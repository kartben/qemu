/*
 * ESP32-C3 general-purpose SPI controller (GP-SPI2).
 *
 * Copyright (c) 2026 Benjamin Cabé
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Zephyr's spi2 sits at 0x60024000 with nothing behind it: the machine wires
 * only spi1, the flash controller. That controller is *not* this one, tempting
 * as the file next door looks. hw/ssi/esp32c3_spi.c models the SPI memory
 * block, whose registers are the `SPI_MEM_*` set (CMD 0x00, USER 0x18, W0
 * 0x58) with the flash command shortcuts and the XTS-AES coupling; GP-SPI2 is
 * a separate peripheral whose map moves nearly everything (USER 0x10, one
 * MS_DLEN at 0x1c for both directions, W0 0x98) and has no flash shortcuts at
 * all. The nearest relative in the tree is hw/ssi/esp32_spi.c, the original
 * Xtensa ESP32's GP-SPI, whose map differs again; this borrows its transaction
 * engine and nothing else.
 *
 * Only what a controller-mode guest uses is modelled: the USER-defined
 * transaction (command, address, data), which is what Zephyr's stock
 * spi_esp32_spim driver drives through the FIFO when a node has no
 * `dma-enabled`. Target mode, segmented transfers and DMA are not here.
 *
 * Two details are load-bearing rather than cosmetic, because the driver spins
 * on both:
 *
 *   - CMD.UPDATE self-clears. spi_ll_apply_config() writes it and loops until
 *     it reads back zero, so a model that latches it hangs the guest before
 *     the first byte moves.
 *   - Completion is DMA_INT_RAW.TRANS_DONE, not CMD.USR. spi_ll_usr_is_done()
 *     reads the interrupt bit, and the driver clears it through DMA_INT_CLR.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/esp32c3_gpspi.h"

/** One transfer, in the phases the USER registers describe. */
typedef struct Esp32C3GpSpiTransaction {
    uint32_t cmd;
    unsigned int cmd_bytes;
    uint32_t addr;
    unsigned int addr_bytes;
    /** Bytes clocked out, in, or both when the part is full duplex. */
    unsigned int data_bytes;
    bool data_out;
    bool data_in;
} Esp32C3GpSpiTransaction;

/* The bitlen registers hold "bits minus one". */
static inline unsigned int bitlen_to_bytes(uint32_t val)
{
    return (val + 1 + 7) / 8;
}

static void esp32c3_gpspi_update_irq(Esp32C3GpSpiState *s)
{
    qemu_set_irq(s->irq, !!(s->int_raw_reg & s->int_ena_reg));
}

/** Byte `i` of the W0..W15 buffer. The driver memcpy's into it, so byte 0 is
 * the low byte of W0. */
static uint8_t data_byte(const Esp32C3GpSpiState *s, unsigned int i)
{
    return (s->data_reg[i / 4] >> (8 * (i % 4))) & 0xff;
}

static void set_data_byte(Esp32C3GpSpiState *s, unsigned int i, uint8_t value)
{
    const unsigned int shift = 8 * (i % 4);

    s->data_reg[i / 4] = (s->data_reg[i / 4] & ~(0xffu << shift))
                         | ((uint32_t)value << shift);
}

/** Lay a command or address word into the run, most significant byte first. */
static unsigned int put_word(uint8_t *buf, uint32_t word, unsigned int bytes)
{
    for (unsigned int i = 0; i < bytes; i++) {
        buf[i] = (word >> (8 * (bytes - 1 - i))) & 0xff;
    }
    return bytes;
}

static void esp32c3_gpspi_cs_set(Esp32C3GpSpiState *s, int level)
{
    const uint32_t disabled = FIELD_EX32(s->misc_reg, GPSPI_MISC, CS_DIS);

    for (int i = 0; i < ESP32C3_GPSPI_CS_COUNT; i++) {
        /* CSn_DIS set means that line stays idle; the rest follow the
         * transfer, and chip selects are active low. */
        qemu_set_irq(s->cs_gpio[i], (disabled & BIT(i)) ? 1 : level);
    }
}

static void esp32c3_gpspi_transaction(Esp32C3GpSpiState *s,
                                      const Esp32C3GpSpiTransaction *t)
{
    /* Command and address are 2 and 4 bytes at most; data is the W buffer. */
    uint8_t run[8 + ESP32C3_GPSPI_BUF_WORDS * 4];
    unsigned int prefix = 0;
    unsigned int len;
    bool cs_release = FIELD_EX32(s->misc_reg, GPSPI_MISC, CS_KEEP_ACTIVE) == 0;

    if (!s->cs_held) {
        esp32c3_gpspi_cs_set(s, 0);
    }

    prefix += put_word(run + prefix, t->cmd, t->cmd_bytes);
    prefix += put_word(run + prefix, t->addr, t->addr_bytes);
    for (unsigned int i = 0; i < t->data_bytes; i++) {
        /* A read-only phase still has to clock something out. */
        run[prefix + i] = t->data_out ? data_byte(s, i) : 0xff;
    }
    len = prefix + t->data_bytes;

    /*
     * One call when the peripheral can take the whole run (the browser bridge
     * does, and a round trip per byte would be unaffordable), a byte at a time
     * for everything else. The prefix travels with it either way: on the wire
     * a command and its data are one unbroken stream, and a device that
     * decodes the stream needs to see the command that started it.
     */
    if (!ssi_transfer_buffer(s->spi, run, run, len, cs_release)) {
        for (unsigned int i = 0; i < len; i++) {
            run[i] = ssi_transfer(s->spi, run[i]) & 0xff;
        }
    }

    if (t->data_in) {
        for (unsigned int i = 0; i < t->data_bytes; i++) {
            set_data_byte(s, i, run[prefix + i]);
        }
    }

    /*
     * A command and its data reach this model as two SPI_USR transfers, and a
     * chip that saw the select drop in between would treat the second as a new
     * command. The driver says so with CS_KEEP_ACTIVE, set for every buffer of
     * a transfer but the last.
     */
    s->cs_held = !cs_release;
    if (!s->cs_held) {
        esp32c3_gpspi_cs_set(s, 1);
    }
}

static void esp32c3_gpspi_do_user_transfer(Esp32C3GpSpiState *s)
{
    Esp32C3GpSpiTransaction t = { 0 };

    if (FIELD_EX32(s->user_reg, GPSPI_USER, COMMAND)) {
        t.cmd = FIELD_EX32(s->user2_reg, GPSPI_USER2, COMMAND_VALUE);
        t.cmd_bytes = bitlen_to_bytes(
            FIELD_EX32(s->user2_reg, GPSPI_USER2, COMMAND_BITLEN));
    }
    if (FIELD_EX32(s->user_reg, GPSPI_USER, ADDR)) {
        t.addr_bytes = bitlen_to_bytes(
            FIELD_EX32(s->user1_reg, GPSPI_USER1, ADDR_BITLEN));
        t.addr = s->addr_reg >> (32 - t.addr_bytes * 8);
    }
    if (FIELD_EX32(s->user_reg, GPSPI_USER, DUMMY)) {
        /* Nothing on this bus is clocked without a byte to carry it, and no
         * guest here asks for dummy cycles. Say so rather than silently
         * shifting the data phase. */
        qemu_log_mask(LOG_UNIMP, "esp32c3_gpspi: dummy cycles ignored\n");
    }

    t.data_out = FIELD_EX32(s->user_reg, GPSPI_USER, MOSI) != 0;
    t.data_in = FIELD_EX32(s->user_reg, GPSPI_USER, MISO) != 0;
    if (t.data_out || t.data_in) {
        unsigned int bytes = bitlen_to_bytes(
            FIELD_EX32(s->ms_dlen_reg, GPSPI_MS_DLEN, MS_DATA_BITLEN));

        if (bytes > ESP32C3_GPSPI_BUF_WORDS * 4) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32c3_gpspi: %u byte transfer exceeds the %d byte"
                          " buffer\n", bytes, ESP32C3_GPSPI_BUF_WORDS * 4);
            bytes = ESP32C3_GPSPI_BUF_WORDS * 4;
        }
        t.data_bytes = bytes;
    }

    esp32c3_gpspi_transaction(s, &t);

    s->int_raw_reg = FIELD_DP32(s->int_raw_reg, GPSPI_DMA_INT_RAW,
                                TRANS_DONE, 1);
    esp32c3_gpspi_update_irq(s);
}

static uint64_t esp32c3_gpspi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32C3GpSpiState *s = ESP32C3_GPSPI(opaque);

    switch (addr) {
    case A_GPSPI_CMD:
        /* USR and UPDATE both complete within the write that set them. */
        return 0;
    case A_GPSPI_ADDR:
        return s->addr_reg;
    case A_GPSPI_CTRL:
        return s->ctrl_reg;
    case A_GPSPI_CLOCK:
        return s->clock_reg;
    case A_GPSPI_USER:
        return s->user_reg;
    case A_GPSPI_USER1:
        return s->user1_reg;
    case A_GPSPI_USER2:
        return s->user2_reg;
    case A_GPSPI_MS_DLEN:
        return s->ms_dlen_reg;
    case A_GPSPI_MISC:
        return s->misc_reg;
    case A_GPSPI_DMA_CONF:
        return s->dma_conf_reg;
    case A_GPSPI_DMA_INT_ENA:
        return s->int_ena_reg;
    case A_GPSPI_DMA_INT_RAW:
        return s->int_raw_reg;
    case A_GPSPI_DMA_INT_ST:
        return s->int_raw_reg & s->int_ena_reg;
    case A_GPSPI_W0 ... A_GPSPI_W15:
        return s->data_reg[(addr - A_GPSPI_W0) / 4];
    case A_GPSPI_SLAVE:
        return s->slave_reg;
    case A_GPSPI_CLK_GATE:
        return s->clk_gate_reg;
    case A_GPSPI_DATE:
        /* Version word the driver reads but does not check. */
        return 0x02007000;
    default:
        return 0;
    }
}

static void esp32c3_gpspi_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    Esp32C3GpSpiState *s = ESP32C3_GPSPI(opaque);

    switch (addr) {
    case A_GPSPI_CMD:
        /* UPDATE latches the configuration registers, which for a model that
         * reads them at transfer time is a no-op that has to clear itself. */
        if (FIELD_EX32(value, GPSPI_CMD, USR)) {
            esp32c3_gpspi_do_user_transfer(s);
        }
        break;
    case A_GPSPI_ADDR:
        s->addr_reg = value;
        break;
    case A_GPSPI_CTRL:
        s->ctrl_reg = value;
        break;
    case A_GPSPI_CLOCK:
        s->clock_reg = value;
        break;
    case A_GPSPI_USER:
        s->user_reg = value;
        break;
    case A_GPSPI_USER1:
        s->user1_reg = value;
        break;
    case A_GPSPI_USER2:
        s->user2_reg = value;
        break;
    case A_GPSPI_MS_DLEN:
        s->ms_dlen_reg = value;
        break;
    case A_GPSPI_MISC:
        s->misc_reg = value;
        break;
    case A_GPSPI_DMA_CONF:
        s->dma_conf_reg = value;
        break;
    case A_GPSPI_DMA_INT_ENA:
        s->int_ena_reg = value;
        esp32c3_gpspi_update_irq(s);
        break;
    case A_GPSPI_DMA_INT_CLR:
        s->int_raw_reg &= ~(uint32_t)value;
        esp32c3_gpspi_update_irq(s);
        break;
    case A_GPSPI_W0 ... A_GPSPI_W15:
        s->data_reg[(addr - A_GPSPI_W0) / 4] = value;
        break;
    case A_GPSPI_SLAVE:
        s->slave_reg = value;
        break;
    case A_GPSPI_CLK_GATE:
        s->clk_gate_reg = value;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps esp32c3_gpspi_ops = {
    .read = esp32c3_gpspi_read,
    .write = esp32c3_gpspi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_gpspi_reset_hold(Object *obj, ResetType type)
{
    Esp32C3GpSpiState *s = ESP32C3_GPSPI(obj);

    s->addr_reg = 0;
    s->ctrl_reg = 0;
    s->clock_reg = 0;
    s->user_reg = 0;
    s->user1_reg = FIELD_DP32(0, GPSPI_USER1, ADDR_BITLEN, 23);
    s->user1_reg = FIELD_DP32(s->user1_reg, GPSPI_USER1, DUMMY_CYCLELEN, 7);
    s->user2_reg = FIELD_DP32(0, GPSPI_USER2, COMMAND_BITLEN, 7);
    s->ms_dlen_reg = 0;
    /* Every chip select idle out of reset. */
    s->misc_reg = FIELD_DP32(0, GPSPI_MISC, CS_DIS, 0x3f);
    s->dma_conf_reg = 0;
    s->int_ena_reg = 0;
    s->int_raw_reg = 0;
    s->slave_reg = 0;
    s->clk_gate_reg = 0;
    memset(s->data_reg, 0, sizeof(s->data_reg));
    s->cs_held = false;
    esp32c3_gpspi_cs_set(s, 1);
}

static void esp32c3_gpspi_init(Object *obj)
{
    Esp32C3GpSpiState *s = ESP32C3_GPSPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_gpspi_ops, s,
                          TYPE_ESP32C3_GPSPI, ESP32C3_GPSPI_IO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    /* Not "spi": spi1's flash controller already has a bus by that name, and
     * a distinct one is what makes `-device <part>,bus=gpspi` reach this
     * controller rather than that one. */
    s->spi = ssi_create_bus(DEVICE(s), "gpspi");
    qdev_init_gpio_out_named(DEVICE(s), s->cs_gpio, SSI_GPIO_CS,
                             ESP32C3_GPSPI_CS_COUNT);
}

static void esp32c3_gpspi_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32c3_gpspi_reset_hold;
}

static const TypeInfo esp32c3_gpspi_type_info = {
    .name = TYPE_ESP32C3_GPSPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32C3GpSpiState),
    .instance_init = esp32c3_gpspi_init,
    .class_init = esp32c3_gpspi_class_init,
};

static void esp32c3_gpspi_register_types(void)
{
    type_register_static(&esp32c3_gpspi_type_info);
}

type_init(esp32c3_gpspi_register_types)
