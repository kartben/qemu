/*
 * SPI peripheral backed by device models in the browser.
 *
 * Copyright (c) 2026 Benjamin Cabé
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The SPI half of what hw/i2c/host_i2c.c does for I2C, and deliberately the
 * same shape: a struct in the wasm heap that both sides read and write, a
 * presence mask so "is anything on this chip select" costs nothing, and one
 * request at a time with the guest's thread futex-parked until the page
 * answers. See docs/esp32.md for the layout and why it is safe to block from
 * guest context under Asyncify.
 *
 * What differs is the granularity. SPI is full duplex, so a byte cannot be
 * answered before it is sent, and a per-byte round trip would make a strip of
 * LEDs or a flash page program hundreds of them. The controller knows how long
 * its run is, so ssi_transfer_buffer() hands the whole run over at once, and
 * this is the peripheral that implements it. A run is what the page's chip
 * models already expect: one full-duplex exchange, plus whether the select
 * drops afterwards, which is how a chip knows its command ended.
 *
 * One instance serves one chip select, because that is what an SSI peripheral
 * is; the machine puts it on CS0, which is where every chip the page models
 * sits.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/ssi/host_spi.h"
#include "hw/qdev-properties.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/threading.h>
#endif

/* A WS2812 strip frame and a flash page program are the big ones. */
#define HOST_SPI_BUF 4096

#define HOST_SPI_TIMEOUT_MS 250

#define HOST_SPI_MAGIC   0x42535053u /* "SPSB" */
#define HOST_SPI_VERSION 1

#define HOST_SPI_OP_TRANSFER 1

/** The controller deasserts the select after this run. */
#define HOST_SPI_F_CS_RELEASE (1u << 0)

#define HOST_SPI_STATUS_OK  0
#define HOST_SPI_STATUS_ERR 1

#ifdef __EMSCRIPTEN__

typedef struct HostSpiArea {
    uint32_t magic;
    uint32_t version;
    /* Bit N set = the page has a chip on chip select N. Written by the page. */
    uint32_t present;
    /* Set by the page while it is listening; cleared when it goes away. */
    uint32_t attached;
    int32_t req_seq;
    int32_t rsp_seq;
    uint32_t op;
    uint32_t cs;
    uint32_t len;
    uint32_t flags;
    int32_t status;
    uint32_t reserved;
    uint8_t data[HOST_SPI_BUF];
} HostSpiArea;

static HostSpiArea host_spi_area = {
    .magic = HOST_SPI_MAGIC,
    .version = HOST_SPI_VERSION,
};

static bool host_spi_realized;

#endif /* __EMSCRIPTEN__ */

struct HostSPIState {
    SSIPeripheral parent_obj;

    /*
     * Native builds have no page. A MOSI-to-MISO wire is the honest stand-in
     * and gives the path something to answer with, so it can be exercised
     * outside a browser:
     *
     *   -global qemu-host-spi.loopback=on
     */
    bool loopback;
};

/* ------------------------------------------------------------------ page */

#ifdef __EMSCRIPTEN__

/** Pointer to the shared area, or 0 when this machine has no bridge. */
EMSCRIPTEN_KEEPALIVE
uint32_t qemu_host_spi_area(void)
{
    return host_spi_realized ? (uint32_t)(uintptr_t)&host_spi_area : 0;
}

static bool host_spi_page_attached(void)
{
    return qatomic_read(&host_spi_area.attached) != 0;
}

static bool host_spi_page_present(uint8_t cs)
{
    return (qatomic_read(&host_spi_area.present) >> (cs & 31)) & 1;
}

static bool host_spi_page_transfer(uint8_t cs, const uint8_t *tx, uint8_t *rx,
                                   uint32_t len, uint32_t flags)
{
    int32_t seq;
    int64_t deadline;

    if (!host_spi_page_attached() || emscripten_is_main_browser_thread()) {
        return false;
    }

    host_spi_area.op = HOST_SPI_OP_TRANSFER;
    host_spi_area.cs = cs;
    host_spi_area.len = len;
    host_spi_area.flags = flags;
    host_spi_area.status = HOST_SPI_STATUS_ERR;
    if (len > 0) {
        memcpy(host_spi_area.data, tx, len);
    }

    seq = host_spi_area.req_seq + 1;
    qatomic_set(&host_spi_area.req_seq, seq);
    emscripten_futex_wake(&host_spi_area.req_seq, INT32_MAX);

    deadline = g_get_monotonic_time() + HOST_SPI_TIMEOUT_MS * 1000;
    while (qatomic_read(&host_spi_area.rsp_seq) != seq) {
        int64_t now = g_get_monotonic_time();

        if (now >= deadline) {
            qemu_log_mask(LOG_UNIMP,
                          "host-spi: no answer from the page in %d ms\n",
                          HOST_SPI_TIMEOUT_MS);
            return false;
        }
        emscripten_futex_wait(&host_spi_area.rsp_seq, (uint32_t)(seq - 1),
                              (double)(deadline - now) / 1000.0);
    }

    if (host_spi_area.status != HOST_SPI_STATUS_OK) {
        return false;
    }
    if (len > 0) {
        memcpy(rx, host_spi_area.data, len);
    }
    return true;
}

#endif /* __EMSCRIPTEN__ */

/* ----------------------------------------------------------------- chips */

/** The chip select line this instance sits on, from the SSI `cs` property. */
static uint8_t host_spi_cs(HostSPIState *s)
{
    return SSI_PERIPHERAL(s)->cs_index;
}

static bool host_spi_present(HostSPIState *s)
{
#ifdef __EMSCRIPTEN__
    if (host_spi_page_attached()) {
        return host_spi_page_present(host_spi_cs(s));
    }
#endif
    return s->loopback;
}

static bool host_spi_exchange(HostSPIState *s, const uint8_t *tx, uint8_t *rx,
                              uint32_t len, bool cs_release)
{
#ifdef __EMSCRIPTEN__
    if (host_spi_page_attached()) {
        return host_spi_page_transfer(host_spi_cs(s), tx, rx, len,
                                      cs_release ? HOST_SPI_F_CS_RELEASE : 0);
    }
#endif
    if (!s->loopback) {
        return false;
    }
    /* MOSI shorted to MISO: every byte comes straight back. */
    if (rx != tx) {
        memcpy(rx, tx, len);
    }
    return true;
}

/* --------------------------------------------------------- SSIPeripheral */

static bool host_spi_transfer_buffer(SSIPeripheral *dev, const uint8_t *tx,
                                     uint8_t *rx, uint32_t len,
                                     bool cs_release)
{
    HostSPIState *s = HOST_SPI(dev);

    if (!host_spi_present(s)) {
        return false;
    }
    if (len > HOST_SPI_BUF) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "host-spi: %u byte run exceeds the %d byte buffer\n",
                      len, HOST_SPI_BUF);
        return false;
    }
    return host_spi_exchange(s, tx, rx, len, cs_release);
}

/*
 * A controller that clocks byte at a time still works, one round trip per
 * byte. Nothing on this machine does — the ESP32 GP-SPI announces its run —
 * but a peripheral that only answers in bulk would be a trap for the next one.
 */
static uint32_t host_spi_transfer(SSIPeripheral *dev, uint32_t val)
{
    HostSPIState *s = HOST_SPI(dev);
    uint8_t tx = val & 0xff;
    uint8_t rx = 0;

    /*
     * Zero, not 0xff, when the page has nothing on this select: ssi_transfer()
     * ORs its peripherals together and an empty bus reads as zero, so this
     * leaves a chip select with no chip indistinguishable from one with no
     * bridge. It matters — a flash driver reading 0xff as its status register
     * sees the write-in-progress bit set and waits for it forever.
     */
    if (!host_spi_present(s) || !host_spi_exchange(s, &tx, &rx, 1, false)) {
        return 0;
    }
    return rx;
}

static void host_spi_realize(SSIPeripheral *dev, Error **errp)
{
#ifdef __EMSCRIPTEN__
    host_spi_realized = true;
#endif
}

static const Property host_spi_props[] = {
    DEFINE_PROP_BOOL("loopback", HostSPIState, loopback, false),
};

static void host_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = host_spi_realize;
    k->transfer = host_spi_transfer;
    k->transfer_buffer = host_spi_transfer_buffer;
    k->cs_polarity = SSI_CS_LOW;
    device_class_set_props(dc, host_spi_props);
}

static const TypeInfo host_spi_type_info = {
    .name = TYPE_HOST_SPI,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(HostSPIState),
    .class_init = host_spi_class_init,
};

static void host_spi_register_types(void)
{
    type_register_static(&host_spi_type_info);
}

type_init(host_spi_register_types)
