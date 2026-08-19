/*
 * ESP32-C3 general-purpose SPI controller (GP-SPI2).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/ssi/ssi.h"

#define TYPE_ESP32C3_GPSPI "ssi.esp32c3.gpspi"
#define ESP32C3_GPSPI(obj) OBJECT_CHECK(Esp32C3GpSpiState, (obj), TYPE_ESP32C3_GPSPI)

#define ESP32C3_GPSPI_IO_SIZE   0x1000
/* W0..W15: the 64-byte CPU-controlled data buffer. */
#define ESP32C3_GPSPI_BUF_WORDS 16
/* CS0..CS5. Only CS0 is pinned out on the DevKitC. */
#define ESP32C3_GPSPI_CS_COUNT  6

typedef struct Esp32C3GpSpiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    SSIBus *spi;
    qemu_irq cs_gpio[ESP32C3_GPSPI_CS_COUNT];

    uint32_t addr_reg;
    uint32_t ctrl_reg;
    uint32_t clock_reg;
    uint32_t user_reg;
    uint32_t user1_reg;
    uint32_t user2_reg;
    uint32_t ms_dlen_reg;
    uint32_t misc_reg;
    uint32_t dma_conf_reg;
    uint32_t int_ena_reg;
    uint32_t int_raw_reg;
    uint32_t slave_reg;
    uint32_t clk_gate_reg;
    uint32_t data_reg[ESP32C3_GPSPI_BUF_WORDS];

    /** Whether a chip select is still held from a previous transfer. */
    bool cs_held;
} Esp32C3GpSpiState;

/*
 * Register map. This is GP-SPI2, not the `SPI_MEM_*` flash controller in
 * esp32c3_spi.c: same peripheral family, different block, and every offset
 * moves. Taken from the ESP32-C3 TRM by way of soc/spi_reg.h.
 */
REG32(GPSPI_CMD, 0x000)
    FIELD(GPSPI_CMD, CONF_BITLEN, 0, 18)
    /* Self-clearing: spi_ll_apply_config() spins until the model drops it. */
    FIELD(GPSPI_CMD, UPDATE, 23, 1)
    FIELD(GPSPI_CMD, USR, 24, 1)

REG32(GPSPI_ADDR, 0x004)

REG32(GPSPI_CTRL, 0x008)

REG32(GPSPI_CLOCK, 0x00c)

REG32(GPSPI_USER, 0x010)
    FIELD(GPSPI_USER, DOUTDIN, 0, 1)
    FIELD(GPSPI_USER, MISO_HIGHPART, 24, 1)
    FIELD(GPSPI_USER, MOSI_HIGHPART, 25, 1)
    FIELD(GPSPI_USER, MOSI, 27, 1)
    FIELD(GPSPI_USER, MISO, 28, 1)
    FIELD(GPSPI_USER, DUMMY, 29, 1)
    FIELD(GPSPI_USER, ADDR, 30, 1)
    FIELD(GPSPI_USER, COMMAND, 31, 1)

REG32(GPSPI_USER1, 0x014)
    FIELD(GPSPI_USER1, DUMMY_CYCLELEN, 0, 8)
    FIELD(GPSPI_USER1, ADDR_BITLEN, 27, 5)

REG32(GPSPI_USER2, 0x018)
    FIELD(GPSPI_USER2, COMMAND_VALUE, 0, 16)
    FIELD(GPSPI_USER2, COMMAND_BITLEN, 28, 4)

/* One length for both directions: this part is full duplex by default. */
REG32(GPSPI_MS_DLEN, 0x01c)
    FIELD(GPSPI_MS_DLEN, MS_DATA_BITLEN, 0, 18)

REG32(GPSPI_MISC, 0x020)
    FIELD(GPSPI_MISC, CS_DIS, 0, 6)
    FIELD(GPSPI_MISC, CK_IDLE_EDGE, 29, 1)
    /* Hold the selected chip across the next transfer's end, so a command and
     * its data can be two SPI_USR transfers within one chip select. */
    FIELD(GPSPI_MISC, CS_KEEP_ACTIVE, 30, 1)

REG32(GPSPI_DMA_CONF, 0x030)

REG32(GPSPI_DMA_INT_ENA, 0x034)
    FIELD(GPSPI_DMA_INT_ENA, TRANS_DONE, 12, 1)
REG32(GPSPI_DMA_INT_CLR, 0x038)
    FIELD(GPSPI_DMA_INT_CLR, TRANS_DONE, 12, 1)
REG32(GPSPI_DMA_INT_RAW, 0x03c)
    FIELD(GPSPI_DMA_INT_RAW, TRANS_DONE, 12, 1)
REG32(GPSPI_DMA_INT_ST, 0x040)

REG32(GPSPI_W0, 0x098)
REG32(GPSPI_W15, 0x0d4)

REG32(GPSPI_SLAVE, 0x0e0)
REG32(GPSPI_SLAVE1, 0x0e4)
REG32(GPSPI_CLK_GATE, 0x0e8)
REG32(GPSPI_DATE, 0x0f0)
