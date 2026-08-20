#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_GPIO "esp32.gpio"
#define ESP32_GPIO(obj)             OBJECT_CHECK(Esp32GpioState, (obj), TYPE_ESP32_GPIO)
#define ESP32_GPIO_GET_CLASS(obj)   OBJECT_GET_CLASS(Esp32GpioClass, obj, TYPE_ESP32_GPIO)
#define ESP32_GPIO_CLASS(klass)     OBJECT_CLASS_CHECK(Esp32GpioClass, klass, TYPE_ESP32_GPIO)

/* Low-bank registers. These offsets are the same on the ESP32 and on the
 * RISC-V parts; only the per-pin block and the CPU interrupt register move,
 * so those two are class fields instead. GPIO32-39 on the ESP32 (the OUT1 /
 * IN1 / ENABLE1 bank) are not modelled. */
REG32(GPIO_OUT,          0x0004)
REG32(GPIO_OUT_W1TS,     0x0008)
REG32(GPIO_OUT_W1TC,     0x000c)
REG32(GPIO_ENABLE,       0x0020)
REG32(GPIO_ENABLE_W1TS,  0x0024)
REG32(GPIO_ENABLE_W1TC,  0x0028)
REG32(GPIO_STRAP,        0x0038)
REG32(GPIO_IN,           0x003c)
REG32(GPIO_STATUS,       0x0044)
REG32(GPIO_STATUS_W1TS,  0x0048)
REG32(GPIO_STATUS_W1TC,  0x004c)

/* GPIO_PINn_REG fields used by the guest to arm an interrupt. */
#define ESP32_GPIO_PIN_INT_TYPE_SHIFT 7
#define ESP32_GPIO_PIN_INT_TYPE_MASK  0x7
#define ESP32_GPIO_PIN_INT_ENA_SHIFT  13
#define ESP32_GPIO_PIN_INT_ENA_MASK   0x1f

/* GPIO_PINn_REG int_type encodings. */
enum {
    ESP32_GPIO_INT_DISABLED = 0,
    ESP32_GPIO_INT_POSEDGE  = 1,
    ESP32_GPIO_INT_NEGEDGE  = 2,
    ESP32_GPIO_INT_ANYEDGE  = 3,
    ESP32_GPIO_INT_LOLEVEL  = 4,
    ESP32_GPIO_INT_HILEVEL  = 5,
};

#define ESP32_GPIO_PIN_COUNT 40

#define ESP32_STRAP_MODE_FLASH_BOOT 0x12
#define ESP32_STRAP_MODE_UART_BOOT  0x0f

typedef struct Esp32GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t strap_mode;

    /* Pin state, low bank. `in` is driven from outside the guest: by
     * qemu_host_gpio_set_inputs() in an Emscripten build, and by the
     * "gpio-in" named GPIO lines otherwise. */
    uint32_t out;
    uint32_t enable;
    uint32_t in;
    uint32_t status;
    /* Per-pin GPIO_PINn_REG, only the interrupt fields of which are acted on. */
    uint32_t pin[ESP32_GPIO_PIN_COUNT];

    /* Browser bridge: the page writes `bridge_inputs` from a thread that holds
     * no lock, and `bridge_timer` applies it here, where the BQL is held. See
     * qemu_host_gpio_set_inputs() in hw/gpio/esp32_gpio.c. */
    uint32_t bridge_inputs;
    QEMUTimer *bridge_timer;
} Esp32GpioState;

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;

    /* Offset of GPIO_PIN0_REG: 0x88 on the ESP32, 0x74 on the RISC-V parts. */
    hwaddr pin_reg_base;
    /* Offset of GPIO_PCPU_INT_REG: 0x68 on the ESP32, 0x5c on the RISC-V parts. */
    hwaddr pcpu_int_reg;
    /* Pins this SoC actually brings out. */
    unsigned ngpios;
} Esp32GpioClass;

/*
 * Drive an input pin from the host side and re-evaluate the interrupt state.
 * Used by the browser bridge and by the qdev "gpio-in" lines.
 */
void esp32_gpio_set_input(Esp32GpioState *s, unsigned pin, bool level);
