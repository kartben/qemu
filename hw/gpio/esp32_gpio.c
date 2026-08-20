/*
 * ESP32 GPIO emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"
#include "qemu/main-loop.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

/*
 * Only the low bank is modelled: OUT / ENABLE / IN / STATUS and the per-pin
 * configuration register, which is enough for a guest to drive an LED, read a
 * button and take an interrupt from one. The GPIO matrix and IO MUX are not
 * decoded here; a pin's function select lives in the IO MUX block and the
 * default (simple GPIO) is what the registers below describe.
 */

static void esp32_gpio_update_irq(Esp32GpioState *s)
{
    /* GPIO_PINn_REG carries a 5-bit int_ena, one bit per CPU/target. Any of
     * them set is enough to forward the pin to the single IRQ line here. */
    Esp32GpioClass *class = ESP32_GPIO_GET_CLASS(s);
    uint32_t pending = 0;

    for (unsigned i = 0; i < class->ngpios && i < ESP32_GPIO_PIN_COUNT; i++) {
        uint32_t ena = (s->pin[i] >> ESP32_GPIO_PIN_INT_ENA_SHIFT)
                       & ESP32_GPIO_PIN_INT_ENA_MASK;
        if (ena && (s->status & BIT(i))) {
            pending |= BIT(i);
        }
    }
    qemu_set_irq(s->irq, pending != 0);
}

/* Latch an edge or level into GPIO_STATUS according to GPIO_PINn_REG. */
static void esp32_gpio_latch_status(Esp32GpioState *s, unsigned pin,
                                    bool old_level, bool new_level)
{
    uint32_t type = (s->pin[pin] >> ESP32_GPIO_PIN_INT_TYPE_SHIFT)
                    & ESP32_GPIO_PIN_INT_TYPE_MASK;
    bool fire = false;

    switch (type) {
    case ESP32_GPIO_INT_POSEDGE:
        fire = !old_level && new_level;
        break;
    case ESP32_GPIO_INT_NEGEDGE:
        fire = old_level && !new_level;
        break;
    case ESP32_GPIO_INT_ANYEDGE:
        fire = old_level != new_level;
        break;
    case ESP32_GPIO_INT_LOLEVEL:
        fire = !new_level;
        break;
    case ESP32_GPIO_INT_HILEVEL:
        fire = new_level;
        break;
    default:
        break;
    }

    if (fire) {
        s->status |= BIT(pin);
    }
}

void esp32_gpio_set_input(Esp32GpioState *s, unsigned pin, bool level)
{
    Esp32GpioClass *class = ESP32_GPIO_GET_CLASS(s);

    if (pin >= class->ngpios || pin >= ESP32_GPIO_PIN_COUNT) {
        return;
    }

    bool old_level = (s->in & BIT(pin)) != 0;
    if (level) {
        s->in |= BIT(pin);
    } else {
        s->in &= ~BIT(pin);
    }

    esp32_gpio_latch_status(s, pin, old_level, level);
    esp32_gpio_update_irq(s);
}

/*
 * A pin the guest drives reads back its own output, which is what a real pad
 * does when the output driver is enabled. Pins the guest is not driving keep
 * whatever the host put in `in`.
 */
static uint32_t esp32_gpio_in_value(Esp32GpioState *s)
{
    return (s->in & ~s->enable) | (s->out & s->enable);
}

static void esp32_gpio_set_out(Esp32GpioState *s, uint32_t value)
{
    Esp32GpioClass *class = ESP32_GPIO_GET_CLASS(s);
    uint32_t changed = (s->out ^ value) & s->enable;

    s->out = value;

    /* A pin driven by the guest can raise its own interrupt. */
    for (unsigned i = 0; i < class->ngpios && i < ESP32_GPIO_PIN_COUNT; i++) {
        if (changed & BIT(i)) {
            esp32_gpio_latch_status(s, i, !(value & BIT(i)), (value & BIT(i)) != 0);
        }
    }
    esp32_gpio_update_irq(s);
}

static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    Esp32GpioClass *class = ESP32_GPIO_GET_CLASS(s);
    uint64_t r = 0;

    if (addr >= class->pin_reg_base &&
        addr < class->pin_reg_base + ESP32_GPIO_PIN_COUNT * 4) {
        return s->pin[(addr - class->pin_reg_base) / 4];
    }
    if (addr == class->pcpu_int_reg) {
        return s->status;
    }

    switch (addr) {
    case A_GPIO_STRAP:
        r = s->strap_mode;
        break;
    case A_GPIO_OUT:
        r = s->out;
        break;
    case A_GPIO_ENABLE:
        r = s->enable;
        break;
    case A_GPIO_IN:
        r = esp32_gpio_in_value(s);
        break;
    case A_GPIO_STATUS:
        r = s->status;
        break;
    default:
        break;
    }
    return r;
}

static void esp32_gpio_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    Esp32GpioClass *class = ESP32_GPIO_GET_CLASS(s);

    if (addr >= class->pin_reg_base &&
        addr < class->pin_reg_base + ESP32_GPIO_PIN_COUNT * 4) {
        s->pin[(addr - class->pin_reg_base) / 4] = value;
        /* Arming a level-triggered interrupt on a pin already at that level
         * has to latch immediately, not wait for the next transition. */
        unsigned pin = (addr - class->pin_reg_base) / 4;
        bool level = (esp32_gpio_in_value(s) & BIT(pin)) != 0;
        esp32_gpio_latch_status(s, pin, level, level);
        esp32_gpio_update_irq(s);
        return;
    }

    switch (addr) {
    case A_GPIO_OUT:
        esp32_gpio_set_out(s, value);
        break;
    case A_GPIO_OUT_W1TS:
        esp32_gpio_set_out(s, s->out | value);
        break;
    case A_GPIO_OUT_W1TC:
        esp32_gpio_set_out(s, s->out & ~value);
        break;
    case A_GPIO_ENABLE:
        s->enable = value;
        break;
    case A_GPIO_ENABLE_W1TS:
        s->enable |= value;
        break;
    case A_GPIO_ENABLE_W1TC:
        s->enable &= ~value;
        break;
    case A_GPIO_STATUS:
        s->status = value;
        esp32_gpio_update_irq(s);
        break;
    case A_GPIO_STATUS_W1TS:
        s->status |= value;
        esp32_gpio_update_irq(s);
        break;
    case A_GPIO_STATUS_W1TC:
        s->status &= ~value;
        esp32_gpio_update_irq(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps uart_ops = {
    .read =  esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * Browser bridge. The page drives inputs and polls outputs through these two
 * exported functions, the same pair the Cortex-M3's qemu,host-gpio device
 * exports, so src/hostGpio.ts binds to either without knowing the difference.
 *
 * Only one GPIO controller exists per machine, so a file-static pointer is
 * enough; it is set when the device is realized.
 */
#ifdef __EMSCRIPTEN__
static Esp32GpioState *esp32_gpio_bridge;

EMSCRIPTEN_KEEPALIVE
void qemu_host_gpio_set_inputs(uint32_t value)
{
    Esp32GpioState *s = esp32_gpio_bridge;
    if (s == NULL) {
        return;
    }
    /* Called from the page, which is not a vCPU thread and holds no lock. A
     * pin change can raise the controller's interrupt, and delivering one ends
     * up in cpu_interrupt(), which asserts bql_locked(). Take the lock for the
     * whole update so the guest also sees every pin move at once.
     *
     * Only the ESP32 tripped this in practice, because its interrupt matrix
     * routes per-CPU and kicks the vCPU directly; the C3 survived a press
     * without it. The requirement is the same on both, so the lock belongs
     * here rather than in either machine. */
    bql_lock();
    for (unsigned i = 0; i < ESP32_GPIO_PIN_COUNT; i++) {
        esp32_gpio_set_input(s, i, (value & BIT(i)) != 0);
    }
    bql_unlock();
}

EMSCRIPTEN_KEEPALIVE
uint32_t qemu_host_gpio_get_outputs(void)
{
    Esp32GpioState *s = esp32_gpio_bridge;
    return s == NULL ? 0 : (s->out & s->enable);
}
#endif

static void esp32_gpio_reset_hold(Object *obj, ResetType type)
{
    Esp32GpioState *s = ESP32_GPIO(obj);

    s->out = 0;
    s->enable = 0;
    s->in = 0;
    s->status = 0;
    memset(s->pin, 0, sizeof(s->pin));
    qemu_set_irq(s->irq, 0);
}

static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
#ifdef __EMSCRIPTEN__
    esp32_gpio_bridge = ESP32_GPIO(dev);
#endif
}

static void esp32_gpio_gpio_in(void *opaque, int n, int level)
{
    esp32_gpio_set_input(ESP32_GPIO(opaque), n, level != 0);
}

static void esp32_gpio_init(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Set the default value for the strap_mode property */
    object_property_set_int(obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT, &error_fatal);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), esp32_gpio_gpio_in, "gpio-in",
                            ESP32_GPIO_PIN_COUNT);
}

static const Property esp32_gpio_properties[] = {
    /* The strap_mode needs to be explicitly set in the instance init, thus, set
     * the default value to 0. */
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
};

static void esp32_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    Esp32GpioClass *gc = ESP32_GPIO_CLASS(klass);

    rc->phases.hold = esp32_gpio_reset_hold;
    dc->realize = esp32_gpio_realize;
    device_class_set_props(dc, esp32_gpio_properties);

    /* ESP32 layout; the RISC-V parts override these. */
    gc->pin_reg_base = 0x88;
    gc->pcpu_int_reg = 0x68;
    gc->ngpios = 40;
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
