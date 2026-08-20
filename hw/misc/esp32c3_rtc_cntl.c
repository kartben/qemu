/*
 * ESP32-C3 RTC CNTL
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
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
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/misc/esp32c3_rtc_cntl.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif


#define RTCCNTL_DEBUG     0
#define RTCCNTL_WARNING   0


/** Nominal RTC_SLOW_CLK: the 150 kHz RC oscillator the C3 boots on. */
#define ESP32C3_RTC_SLOW_CLK_HZ 150000

/*
 * What the page shows on its power card.
 *
 * Read-only and one-directional, so unlike the I2C, SPI and CAN bridges there
 * is no protocol here: the model updates it at the two moments that matter and
 * the page reads it whenever it repaints. Sleep is otherwise invisible - the
 * guest stops printing and there is nothing to see - which is exactly the kind
 * of thing this project puts in the dock.
 */
#define ESP32C3_RTC_STATUS_MAGIC   0x53435452u /* "RTCS" */
#define ESP32C3_RTC_STATUS_VERSION 1

/** Values of `state`. */
#define ESP32C3_RTC_AWAKE       0
#define ESP32C3_RTC_LIGHT_SLEEP 1
#define ESP32C3_RTC_DEEP_SLEEP  2

typedef struct Esp32C3RtcStatus {
    uint32_t magic;
    uint32_t version;
    /** Awake, or which kind of sleep is in progress. */
    uint32_t state;
    /** Reset reason, as ESP32C3ResetReason. Survives a deep sleep. */
    uint32_t reset_reason;
    /** Sleeps entered, and sleeps the model refused for want of a wake source. */
    uint32_t sleep_count;
    uint32_t reject_count;
    /** RTC_CNTL_SLP_WAKEUP_CAUSE after the last wake. */
    uint32_t wake_cause;
    /** Duration the last sleep asked for, and the total so far, in microseconds. */
    uint32_t last_sleep_us;
    uint32_t total_sleep_us;
    /** RTC slow-clock counter, as of the last sleep, wake or guest read. */
    uint32_t rtc_ticks_low;
    uint32_t rtc_ticks_high;
} Esp32C3RtcStatus;

static Esp32C3RtcStatus esp32c3_rtc_status = {
    .magic = ESP32C3_RTC_STATUS_MAGIC,
    .version = ESP32C3_RTC_STATUS_VERSION,
};

#ifdef __EMSCRIPTEN__
EMSCRIPTEN_KEEPALIVE
uint32_t qemu_esp32c3_rtc_status(void)
{
    return (uint32_t)(uintptr_t)&esp32c3_rtc_status;
}
#endif

static void esp32c3_rtc_status_time(uint64_t ticks)
{
    esp32c3_rtc_status.rtc_ticks_low = (uint32_t)ticks;
    esp32c3_rtc_status.rtc_ticks_high = (uint32_t)(ticks >> 32);
}

static uint32_t esp32c3_ticks_to_us(uint64_t ticks)
{
    return (uint32_t)muldiv64(ticks, 1000000, ESP32C3_RTC_SLOW_CLK_HZ);
}

static uint64_t esp32c3_rtc_slow_ticks(void)
{
    uint64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    return muldiv64(ns, ESP32C3_RTC_SLOW_CLK_HZ, NANOSECONDS_PER_SECOND);
}



static void esp32c3_reset_request(void *opaque, int n, int level)
{
    ESP32C3RtcCntlState *s = ESP32C3_RTC_CNTL(opaque);
    /* Make sure the "reset reason" is correct */
    assert(n < ESP32C3_COUNT_RESET);

    if (level) {
        s->reason = n;
        esp32c3_rtc_status.reset_reason = n;
        qemu_irq_raise(s->cpu_reset);
    }
}


/*
 * Sleep, and the counter that ends it.
 *
 * The chip sleeps by arming a target on the RTC slow clock and setting
 * STATE0.SLEEP_EN; ESP-IDF then spins on INT_RAW waiting for either a wakeup
 * or a rejection. Deep sleep never finishes that loop on hardware, because the
 * digital core is powered down and the part comes back through reset - which
 * is why the reason register and the RTC scratch registers survive one, and
 * how a guest tells a cold boot from a wake.
 *
 * Modelled at that level and no deeper: a virtual-clock timer for the target,
 * a reset for deep sleep, and the wakeup interrupt for light sleep. The two
 * are told apart by DIG_PWC.DG_WRAP_PD_EN, which is what powers the digital
 * core down.
 */

static void esp32c3_rtc_cntl_wake(void *opaque)
{
    ESP32C3RtcCntlState *s = ESP32C3_RTC_CNTL(opaque);

    /* Whatever was enabled as a wake source got us here; the timer is the
     * only one this models, so report the set the guest asked for. */
    s->slp_wakeup_cause = FIELD_EX32(s->wakeup_state, RTC_CNTL_RTC_WAKEUP_STATE,
                                     RTC_WAKEUP_ENA);
    s->state0 = FIELD_DP32(s->state0, RTC_CNTL_RTC_STATE0, SLEEP_EN, 0);

    esp32c3_rtc_status.state = ESP32C3_RTC_AWAKE;
    esp32c3_rtc_status.wake_cause = s->slp_wakeup_cause;
    esp32c3_rtc_status.total_sleep_us += esp32c3_rtc_status.last_sleep_us;
    esp32c3_rtc_status_time(esp32c3_rtc_slow_ticks());

    if (s->sleep_is_deep) {
        /*
         * Deliberately no wakeup interrupt: on hardware the core is off and
         * never observes one. The guest is still spinning in rtc_sleep_start()
         * here, and the reset is what takes it out.
         */
        esp32c3_reset_request(opaque, ESP32C3_DEEPSLEEP_RESET, 1);
        return;
    }

    s->state0 = FIELD_DP32(s->state0, RTC_CNTL_RTC_STATE0, SLP_WAKEUP, 1);
    s->int_raw = FIELD_DP32(s->int_raw, RTC_CNTL_INT_RAW_RTC, SLP_WAKEUP_INT_RAW, 1);
}

static void esp32c3_rtc_cntl_sleep(ESP32C3RtcCntlState *s)
{
    uint64_t target = ((uint64_t)(s->slp_timer1 & 0xffff) << 32) | s->slp_timer0;
    uint64_t now = esp32c3_rtc_slow_ticks();
    uint64_t ticks = target > now ? target - now : 0;

    s->sleep_is_deep = FIELD_EX32(s->dig_pwc, RTC_CNTL_DIG_PWC,
                                  DG_WRAP_PD_EN) != 0;

    if (!FIELD_EX32(s->slp_timer1, RTC_CNTL_RTC_SLP_TIMER1, RTC_MAIN_TIMER_ALARM_EN)) {
        /*
         * No timer armed and nothing else here can wake the part. Reject the
         * sleep rather than stopping forever: a guest that asked for a sleep
         * it cannot leave gets an error it can report.
         */
        s->int_raw = FIELD_DP32(s->int_raw, RTC_CNTL_INT_RAW_RTC,
                                SLP_REJECT_INT_RAW, 1);
        s->state0 = FIELD_DP32(s->state0, RTC_CNTL_RTC_STATE0, SLP_REJECT, 1);
        s->state0 = FIELD_DP32(s->state0, RTC_CNTL_RTC_STATE0, SLEEP_EN, 0);
        esp32c3_rtc_status.reject_count++;
        return;
    }

    esp32c3_rtc_status.state = s->sleep_is_deep ? ESP32C3_RTC_DEEP_SLEEP
                                                : ESP32C3_RTC_LIGHT_SLEEP;
    esp32c3_rtc_status.sleep_count++;
    esp32c3_rtc_status.last_sleep_us = esp32c3_ticks_to_us(ticks);
    esp32c3_rtc_status_time(now);

    timer_mod_ns(s->sleep_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                 + muldiv64(ticks, NANOSECONDS_PER_SECOND,
                            ESP32C3_RTC_SLOW_CLK_HZ));
}

static uint64_t esp32c3_rtc_cntl_read(void* opaque, hwaddr addr, unsigned int size)
{
    ESP32C3RtcCntlState *s = ESP32C3_RTC_CNTL(opaque);
    uint64_t r = 0;

    switch(addr) {
        case A_RTC_CNTL_RTC_OPTIONS0:
            r = s->options0;
            break;
        case A_RTC_CNTL_RTC_RESET_STATE:
            r = s->reason;
            break;

        case A_RTC_CNTL_RTC_SLP_TIMER0:
            r = s->slp_timer0;
            break;
        case A_RTC_CNTL_RTC_SLP_TIMER1:
            r = s->slp_timer1;
            break;
        case A_RTC_CNTL_RTC_TIME_LOW0:
            r = s->time_low;
            break;
        case A_RTC_CNTL_RTC_TIME_HIGH0:
            r = s->time_high;
            break;
        case A_RTC_CNTL_RTC_STATE0:
            r = s->state0;
            break;
        case A_RTC_CNTL_RTC_WAKEUP_STATE:
            r = s->wakeup_state;
            break;
        case A_RTC_CNTL_INT_RAW_RTC:
            r = s->int_raw;
            break;
        case A_RTC_CNTL_INT_ENA_RTC:
            r = s->int_ena;
            break;
        case A_RTC_CNTL_INT_ST_RTC:
            r = s->int_raw & s->int_ena;
            break;
        case A_RTC_CNTL_DIG_PWC:
            r = s->dig_pwc;
            break;
        case A_RTC_CNTL_RTC_SLP_WAKEUP_CAUSE:
            r = s->slp_wakeup_cause;
            break;

        case A_RTC_CNTL_RTC_STORE0:
        case A_RTC_CNTL_RTC_STORE1:
        case A_RTC_CNTL_RTC_STORE2:
        case A_RTC_CNTL_RTC_STORE3:
            r = s->scratch_reg[(addr - A_RTC_CNTL_RTC_STORE0) / 4];
            break;

        case A_RTC_CNTL_RTC_STORE4:
        case A_RTC_CNTL_RTC_STORE5:
        case A_RTC_CNTL_RTC_STORE6:
        case A_RTC_CNTL_RTC_STORE7:
            r = s->scratch_reg[(addr - A_RTC_CNTL_RTC_STORE4) / 4 + 4];
            break;
        default:
#if RTCCNTL_WARNING
            /* Other registers are not supported yet */
            warn_report("[RTCCNTL] Unsupported read to %08lx", addr);
#endif
            break;
    }

    return r;
}


static void esp32c3_rtc_cntl_write(void* opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32C3RtcCntlState *s = ESP32C3_RTC_CNTL(opaque);
    const uint32_t c_value = value;

    switch(addr) {
        case A_RTC_CNTL_RTC_OPTIONS0:
            CLEAR_BIT(value, R_RTC_CNTL_RTC_OPTIONS0_SW_SYS_RST_SHIFT);
            CLEAR_BIT(value, R_RTC_CNTL_RTC_OPTIONS0_SW_PROCPU_RST_SHIFT);
            s->options0 = value;
            /* Check if we have to reset the CPU/machine */
            if (FIELD_EX32(c_value, RTC_CNTL_RTC_OPTIONS0, SW_SYS_RST)) {
                esp32c3_reset_request(opaque, ESP32C3_RTC_SW_SYS_RESET, 1);
            } else if (FIELD_EX32(c_value, RTC_CNTL_RTC_OPTIONS0, SW_PROCPU_RST)) {
                esp32c3_reset_request(opaque, ESP32C3_RTC_SW_CPU_RESET, 1);
            }
            break;

        case A_RTC_CNTL_RTC_SLP_TIMER0:
            s->slp_timer0 = value;
            break;
        case A_RTC_CNTL_RTC_SLP_TIMER1:
            s->slp_timer1 = value;
            break;

        case A_RTC_CNTL_RTC_TIME_UPDATE:
            /* Bit 31 latches the free-running counter into TIME_LOW/HIGH; the
             * guest then polls for it to clear, which it already has. */
            if (FIELD_EX32(c_value, RTC_CNTL_RTC_TIME_UPDATE, RTC_TIME_UPDATE)) {
                uint64_t ticks = esp32c3_rtc_slow_ticks();

                s->time_low = (uint32_t)ticks;
                s->time_high = (uint32_t)(ticks >> 32);
                esp32c3_rtc_status_time(ticks);
            }
            break;

        case A_RTC_CNTL_RTC_STATE0:
            s->state0 = value;
            if (FIELD_EX32(c_value, RTC_CNTL_RTC_STATE0, SLEEP_EN)) {
                esp32c3_rtc_cntl_sleep(s);
            }
            break;

        case A_RTC_CNTL_RTC_WAKEUP_STATE:
            s->wakeup_state = value;
            break;

        case A_RTC_CNTL_INT_ENA_RTC:
            s->int_ena = value;
            break;

        case A_RTC_CNTL_INT_CLR_RTC:
            s->int_raw &= ~(uint32_t)value;
            break;

        case A_RTC_CNTL_DIG_PWC:
            s->dig_pwc = value;
            break;

        case A_RTC_CNTL_RTC_STORE0:
        case A_RTC_CNTL_RTC_STORE1:
        case A_RTC_CNTL_RTC_STORE2:
        case A_RTC_CNTL_RTC_STORE3:
            s->scratch_reg[(addr - A_RTC_CNTL_RTC_STORE0) / 4] = value;
            break;

        case A_RTC_CNTL_RTC_STORE4:
        case A_RTC_CNTL_RTC_STORE5:
        case A_RTC_CNTL_RTC_STORE6:
        case A_RTC_CNTL_RTC_STORE7:
            s->scratch_reg[(addr - A_RTC_CNTL_RTC_STORE4) / 4 + 4] = value;
            break;

        default:
#if RTCCNTL_WARNING
            /* Other registers are not supported yet */
            warn_report("[RTCCNTL] Unsupported write to %08lx (%08lx)", addr, value);
#endif
            break;
    }
}


static const MemoryRegionOps esp_rtc_cntl_ops = {
    .read =  esp32c3_rtc_cntl_read,
    .write = esp32c3_rtc_cntl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};


static void esp32c3_rtc_cntl_reset_hold(Object *obj, ResetType type)
{
    static bool first_boot = true;
    ESP32C3RtcCntlState *s = ESP32C3_RTC_CNTL(obj);
    s->options0 = 0;

    /*
     * The RTC domain is what a deep sleep keeps alive, so `reason` and the
     * scratch registers deliberately survive this - that is how the guest
     * tells a wake from a cold boot. Everything else is digital-side state
     * that the reset really does clear.
     */
    s->state0 = 0;
    s->int_raw = 0;
    s->int_ena = 0;
    s->dig_pwc = 0;
    s->wakeup_state = 0;
    s->sleep_is_deep = false;
    if (s->sleep_timer) {
        timer_del(s->sleep_timer);
    }

    if (first_boot) {
        s->reason = ESP32C3_POWERON_RESET;
        first_boot = false;
    }
    esp32c3_rtc_status.reset_reason = s->reason;
    esp32c3_rtc_status.state = ESP32C3_RTC_AWAKE;

    qemu_irq_lower(s->cpu_reset);
}


static void esp32c3_rtc_cntl_realize(DeviceState *dev, Error **errp)
{
    ESP32C3RtcCntlState *s = ESP32C3_RTC_CNTL(dev);
    esp32c3_rtc_cntl_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
    (void) s;
}


static void esp32c3_rtc_cntl_init(Object *obj)
{
    ESP32C3RtcCntlState *s = ESP32C3_RTC_CNTL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp_rtc_cntl_ops, s,
                          TYPE_ESP32C3_RTC_CNTL, ESP32C3_RTC_CNTL_IO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    /* Initialize ESP32C3_COUNT_RESET input lines, each representing a reset source (reason) */
    qdev_init_gpio_in(DEVICE(s), esp32c3_reset_request, ESP32C3_COUNT_RESET);
    /* Initialize the GPIO that will notify the CPU to reset itself */
    qdev_init_gpio_out_named(DEVICE(s), &s->cpu_reset, ESP32C3_RTC_CPU_RESET_GPIO, 1);

    s->sleep_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, esp32c3_rtc_cntl_wake, s);
}


static void esp32c3_rtc_cntl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32c3_rtc_cntl_reset_hold;
    dc->realize = esp32c3_rtc_cntl_realize;
}


static const TypeInfo esp32c3_rtc_cntl_info = {
    .name = TYPE_ESP32C3_RTC_CNTL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32C3RtcCntlState),
    .instance_init = esp32c3_rtc_cntl_init,
    .class_init = esp32c3_rtc_cntl_class_init
};


static void esp32c3_rtc_cntl_register_types(void)
{
    type_register_static(&esp32c3_rtc_cntl_info);
}


type_init(esp32c3_rtc_cntl_register_types)
