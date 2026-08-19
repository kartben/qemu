/*
 * CAN bus client backed by a bus model running in the browser.
 *
 * Copyright (c) 2026 Benjamin Cabé
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The third browser bridge on the ESP32-C3, after hw/i2c/host_i2c.c and
 * hw/ssi/host_spi.c, and the one that is shaped differently. Those two answer
 * questions the guest asks, so a request/response mailbox fits and the guest's
 * own thread can wait for the answer. CAN is not like that: a frame arrives
 * because some other node decided to send one, and the guest may be idle.
 *
 * So this is a ring pair in the wasm heap, the shape net/browser.c already uses
 * for Ethernet: QEMU appends transmitted frames to the TX ring and the page
 * drains them, the page appends received frames to the RX ring and a
 * QEMU_CLOCK_VIRTUAL timer drains them under the BQL. Frames are fixed-size
 * records, which is the one simplification Ethernet does not get - a classic
 * CAN frame is an id, a length and at most eight bytes, so there is no
 * length-prefix or wrap-skip machinery here.
 *
 * Injection has to happen on the QEMU thread, hence the timer rather than a
 * JS-visible "send" entry point. On this `-icount sleep=on` machine the virtual
 * clock warps to the next deadline whenever the guest idles, so a 5 ms poll
 * stays responsive without burning host cycles.
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "net/can_emu.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

/* Power of two: the indices are free-running and masked on use. */
#define CAN_BROWSER_RING_SLOTS 256
#define CAN_BROWSER_POLL_MS    5

/**
 * One frame on the wire between the two sides. Little-endian, 16 bytes, laid
 * out so the page can read it with a DataView and no padding surprises.
 */
typedef struct CanBrowserSlot {
    uint32_t can_id;   /* QEMU_CAN_* flags in the top bits, as qemu_canid_t */
    uint8_t  dlc;      /* 0..8; longer frames are dropped, this is not FD */
    uint8_t  flags;    /* qemu_can_frame.flags */
    uint8_t  pad[2];
    uint8_t  data[8];
} CanBrowserSlot;

typedef struct CanBrowserState {
    CanBusClientState bus_client;
    QEMUTimer *poll_timer;
} CanBrowserState;

static CanBrowserState *can_browser;                     /* singleton */

static CanBrowserSlot can_browser_tx[CAN_BROWSER_RING_SLOTS];  /* guest -> page */
static CanBrowserSlot can_browser_rx[CAN_BROWSER_RING_SLOTS];  /* page -> guest */
static uint32_t can_browser_tx_wr, can_browser_tx_rd;    /* wr: QEMU, rd: page */
static uint32_t can_browser_rx_wr, can_browser_rx_rd;    /* wr: page, rd: QEMU */

/* --- entry points for the page (Module._qemu_can_browser_*) -------------- */

EMSCRIPTEN_KEEPALIVE
uint32_t qemu_can_browser_ready(void)
{
    return can_browser != NULL;
}

EMSCRIPTEN_KEEPALIVE
uint32_t qemu_can_browser_ring_slots(void)
{
    return CAN_BROWSER_RING_SLOTS;
}

EMSCRIPTEN_KEEPALIVE
uintptr_t qemu_can_browser_tx_ring(void)
{
    return (uintptr_t)can_browser_tx;
}

EMSCRIPTEN_KEEPALIVE
uintptr_t qemu_can_browser_rx_ring(void)
{
    return (uintptr_t)can_browser_rx;
}

EMSCRIPTEN_KEEPALIVE
uint32_t qemu_can_browser_tx_write_index(void)
{
    return qatomic_read(&can_browser_tx_wr);
}

EMSCRIPTEN_KEEPALIVE
uint32_t qemu_can_browser_tx_read_index(void)
{
    return qatomic_read(&can_browser_tx_rd);
}

EMSCRIPTEN_KEEPALIVE
void qemu_can_browser_tx_set_read_index(uint32_t value)
{
    qatomic_set(&can_browser_tx_rd, value);
}

EMSCRIPTEN_KEEPALIVE
uint32_t qemu_can_browser_rx_write_index(void)
{
    return qatomic_read(&can_browser_rx_wr);
}

EMSCRIPTEN_KEEPALIVE
uint32_t qemu_can_browser_rx_read_index(void)
{
    return qatomic_read(&can_browser_rx_rd);
}

EMSCRIPTEN_KEEPALIVE
void qemu_can_browser_rx_set_write_index(uint32_t value)
{
    qatomic_set(&can_browser_rx_wr, value);
}

/* --- guest -> page (QEMU thread, BQL held) ------------------------------- */

static bool can_browser_can_receive(CanBusClientState *client)
{
    uint32_t used = can_browser_tx_wr - qatomic_read(&can_browser_tx_rd);

    return used < CAN_BROWSER_RING_SLOTS;
}

static ssize_t can_browser_receive(CanBusClientState *client,
                                   const qemu_can_frame *frames,
                                   size_t frames_cnt)
{
    size_t sent = 0;

    for (size_t i = 0; i < frames_cnt; i++) {
        const qemu_can_frame *f = &frames[i];
        uint32_t wr = can_browser_tx_wr;
        CanBrowserSlot *slot;

        if (f->can_dlc > 8 || (f->flags & QEMU_CAN_FRMF_TYPE_FD)) {
            /* Classic CAN only. Counted as sent so the controller does not
             * stall retrying something this bridge will never carry. */
            sent++;
            continue;
        }
        if (wr - qatomic_read(&can_browser_tx_rd) >= CAN_BROWSER_RING_SLOTS) {
            break;                              /* page is behind; back-pressure */
        }

        slot = &can_browser_tx[wr % CAN_BROWSER_RING_SLOTS];
        memset(slot, 0, sizeof(*slot));
        slot->can_id = f->can_id;
        slot->dlc = f->can_dlc;
        slot->flags = f->flags;
        memcpy(slot->data, f->data, f->can_dlc);

        /* Publish the record before the index that reveals it. */
        smp_wmb();
        qatomic_set(&can_browser_tx_wr, wr + 1);
        sent++;

#ifndef __EMSCRIPTEN__
        /*
         * Native builds have no page, so nothing would ever answer. Reflect
         * the frame back as if a node out there had echoed it, which is enough
         * for a guest to prove its controller both transmits and receives
         * without a browser. Deliberately not compiled into the wasm artifact:
         * there the page is the other end and an echo would be a phantom node.
         */
        {
            uint32_t rxw = qatomic_read(&can_browser_rx_wr);

            if (rxw - qatomic_read(&can_browser_rx_rd) < CAN_BROWSER_RING_SLOTS) {
                can_browser_rx[rxw % CAN_BROWSER_RING_SLOTS] = *slot;
                smp_wmb();
                qatomic_set(&can_browser_rx_wr, rxw + 1);
            }
        }
#endif
    }
    return sent;
}

/* --- page -> guest (timer, QEMU thread, BQL held) ------------------------ */

static void can_browser_poll(void *opaque)
{
    CanBrowserState *s = opaque;
    uint32_t rd = can_browser_rx_rd;
    uint32_t wr = qatomic_read(&can_browser_rx_wr);

    while (rd != wr) {
        const CanBrowserSlot *slot = &can_browser_rx[rd % CAN_BROWSER_RING_SLOTS];
        qemu_can_frame frame = { 0 };

        /* Read the record only after the index that revealed it. */
        smp_rmb();
        frame.can_id = slot->can_id;
        frame.can_dlc = slot->dlc > 8 ? 8 : slot->dlc;
        frame.flags = slot->flags;
        memcpy(frame.data, slot->data, frame.can_dlc);

        can_bus_client_send(&s->bus_client, &frame, 1);
        rd++;
        qatomic_set(&can_browser_rx_rd, rd);
    }

    timer_mod(s->poll_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)
                             + CAN_BROWSER_POLL_MS);
}

static CanBusClientInfo can_browser_bus_client_info = {
    .can_receive = can_browser_can_receive,
    .receive = can_browser_receive,
};

/**
 * can_browser_connect: put the page on @bus.
 *
 * One instance per process; a second call is a no-op, because the page has one
 * CAN bus model and there is one machine in it.
 */
bool can_browser_connect(CanBusState *bus, Error **errp)
{
    CanBrowserState *s;

    if (can_browser != NULL) {
        return true;
    }

    s = g_new0(CanBrowserState, 1);
    s->bus_client.info = &can_browser_bus_client_info;
    if (can_bus_insert_client(bus, &s->bus_client) < 0) {
        error_setg(errp, "can_browser: cannot insert the page onto the CAN bus");
        g_free(s);
        return false;
    }

    s->poll_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, can_browser_poll, s);
    timer_mod(s->poll_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)
                             + CAN_BROWSER_POLL_MS);
    can_browser = s;
    return true;
}
