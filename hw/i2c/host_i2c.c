/*
 * I2C slave backed by device models running in the browser.
 *
 * Copyright (c) 2026 Benjamin Cabé
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * One device that answers for every chip the page has on the bus, so a guest
 * driving the SoC's own I2C controller reaches a TypeScript model of an
 * EEPROM, a sensor or a display without QEMU knowing what any of them are.
 *
 * The shape is the GPIO bridge's: a small struct in the wasm heap, exported to
 * the page, that both sides read and write. Two things travel over it.
 *
 * - **Which addresses answer**, as a 128-bit mask the page writes whenever a
 *   chip is attached or detached. Presence is the one question asked on every
 *   single transfer, and `i2c scan` asks it 116 times in a row, so it is a
 *   plain load from shared memory rather than a round trip.
 *
 * - **The transfers themselves**, one request at a time: QEMU fills the
 *   request fields, bumps `req_seq` and futex-waits on `rsp_seq` until the page
 *   answers. The page's chip models are synchronous and live on the browser's
 *   main thread, which is not the thread running the guest, so this is a real
 *   blocking wait — a slow browser stalls the guest exactly as a slow I2C
 *   device would, and a page that never answers is bounded by a timeout.
 *
 * Nothing here re-enters the block layer or a coroutine, which is what makes
 * it safe to do from guest context under Asyncify.
 *
 * Granularity is one *message* per request on the write side, and one
 * uninterrupted *run* on the read side: the ESP32 driver splits a read of N
 * bytes into N-1 and 1 so it can NAK the last one, and each run arrives here
 * through i2c_announce_recv() before the master pulls its bytes. Because a run
 * is not a whole message, the page is told when a message starts (the FIRST
 * flag) so a chip whose read position is scoped to one message can reset it.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/host_i2c.h"
#include "hw/qdev-properties.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/threading.h>
#endif

/*
 * Enough for any message this can see: the ESP32 command register carries an
 * 8-bit byte count, and its FIFO is 32 bytes deep.
 */
#define HOST_I2C_BUF 256

/* How long to wait for the page before giving up on a transfer and NAKing. */
#define HOST_I2C_TIMEOUT_MS 250

#define HOST_I2C_MAGIC   0x42433249u /* "I2CB" */
#define HOST_I2C_VERSION 1

#define HOST_I2C_OP_WRITE 1
#define HOST_I2C_OP_READ  2

/* This read run opens a message; a message-scoped chip rewinds on it. */
#define HOST_I2C_F_FIRST (1u << 0)

#define HOST_I2C_STATUS_ACK 0
#define HOST_I2C_STATUS_NAK 1

#ifdef __EMSCRIPTEN__

/*
 * The shared area, at a fixed address for the life of the process — the heap
 * never moves (TOTAL_MEMORY with no growth), so the page can keep typed-array
 * views over it. Layout is part of the contract in docs/esp32.md; do not
 * reorder without changing src/hostI2c.ts.
 */
typedef struct HostI2cArea {
    uint32_t magic;
    uint32_t version;
    /* Bit N set = a chip answers at 7-bit address N. Written by the page. */
    uint32_t present[4];
    /* Set by the page while it is listening; cleared when it goes away. */
    uint32_t attached;
    /* Bumped by QEMU once a request is filled in; futex-woken. */
    int32_t req_seq;
    /* Set to req_seq by the page once the answer is filled in; futex-woken. */
    int32_t rsp_seq;
    uint32_t op;
    uint32_t addr;
    uint32_t len;
    uint32_t flags;
    int32_t status;
    uint32_t reserved;
    uint8_t data[HOST_I2C_BUF];
} HostI2cArea;

static HostI2cArea host_i2c_area = {
    .magic = HOST_I2C_MAGIC,
    .version = HOST_I2C_VERSION,
};

/*
 * Only set once a bridge device is realized. The riscv32 artifact carries this
 * file for every machine it can boot, so the export has to say "this machine
 * has one" rather than "this build was compiled with one".
 */
static bool host_i2c_realized;

#endif /* __EMSCRIPTEN__ */

struct HostI2CState {
    I2CSlave parent_obj;

    /* Address the open transfer matched. Meaningless when none is open. */
    uint8_t active;

    /* What the master has sent since START, handed over whole at FINISH. */
    uint8_t wbuf[HOST_I2C_BUF];
    unsigned int wlen;
    bool wtruncated;

    /* The run the master is reading now, fetched by i2c_announce_recv(). */
    uint8_t rbuf[HOST_I2C_BUF];
    unsigned int rlen;
    unsigned int rpos;
    /* Set by START_RECV, cleared by the first run fetched for that message. */
    bool rfirst;

    /*
     * Native builds have no page. A loopback part with AT24 manners gives the
     * whole path — match, write, run-at-a-time read — something to answer, so
     * it can be exercised outside a browser:
     *
     *   -global qemu-host-i2c.loopback=on
     */
    bool loopback;
    uint8_t loopback_address;
    uint8_t loopback_mem[256];
    uint8_t loopback_ptr;
};

/* ------------------------------------------------------------------ page */

#ifdef __EMSCRIPTEN__

/** Pointer to the shared area, or 0 when this machine has no bridge. */
EMSCRIPTEN_KEEPALIVE
uint32_t qemu_host_i2c_area(void)
{
    return host_i2c_realized ? (uint32_t)(uintptr_t)&host_i2c_area : 0;
}

static bool host_i2c_page_attached(void)
{
    return qatomic_read(&host_i2c_area.attached) != 0;
}

static bool host_i2c_page_present(uint8_t address)
{
    uint32_t word = qatomic_read(&host_i2c_area.present[(address >> 5) & 3]);

    return (word >> (address & 31)) & 1;
}

/**
 * Post one request and block until the page answers it. Returns false on a NAK
 * and on a page that stopped answering; a caller that wanted bytes gets none.
 */
static bool host_i2c_page_request(uint32_t op, uint8_t address,
                                  const uint8_t *out, uint8_t *in,
                                  unsigned int len, uint32_t flags)
{
    int32_t seq;
    int64_t deadline;

    if (!host_i2c_page_attached()) {
        return false;
    }
    /*
     * The browser's main thread is where the answer comes from, so waiting for
     * it there would deadlock. Guest code never runs there under
     * PROXY_TO_PTHREAD; belt and braces for anything that later does.
     */
    if (emscripten_is_main_browser_thread()) {
        return false;
    }

    host_i2c_area.op = op;
    host_i2c_area.addr = address;
    host_i2c_area.len = len;
    host_i2c_area.flags = flags;
    host_i2c_area.status = HOST_I2C_STATUS_NAK;
    if (out != NULL && len > 0) {
        memcpy(host_i2c_area.data, out, len);
    }

    seq = host_i2c_area.req_seq + 1;
    qatomic_set(&host_i2c_area.req_seq, seq);
    emscripten_futex_wake(&host_i2c_area.req_seq, INT32_MAX);

    deadline = g_get_monotonic_time() + HOST_I2C_TIMEOUT_MS * 1000;
    while (qatomic_read(&host_i2c_area.rsp_seq) != seq) {
        int64_t now = g_get_monotonic_time();

        if (now >= deadline) {
            qemu_log_mask(LOG_UNIMP,
                          "host-i2c: no answer from the page in %d ms\n",
                          HOST_I2C_TIMEOUT_MS);
            return false;
        }
        /*
         * -EWOULDBLOCK just means the answer landed between the check and the
         * wait, which the loop picks up on the next pass.
         */
        emscripten_futex_wait(&host_i2c_area.rsp_seq, (uint32_t)(seq - 1),
                              (double)(deadline - now) / 1000.0);
    }

    if (host_i2c_area.status != HOST_I2C_STATUS_ACK) {
        return false;
    }
    if (in != NULL && len > 0) {
        memcpy(in, host_i2c_area.data, len);
    }
    return true;
}

#endif /* __EMSCRIPTEN__ */

/* ---------------------------------------------------------------- native */

static bool host_i2c_loopback_write(HostI2CState *s, const uint8_t *buf,
                                    unsigned int len)
{
    if (len == 0) {
        return true;
    }
    /* First byte is the word address, as on an AT24. */
    s->loopback_ptr = buf[0];
    for (unsigned int i = 1; i < len; i++) {
        s->loopback_mem[s->loopback_ptr++] = buf[i];
    }
    return true;
}

static bool host_i2c_loopback_read(HostI2CState *s, uint8_t *buf,
                                   unsigned int len)
{
    for (unsigned int i = 0; i < len; i++) {
        buf[i] = s->loopback_mem[s->loopback_ptr++];
    }
    return true;
}

/* ----------------------------------------------------------------- chips */

static bool host_i2c_present(HostI2CState *s, uint8_t address)
{
#ifdef __EMSCRIPTEN__
    if (host_i2c_page_attached()) {
        return host_i2c_page_present(address);
    }
#endif
    return s->loopback && address == s->loopback_address;
}

static bool host_i2c_write(HostI2CState *s, const uint8_t *buf,
                           unsigned int len)
{
#ifdef __EMSCRIPTEN__
    if (host_i2c_page_attached()) {
        return host_i2c_page_request(HOST_I2C_OP_WRITE, s->active, buf, NULL,
                                     len, 0);
    }
#endif
    if (s->loopback && s->active == s->loopback_address) {
        return host_i2c_loopback_write(s, buf, len);
    }
    return false;
}

static bool host_i2c_read(HostI2CState *s, uint8_t *buf, unsigned int len,
                          uint32_t flags)
{
#ifdef __EMSCRIPTEN__
    if (host_i2c_page_attached()) {
        return host_i2c_page_request(HOST_I2C_OP_READ, s->active, NULL, buf,
                                     len, flags);
    }
#endif
    if (s->loopback && s->active == s->loopback_address) {
        return host_i2c_loopback_read(s, buf, len);
    }
    return false;
}

/* -------------------------------------------------------------- I2CSlave */

static bool host_i2c_match_and_add(I2CSlave *candidate, uint8_t address,
                                   bool broadcast, I2CNodeList *current_devs)
{
    HostI2CState *s = HOST_I2C(candidate);
    I2CNode *node;

    /*
     * Broadcast (the SMBus general call) is deliberately not answered: the
     * page's models are addressed parts, and claiming address 0 would make
     * every general call look like a device that is there.
     */
    if (broadcast || !host_i2c_present(s, address)) {
        return false;
    }

    s->active = address;
    node = g_new(struct I2CNode, 1);
    node->elt = candidate;
    QLIST_INSERT_HEAD(current_devs, node, next);
    return true;
}

static int host_i2c_event(I2CSlave *i2c, enum i2c_event event)
{
    HostI2CState *s = HOST_I2C(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->wlen = 0;
        s->wtruncated = false;
        break;

    case I2C_START_RECV:
        s->rlen = 0;
        s->rpos = 0;
        s->rfirst = true;
        break;

    case I2C_FINISH:
        if (s->wlen > 0) {
            if (s->wtruncated) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "host-i2c: write to 0x%02x longer than %d bytes,"
                              " tail dropped\n", s->active, HOST_I2C_BUF);
            }
            /*
             * The answer arrives too late to NAK a byte with: this controller
             * ignores per-byte ACKs anyway, and the address phase — the one a
             * driver does notice — was already answered from the present mask.
             */
            host_i2c_write(s, s->wbuf, s->wlen);
        }
        s->wlen = 0;
        s->rlen = 0;
        s->rpos = 0;
        s->rfirst = false;
        break;

    case I2C_NACK:
        s->wlen = 0;
        s->rlen = 0;
        s->rpos = 0;
        break;

    default:
        break;
    }
    return 0;
}

static int host_i2c_send(I2CSlave *i2c, uint8_t data)
{
    HostI2CState *s = HOST_I2C(i2c);

    if (s->wlen < HOST_I2C_BUF) {
        s->wbuf[s->wlen++] = data;
    } else {
        s->wtruncated = true;
    }
    return 0;
}

/*
 * The master is about to pull `len` bytes without letting go of the bus. One
 * fetch answers the whole run, which is what keeps a chip model that encodes a
 * multi-byte register (a thermometer's two bytes) from being asked for byte 0
 * twice.
 */
static void host_i2c_announce_recv(I2CSlave *i2c, unsigned int len)
{
    HostI2CState *s = HOST_I2C(i2c);
    uint32_t flags = s->rfirst ? HOST_I2C_F_FIRST : 0;

    if (len == 0) {
        return;
    }
    if (len > HOST_I2C_BUF) {
        len = HOST_I2C_BUF;
    }
    if (!host_i2c_read(s, s->rbuf, len, flags)) {
        /* Nobody answered: an open bus reads as all ones. */
        memset(s->rbuf, 0xff, len);
    }
    s->rlen = len;
    s->rpos = 0;
    s->rfirst = false;
}

static uint8_t host_i2c_recv(I2CSlave *i2c)
{
    HostI2CState *s = HOST_I2C(i2c);

    if (s->rpos < s->rlen) {
        return s->rbuf[s->rpos++];
    }

    /*
     * A master that pulls bytes without announcing the run — nothing here does
     * today — still gets served, one byte at a time.
     */
    host_i2c_announce_recv(i2c, 1);
    return s->rpos < s->rlen ? s->rbuf[s->rpos++] : 0xff;
}

static void host_i2c_realize(DeviceState *dev, Error **errp)
{
#ifdef __EMSCRIPTEN__
    host_i2c_realized = true;
#endif
}

static const Property host_i2c_props[] = {
    DEFINE_PROP_BOOL("loopback", HostI2CState, loopback, false),
    DEFINE_PROP_UINT8("loopback-address", HostI2CState, loopback_address, 0x50),
};

static void host_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->realize = host_i2c_realize;
    device_class_set_props(dc, host_i2c_props);
    sc->match_and_add = host_i2c_match_and_add;
    sc->event = host_i2c_event;
    sc->send = host_i2c_send;
    sc->recv = host_i2c_recv;
    sc->announce_recv = host_i2c_announce_recv;
}

static const TypeInfo host_i2c_type_info = {
    .name = TYPE_HOST_I2C,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HostI2CState),
    .class_init = host_i2c_class_init,
};

static void host_i2c_register_types(void)
{
    type_register_static(&host_i2c_type_info);
}

type_init(host_i2c_register_types)
