/*
 * CAN bus client backed by a bus model running in the browser.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef NET_CAN_BROWSER_H
#define NET_CAN_BROWSER_H

#include "net/can_emu.h"

bool can_browser_connect(CanBusState *bus, Error **errp);

#endif /* NET_CAN_BROWSER_H */
