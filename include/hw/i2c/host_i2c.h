/*
 * Browser-backed I2C slave.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_I2C_HOST_I2C_H
#define HW_I2C_HOST_I2C_H

#include "hw/i2c/i2c.h"

#define TYPE_HOST_I2C "qemu-host-i2c"

OBJECT_DECLARE_SIMPLE_TYPE(HostI2CState, HOST_I2C)

#endif /* HW_I2C_HOST_I2C_H */
