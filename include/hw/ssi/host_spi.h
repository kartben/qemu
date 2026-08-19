/*
 * SPI peripheral backed by device models in the browser.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_HOST_SPI_H
#define HW_SSI_HOST_SPI_H

#include "hw/ssi/ssi.h"

#define TYPE_HOST_SPI "qemu-host-spi"

OBJECT_DECLARE_SIMPLE_TYPE(HostSPIState, HOST_SPI)

#endif /* HW_SSI_HOST_SPI_H */
