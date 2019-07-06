/*
 * Copyright © 2010-2014 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1+
 */

#include "config.h"

#include <string.h>
#include <ctype.h>
#include <sys/types.h>

#ifndef HAVE_MODBUS_GET_FLOAT_ABCD

/* Get a float from 4 bytes (Modbus) without any conversion (ABCD) */
float modbus_get_float_abcd(const uint16_t *src)
{
    float f;
    uint32_t i;
    uint8_t a, b, c, d;

    a = (src[0] >> 8) & 0xFF;
    b = (src[0] >> 0) & 0xFF;
    c = (src[1] >> 8) & 0xFF;
    d = (src[1] >> 0) & 0xFF;

    i = (a << 24) |
        (b << 16) |
        (c << 8) |
        (d << 0);
    memcpy(&f, &i, 4);

    return f;
}

#endif /* !defined(MODBUS_GET_FLOAT_ABCD) */
