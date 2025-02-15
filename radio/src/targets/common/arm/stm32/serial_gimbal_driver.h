/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#pragma once

#include "hal/serial_port.h"

// Max packet size = 1 byte header + 1 byte ID + 1 byte length + 25 bytes payload + 2 bytes CRC = 30 bytes
// using 32 bytes is more than enough
#define SERIAL_GIMBAL_BUFF_SIZE           ( 32 )
#define SERIAL_GIMBAL_BAUDRATE            ( 921600 )
#define SERIAL_GIMBAL_CHANNEL_COUNT       ( 4 )

// This value has been chosen arbitrarily to allow
// for 13-bit precision.
//
// Note: Serial gimbals provide signed 16-bit values, whereby
//       ADC sampling uses unsigned 16-bit values.
//
#define SERIAL_GIMBAL_OFFSET_VALUE        ( 1 << 12 )

#define SERIAL_GIMBAL_PROTOLO_HEAD        0x55
#define SERIAL_GIMBAL_RESP_TYPE_VALUES    0x0c

typedef struct __attribute__ ((packed))
{
  uint8_t start;
  uint8_t senderID:2;
  uint8_t receiverID:2;
  uint8_t packetID:4;
  uint8_t length;
  int16_t  channel[4];
  uint8_t  stickState;
  uint16_t checkSum;
} serial_gimbal_frame;

enum TRANSFER_DIR_E {
  TRANSFER_DIR_HALLSTICK,
  TRANSFER_DIR_TXMCU,
  TRANSFER_DIR_HOSTPC,
  TRANSFER_DIR_RFMODULE,
};

// returns true if the gimbals were detected properly
bool serial_gimbal_init();

void serial_gimbal_deinit();
const etx_serial_port_t* serial_gimbal_get_port();
