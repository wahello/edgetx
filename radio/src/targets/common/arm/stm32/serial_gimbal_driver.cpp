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

#include "serial_gimbal_driver.h"
#include "stm32_serial_driver.h"
#include "stm32_gpio.h"
#include "stm32_adc.h"

#include "delays_driver.h"
#include "hal/adc_driver.h"
#include "hal/gpio.h"

#include "hal.h"
#include "crc.h"

#include <string.h>
#include "edgetx_helpers.h"

#include "debug.h"

static const stm32_usart_t fsUSART = {
  .USARTx = GIMBAL_HALL_SERIAL_USART,
  .txGPIO = GIMBAL_HALL_SERIAL_TX_GPIO,
  .rxGPIO = GIMBAL_HALL_SERIAL_RX_GPIO,
  .IRQn = GIMBAL_HALL_SERIAL_USART_IRQn,
  .IRQ_Prio = 4,
  .txDMA = nullptr,
  .txDMA_Stream = 0,
  .txDMA_Channel = 0,
  .rxDMA = GIMBAL_HALL_SERIAL_DMA,
  .rxDMA_Stream = GIMBAL_HALL_SERIAL_DMA_Stream_RX,
  .rxDMA_Channel = GIMBAL_HALL_SERIAL_DMA_Channel,
};

DEFINE_STM32_SERIAL_PORT(FSGimbal, fsUSART, HALLSTICK_BUFF_SIZE, 0);

static const etx_serial_port_t _fs_gimbal_serial_port = {
  .name = "gimbals",
  .uart = &STM32SerialDriver,
  .hw_def = REF_STM32_SERIAL_PORT(FSGimbal),
  .set_pwr = nullptr,
};

static void* _fs_usart_ctx = nullptr;

static int _fs_get_frame(serial_gimbal_frame* data)
{
  unsigned int len = STM32SerialDriver.getBufferedBytes (_fs_usart_ctx);
  if ((len == 0) || (len > sizeof(serial_gimbal_frame)))
    return 0;

  STM32SerialDriver.copyRxBuffer(_fs_usart_ctx, (uint8_t*)data, len);

  return len;
}

static volatile bool _fs_gimbal_detected;

static void serial_gimbal_process()
{
  serial_gimbal_frame serial_data = { 0 };

  auto len = _fs_get_frame(&serial_data);

  if (len == 0)
    return;

  // Check header is valid
  if (serial_data.start != GIMBAL_HALL_PROTOLO_HEAD) {
    memclear(&serial_data, sizeof(serial_data));
    TRACE("serial gimbal: invalid header");
    return;
  }

  // Check CRC
  if (serial_data.checkSum != crc16(CRC_1021, (const uint8_t*) &serial_data, serial_data.length + 3, 0xffff)) {
    TRACE("serial gimbal: invalid CRC");
    memclear(&serial_data, sizeof(serial_data));
    return;
  }

  // Channel data
  if (serial_data.receiverID == TRANSFER_DIR_TXMCU || serial_data.receiverID == TRANSFER_DIR_RFMODULE) {
    if (serial_data.packetID == GIMBAL_HALL_RESP_TYPE_VALUES) {
      int16_t *p_values = (int16_t *) serial_data.channel;
      uint16_t *adcValues = getAnalogValues();
      for (uint8_t i = 0; i < 4; i++) {
        adcValues[i] = GIMBAL_HALL_OFFSET_VALUE - p_values[i];
      }
    _fs_gimbal_detected = true;
    }
  }
}

static void serial_gimbal_loop(void*)
{
  serial_gimbal_process();
}

void serial_gimbal_deinit()
{
  STM32SerialDriver.deinit(_fs_usart_ctx);
}

bool serial_gimbal_init()
{
  etx_serial_init cfg = {
    .baudrate = GIMBAL_HALL_BAUDRATE,
    .encoding = ETX_Encoding_8N1,
    .direction = ETX_Dir_RX,
    .polarity = ETX_Pol_Normal,
  };

  _fs_gimbal_detected = false;
  _fs_usart_ctx = STM32SerialDriver.init(REF_STM32_SERIAL_PORT(FSGimbal), &cfg);
  STM32SerialDriver.setIdleCb(_fs_usart_ctx, serial_gimbal_loop, 0);

  // Wait 70ms for FlySky gimbals to respond. According to LA trace, minimally 23ms is required
  for (uint8_t i = 0; i < 70; i++) {
    delay_ms(1);
    if (_fs_gimbal_detected) {
      // Mask the first 4 inputs (sticks)
      stm32_hal_set_inputs_mask(0xF);
      return true;
    }
  }

  //serial_gimbal_deinit();
  return false;
}

const etx_serial_port_t* serial_gimbal_get_port()
{
  return &_fs_gimbal_serial_port;
}
