/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"

#include "wiring_constants.h"
#include "wiring_digital.h"
#include "nrf.h"

const uint32_t g_ADigitalPinMap[] = {
  0,  // xtal 1
  1,  // xtal 2
  2,  // A0
  3,  // A1
  4,  // A2
  5,  // A3
  6,  // SDA
  7,  // SCL
  8,  // D14 (LED2)
  9,  // NFC1
  10, // NFC2
  11, // TXD
  12, // RXD
  13, // D15 (LED1)
  14, // MISO
  15, // SW1
  16, // D5
  17, // D6
  18, // D7
  19, // D8
  20, // D9
  21, // Reset
  22, // D10
  23, // D11
  24, // D12
  25, // MOSI
  26, // MISO
  27, // SCK
  28, // SS
  29, // A5
  30, // A6
  31, // A7
};

void initVariant()
{
  // LED1 & LED2
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);

  pinMode(PIN_LED2, OUTPUT);
  ledOff(PIN_LED2);
}

