/*
 * Copyright (C) 2019 Daniel Pickem (daniel.pickem@gmail.com)
 * http://www.danielpickem.com - All Rights Reserved
 * 
 * This software is part of the firmware running on home automation
 * sensor nodes. You may use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software but
 * you must include this copyright notice and this permission in
 * all copies or substantial portions of the software.
 *
 * MIT-License
 */

#ifndef _SENSOR_NODE_UTILITIES_
#define _SENSOR_NODE_UTILITIES_

// Include Arduino libraries.
#include "Arduino.h"
#include "Wire.h"

// Include Adafruit libraries.
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_VEML6075.h"

bool ping_i2c_device(uint8_t address);
bool ping_i2c_device_veml_6075(uint8_t address);

#endif
