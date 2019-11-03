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

#include "utilities.h"

bool ping_i2c_device(uint8_t address) {
    /* Try to start a transmission to an I2C address, which is
     * the equivalent of a ping command. If the device is present
     * it will respond with a zero error code.
     */
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission(true);
    return (error == 0);
}

bool ping_i2c_device_veml_6075(uint8_t address) {
    /* The VEML6075 and VEML7700 sensor use the same I2C address.
     * To be able to differentiate between the two one needs to
     * check the ID register which is only available on the 
     * VEML6075 device.
     */

    // Instantiate a new I2C device interface.
    Adafruit_I2CDevice *i2c_device = 
      new Adafruit_I2CDevice(VEML6075_ADDR, &Wire);

    if (!i2c_device->begin()) {
        delete i2c_device;
        return false;
    }

    // Instantiate a new I2C register.
    Adafruit_I2CRegister id_register = 
      Adafruit_I2CRegister(i2c_device, VEML6075_REG_ID, 2);

    uint16_t id;
    if (!id_register.read(&id) or id != 0x26) {
        delete i2c_device;
        return false;
    }

    delete i2c_device;
    return true;
}
