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

#ifndef _SENSOR_NODE_CONFIG_
#define _SENSOR_NODE_CONFIG_

/* I2C sensor addresses for reference.
 *
 * ----------------------
 * Unique addresses:
 * ----------------------
 *
 * BME680_DEFAULT_ADDRESS           0x77
 * MPL3115A2_ADDRESS                0x60 
 * SEESAW_ADDRESS (no solder gate)  0x36
 * SEESAW_ADDRESS (A0 solder gate)  0x37
 * SEESAW_ADDRESS (A1 solder gate)  0x38
 * SEESAW_ADDRESS (both gates)      0x39
 * SGP30_I2CADDR_DEFAULT            0x58
 * SI7021_DEFAULT_ADDRESS           0x40
 * TSL2591_ADDR                     0x29
 * AS726x_ADDRESS                   0x49
 *
 * ----------------------
 * Conflicting addresses:
 * ----------------------
 * VEML6075_ADDR                    0x10
 * VEML7700_I2CADDR_DEFAULT         0x10
 */

// Pin definitions.
// #define MOTION_SENSOR_PIN A0 // This is for Huzzah.
#define MOTION_SENSOR_PIN 4  // This is the mini sensor node.

// Update rates.
#define DATA_COLLECT_RATE 500
#define AS7262_SENSOR_COLLECT_RATE 1000
#define BME680_SENSOR_COLLECT_RATE 1000
#define MOTION_SENSOR_COLLECT_RATE 1000
#define MPL3115A2_SENSOR_COLLECT_RATE 1000
#define SGP30_SENSOR_COLLECT_RATE 1000
#define SI7021_SENSOR_COLLECT_RATE 1000
#define SOIL_MOISTURE_SENSOR_COLLCT_RATE 1000
#define TSL2591_SENSOR_COLLECT_RATE 1000
#define VEML6075_SENSOR_COLLECT_RATE 1000
#define VEML7700_SENSOR_COLLECT_RATE 1000

// ----------------------------------------
//  Define non-standard sensor addresses 
// ----------------------------------------
#define SEESAW_ADDRESS_DEFAULT 0x36
#define SEESAW_ADDRESS_A0 0x37
#define SEESAW_ADDRESS_A1 0x38
#define SEESAW_ADDRESS_A0_A1 0x39

// ----------------------------------------
//         Define sensor constants
// ----------------------------------------

// Motion sensor.
#define MOTION_SENSOR_CONNECTED false

// BME680 constants.
#define SEALEVELPRESSURE_HPA (1017.1)  // in San Jose.

// SGP30 constants.
#define SGP30_BASELINE_MEASUREMENT_TIME_SEC 30

// ----------------------------------------
// Other numeric constants
// ----------------------------------------
#define FLOAT_INVALID std::numeric_limits<float>::min()

#endif
