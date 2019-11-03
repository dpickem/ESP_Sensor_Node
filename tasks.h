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

#ifndef _TASKS_
#define _TASKS_

// Include Arduino libraries.
#include "Arduino.h"
#include "Wire.h"

// Include sensor node dependencies.
#include "config.h"
#include "data_manager.h"
#include "mqtt_interface.h"
#include "task.h"
#include "utilities.h"

// Include sensor libraries.
#include "Adafruit_Sensor.h"
#include "Adafruit_AS726x.h"
#include "Adafruit_BME680.h"
#include "Adafruit_MPL3115A2.h"
#include "Adafruit_seesaw.h"
#include "Adafruit_SGP30.h"
#include "Adafruit_Si7021.h"
#include "Adafruit_TSL2591.h"
#include "Adafruit_VEML7700.h"


// AS7262 light spectrometer sensor class as TimedTask.
class AS7262SensorDataCollector : public TimedTask {
 public:
    // Constructor.
    AS7262SensorDataCollector(
            uint32_t rate,
            DataManager<float>* dm);

    // Overloaded setup and run methods.
    bool is_connected();
    bool setup_helper();
    void run_helper();

 private:
    bool measurement_started_;
    uint16_t sensor_data_[AS726x_NUM_CHANNELS];
    Adafruit_AS726x* sensor_; 
    DataManager<float>* dm_;
};

// BME680 temperature, humidity, gas, altitude sensor class as TimedTask.
class BME680SensorDataCollector : public TimedTask {
 public:
    // Constructor.
    BME680SensorDataCollector(
            uint32_t rate,
            DataManager<float>* dm);

    // Overloaded setup and run methods.
    bool is_connected();
    bool setup_helper();
    void run_helper();

 private:
    Adafruit_BME680* sensor_;
    DataManager<float>* dm_;
};

// Motion sensor class as TimedTask.
class MotionSensorDataCollector : public TimedTask {
 public:
    // Constructor.
    MotionSensorDataCollector(
            uint32_t rate,
            uint8_t sensor_pin,
            DataManager<float>* dm,
            uint16_t threshold = 500);

    // Overloaded setup and run methods.
    bool is_connected();
    bool setup_helper();
    void run_helper();

 private:
    uint16_t threshold_;
    uint8_t sensor_pin_;
    DataManager<float>* dm_;
};

// MPL3115A2 temperature, pressure, altitude sensor class as TimedTask.
class MPL3115A2SensorDataCollector : public TimedTask {
 public:
    // Constructor.
    MPL3115A2SensorDataCollector(
            uint32_t rate,
            DataManager<float>* dm);

    // Overloaded setup and run methods.
    bool is_connected();
    bool setup_helper();
    void run_helper();

 private:
    Adafruit_MPL3115A2* sensor_;
    DataManager<float>* dm_;
};

// Si7021 temperature and humidity sensor class as TimedTask.
class Si7021SensorDataCollector : public TimedTask {
 public:
    // Constructor.
    Si7021SensorDataCollector(uint32_t rate, DataManager<float>* dm);

    // Overloaded setup and run methods.
    bool is_connected();
    bool setup_helper();
    void run_helper();

 private:
    Adafruit_Si7021* sensor_;
    DataManager<float>* dm_;
};

// Soil moisture sensor class as TimedTask.
class SoilMoistureSensorDataCollector : public TimedTask {
 public:
    // Constructor.
    SoilMoistureSensorDataCollector(
            uint32_t rate,
            DataManager<float>* dm,
            uint8_t address = SEESAW_ADDRESS_DEFAULT,
            String name = "soil_moisture");

    // Overloaded setup and run methods.
    bool is_connected();
    bool setup_helper();
    void run_helper();

 private:
    uint8_t address_;
    Adafruit_seesaw* sensor_;
    DataManager<float>* dm_;
};

// SGP30 eCO2 and VOC sensor class as TimedTask.
class SGP30SensorDataCollector : public TimedTask {
 public:
    // Constructor.
    SGP30SensorDataCollector(
            uint32_t rate,
            DataManager<float>* dm);

    // Overloaded setup and run methods.
    bool is_connected();
    bool setup_helper();
    void run_helper();

    // Utility functions.
    uint32_t getAbsoluteHumidity(float temperature, float humidity);

 private:
    uint32_t baseline_measurement_start_time_;
    Adafruit_SGP30* sensor_;
    DataManager<float>* dm_;
};

// TSL2591 light sensor class as TimedTask.
class TSL2591SensorDataCollector : public TimedTask {
 public:
    // Constructor.
    TSL2591SensorDataCollector(
            uint32_t rate,
            TwoWire &wire,
            DataManager<float>* dm);

    // Overloaded setup and run methods.
    bool is_connected();
    bool setup_helper();
    void run_helper();

 private:
    uint8_t id_;
    Adafruit_TSL2591* sensor_;
    DataManager<float>* dm_;
};

// VEML6075 UVA, UVB, UV index sensor class as TimedTask.
class VEML6075SensorDataCollector : public TimedTask {
 public:
    // Constructor.
    VEML6075SensorDataCollector(
            uint32_t rate,
            DataManager<float>* dm);

    // Overloaded setup and run methods.
    bool is_connected();
    bool setup_helper();
    void run_helper();

 private:
    Adafruit_VEML6075* sensor_;
    DataManager<float>* dm_;
};

// VEML7700 light sensor class as TimedTask.
class VEML7700SensorDataCollector : public TimedTask {
 public:
    // Constructor.
    VEML7700SensorDataCollector(
            uint32_t rate,
            DataManager<float>* dm);

    // Overloaded setup and run methods.
    bool is_connected();
    bool setup_helper();
    void run_helper();

 private:
    Adafruit_VEML7700* sensor_; 
    DataManager<float>* dm_;
};

// DataCollector task as TimedTask.
class DataCollector : public TimedTask {
 public:
    // Constructor.
    DataCollector(
            uint32_t rate,
            MQTTInterface* mqtt_interface,
            DataManager<float>* dm,
            bool batch_publish = false);

    // Overloaded setup and run methods.
    bool is_connected();
    bool setup_helper();
    void run_helper();

    // Utility functions.
    void publish_new_data();
    void publish_new_data_batch();
    String get_new_data_as_json_string();
    String get_mac_address();

 private:
    DataManager<float>* dm_;
    MQTTInterface* mqtt_interface_;
    String mqtt_topic_;
    bool batch_publish_;
};

#endif
