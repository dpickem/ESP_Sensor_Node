/*
 * Copyright (C) 2019 Daniel Pickem (daniel.pickem@gmail.com)
 * http://www.danielpickem.com - All Rights Reserved
 *
 * This is the main application sketch for an environmental
 * sensor node.
 * 
 * This software is part of the firmware running on home automation
 * sensor nodes. You may use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software but
 * you must include this copyright notice and this permission in
 * all copies or substantial portions of the software.
 *
 * MIT-License
 */

// Include LinkedList class.
#include "LinkedList.h"

// Include sensor node dependencies.
#include "data_manager.h"
#include "mqtt_interface.h"
#include "tasks.h"

// Instantiate a DataManager object.
DataManager<float>* dm = new DataManager<float>();

// Container for all tasks.
LinkedList<TimedTask*> tasks;

// Instantiate MQTTInterface with default callback handler.
MQTTInterface mqtt_interface;

// Instantiate data collector task.
DataCollector* dm_task = new DataCollector(1000, &mqtt_interface, dm);

// Instantiate tasks as pointers to the appropriate class.
// NOTE: This sketch will crash for instantiations of the following form:
//       TimedTask* task = new SensorTask(...)
AS7262SensorDataCollector* as7262_task = new AS7262SensorDataCollector(AS7262_SENSOR_COLLECT_RATE, dm);
BME680SensorDataCollector* bme680_task = new BME680SensorDataCollector(BME680_SENSOR_COLLECT_RATE, dm);
MotionSensorDataCollector* ms_task = new MotionSensorDataCollector(MOTION_SENSOR_COLLECT_RATE, MOTION_SENSOR_PIN, dm);
MPL3115A2SensorDataCollector* mpl3115_task = new MPL3115A2SensorDataCollector(MPL3115A2_SENSOR_COLLECT_RATE, dm);
SGP30SensorDataCollector* spg30_task = new SGP30SensorDataCollector(SGP30_SENSOR_COLLECT_RATE, dm);
Si7021SensorDataCollector* si7021_task = new Si7021SensorDataCollector(SI7021_SENSOR_COLLECT_RATE, dm);
TSL2591SensorDataCollector* tsl2591_task = new TSL2591SensorDataCollector(TSL2591_SENSOR_COLLECT_RATE, Wire, dm);
VEML6075SensorDataCollector* veml6075_task = new VEML6075SensorDataCollector(VEML6075_SENSOR_COLLECT_RATE, dm);
VEML7700SensorDataCollector* veml7700_task = new VEML7700SensorDataCollector(VEML7700_SENSOR_COLLECT_RATE, dm);

SoilMoistureSensorDataCollector* soil_task_1 = new SoilMoistureSensorDataCollector(SOIL_MOISTURE_SENSOR_COLLECT_RATE, dm, SEESAW_ADDRESS_DEFAULT, "soil_moisture_1");
SoilMoistureSensorDataCollector* soil_task_2 = new SoilMoistureSensorDataCollector(SOIL_MOISTURE_SENSOR_COLLECT_RATE, dm, SEESAW_ADDRESS_A0, "soil_moisture_2");
SoilMoistureSensorDataCollector* soil_task_3 = new SoilMoistureSensorDataCollector(SOIL_MOISTURE_SENSOR_COLLECT_RATE, dm, SEESAW_ADDRESS_A1, "soil_moisture_3");
SoilMoistureSensorDataCollector* soil_task_4 = new SoilMoistureSensorDataCollector(SOIL_MOISTURE_SENSOR_COLLECT_RATE, dm, SEESAW_ADDRESS_A0_A1, "soil_moisture_4");


bool add_task(TimedTask* task) {
    // Buffer task name.
    String task_name = task->get_name();

    // Check whether the sensor corresponding to the input task is connected.
    if (task->is_connected()) {
      if (task->is_setup()) {
        tasks.add(task);
        return true;
      } else {
        Serial.print("Failed to add task "); Serial.print(task_name);
        Serial.println(". Task is not setup.");
      }
    } else {
        Serial.print("Failed to add task "); Serial.print(task_name);
        Serial.println(". Sensor is not connected.");
    }

    return false;
}


void setup() {
    // Set up serial connection.
    Serial.begin(115200);

    // Set up I2C.
    Wire.begin();
    
    // Add the set up data collector task as the first one, since all other sensor tasks
    // depend on a properly initialized data manager.
    add_task(dm_task->setup_inline());
    
    // Add all sensor tasks.
    // NOTE: It is necessary to set up a task before it is added to the task list. Otherwise,
    //       due to some bizarre I2C ownership issue the task crashes once it's executed in
    //       the main loop.
    // NOTE: A yet to be triaged ordering issue requires the Si7021 task to be set up after a
    //       VEML6075 or VEML7700 task and possible as the last task.
    add_task(as7262_task->setup_inline());
    add_task(bme680_task->setup_inline());
    add_task(ms_task->setup_inline());
    add_task(mpl3115_task->setup_inline());
    add_task(spg30_task->setup_inline());
    add_task(tsl2591_task->setup_inline());
    add_task(veml6075_task->setup_inline());
    add_task(veml7700_task->setup_inline());
    add_task(si7021_task->setup_inline());

    add_task(soil_task_1->setup_inline());
    add_task(soil_task_2->setup_inline());
    add_task(soil_task_3->setup_inline());
    add_task(soil_task_4->setup_inline());
    
    Serial.print("Number of added tasks: "); Serial.println(tasks.size());
}

void loop() {
  // Get current timestamp.
  uint32_t now = millis();

  // Execute all tasks depending on whether a task is set up and whether it can currently run
  // (depending on rate).
  for (int i = 0; i < tasks.size(); i++) {
    tasks.get(i)->run(now, false); 
  }

  delay(10);
}
