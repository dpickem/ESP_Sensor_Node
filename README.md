# ESP Sensor Node

The ESP sensor node library implements a flexible framework for managing timed tasks that are meant to be executed periodically in a non-blocking asynchronous fashion. A number of sensors are supported out of the box but at the same time it is trivial to add a new sensor to the framework. Since this class depends on Wifi connectivity, it can only be used on Wifi-enabled microcontrollers such as the **ESP8266** or **ESP32**.

## Installation

Clone the repository from [here](https://github.com/dpickem/ESP_Sensor_Node.git)

## Dependencies

The sensor node framework has a number of 2nd and 3rd party dependencies which include sensor libraries (mostly Adafruit libraries), data structure libraries, as well as connectivity and data management dependencies. 

### 2nd party dependencies

* **MQTTInterface**: https://github.com/dpickem/Arduino_MQTTInterface
* **DataManager**: https://github.com/dpickem/Arduino_DataManager

### 3rd party data structure dependencies

* **LinkedList**: https://github.com/ivanseidel/LinkedList

### 3rd party sensor libraries

* **Adafruit Sensor Library**: https://github.com/adafruit/Adafruit_Sensor.git
* **Adafruit BusIO Library**: https://github.com/adafruit/Adafruit_BusIO.git
* **AS7262**: https://github.com/adafruit/Adafruit_AS726x.git
* **BME 680**: https://github.com/adafruit/Adafruit_BME680.git
* **MPL3115A2**:https://github.com/adafruit/Adafruit_MPL3115A2_Library.git
* **SGP30**: https://github.com/adafruit/Adafruit_SGP30.git
* **Si7021**: https://github.com/adafruit/Adafruit_Si7021
* **Soil moisturer sensor**: https://github.com/adafruit/Adafruit_Seesaw.git
* **TSL2591**: https://github.com/adafruit/Adafruit_TSL2591_Library
* **VEML 6075**: https://github.com/adafruit/Adafruit_VEML6075.git
* **VEML 7700**: https://github.com/adafruit/Adafruit_VEML7700.git

## Getting started

The easiest way to get started is to deploy the main example sketch, which should work out of the box after setting Wifi and MQTT credentials. The sketch and task framework is set up such that connected sensors are automatically recognized and setup (with the exception of motion sensors which need to be activated via a flag in config.h).

### Deploying a sensor node.

* Set Wifi and MQTT broker credentials in the config file of the MQTT interface class.
* Ensure that an MQTT broker is running.
* Compile and upload the main sketch to an ESP8266 or ESP32 (untested) microcontroller.
* Watch the data stream to your MQTT broker.

### Adding a new task.

* Add a new task declaration to the tasks.h file.
* Each new task class needs to declare and implement the following three
  functions:
  * bool is_connected(): To determine if a sensor is connected.
  * bool setup_helper(): To setup the sensor and determine if the setup was successful.
  * void run_helper(): To execute the data collection step in a non-blocking way. This can either be accomplished by taking a sensor reading very quickly or by asynchronously measuring data (an example of asynchronous measurements is shown in the class SGP30SensorDataCollector).
* Implement each required function for the new task in tasks.cpp.
* Instantiate an instance of the new task in main.ino and add the task via add_task() in the Arduino setup method. The add_task() methodwill take care of setting the task up and adding it to the list of executable tasks.

### Version History

* `1.0 (2019-11-02)`: Original release
