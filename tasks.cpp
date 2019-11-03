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

// Include tasks header file.
#include "tasks.h"

// --------------------------------------------------------
//            AS7262SensorDataCollectorClass
// --------------------------------------------------------
AS7262SensorDataCollector::AS7262SensorDataCollector(
        uint32_t rate,
        DataManager<float>* dm)
    : TimedTask(millis(), rate, "as7262"), dm_(dm) {
    // Instantiate sensor class.
    sensor_ = new Adafruit_AS726x();

    // Initialize internals.
    measurement_started_ = false;
}

bool AS7262SensorDataCollector::is_connected() {
  return ping_i2c_device(AS726x_ADDRESS);
}

bool AS7262SensorDataCollector::setup_helper() {
  // Only set up the sensor if it is connected.
  if (is_connected()) {
    Serial.println("Found sensor. Trying to setup...");
    // Initialize sensor.
    while (!sensor_->begin()) {
      Serial.println(F("Did not find a valid AS7262 sensor!"));
      delay(1000);
    }

    return true;
  }

  return false;
}

void AS7262SensorDataCollector::run_helper() {
  // Start a color measurement if none is currently pending.
  if (!measurement_started_) {
    Serial.println("Starting measurement...");
    sensor_->startMeasurement();

    // Update internal state.
    measurement_started_ = true;
  } else {
    // Check if data is available.
    Serial.print("Waiting for measurement to complete.");

    if (sensor_->dataReady()) {
      // Read data for all 6 channels.
      Serial.println(); Serial.println("Reading raw values...");
      sensor_->readRawValues(sensor_data_);

      /* The sensor readings have the following units:
       * 
       * Datasheet and application notes: https://ams.com/as7262
       *
       * NOTE: The 6-channel color measurements can take over 300
       *       ms. This call cannot block for that long and
       *       therefore needs to be implemented in an
       *       asynchronous way.
       *
       * Each channel: counts/(μW/cm2) (16 bit value, according to
       *                                spectral responsivity curves
       *                                in Fig. 13 in the datasheet)
       */
      dm_->update(name_ + "_violet", sensor_data_[AS726x_VIOLET]);
      dm_->update(name_ + "_blue", sensor_data_[AS726x_BLUE]);
      dm_->update(name_ + "_green", sensor_data_[AS726x_GREEN]);
      dm_->update(name_ + "_yellow", sensor_data_[AS726x_YELLOW]);
      dm_->update(name_ + "_orange", sensor_data_[AS726x_ORANGE]);
      dm_->update(name_ + "_red", sensor_data_[AS726x_RED]);

      // Read the device temperature and update data manager.
      uint8_t temperature = sensor_->readTemperature();
      dm_->update(name_ + "_temperature", temperature);

      // Update internal state.
      measurement_started_ = false;
    }
  }
}

// --------------------------------------------------------
//            BME680SensorDataCollectorClass
// --------------------------------------------------------
BME680SensorDataCollector::BME680SensorDataCollector(
        uint32_t rate,
        DataManager<float>* dm)
    : TimedTask(millis(), rate, "bme680"),
      dm_(dm) {
  // Instantiate sensor.
  sensor_ = new Adafruit_BME680();
}

bool BME680SensorDataCollector::is_connected() {
  return ping_i2c_device(BME680_DEFAULT_ADDRESS);
}

bool BME680SensorDataCollector::setup_helper() {
  // Only set up the sensor if it is connected.
  if (is_connected()) {
    // Initialize sensor.
    while (!sensor_->begin()) {
      Serial.println(F("Did not find a valid BME680 sensor!"));
      delay(1000);
    }

    // Set up oversampling and filter initialization
    sensor_->setTemperatureOversampling(BME680_OS_8X);
    sensor_->setHumidityOversampling(BME680_OS_2X);
    sensor_->setPressureOversampling(BME680_OS_4X);
    sensor_->setIIRFilterSize(BME680_FILTER_SIZE_3);
    sensor_->setGasHeater(320, 150); // 320*C for 150 ms

    return true;
  }

  return false;
}

void BME680SensorDataCollector::run_helper() {
  // Read temperature, pressure, humidity, gas, and altitued data.
  if (sensor_->performReading()) {
    /* The sensor readings have the following units:
     *
     * Datsheet and application notes:
     *
     *    https://www.bosch-sensortec.com/bst/products/
     *    all_products/bme680
     *
     * Temperature   : *C
     * Pressure      : hPa
     * Humidity      : %
     * Gas resistance: KOhms
     * Altitude      : m
     */
    dm_->update(name_ + "_temperature", sensor_->temperature);
    dm_->update(name_ + "_pressure", sensor_->pressure / 100.0);
    dm_->update(name_ + "_humidity", sensor_->humidity);
    dm_->update(name_ + "_gas_resistance",
            sensor_->gas_resistance / 1000.0);
    dm_->update(name_ + "_altitude",
            sensor_->readAltitude(SEALEVELPRESSURE_HPA));
  } else {
    /* Since a task does not have access to the MQTT interface
     * directly, error handling just sets the sensor values to
     * invalid values.
     */
    dm_->update(name_ + "_temperature", FLOAT_INVALID);
    dm_->update(name_ + "_pressure", FLOAT_INVALID);
    dm_->update(name_ + "_humidity", FLOAT_INVALID);
    dm_->update(name_ + "_gas_resistance", FLOAT_INVALID);
    dm_->update(name_ + "_altitude", FLOAT_INVALID);
  }
}


// --------------------------------------------------------
//       MotionSensorDataCollector Class
// --------------------------------------------------------
MotionSensorDataCollector::MotionSensorDataCollector(
        uint32_t rate,
        uint8_t sensor_pin,
        DataManager<float>* dm,
        uint16_t threshold)
    : TimedTask(millis(), rate, "motion"),
      dm_(dm),
      sensor_pin_(sensor_pin),
      threshold_(threshold) {}

bool MotionSensorDataCollector::is_connected() {
  /* Since a motion sensor just uses a single analog or digital
   * input pin (and not I2C) the same ping method as for I2C
   * devices cannot be used here. For motion sensors, this method
   * therefore relies on a flag being set in the config.h file.
   */
  return MOTION_SENSOR_CONNECTED;
}

bool MotionSensorDataCollector::setup_helper() {
  // Only set up the sensor if it is connected.
  if (is_connected()) {
    // Set up motion sensor pin.
    pinMode(MOTION_SENSOR_PIN, INPUT);
    return true;
  }

  return false;
}

void MotionSensorDataCollector::run_helper() {
  // Read motion sensor data.
  uint16_t motion_level = analogRead(sensor_pin_);

  /* If sensor level exceeds a threshold add new data.
   * 
   * Datasheet and tutorials:
   *
   *    https://learn.adafruit.com/pir-passive-infrared-proximity-
   *    motion-sensor/overview
   *
   * NOTE: If no motion was detected, the motion state is reset
   *       in the DataCollector.
   */
  if (motion_level > threshold_) {
    dm_->update(name_ + "_alert", true);
  }
}

// --------------------------------------------------------
//            MPL3115A2SensorDataCollectorClass
// --------------------------------------------------------
MPL3115A2SensorDataCollector::MPL3115A2SensorDataCollector(
        uint32_t rate,
        DataManager<float>* dm)
    : TimedTask(millis(), rate, "mpl3115a2"),
      dm_(dm) {
  // Instantiate sensor class.
  sensor_ = new Adafruit_MPL3115A2();
}

bool MPL3115A2SensorDataCollector::is_connected() {
  return ping_i2c_device(MPL3115A2_ADDRESS);
}

bool MPL3115A2SensorDataCollector::setup_helper() {
  // Only set up the sensor if it is connected.
  if (is_connected()) {
    // Initialize sensor.
    while (!sensor_->begin()) {
      Serial.println(F("Did not find a valid MPL3115A2 sensor!"));
      delay(1000);
    }

    // Set sea pressure level in pascal.
    sensor_->setSeaPressure(SEALEVELPRESSURE_HPA * 100.0);

    return true;
  }

  return false;
}

void MPL3115A2SensorDataCollector::run_helper() {
  /* The sensor readings have the following units:
   *
   * Datasheet:
   *
   *    http://cache.freescale.com/files/sensors/doc/
   *    data_sheet/MPL3115A2.pdf
   *
   * NOTE: Taking temperature readings seem to set the sensor into
   *       a different mode that messes up altitude and barometric
   *       pressure readings. This issue only seems to occur when
   *       the Adafruit library (or also the Sparkfun one) is used
   *       as part of this timed task setup.
   * TODO: Triage this issue and fix this bug. Until then, don't
   *       measure temperature with this sensor.
   *
   * Altitude      : m
   * Pressure      : hPa
   * Temperature   : *C
   */
  // Get sensor data readings.
  float pressure = sensor_->getPressure() / 100.0;
  float altitude = sensor_->getAltitude();

  dm_->update(name_ + "_altitude", altitude);
  dm_->update(name_ + "_pressure", pressure);

  //float temperature = sensor_->getTemperature();
  //dm_->update(name_ + "_temperature", temperature);
}

// --------------------------------------------------------
//            SGP30SensorDataCollectorClass
// --------------------------------------------------------
SGP30SensorDataCollector::SGP30SensorDataCollector(
        uint32_t rate,
        DataManager<float>* dm)
    : TimedTask(millis(), rate, "sgp30"),
      dm_(dm) {
  // Instantiate sensor class.
  sensor_ = new Adafruit_SGP30();

  // Initialize counter variable.
  baseline_measurement_start_time_ = millis();
}

bool SGP30SensorDataCollector::is_connected() {
  return ping_i2c_device(SGP30_I2CADDR_DEFAULT);
}

bool SGP30SensorDataCollector::setup_helper() {
  // Only set up the sensor if it is connected.
  if (is_connected()) {
    // Initialize sensor.
    while (!sensor_->begin()) {
      Serial.println(F("Did not find a valid SGP30 sensor!"));
      delay(1000);
    }

    return true;
  }

  return false;
}

void SGP30SensorDataCollector::run_helper() {
  /* If a temperature / humidity sensor is available, one can set
   * the absolute humidity to enable the humditiy compensation for
   * the air quality signals.
   *
   * For exmple:
   *   float temperature = 22.1;  // [°C]
   *   float humidity = 45.2;     // [%RH]
   *   float absolute_humidity =  // [mg/m^3]
   *       getAbsoluteHumidity(temperature, humidity));
   *   sensor_->setHumidity(absolute_humidity);
   */
  if (sensor_->IAQmeasure()) {
    /* Update Total Volatile Organic Compound (TVOC) measurements.
     *
     * Datasheet:
     *
     *    https://www.sensirion.com/fileadmin/user_upload/customers/
     *    sensirion/Dokumente/0_Datasheets/Gas/
     *    Sensirion_Gas_Sensors_SGP30_Datasheet.pdf
     *
     * The sensor readings have the following units:
     *
     * eCO2: ppm
     * TVOC: ppb
     */
    dm_->update(name_ + "_tvoc", sensor_->TVOC);
    dm_->update(name_ + "_eco2", sensor_->eCO2);
  } else {
    // Set values indicating data acquisition failure.
    dm_->update(name_ + "_tvoc", FLOAT_INVALID);
    dm_->update(name_ + "_eco2", FLOAT_INVALID);
  }

  if (sensor_->IAQmeasureRaw()) {
    /* The sensor readings have the following units:
     *
     * Raw H2     : ppm
     * Raw ethanol: ppm
     */
    dm_->update(name_ + "_raw_h2", sensor_->rawH2);
    dm_->update(name_ + "_raw_ethanol", sensor_->rawEthanol);
  } else {
    // Set values indicating data acquisition failure.
    dm_->update(name_ + "_raw_h2", FLOAT_INVALID);
    dm_->update(name_ + "_raw_ethanol", FLOAT_INVALID);
  }

  /* NOTE: The following measurements should be done after a 30
   *       second warmup period. The 30 second wait period is
   *       implemented asynchronously to avoid blocking.
   */
  if ((millis() - baseline_measurement_start_time_) > 
          (SGP30_BASELINE_MEASUREMENT_TIME_SEC * 1000)) {
    // Reset baseline measurement time.
    baseline_measurement_start_time_ = millis();
    Serial.print("Starting SGP baseline measurement...");

    uint16_t TVOC_base, eCO2_base;
    if (sensor_->getIAQBaseline(&eCO2_base, &TVOC_base)) {
    /* The sensor readings have the following units:
     *
     * Baseline eCO2: ppm
     * Baseline TVOC: ppb
     */
      dm_->update(name_ + "_baseline_eco2", eCO2_base);
      dm_->update(name_ + "_baseline_tvoc", TVOC_base);
    } else {
      // Set values indicating data acquisition failure.
      dm_->update(name_ + "_baseline_eco2", FLOAT_INVALID);
      dm_->update(name_ + "_baseline_tvoc", FLOAT_INVALID);
    }
  }
}

// --------------------------------------------------------
//            Si7021SensorDataCollectorClass
// --------------------------------------------------------
Si7021SensorDataCollector::Si7021SensorDataCollector(
        uint32_t rate,
        DataManager<float>* dm)
    : TimedTask(millis(), rate, "si7021"),
      dm_(dm) {
  // Set up sensor class.
  sensor_ = new Adafruit_Si7021();
}

bool Si7021SensorDataCollector::is_connected() {
  return ping_i2c_device(SI7021_DEFAULT_ADDRESS);
}

bool Si7021SensorDataCollector::setup_helper() {
  // Only set up the sensor if it is connected.
  if (is_connected()) {
    // Initialize sensor.
    while (!sensor_->begin()) {
      Serial.println(F("Did not find Si7021 sensor!"));
    }

    return true;
  }

  return false;
}

void Si7021SensorDataCollector::run_helper() {
  // Read temperature and humidity sensor data.
  float humidity = sensor_->readHumidity();
  float temperature = sensor_->readTemperature();

  /* The sensor readings have the following units:
   *
   * Datasheet:
   *
   *    https://www.silabs.com/documents/public/data-sheets/
   *    Si7021-A20.pdf
   *
   * Application notes:
   *
   *    https://www.silabs.com/documents/public/application-notes/
   *    AN607.pdf
   *
   * Temperature   : *C
   * Humidity      : %
   */
  dm_->update(name_ + "_temperature", temperature);
  dm_->update(name_ + "_humidity", humidity);
}


uint32_t SGP30SensorDataCollector::getAbsoluteHumidity(
    float temperature,
    float humidity) {
  /* Approximation formula from Sensirion SGP30 Driver
   * Integration chapter 3.15.
   *
   * Intermediate quantities have the following units:
   *
   * Absolute humidity       : [g/m^3]
   * Absolute humidity scaled: [g/m^3]
   */
  const float exponent = (17.62f * temperature) /
                         (243.12f + temperature);

  const float absolute_humidity = 216.7f *
    ((humidity / 100.0f) * 6.112f * 
     exp(exponent) / (273.15f + temperature));

  const uint32_t absolute_humidity_scaled =
    static_cast<uint32_t>(1000.0f * absolute_humidity);

  return absolute_humidity_scaled;
}

// --------------------------------------------------------
//            SoilMoistureSensorDataCollectorClass
// --------------------------------------------------------
SoilMoistureSensorDataCollector::SoilMoistureSensorDataCollector(
        uint32_t rate,
        DataManager<float>* dm,
        uint8_t address,
        String name)
    : TimedTask(millis(), rate, name),
      address_(address),
      dm_(dm) {
  // Instantiate sensor.      
  sensor_ = new Adafruit_seesaw();
}

bool SoilMoistureSensorDataCollector::is_connected() {
  return ping_i2c_device(address_);
}

bool SoilMoistureSensorDataCollector::setup_helper() {
  // Only set up the sensor if it is connected.
  if (is_connected()) {
    // Initialize sensor.
    while (!sensor_->begin(address_)) {
      Serial.println(F("Did not find a valid soil moisture sensor!"));
      delay(1000);
    }

    return true;
  }

  return false;
}

void SoilMoistureSensorDataCollector::run_helper() {
  // Read sensor data.
  uint16_t capacitive_read = sensor_->touchRead(0);
  float temperature = sensor_->getTemp();

  /* The sensor readings have the following units:
   *
   * Tutorial:
   *    https://learn.adafruit.com/adafruit-stemma-soil-sensor-
   *    i2c-capacitive-moisture-sensor/overview
   *
   * Temperature   : *C
   * Capacitance   : ~F (using capacitive touch of the built
   *                     in ATSAMD10 chip)
   */
  dm_->update(name_ + "_capacitive", capacitive_read);
  dm_->update(name_ + "_temperature", temperature);
}

// --------------------------------------------------------
//            TSL2591SensorDataCollector Class
// --------------------------------------------------------
TSL2591SensorDataCollector::TSL2591SensorDataCollector(
        uint32_t rate,
        TwoWire &wire,
        DataManager<float>* dm)
    : TimedTask(millis(), rate, "tsl2591"),
      dm_(dm) {
  // Instantiate sensor.
  sensor_ = new Adafruit_TSL2591(2591);
}

bool TSL2591SensorDataCollector::is_connected() {
  return ping_i2c_device(TSL2591_ADDR);
}

bool TSL2591SensorDataCollector::setup_helper() {
  // Only set up the sensor if it is connected.
  if (is_connected()) {
    // Initialize sensor.
    while (!sensor_->begin()) {
      Serial.println(F("Did not find TSL2591 sensor!"));
      delay(1000);
    }

    /* Set sensor gain.
     * 
     * Possible values are the following:
     *
     *  - TSL2591_GAIN_LOW :   1x gain (bright light)
     *  - TSL2591_GAIN_MED :  25x gain
     *  - TSL2591_GAIN_HIGH: 428x gain (dim light)
     */
    sensor_->setGain(TSL2591_GAIN_MED);

    /* Set integration time.
     *
     * Changing the integration time results in a longer time over
     * which to sense light. Larger integration times result in
     * slower readings, but are good in very low light situtations. 
     *
     * Possible values are the following:
     * (from shortest integration time to longest)
     * 
     *  - TSL2591_INTEGRATIONTIME_100MS: (bright light)
     *  - TSL2591_INTEGRATIONTIME_200MS
     *  - TSL2591_INTEGRATIONTIME_300MS
     *  - TSL2591_INTEGRATIONTIME_400MS
     *  - TSL2591_INTEGRATIONTIME_500MS
     *  - TSL2591_INTEGRATIONTIME_600MS: (dim light)
     */
    sensor_->setTiming(TSL2591_INTEGRATIONTIME_300MS);

    return true;
  }

  return false;
}

void TSL2591SensorDataCollector::run_helper() {
  // Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum.
  uint32_t luminosity = sensor_->getFullLuminosity();
  uint16_t ir = luminosity >> 16;
  uint16_t full = luminosity & 0xFFFF;
  float lux = sensor_->calculateLux(full, ir);
  
  /* The sensor readings have the following units:
   *
   * Datasheet:
   *    https://ams.com/documents/20143/36005/TSL2591_DS000338_6
   *    -00.pdf/090eb50d-bb18-5b45-4938-9b3672f86b80
   *
   * Luminosity: Normalized spectral responsivity 32 bits
   * Full      : Normalized spectral responsivity 16 LSB
   * IR        : Normalized spectral responsivity 16 MSB
   * Visible   : Normalized spectral responsivity (Full - IR)
   * Lux       : Lux
   */
  dm_->update(name_ + "_full", full);
  dm_->update(name_ + "_ir", ir);
  dm_->update(name_ + "_luminosity", luminosity);
  dm_->update(name_ + "_visible", full - ir);
  dm_->update(name_ + "_lux", lux);
}

// --------------------------------------------------------
//            VEML6075SensorDataCollectorClass
// --------------------------------------------------------
VEML6075SensorDataCollector::VEML6075SensorDataCollector(
        uint32_t rate,
        DataManager<float>* dm)
    : TimedTask(millis(), rate, "veml6075"),
      dm_(dm) {
  // Instantiate sensor class.
  sensor_ = new Adafruit_VEML6075();
}

bool VEML6075SensorDataCollector::is_connected() {
  /* Since this device uses a conflicting I2C address (0x10) with
   * the VEML7700 sensor, the is_connected() function needs to
   * check the ID register of the device for a match (as done int
   * he VEML6075 Adafruit library).
   */
  return ping_i2c_device_veml_6075(VEML6075_ADDR);
}

bool VEML6075SensorDataCollector::setup_helper() {
  // Only set up the sensor if it is connected.
  if (is_connected()) {
    // Initialize sensor.
    while (!sensor_->begin()) {
      Serial.println(F("Did not find a valid VEML6075 sensor!"));
      delay(1000);
    }

    // Set the integration constant.
    sensor_->setIntegrationTime(VEML6075_100MS);

    // Set the high dynamic mode.
    sensor_->setHighDynamic(true);

    // Set the mode.
    sensor_->setForcedMode(false);

    // Set the calibration coefficients (two coefficients for
    // UVA_A and UVA_B, two for UVB_C, UVB_D, and two for UVA and
    // UVB responses).
    sensor_->setCoefficients(2.22, 1.33,
                             2.95, 1.74,
                             0.001461, 0.002591);

    return true;
  }

  return false;
}

void VEML6075SensorDataCollector::run_helper() {
  /* The sensor readings have the following units:
   *
   * Datasheet:
   *    https://cdn.sparkfun.com/assets/3/c/3/2/f/veml6075.pdf
   *
   * See http://www.patarnott.com/atms748/pdf/Class2018/
   *     designingveml6075_VishayUVaUVbsensors.pdf
   *
   * As already mentioned, this UVA / UVB sensor will need a
   * well-selected cover that is not only completely transmissive
   * to visible light (400 nm to 700 nm), but also to UVA and UVB
   * wavelengths (280 nm to 400 nm).
   *
   * Teflon or polytetrafluoroethylene (PTFE) is a known optical
   * material that allows transmission of UV up to near infrared
   * signals. A teflon diffusor (PTFE sheet) radiates like Lambert’s
   * cosine law. Thus PTFE enables a cosine angular response for a
   * detector measuring the optical radiation power at a surface.
   *
   * UVA / UVB sensing resolution:
   *
   * - 0.0011 UVI per count OR
   * - 9 counts per 0.01 UVI OR
   * - 909 counts per 1 UVI
   *
   * UVA     : counts (sensing resolution 909 counts per 1 UVI)
   * UVB     : counts
   * UV Index: computed index ranging from 0 - 12
   */
  dm_->update(name_ + "_uva", sensor_->readUVA());
  dm_->update(name_ + "_uvb", sensor_->readUVB());
  dm_->update(name_ + "_uv_index", sensor_->readUVI());
}

// --------------------------------------------------------
//            VEML7700SensorDataCollectorClass
// --------------------------------------------------------
VEML7700SensorDataCollector::VEML7700SensorDataCollector(
        uint32_t rate,
        DataManager<float>* dm)
    : TimedTask(millis(), rate, "veml7700"),
      dm_(dm) {
  // Instantiate sensor class.
  sensor_ = new Adafruit_VEML7700();
}

bool VEML7700SensorDataCollector::is_connected() {
  /*
   * The VEML7700 shares the same I2C address (0x10) as the
   * VEML6075. To be able to tell them apart, one can check the ID
   * register on the VEML6075. That register is not available on
   * the VEML7700, which should result in the
   * ping_i2c_device_veml_6075() method to fail.
   */
  return ping_i2c_device(VEML7700_I2CADDR_DEFAULT) && 
         !ping_i2c_device_veml_6075(VEML6075_ADDR);
}

bool VEML7700SensorDataCollector::setup_helper() {
  // Only set up the sensor if it is connected.
  if (is_connected()) {
    // Initialize sensor.
    while (!sensor_->begin()) {
      Serial.println(F("Did not find a valid VEML7700 sensor!"));
      delay(1000);
    }

    /* Set sensor gain and integration time.
     *
     * NOTE: The multiplier in the Adafruit library is hard-coded
     *       to 0.0576 which is only valid for the following
     *       combinations for gain and integration time:
     *
     * - VEML7700_GAIN_1_8 - VEML7700_IT_800MS
     * - VEML7700_GAIN_1_4 - VEML7700_IT_400MS
     * - VEML7700_GAIN_1   - VEML7700_IT_100MS
     * - VEML7700_GAIN_2   - VEML7700_IT_50MS
     */
    sensor_->setGain(VEML7700_GAIN_1);
    sensor_->setIntegrationTime(VEML7700_IT_100MS);

    return true;
  }

  return false;
}

void VEML7700SensorDataCollector::run_helper() {
  /* The sensor readings have the following units:
   * 
   * Datasheet:
   *   https://www.vishay.com/docs/84286/veml7700.pdf
   *
   * Application note:
   *   https://www.vishay.com/docs/84323/designingveml7700.pdf
   *
   * Note that the multiplier of 0.0576 that is hard-coded in the
   * Adafruit library is only valid for a gain of 1 and an
   * integration time of 100 ms.
   *
   * Lux        : Lux (0 lx to ~ 120 klx, follows human eye curve,
   *                   Fig. 12 in appliation note)
   * White light: Lux (0 lx to ~ 120 klx, follows white light curve,
   *                   Fig. 13 in appliation note)
   * Raw ALS    : Raw ambient light sensor (ALS) count with
   *              resolution as low as 0.0036 lx/ct (follows the
   *              the so-called human eye curve)
   *
   */
  dm_->update(name_ + "_lux", sensor_->readLux());
  dm_->update(name_ + "_white_light", sensor_->readWhite());
  dm_->update(name_ + "_raw_als", sensor_->readALS());
}

// --------------------------------------------------------
//               DataCollector Class
// --------------------------------------------------------
DataCollector::DataCollector(uint32_t rate,
                             MQTTInterface* mqtt_interface,
                             DataManager<float>* dm,
                             bool batch_publish)
    : TimedTask(millis(), rate, "data_collector"),
      mqtt_interface_(mqtt_interface),
      dm_(dm),
      batch_publish_(batch_publish) {}

bool DataCollector::is_connected() {
  return true;
}

bool DataCollector::setup_helper() {
  // Set up MQTT interface class.
  if (mqtt_interface_ != nullptr) {
    // Set up MQTT interface class.
    mqtt_interface_->setup(true);

    // Compute MQTT topic string.
    mqtt_topic_ = "/sensors/" + get_mac_address();
  }

  return true;
}

void DataCollector::run_helper() {
  // Print all new data that have been collected since last time.
  if (dm_->has_new_data()) {
    if (batch_publish_) {
      publish_new_data_batch();
    } else {
      publish_new_data();
    }
  }
}

void DataCollector::publish_new_data() {
  /* This is the recommended method of publishing data, one
   * key-value pair at a time. While this generates as many
   * messages as new key-value pairs exist at a given execution
   * time, its publishing behavior and resource requirements are a
   * lot more predictable.
   */
  if (mqtt_interface_ != nullptr) {
    // Publish all new key-value data pairs.
    for (int i = 0; i < dm_->get_size(); i++) {
      if (dm_->get_is_new(i)) {
        // Reset is_new flag.
        dm_->set_is_new(i, false);

        // Create JSON string.
        String json_string = "{'" + dm_->get_name(i) + "':" +
          String(dm_->get_value(i)) + "}";

        // Publish JSON string.
        Serial.print("Publishing data: ");
        Serial.println(json_string);
        mqtt_interface_->publish(mqtt_topic_, json_string);
      }
    }
  }
}

void DataCollector::publish_new_data_batch() {
  /* NOTE: Batch publishing, while convenient, may not work on
   *       resource-constrained devices such as the ESP8266 or
   *       ESP32. The main reason seems to be the limited buffer
   *       size of the MQTT client and the fact that no maximum
   *       message length can be guaranteed given that an arbitrary
   *       number of sensors can be connected to a sensor node.
   */
  // Create a single JSON string from all new data.
  String data_json_string = get_new_data_as_json_string();

  // Publish a message via MQTT.
  if (mqtt_interface_ != nullptr) {
    Serial.print("Publishing data to topic ");
    Serial.print(mqtt_topic_); Serial.println(" ...");
    Serial.println(data_json_string);
    mqtt_interface_->publish(mqtt_topic_, data_json_string);
  }
}

String DataCollector::get_new_data_as_json_string() {
  String json_string = "{";

  // Add all key-value data pairs.
  for (int i = 0; i < dm_->get_size(); i++) {
    if (dm_->get_is_new(i)) {
      // Reset is_new flag.
      dm_->set_is_new(i, false);

      // Add key-value pair to JSON string.
      // NOTE: This function assumes numeric data values. Strings
      //       are NOT supported as data values since their values
      //       would have to be encapsulated in quotes.
      json_string += "'" + dm_->get_name(i) + "':";
      json_string += String(dm_->get_value(i)) + ",";
    }
  }

  // Remove trailing comma.
  json_string.remove(json_string.length() -  1);

  return json_string + "}";
}

String DataCollector::get_mac_address() {
  // Get raw mac address as array of bytes.
  unsigned char mac[6];
  WiFi.macAddress(mac);

  // Convert to string.
  String  mac_address = "";
  for (int i = 0; i < 6; ++i) {
    mac_address += String(mac[i], 16);
  }

  return mac_address;
}
