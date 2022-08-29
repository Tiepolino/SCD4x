/*******************************************************************************
 * SCD4x.cpp
 *
 * Interface for the SCD4x CO2, Temperature and Relative Humidity sensor
 *
 * Author				: Tiebolt van der Linden
 * Created			: 2022-07-08 / 17.48
 * Last Changed	: 2022-08-29 / 10.08
 *
 * ToDo
 *    
 * History
 *    20220829 - Updated comments and function descriptions
 *    20220829 - Code tested
 *    20220827 - Bugfix in _writeData(cmd, data) function, crc fixed.
 *    20220826 - Code completed
 *    20220825 - Building up class functions
 *		20220708 - Initial Version
 */

#include "SCD4x.h"

/**
 * @brief Construct a new SCD4x object
 * 
 * @param wire - OneWire object
 */
SCD4x::SCD4x(uint8_t sensorType, TwoWire *wire) : _wire(wire) {
  _co2 = 0;
  _temperature = 0.0;
  _humidity = 0;
  _sensorType = sensorType;
}

#if defined (ESP8266) || defined (ESP32)
/**
 * @brief Startup the DHT20 sensor
 * 
 * @param i2cAddress  - optional - i2c Address of the sensor
 * @param sda         - optional - SDA pin
 * @param scl         - optional - SCL pin
 * @return true       - On success
 * @return false      - On failure
 */
bool SCD4x::begin(uint8_t i2cAddress, uint8_t sda, uint8_t scl) {
  uint8_t cmd[3] = {0x71};                                                      // Get status command
  uint8_t result;

  while(millis() < SCD4x_INIT_DELAY);                                           // Make sure the sensor is initialized

  // ---- Check if we need to setup I2C pins ----
  if ((sda < 0xff) && (scl < 0xff)) {
    _wire.begin(sda, scl);
  } else {
    _wire.begin();
  }

  _i2cAddress = i2cAddress;                                                     // Set the i2c address

  _wire->beginTransmission(_i2cAddress);                                        // Try sending to the i2c address
  result = _wire->endTransmission();                                            // Get the result from the sensor
  if (result != 0) {                                                            // Check if the sensor is there
    _lastError = SCD4x_ERROR_NOSENSOR;                                          // If not set lastError
    return false;
  }

  // ---- Get the serial number from the sensor ----
  result = _getSerialNumber();                                                  // Stop measurements before reading Serial Number
  if (!result) {
    return false;
  }

  return true;
}
#else
/**
 * @brief Startup the DHT20 sensor
 * 
 * @param i2cAddress  - optional - i2c address of the sensor
 * @return true       - On success
 * @return false      - On failure
 */
bool SCD4x::begin(uint8_t i2cAddress) {
  uint8_t result = 0;
  _i2cAddress = i2cAddress;                                                     // Set the i2c address

  // ---- Check if there is a sensor at the i2c address ----
  while(millis() < SCD4x_INIT_DELAY);                                           // Make sure the sensor is initialized
  _wire->begin();                                                               // Startup the i2c bus
  _wire->beginTransmission(_i2cAddress);                                        // Try sending to the i2c address
  result = _wire->endTransmission();                                            // Get the result from the sensor
  if (result != 0) {                                                            // Check if the sensor is there
    _lastError = SCD4x_ERROR_NOSENSOR;                                          // If not set lastError
    return false;
  }

  // ---- Get the serial number from the sensor ----
  result = _getSerialNumber();                                                  // Stop measurements before reading Serial Number
  if (!result) {
    return false;
  }

  return true;
}
#endif

/**
 * @brief Instructs the sensor to start measuring
 * 
 * @param lowPower - Use low power measurements if true
 * @return true    - On success
 * @return false   - On Failure, lastError will be set
 */
bool SCD4x::startMeasuring(bool lowPower) {
  uint16_t cmd = (lowPower) ? SCD4x_START_MEASURING_LP : SCD4x_START_MEASURING;
  bool result = true;

  _lastError = SCD4x_ERROR_NONE;                                                // Just started so no errors

  result = _writeCommand(cmd);                                                  // Write the command to the sensor
  if (result) {
    _measuring = true;                                                          // We are now measuring
    _lowPower = lowPower;                                                       // Store if we are operating in low power or not
  }

  return result;
}

/**
 * @brief Instructs the sensor to stop measuring
 * 
 * @return true   - On success
 * @return false  - On Failure, lastError will be set
 */
bool SCD4x::stopMeasuring(void) {
  bool result = true;;

  _lastError = SCD4x_ERROR_NONE;                                                // Just started so no errors

  result = _writeCommand(SCD4x_STOP_MEASURING);                                 // Write the command to the sensor
  if (result) {
    _measuring = false;                                                         // We are no longer measuring
    _lowPower = false;                                                          // Set the low power setting to the default

    // ---- Wait for 500 miliseconds ----
    unsigned long now = millis();
    while (millis() - now < 500);
  }

  return result;
}

/**
 * @brief Instructs the SCD41 to perform a single measurement
 * 
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool SCD4x::singleMeasurement(void) {
  if (_sensorType != SCD41) {                                                   // Only supported with SCD41
    _lastError = SCD4x_ERROR_NOT_SUPPORTED;
    return false;
  }

  if (_measuring) {                                                             // Only supported while not measuring
    _lastError = SCD4x_ERROR_MEASURING;
    return false;
  }

  return _writeCommand(SCD4x_SINGLE_MEASUREMENT);                               // Issue the command. Data ready in 5 sec.
}

/**
 * @brief Instructs the SCD41 to perform a single Temp and Humidity measurement only
 * 
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool SCD4x::singleMeasurement_rht(void) {
  if (_sensorType != SCD41 || _measuring == true) {                             // Only supported with SCD41 and while not measuring
    _lastError = SCD4x_ERROR_NOT_SUPPORTED;
    return false;
  }

  return _writeCommand(SCD4x_SINGLE_MEASUREMENT_RHT);                           // Issue the command. Data ready in 50 ms.
}

/**
 * @brief Reads the measurement data from the sensor
 * 
 * @return true  - On success
 * @return false - On failure (lastError will be set)
 */
bool SCD4x::readSensorData(void) {
  uint16_t buffer = 0;

  _lastError = SCD4x_ERROR_NONE;

  // ---- Check if we have data available ----
  if(!dataReady()) {
    _lastError = SCD4x_ERROR_MFAIL;
    return false;
  }

  if(!_writeCommand(SCD4x_READ_MEASUREMENT)) return false;                      // Send the command to the server
  delay(1);                                                                     // Weat for command to complete

  _wire->requestFrom((uint8_t) _i2cAddress, (uint8_t) 9);                       // Read 9 bytes from the sensor
  for (uint8_t step = 0; step < 3; step++) {
    buffer = ((uint16_t) _wire->read()) << 8;                                   // MSB
    buffer |= _wire->read();                                                    // LSB
    
    if (!(_crc8((uint8_t *) &buffer, 2) == _wire->read())) {
      _lastError = SCD4x_ERROR_CRC;
    }

    switch(step) {
      case 0:   // ---- CO2 ----
          _co2 = buffer;
          break;
      case 1:   // ---- Temperature ----
          _temperature = -45 + (((float) buffer) * 175.0 / 65535.0);
          break;
      case 2:   // ---- Relative Humidity ----
          _humidity = ((float) buffer) * 100.0 / 65535.0;
          break;
    }
  }

  return true;
}

/**
 * @brief Returns the measured CO2 value in ppm
 * 
 * @return uint16_t - The CO2 value in ppm
 */
uint16_t SCD4x::getCO2(void) {
  return _co2;
}

/**
 * @brief Returns the temperature measured in the requested scale
 * 
 * @param scale  - Scale to be used with the temperature
 * @return float - Temperature corrected for requested scale
 */
float SCD4x::getTemperature(uint8_t scale) {
  switch(scale) {
    case _CELCIUS :
        return _temperature;
        break;
    case _FAHRENHEIT :
        return (_temperature * 1.8) + 32;
        break;
    case _KELVIN :
        return _temperature + 273.15;
        break;
  }

  return 0;
}

/**
 * @brief Returns the measured humidity in % RH (Relative Humidity)
 * 
 * @return uint8_t - The relative humidity in %
 */
uint8_t SCD4x::getHumidity(void) {
  return _humidity;
}

/**
 * @brief Set the positive temperature offset for the sensor
 * 
 * @param toff    - Temperature offset (0 - 174)°C
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool SCD4x::setTempOffset(float toff) {
  uint16_t offset = 0;

  if (_measuring) {                                                             // Only available while not measuring
    _lastError = SCD4x_ERROR_MEASURING;
    return false;
  }

  if (toff < 0 || toff >= 175) {                                                // Check if offset > 0°C and < 175°C
    _lastError = SCD4x_ERROR_VALUE_OOB;
    return false;
  }

  offset = (uint16_t) (toff * 65536 / 175);                                     // Calculate the temperature offset 
  if (!_writeCommand(SCD4x_SET_TEMPERATURE_OFFSET, offset)) return false;       // Issue the command and write the offset

  delay(1);                                                                     // Give the sensor some time to process
  return true;
}

/**
 * @brief Retrieves the temperature offset from the sensor
 * 
 * @return float - Temperature offset or 175.0 in case of an error
 */
float SCD4x::getTempOffset(void) {
  uint16_t offset = 0;

  if (_measuring) {                                                             // Only available if not measuring
    _lastError = SCD4x_ERROR_MEASURING;
    return false;
  }

  if(!_readData(SCD4x_GET_TEMPERATURE_OFFSET, &offset, 1)) return 175.0;        // Read the result from the sensor

  return ((float) offset) * 175.0 / 65535.0;                                    // Calculate and return the offset temperature
}

/**
 * @brief Sets the hight in meters above sea level this sensor is operating on
 * 
 * @param alt     - Altitude in meters
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool SCD4x::setSensorAltitude(uint16_t alt) {
  if (_measuring) {                                                             // Only available is not measuring
    _lastError = SCD4x_ERROR_MEASURING;
    return false;
  }

  if (!_writeCommand(SCD4x_SET_SENSOR_ALTITUDE, alt)) return false;             // Issue the command and write the altitude

  delay(1);                                                                     // Give the sensor some time to process
  return true;
}

/**
 * @brief Reads the stored altitude from the sensor
 * 
 * @return uint16_t - Altitude in meters above sea level
 */
uint16_t SCD4x::getSensorAltitude(void) {
  uint16_t altitude = 0;

  _lastError = SCD4x_ERROR_NONE;                                                // Just started, no error

  if (_measuring) {                                                             // Only available is not measuring
    _lastError = SCD4x_ERROR_MEASURING;
    return false;
  }

  if(!_readData(SCD4x_GET_SENSOR_ALTITUDE, &altitude, 1)) return 0xFFFF;        // Read the result from the sensor

  return altitude;                                                              // Return the sensor altitude
}

/**
 * @brief Set the ambient pressure in mbar (Pa / 100)
 * 
 * @param mbar    - The current ambient pressure
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool SCD4x::setAmbientPressure(uint16_t mbar) {
  _lastError = SCD4x_ERROR_NONE;                                                // No error, just strated

  if (mbar < 850 || mbar > 1100) {                                              // Check if the value is valid
    _lastError = SCD4x_ERROR_VALUE_OOB;
    return false;
  }

  if (!_writeCommand(SCD4x_SET_SENSOR_ALTITUDE, mbar)) return false;

  delay(1);
  return true;
}

/**
 * @brief Writes the settings from ram to eeprom
 * 
 * @return true   - On succes
 * @return false  - On failure (lastError will be set)
 */
bool SCD4x::persistSettings(void) {
  _lastError = SCD4x_ERROR_NONE;

  if (_measuring) {                                                             // Only available is not measuring
    _lastError = SCD4x_ERROR_MEASURING;
    return false;
  }

  if(!_writeCommand(SCD4x_PERSIST_SETTINGS)) return false;                      // Write the command to the sensor
  
  delay(800);                                                                   // Operation takes max 800ms
  return true;
}

/**
 * @brief Forces the calibration of the sensor
 * 
 * @param concentration   - The current CO2 concentration in PPM
 * @return int16_t        - The correction value against the old calibration or 0xFFFF on error
 */
int16_t SCD4x::forceRecalibration(uint16_t concentration) {
  uint16_t correction = 0;

  _lastError = SCD4x_ERROR_NONE;

  if (_measuring) {                                                             // Only available is not measuring
    _lastError = SCD4x_ERROR_MEASURING;
    return false;
  }

  if (!_writeCommand(SCD4x_PERFORM_FORCED_CALIBRATION)) return false;           // Initiate the forced calibration
  delay(400);                                                                   // Wait for calibration to be executed

  // --- Get the correction value ----
  _wire->requestFrom((uint8_t) _i2cAddress, (uint8_t) 3);                       // Request the result from the sensor
  correction = ((uint16_t) _wire->read()) << 8;                                 // MSB
  correction |= _wire->read();                                                  // LSB
  if (_crc8((uint8_t *) &correction, 2) != _wire->read()) {                     // Check CRC
    _lastError = SCD4x_ERROR_CRC;
    return false;
  }

  return (correction == 0xFFFF) ? correction : correction - 0x8000;             // Return correction or 0xFFFF on error
}

/**
 * @brief Enable or Disable the Self Calibration of the sensor
 * 
 * @param state   - true = enable, false = disable
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool SCD4x::enableSelfCalibration(bool state) {
  _lastError = SCD4x_ERROR_NONE;

  if (_measuring) {                                                             // Only available is not measuring
    _lastError = SCD4x_ERROR_MEASURING;
    return false;
  }

  if (!_writeCommand(SCD4x_AUTO_SELF_CALIBRATION, (uint16_t) state))            // Write status to the sensor
    return false;
  
  delay(1);
  return true;
}

/**
 * @brief Read the self calibration status from the sensor
 *
 * @return true  - Self calibration enabled
 * @return false - Self calibration disabled (check lastError for errors!)
 */
bool SCD4x::selfCalibrationEnabled(void) {
  uint16_t state = false;

  _lastError = SCD4x_ERROR_NONE;

  if (_measuring) {                                                             // Only available is not measuring
    _lastError = SCD4x_ERROR_MEASURING;
    return false;
  }

  if (!_readData(SCD4x_AUTO_SELF_CALIBRATION_STATE, &state, 1)) return false;   // Read the status from the sensor
  return (bool) state;
}

/**
 * @brief Checks if the sensor is ready with the measurement
 * 
 * @return true   - Measurement data is available
 * @return false  - Sensor is still measuring
 */
bool SCD4x::dataReady() {
  uint16_t status = 0x00;

  if(!_readData(SCD4x_GET_DATA_READY_STATUS, &status, 2)) return false;         // Check if the sensor is done measuring
  return ((status & 0x07FF) == 0x0000) ? false : true;                          // Return the result
}

/**
 * @brief Retrieves the stored serial number of this sensor
 * 
 * @return char* - Pointer to the serial number
 */
char* SCD4x::getSerialNumber(void) {
  return (char*) serialNumber;
}

/**
 * @brief Powerdown a SCD41 sensor
 * 
 * @return true   - On succes
 * @return false  - On Failure (lastError will be set)
 */
bool SCD4x::powerdown(void) {
  _lastError = SCD4x_ERROR_NONE;

  if (_sensorType != SCD41) {                                                   // Only available in a SCD41 sensor
    _lastError = SCD4x_ERROR_NOT_SUPPORTED;
    return false;
  }

  if (_measuring) {                                                             // Only available when not measuring
    _lastError = SCD4x_ERROR_MEASURING;
    return false;
  }
  
  if (!_writeCommand(SCD4x_POWER_DOWN)) return false;                          // Write the command to the sensor

  delay(1);
  return true;
}

/**
 * @brief Wakes the SCD41 sensor, please discard the first reading after wakeup!
 * 
 * @return true  - On success
 * @return false - On failure (lastError will be set)
 */
bool SCD4x::wakeup(void) {
  _lastError = SCD4x_ERROR_NONE;

  if (_sensorType != SCD41) {                                                   // Only available in a SCD41 sensor
    _lastError = SCD4x_ERROR_NOT_SUPPORTED;
    return false;
  }

  if (!_writeCommand(SCD4x_WAKE_UP)) return false;                              // Write the command to the sensor

  delay(20);                                                                    // Wait for the sensor to complete
  return _getSerialNumber();                                                    // Read out the serial number to verify the sensor is alive
}

/**
 * @brief Instructs the sensor to perform a selftest
 * 
 * @return true   - Self test oké
 * @return false  - Self test failed (Check lastError for other errors)
 */
bool SCD4x::performSelfTest(void) {
  uint16_t result = 0;

  _lastError = SCD4x_ERROR_NONE;

  if (_measuring) {                                                             // Only available when not measuring
    _lastError = SCD4x_ERROR_MEASURING;
    return false;
  }

  if(!_readData(SCD4x_SELF_TEST, &result, 10000)) return false;                 // Initiate the selftest

  return (result == 0x0000);                                                    // All oké if sensor responds with 0x0000
}

/**
 * @brief Reinitialize the sensor
 * 
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool SCD4x::reinitialize(void) {
  _lastError = SCD4x_ERROR_NONE;

  if (!stopMeasuring()) return false;                                           // Stop measuring
  if (!_writeCommand(SCD4x_REINIT)) return false;                               // Issue the reinit command to the sensor

  delay(20);                                                                    // Wait for sensor to complete reinitialization
  return true;
}

/**
 * @brief Have the sensor perform a factory reset, clears EEPROM as well
 * 
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool SCD4x::factoryReset(void) {
  _lastError = SCD4x_ERROR_NONE;

  if (!_writeCommand(SCD4x_FACTORY_RESET)) return false;                        // Issue the command to the sensor

  delay(1200);                                                                  // Wait for the sensor to complete the factory reset
  return true;
}

/**
 * @brief Returns the _lastError value
 * 
 * @return uint8_t - The value of the last error that occured
 */
uint8_t SCD4x::getLastError() {
  return _lastError;
}

/**
 * @brief Gets the serial number from the sensor and stores it for later use
 * 
 * @return true   - On success
 * @return false  - On failure (_lastError will be set)
 */
bool SCD4x::_getSerialNumber(void) {
  uint8_t buffer[2];                                                            // Read buffer
  uint8_t crc = 0, digit = 0;
  _lastError = SCD4x_ERROR_NONE;                                                // Reset the last error state

  if (!stopMeasuring()) return false;                                           // Stop measuring

  if (!_writeCommand(SCD4x_GET_SERIAL_NUMBER)) return false;                    // Send the command to the sensor
  delay(1);                                                                     // Specified in the datasheet
  _wire->requestFrom((uint8_t) _i2cAddress, (uint8_t) 9);                       // Request 9 bytes from the sensor
  if (_wire->available()) {
    for (uint8_t word = 0; word < 3; word++) {
      buffer[0] = _wire->read();                                                // MSB
      buffer[1] = _wire->read();                                                // LSB
      crc = _wire->read();                                                      // CRC

      if ((_crc8((uint8_t *) &buffer, 2) == crc)) {
        serialNumber[digit++] = _hex2ascii(buffer[0] >> 4);                     // 0x#...
        serialNumber[digit++] = _hex2ascii(buffer[0] & 0x0F);                   // 0x.#..
        serialNumber[digit++] = _hex2ascii(buffer[1] >> 4);                     // 0x..#.
        serialNumber[digit++] = _hex2ascii(buffer[1] & 0x0F);                   // 0x...#
      } else {
        _lastError = SCD4x_ERROR_CRC;
        return false;
      }
    }
  } else {
    _lastError = SCD4x_ERROR_I2CREAD;
    return false;
  }

  return true;
}

/**
 * @brief Writes a command to the sensor
 * 
 * @param cmd     - The command to be written
 * @return true   - On succes
 * @return false  - On Failure (_lastError will be set)
 */
bool SCD4x::_writeCommand(uint16_t cmd) {
  _lastError = SCD4x_ERROR_NONE;                                                // Clear the last error status

  _wire->beginTransmission(_i2cAddress);                                        // Start the communication with the sensor
  _wire->write(cmd >> 8);                                                       // MSB value of the command
  _wire->write(cmd & 0xFF);                                                     // LSB value of the command
  
  // ---- Check if transmission was successfull ----
  if (_wire->endTransmission() != 0) {
    _lastError = SCD4x_ERROR_I2CWRITE;
    return false;
  }

  return true;
}

/**
 * @brief Writes a command with extra data to the sensor
 * 
 * @param cmd     - Command to be executed
 * @param data    - Data to be send
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool SCD4x::_writeCommand(uint16_t cmd, uint16_t data) {
  uint8_t payload[2] = { (uint8_t) (data >> 8), (uint8_t) (data & 0xFF) };

  _wire->beginTransmission(_i2cAddress);
  _wire->write(cmd >> 8);                                                        // MSB of the command
  _wire->write(cmd & 0xFF);                                                      // LSB of the command
  _wire->write(payload[0]);                                                      // MSB of the data
  _wire->write(payload[1]);                                                      // LSB of the data
  _wire->write(_crc8((uint8_t *) &payload, 2));                                  // CRC over the data

  // ---- Check if transmission was successfull ----
  if (_wire->endTransmission() != 0) {
    _lastError = SCD4x_ERROR_I2CWRITE;
    return false;
  }

  return true;
}

/**
 * @brief Sends a command to the sensor and reads the response
 * 
 * @param cmd     - Command to be executed
 * @param data    - uint16_t pointer where to store the data
 * @param msDelay - Time in ms it takes for the command to complete
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool SCD4x::_readData(uint16_t cmd, uint16_t *data, uint16_t msDelay) {
  uint8_t buffer[2] = { 0x00 }, crc = 0;

  _lastError = SCD4x_ERROR_NONE;                                                // Clear the last error status

  _wire->beginTransmission(_i2cAddress);                                        // Start the communication with the sensor
  _wire->write(cmd >> 8);                                                       // MSB value of the command
  _wire->write(cmd & 0xFF);                                                     // LSB value of the command
  if (_wire->endTransmission() != 0) {
    _lastError = SCD4x_ERROR_I2CWRITE;
    return false;
  }

  delay(msDelay);                                                               // Wait for the command to complete

  _wire->requestFrom((uint8_t) _i2cAddress, (uint8_t) 3);                       // Request 3 bytes from the sensor
  buffer[0] = _wire->read();                                                    // MSB
  buffer[1] = _wire->read();                                                    // LSB
  crc = _wire->read();                                                          // CRC
  if ((_crc8((uint8_t *) &buffer, 2) != crc)) {                                 // Check if CRC is valid
    _lastError = SCD4x_ERROR_CRC;
    return false;
  }
  
  *data = (uint16_t) buffer[0] << 8 | buffer[1];                                // Store the value
  return true;
}

/**
 * @brief Converts a Hex value into a readable ASCII equivalent
 * 
 * @param value     - The hex value to convert
 * @return uint8_t  - The resulting ascii value
 */
uint8_t SCD4x::_hex2ascii(uint8_t value) {
  return ((value <= 9) ? value + 0x30 : value + 0x41 - 10);
}

/**
 * @brief Calculate a checksum over the result from the sensor
 * 
 * @param ptr       - Pointer to the data buffer containing the result
 * @param size      - Number of bytes in the buffer
 * @return uint8_t  - The calculated CRC8 value for this buffer
 */
uint8_t SCD4x::_crc8(uint8_t *ptr, size_t size) {
  uint8_t crc = SCD4x_CRC_INIT;                                                           // CRC base value

  // ---- Calculate the CRC value  ----
  while (size--) {
    crc ^= *ptr++;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) {
        //crc <<= 1;
        //crc ^= SCD4x_CRC_POLYNOMINAL;
        crc = (crc << 1) ^ SCD4x_CRC_POLYNOMINAL;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}
