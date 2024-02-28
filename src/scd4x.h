#ifndef __SENSENET_SCD4x__
#define __SENSENET_SCD4x__

#include <Arduino.h>
#include <Wire.h>

// ---- Sensor Settings ----
#define SCD4x_I2C_ADDRESS                       0x62                            // Address of this sensor
#define SCD4x_MEASURETIME                        5E3                            // Time in ms to wait for a measurement to finish (5 sec.)
#define SCD4x_MEASURETIME_LP                     3E5                            // Time in ms to wait for a measurment to finsih in LP mode (30 sec.)
#define SCD4x_CRC_INIT                          0xFF                            // Init value for the CRC check
#define SCD4x_CRC_POLYNOMINAL                   0x31                            // Seed for the CRC polynominal
#define SCD4x_INIT_DELAY                        1000                            // Sensor needs 1 sec. init time
#define SCD4x_CMD_SIZE                             2                            // Commands consist of 2 bytes

// ---- SCD4x SensorType ----
#define SCD40                                     40                            // Sensor is a SCD40 (default)
#define SCD41                                     41                            // Sensor is a SCD41 with Single Measurement feature

// ---- SCD4x Commands ----
#define SCD4x_START_MEASURING                 0x21B1                            // Start periodic measurement, signal update interval is 5 seconds
#define SCD4x_READ_MEASUREMENT                0xEC05                            // Read sensor output
#define SCD4x_STOP_MEASURING                  0x3F86                            // Stop periodic measurement to change the sensor configuration or to save power
#define SCD4x_SET_TEMPERATURE_OFFSET          0x241D                            // Set the offset for the temperature measurement
#define SCD4x_GET_TEMPERATURE_OFFSET          0x2318                            // Get the current offset for the temperature measurement
#define SCD4x_SET_SENSOR_ALTITUDE             0x2427                            // Writing of the sensor altitude must be done while the SCD4x is in idle mode
#define SCD4x_GET_SENSOR_ALTITUDE             0x2322                            // Reading of the sensor altitude must be done while the SCD4x is in idle mode
#define SCD4x_SET_AMBIENT_PRESSURE            0xE000                            // Can be sent during periodic measurements to enable continuous pressure compensation
#define SCD4x_PERFORM_FORCED_CALIBRATION      0x362F                            // Force re-calibration of the sensor
#define SCD4x_AUTO_SELF_CALIBRATION           0x2416                            // Set the current state of the automatic self-calibration. (Enabled by default)
#define SCD4x_AUTO_SELF_CALIBRATION_STATE     0x2313                            // Check the state of the automatic self-calibration.
#define SCD4x_START_MEASURING_LP              0x21AC                            // Start low power periodic measurement, signal update interval is approximately 30 seconds
#define SCD4x_GET_DATA_READY_STATUS           0xE4B8                            // Data ready if 11 Least Significant Bits are all 0
#define SCD4x_PERSIST_SETTINGS                0x3615                            // Store settings into EEprom
#define SCD4x_GET_SERIAL_NUMBER               0x3682                            // Retrieve the 6 byte serial number
#define SCD4x_SELF_TEST                       0x3639                            // Initiate a sensor self test
#define SCD4x_FACTORY_RESET                   0x3632                            // Resets all configuration settings stored in the EEPROM and erases the FRC and ASC algorithm history.
#define SCD4x_REINIT                          0x3646                            // Re-Initialise the sensor
#define SCD4x_SINGLE_MEASUREMENT              0x219D                            // On-demand measurement of CO2 concentration, relative humidity and temperature
#define SCD4x_SINGLE_MEASUREMENT_RHT          0x2196                            // On-demand measurement of relative humidity and temperature only
#define SCD4x_POWER_DOWN                      0x36E0                            // Put the sensor from idle to sleep to reduce current consumption
#define SCD4x_WAKE_UP                         0x36F6                            // Wake up the sensor from sleep mode into idle mode

// ---- Temperature Scales ----
#define _CELCIUS                                0x01                            // Use Celcius scale for temperature values
#define _FAHRENHEIT                             0x02                            // Use Farenheit scale for temperature values
#define _KELVIN                                 0x03                            // Use Kelvin scale for temperature valuess

// ---- SCD4x Error States ----
#define SCD4x_ERROR_NONE                        0x00                            // No error
#define SCD4x_ERROR_NOSENSOR                    0x01                            // Sensor not found
#define SCD4x_ERROR_CRC                         0x02                            // CRC error
#define SCD4x_ERROR_WAIT                        0x03                            // Measurement not ready
#define SCD4x_ERROR_I2CWRITE                    0x04                            // I2C Write error
#define SCD4x_ERROR_I2CREAD                     0x05                            // I2C Read error
#define SCD4x_ERROR_VALUE_OOB                   0x06                            // Offset value is out of bounds
#define SCD4x_ERROR_NOT_SUPPORTED               0x07                            // Function is not supported by this sensor
#define SCD4x_ERROR_MEASURING                   0x08                            // Function not supported while sensor is measuring
#define SCD4x_ERROR_VALUE_INVALID               0x09                            // The value is invalid, perform a measurement first

class SCD4x {
  public:
    SCD4x(uint8_t sensorType = SCD40, TwoWire *wire = &Wire);                   // Constructor of the SCD4x class
    #if defined (ESP8266) || defined (ESP32)
      bool begin(uint8_t i2cAddress = SCD4x_I2C_ADDRESS, uint8_t sda = 0xff, uint8_t scl = 0xff);
    #else
      bool begin(uint8_t i2cAddress = SCD4x_I2C_ADDRESS);                       // Starts the sensor
    #endif

    // ---- Measurement functions ----
    bool startMeasuring(bool lowPower = false);                                 // Start measurements (5 sec. interval)
    bool stopMeasuring(void);                                                   // Stop measuring
    bool singleMeasurement(void);                                               // Perform a single measurement (SCD41 only)
    bool singleMeasurement_rht(void);                                           // Measure relative humidity and temperature once (SCD41 only)
    bool readSensorData(void);                                                  // Read the data from the sensor (and clears them if requested)
    uint16_t getCO2(void);                                                      // Returns the CO2 value from the last measurement
    float getTemperature(uint8_t scale = _CELCIUS);                             // Returns the Temperature from the last measurement in the selected scale
    float getHumidity(void);                                                    // Returns the Humidity from the last measurement

    // ---- Sensor settings -----
    bool setTempOffset(float toff);                                             // Set the temperature offset
    float getTempOffset(void);                                                  // Get the current temperature offset
    bool setSensorAltitude(uint16_t alt);                                       // Set the altitude (meters above sea level)
    uint16_t getSensorAltitude(void);                                           // Gets the altitude from the sensor
    bool setAmbientPressure(uint16_t mbar);                                     // Sets the ambient pressure in mbar (overrides sensor alititude)
    bool persistSettings(void);                                                 // Make the current settings persistent

    // ---- Calibration settings ----
    int16_t forceRecalibration(uint16_t concentration);                         // Force a recalibration of the sensor returns the correction value
    bool enableSelfCalibration(bool state);                                     // Enable self calibration for this sensor
    bool selfCalibrationEnabled(void);                                          // Returns the current self calibration state

    // ---- Software calibration ----
    bool tempCalibration(uint8_t tLow, float mtLow,                             // Set the two point lineair calibration for this sensor
                         uint8_t tHigh, float mtHigh);
    bool humidCalibration(uint8_t hLow, float mhLow,
                          uint8_t hHigh, float mhHigh);

    // ---- Utility functions ----
    bool dataReady(void);                                                       // Check if the measurement is done
    char* getSerialNumber(void);                                                // Retrieves the stored sensor serial number
    bool powerdown(void);                                                       // PowerDown the sensor
    bool wakeup(void);                                                          // PowerUp the sensor
    bool performSelfTest(void);                                                 // Have the sensor perform a self test
    bool reinitialize(void);                                                    // Re Initalize the sensor
    bool factoryReset(void);                                                    // Factory Reset the sensor, cleares EEPROM also!
    uint8_t getLastError(void);                                                 // Returns the last error encountered

  private:
    TwoWire *_wire = nullptr;                                                   // Handle to I2C class
    uint8_t _i2cAddress = SCD4x_I2C_ADDRESS;                                    // i2c Address for this sensor
    uint8_t _sensorType = SCD40;                                                // Sensor type, defaults to SCD4x
    uint8_t _lastError = SCD4x_ERROR_NONE;                                      // Clear the lastError status
    uint16_t _co2 = 0;                                                          // Variable for CO2
    float _temperature = 0.0, _humidity = 0.0;                                  // Variables for Temperature and Humidity
    bool _co2_valid = false, _temp_valid = false, _humid_valid = false;         // Are the stored values valid or not
    uint8_t tLow = 0, tHigh = 0;                                                // Temperatures to 
    bool _measuring = false;                                                    // Status flag for measuring
    uint8_t serialNumber[13] = { 0x00 };                                        // Sensors serial number

    bool _getSerialNumber(void);                                                // Reads the sensors serial number
    bool _writeCommand(uint16_t cmd);                                           // Writes a command to the sensor
    bool _writeCommand(uint16_t cmd, uint16_t data);                            // Writes a command with data to the sensor
    bool _readData(uint16_t cmd, uint16_t *data, uint16_t msDelay);             // Reads the result from the sensor

    uint8_t _hex2ascii(uint8_t value);                                          // Convert a hex value to printable ascii
    uint8_t _crc8(uint8_t *ptr, size_t size);                                   // Calculate the CRC
};

#endif // __SENSENET_SCD4x__