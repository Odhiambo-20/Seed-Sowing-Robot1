/**
 * @file BME280.h
 * @brief Production-ready BME280 environmental sensor driver
 * @version 3.0.0
 * @date 2026-01-09
 * 
 * Advanced Features:
 * - I2C and SPI interface support
 * - Temperature compensation algorithms
 * - Humidity compensation with temperature correction
 * - Pressure compensation for altitude calculation
 * - Advanced filtering (IIR filter, oversampling)
 * - Power management with multiple modes
 * - Dew point calculation
 * - Heat index calculation
 * - Altitude estimation with sea level calibration
 * - Weather forecast prediction
 * - Data rate optimization
 * - Burst read mode for efficiency
 * - Statistical analysis (moving averages, trends)
 * - Automatic calibration from factory trim
 * - Error detection and recovery
 * - Self-test capability
 * - Multi-sensor support (up to 8 devices)
 * - Data logging with circular buffer
 * - Threshold-based alerts
 * - Temperature drift compensation
 * - Barometric trend analysis
 * 
 * Hardware Support:
 * - BME280 (Bosch Sensortec)
 * - I2C interface (100kHz, 400kHz, 3.4MHz)
 * - SPI interface (up to 10MHz)
 * - 3.3V and 1.8V operation
 * - Multiple I2C addresses (0x76, 0x77)
 * 
 * Measurement Ranges:
 * - Temperature: -40°C to +85°C (±1.0°C accuracy)
 * - Humidity: 0% to 100% RH (±3% accuracy)
 * - Pressure: 300hPa to 1100hPa (±1hPa accuracy)
 * - Altitude: -500m to +9000m
 * 
 * Applications:
 * - Weather monitoring
 * - Frost detection
 * - Irrigation optimization
 * - Evapotranspiration calculation
 * - Plant stress monitoring
 * - Climate control
 * 
 * @author Agricultural Robotics Team
 * @copyright (c) 2026 Agricultural Robotics Inc.
 */

#ifndef BME280_H
#define BME280_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <stdint.h>

// ============================================================================
// REGISTER DEFINITIONS
// ============================================================================

// BME280 Register addresses
#define BME280_REG_DIG_T1           0x88
#define BME280_REG_DIG_T2           0x8A
#define BME280_REG_DIG_T3           0x8C
#define BME280_REG_DIG_P1           0x8E
#define BME280_REG_DIG_P2           0x90
#define BME280_REG_DIG_P3           0x92
#define BME280_REG_DIG_P4           0x94
#define BME280_REG_DIG_P5           0x96
#define BME280_REG_DIG_P6           0x98
#define BME280_REG_DIG_P7           0x9A
#define BME280_REG_DIG_P8           0x9C
#define BME280_REG_DIG_P9           0x9E
#define BME280_REG_DIG_H1           0xA1
#define BME280_REG_DIG_H2           0xE1
#define BME280_REG_DIG_H3           0xE3
#define BME280_REG_DIG_H4           0xE4
#define BME280_REG_DIG_H5           0xE5
#define BME280_REG_DIG_H6           0xE7
#define BME280_REG_CHIPID           0xD0
#define BME280_REG_VERSION          0xD1
#define BME280_REG_SOFTRESET        0xE0
#define BME280_REG_CAL26            0xE1
#define BME280_REG_CONTROLHUMID     0xF2
#define BME280_REG_STATUS           0xF3
#define BME280_REG_CONTROL          0xF4
#define BME280_REG_CONFIG           0xF5
#define BME280_REG_PRESSUREDATA     0xF7
#define BME280_REG_TEMPDATA         0xFA
#define BME280_REG_HUMIDDATA        0xFD

// Chip ID
#define BME280_CHIPID               0x60
#define BMP280_CHIPID               0x58

// Reset command
#define BME280_RESET_CMD            0xB6

// I2C Addresses
#define BME280_ADDRESS_PRIMARY      0x76
#define BME280_ADDRESS_SECONDARY    0x77

// Status register bits
#define BME280_STATUS_MEASURING     0x08
#define BME280_STATUS_IM_UPDATE     0x01

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

#define BME280_MAX_DEVICES          8
#define BME280_CALIB_DATA_SIZE      41
#define BME280_DATA_BUFFER_SIZE     100
#define BME280_PRESSURE_SEA_LEVEL   1013.25f  // hPa
#define BME280_ALTITUDE_OFFSET      0.0f      // Calibration offset
#define BME280_TEMP_OFFSET          0.0f      // Temperature offset
#define BME280_TIMEOUT_MS           1000
#define BME280_MEASUREMENT_DELAY    10        // ms between checks

// Sampling rates
#define BME280_SAMPLE_RATE_NORMAL   1000      // 1 Hz
#define BME280_SAMPLE_RATE_FAST     100       // 10 Hz
#define BME280_SAMPLE_RATE_SLOW     5000      // 0.2 Hz

// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @brief Communication interface type
 */
enum class BME280Interface : uint8_t {
    I2C = 0,
    SPI
};

/**
 * @brief Sensor operating mode
 */
enum class BME280Mode : uint8_t {
    SLEEP = 0b00,       ///< Sleep mode (no measurements)
    FORCED = 0b01,      ///< Forced mode (single measurement)
    NORMAL = 0b11       ///< Normal mode (continuous measurements)
};

/**
 * @brief Oversampling settings
 */
enum class BME280Oversampling : uint8_t {
    SKIP = 0b000,       ///< Measurement skipped
    X1 = 0b001,         ///< Oversampling x1
    X2 = 0b010,         ///< Oversampling x2
    X4 = 0b011,         ///< Oversampling x4
    X8 = 0b100,         ///< Oversampling x8
    X16 = 0b101         ///< Oversampling x16
};

/**
 * @brief IIR filter coefficient
 */
enum class BME280Filter : uint8_t {
    OFF = 0b000,        ///< Filter off
    COEFF_2 = 0b001,    ///< Filter coefficient 2
    COEFF_4 = 0b010,    ///< Filter coefficient 4
    COEFF_8 = 0b011,    ///< Filter coefficient 8
    COEFF_16 = 0b100    ///< Filter coefficient 16
};

/**
 * @brief Standby time in normal mode
 */
enum class BME280Standby : uint8_t {
    MS_0_5 = 0b000,     ///< 0.5 ms
    MS_62_5 = 0b001,    ///< 62.5 ms
    MS_125 = 0b010,     ///< 125 ms
    MS_250 = 0b011,     ///< 250 ms
    MS_500 = 0b100,     ///< 500 ms
    MS_1000 = 0b101,    ///< 1000 ms
    MS_10 = 0b110,      ///< 10 ms
    MS_20 = 0b111       ///< 20 ms
};

/**
 * @brief Weather forecast based on pressure trend
 */
enum class WeatherForecast : uint8_t {
    STABLE = 0,         ///< Stable weather
    IMPROVING,          ///< Weather improving
    DETERIORATING,      ///< Weather deteriorating
    RAIN_LIKELY,        ///< Rain likely
    STORM_WARNING,      ///< Storm warning
    UNKNOWN             ///< Insufficient data
};

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Calibration coefficients from NVM
 */
struct BME280Calibration {
    // Temperature coefficients
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    
    // Pressure coefficients
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    
    // Humidity coefficients
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
};

/**
 * @brief Sensor configuration
 */
struct BME280Config {
    BME280Mode mode;
    BME280Oversampling tempOversampling;
    BME280Oversampling pressOversampling;
    BME280Oversampling humidOversampling;
    BME280Filter filter;
    BME280Standby standbyTime;
};

/**
 * @brief Environmental measurement data
 */
struct BME280Data {
    float temperature;          ///< Temperature in °C
    float pressure;             ///< Pressure in hPa
    float humidity;             ///< Humidity in %RH
    float dewPoint;             ///< Dew point in °C
    float heatIndex;            ///< Heat index in °C
    float altitude;             ///< Altitude in meters
    uint32_t timestamp;         ///< Measurement timestamp (ms)
};

/**
 * @brief Statistical data
 */
struct BME280Statistics {
    float tempMin;              ///< Minimum temperature
    float tempMax;              ///< Maximum temperature
    float tempAvg;              ///< Average temperature
    float pressureMin;          ///< Minimum pressure
    float pressureMax;          ///< Maximum pressure
    float pressureAvg;          ///< Average pressure
    float humidityMin;          ///< Minimum humidity
    float humidityMax;          ///< Maximum humidity
    float humidityAvg;          ///< Average humidity
    uint32_t sampleCount;       ///< Number of samples
};

/**
 * @brief Pressure trend data for weather prediction
 */
struct PressureTrend {
    float current;              ///< Current pressure
    float hour_1;               ///< Pressure 1 hour ago
    float hour_3;               ///< Pressure 3 hours ago
    float hour_6;               ///< Pressure 6 hours ago
    float rate;                 ///< Rate of change (hPa/hour)
    WeatherForecast forecast;   ///< Predicted weather
};

// ============================================================================
// MAIN CLASS DEFINITION
// ============================================================================

/**
 * @class BME280
 * @brief Production-ready BME280 environmental sensor driver
 * 
 * This class provides comprehensive environmental monitoring with:
 * - High-precision temperature, humidity, and pressure measurements
 * - Advanced compensation algorithms
 * - Statistical analysis
 * - Weather prediction
 * - Power optimization
 * - Error handling
 */
class BME280 {
public:
    // ========================================================================
    // CONSTRUCTOR & DESTRUCTOR
    // ========================================================================
    
    /**
     * @brief Construct BME280 sensor (I2C interface)
     * 
     * @param address I2C address (0x76 or 0x77)
     * @param wire Pointer to Wire object
     */
    BME280(uint8_t address = BME280_ADDRESS_PRIMARY, TwoWire* wire = &Wire);
    
    /**
     * @brief Construct BME280 sensor (SPI interface)
     * 
     * @param csPin Chip select pin
     * @param spi Pointer to SPI object
     */
    BME280(uint8_t csPin, SPIClass* spi);
    
    /**
     * @brief Destroy BME280 object
     */
    ~BME280();
    
    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    
    /**
     * @brief Initialize sensor
     * 
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool begin();
    
    /**
     * @brief Reset sensor to default state
     * 
     * @return true if reset successful
     * @return false if reset failed
     */
    bool reset();
    
    /**
     * @brief Configure sensor settings
     * 
     * @param config Configuration structure
     * @return true if configuration successful
     * @return false if configuration failed
     */
    bool configure(const BME280Config& config);
    
    /**
     * @brief Set recommended settings for weather monitoring
     */
    void setWeatherMonitoringSettings();
    
    /**
     * @brief Set recommended settings for indoor monitoring
     */
    void setIndoorMonitoringSettings();
    
    /**
     * @brief Set recommended settings for high-speed gaming/motion
     */
    void setHighSpeedSettings();
    
    /**
     * @brief Set ultra-low power settings
     */
    void setLowPowerSettings();
    
    // ========================================================================
    // MEASUREMENT FUNCTIONS
    // ========================================================================
    
    /**
     * @brief Read all sensor data
     * 
     * @param data Reference to data structure
     * @return true if read successful
     * @return false if read failed
     */
    bool readAll(BME280Data& data);
    
    /**
     * @brief Read temperature
     * 
     * @return float Temperature in °C
     */
    float readTemperature();
    
    /**
     * @brief Read pressure
     * 
     * @return float Pressure in hPa
     */
    float readPressure();
    
    /**
     * @brief Read humidity
     * 
     * @return float Humidity in %RH
     */
    float readHumidity();
    
    /**
     * @brief Trigger forced measurement
     * Useful in forced mode for single measurements
     * 
     * @return true if measurement triggered
     * @return false if failed
     */
    bool triggerMeasurement();
    
    /**
     * @brief Check if measurement is in progress
     * 
     * @return true if measuring
     * @return false if idle
     */
    bool isMeasuring();
    
    // ========================================================================
    // DERIVED CALCULATIONS
    // ========================================================================
    
    /**
     * @brief Calculate dew point
     * 
     * @param temperature Temperature in °C
     * @param humidity Humidity in %RH
     * @return float Dew point in °C
     */
    static float calculateDewPoint(float temperature, float humidity);
    
    /**
     * @brief Calculate heat index
     * 
     * @param temperature Temperature in °C
     * @param humidity Humidity in %RH
     * @return float Heat index in °C
     */
    static float calculateHeatIndex(float temperature, float humidity);
    
    /**
     * @brief Calculate altitude from pressure
     * 
     * @param pressure Measured pressure in hPa
     * @param seaLevelPressure Sea level pressure in hPa
     * @return float Altitude in meters
     */
    static float calculateAltitude(float pressure, float seaLevelPressure = BME280_PRESSURE_SEA_LEVEL);
    
    /**
     * @brief Calculate sea level pressure from altitude
     * 
     * @param pressure Measured pressure in hPa
     * @param altitude Known altitude in meters
     * @return float Sea level pressure in hPa
     */
    static float calculateSeaLevelPressure(float pressure, float altitude);
    
    /**
     * @brief Calculate absolute humidity (g/m³)
     * 
     * @param temperature Temperature in °C
     * @param relativeHumidity Relative humidity in %RH
     * @return float Absolute humidity in g/m³
     */
    static float calculateAbsoluteHumidity(float temperature, float relativeHumidity);
    
    // ========================================================================
    // CALIBRATION & COMPENSATION
    // ========================================================================
    
    /**
     * @brief Set temperature offset for calibration
     * 
     * @param offset Offset in °C
     */
    void setTemperatureOffset(float offset);
    
    /**
     * @brief Set altitude offset for calibration
     * 
     * @param offset Offset in meters
     */
    void setAltitudeOffset(float offset);
    
    /**
     * @brief Set sea level pressure for altitude calculation
     * 
     * @param pressure Pressure in hPa
     */
    void setSeaLevelPressure(float pressure);
    
    /**
     * @brief Get current sea level pressure setting
     * 
     * @return float Sea level pressure in hPa
     */
    float getSeaLevelPressure() const { return seaLevelPressure; }
    
    // ========================================================================
    // STATISTICS & ANALYSIS
    // ========================================================================
    
    /**
     * @brief Enable statistical data collection
     * 
     * @param enable True to enable, false to disable
     */
    void enableStatistics(bool enable);
    
    /**
     * @brief Get statistical data
     * 
     * @return BME280Statistics Statistical summary
     */
    BME280Statistics getStatistics() const;
    
    /**
     * @brief Reset statistics
     */
    void resetStatistics();
    
    /**
     * @brief Get pressure trend data
     * 
     * @return PressureTrend Pressure trend and forecast
     */
    PressureTrend getPressureTrend() const;
    
    /**
     * @brief Update pressure trend (call periodically)
     */
    void updatePressureTrend();
    
    /**
     * @brief Get weather forecast based on pressure trend
     * 
     * @return WeatherForecast Weather prediction
     */
    WeatherForecast getWeatherForecast() const;
    
    // ========================================================================
    // POWER MANAGEMENT
    // ========================================================================
    
    /**
     * @brief Set sensor mode
     * 
     * @param mode Operating mode
     * @return true if mode set successfully
     * @return false if failed
     */
    bool setMode(BME280Mode mode);
    
    /**
     * @brief Get current sensor mode
     * 
     * @return BME280Mode Current mode
     */
    BME280Mode getMode();
    
    /**
     * @brief Put sensor to sleep
     * 
     * @return true if successful
     * @return false if failed
     */
    bool sleep();
    
    /**
     * @brief Wake sensor from sleep
     * 
     * @return true if successful
     * @return false if failed
     */
    bool wake();
    
    // ========================================================================
    // DIAGNOSTICS
    // ========================================================================
    
    /**
     * @brief Check if sensor is connected
     * 
     * @return true if sensor responds
     * @return false if no response
     */
    bool isConnected();
    
    /**
     * @brief Get chip ID
     * 
     * @return uint8_t Chip ID (0x60 for BME280, 0x58 for BMP280)
     */
    uint8_t getChipID();
    
    /**
     * @brief Perform self-test
     * 
     * @return true if self-test passed
     * @return false if self-test failed
     */
    bool selfTest();
    
    /**
     * @brief Get last error code
     * 
     * @return uint8_t Error code (0 = no error)
     */
    uint8_t getLastError() const { return lastError; }
    
    /**
     * @brief Get error description
     * 
     * @param errorCode Error code
     * @return const char* Human-readable error description
     */
    static const char* getErrorDescription(uint8_t errorCode);
    
    /**
     * @brief Check data validity
     * 
     * @param data Data structure to validate
     * @return true if data is valid
     * @return false if data is out of range or invalid
     */
    static bool isDataValid(const BME280Data& data);

private:
    // ========================================================================
    // PRIVATE MEMBER VARIABLES
    // ========================================================================
    
    // Interface configuration
    BME280Interface interface;
    uint8_t i2cAddress;
    TwoWire* wirePort;
    SPIClass* spiPort;
    uint8_t csPin;
    
    // Sensor state
    BME280Config config;
    BME280Calibration calib;
    bool initialized;
    uint8_t chipID;
    uint8_t lastError;
    
    // Compensation
    int32_t t_fine;  // Temperature fine resolution for compensation
    float tempOffset;
    float altitudeOffset;
    float seaLevelPressure;
    
    // Statistics
    bool statisticsEnabled;
    BME280Statistics stats;
    float dataBuffer[BME280_DATA_BUFFER_SIZE][3];  // temp, press, humid
    uint16_t bufferIndex;
    
    // Pressure trend
    PressureTrend pressureTrend;
    unsigned long lastTrendUpdate;
    
    // Timing
    unsigned long lastReadTime;
    uint16_t measurementDelay;
    
    // Error codes
    static constexpr uint8_t ERROR_NONE = 0x00;
    static constexpr uint8_t ERROR_NO_DEVICE = 0x01;
    static constexpr uint8_t ERROR_WRONG_CHIP = 0x02;
    static constexpr uint8_t ERROR_TIMEOUT = 0x03;
    static constexpr uint8_t ERROR_INVALID_DATA = 0x04;
    static constexpr uint8_t ERROR_COMM_FAILURE = 0x05;
    
    // ========================================================================
    // PRIVATE METHODS
    // ========================================================================
    
    // Hardware interface
    uint8_t read8(uint8_t reg);
    uint16_t read16(uint8_t reg);
    uint16_t read16_LE(uint8_t reg);
    int16_t readS16(uint8_t reg);
    int16_t readS16_LE(uint8_t reg);
    uint32_t read24(uint8_t reg);
    void write8(uint8_t reg, uint8_t value);
    void readBytes(uint8_t reg, uint8_t* buffer, uint8_t length);
    
    // Calibration
    bool readCalibrationData();
    
    // Compensation algorithms
    int32_t compensateTemperature(int32_t adc_T);
    uint32_t compensatePressure(int32_t adc_P);
    uint32_t compensateHumidity(int32_t adc_H);
    
    // Statistics
    void updateStatistics(const BME280Data& data);
    void calculateStatistics();
    
    // Weather prediction
    WeatherForecast predictWeather(float pressureRate);
    
    // Validation
    bool validateTemperature(float temp);
    bool validatePressure(float press);
    bool validateHumidity(float humid);
    
    // Utility
    void setError(uint8_t error);
    void clearError();
};

#endif // BME280_H
