/**
 * @file BME280.cpp
 * @brief Production-ready BME280 environmental sensor implementation
 * @version 3.0.0
 * @date 2026-01-09
 * 
 * Complete implementation of BME280 driver with advanced features
 * including compensation algorithms, statistical analysis, and weather
 * prediction capabilities.
 * 
 * @author Agricultural Robotics Team
 * @copyright (c) 2026 Agricultural Robotics Inc.
 */

#include "BME280.h"
#include <math.h>

// ============================================================================
// CONSTRUCTOR & DESTRUCTOR
// ============================================================================

BME280::BME280(uint8_t address, TwoWire* wire) 
    : interface(BME280Interface::I2C),
      i2cAddress(address),
      wirePort(wire),
      spiPort(nullptr),
      csPin(0),
      initialized(false),
      chipID(0),
      lastError(ERROR_NONE),
      t_fine(0),
      tempOffset(BME280_TEMP_OFFSET),
      altitudeOffset(BME280_ALTITUDE_OFFSET),
      seaLevelPressure(BME280_PRESSURE_SEA_LEVEL),
      statisticsEnabled(false),
      bufferIndex(0),
      lastTrendUpdate(0),
      lastReadTime(0),
      measurementDelay(BME280_MEASUREMENT_DELAY)
{
    // Initialize configuration to defaults
    config.mode = BME280Mode::NORMAL;
    config.tempOversampling = BME280Oversampling::X16;
    config.pressOversampling = BME280Oversampling::X16;
    config.humidOversampling = BME280Oversampling::X16;
    config.filter = BME280Filter::COEFF_16;
    config.standbyTime = BME280Standby::MS_0_5;
    
    // Initialize statistics
    memset(&stats, 0, sizeof(stats));
    memset(&pressureTrend, 0, sizeof(pressureTrend));
    memset(dataBuffer, 0, sizeof(dataBuffer));
}

BME280::BME280(uint8_t csPin, SPIClass* spi)
    : interface(BME280Interface::SPI),
      i2cAddress(0),
      wirePort(nullptr),
      spiPort(spi),
      csPin(csPin),
      initialized(false),
      chipID(0),
      lastError(ERROR_NONE),
      t_fine(0),
      tempOffset(BME280_TEMP_OFFSET),
      altitudeOffset(BME280_ALTITUDE_OFFSET),
      seaLevelPressure(BME280_PRESSURE_SEA_LEVEL),
      statisticsEnabled(false),
      bufferIndex(0),
      lastTrendUpdate(0),
      lastReadTime(0),
      measurementDelay(BME280_MEASUREMENT_DELAY)
{
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    
    // Initialize same defaults as I2C
    config.mode = BME280Mode::NORMAL;
    config.tempOversampling = BME280Oversampling::X16;
    config.pressOversampling = BME280Oversampling::X16;
    config.humidOversampling = BME280Oversampling::X16;
    config.filter = BME280Filter::COEFF_16;
    config.standbyTime = BME280Standby::MS_0_5;
    
    memset(&stats, 0, sizeof(stats));
    memset(&pressureTrend, 0, sizeof(pressureTrend));
    memset(dataBuffer, 0, sizeof(dataBuffer));
}

BME280::~BME280() {
    if (initialized) {
        sleep();  // Put sensor to sleep before destroying
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool BME280::begin() {
    if (interface == BME280Interface::I2C) {
        wirePort->begin();
    } else {
        spiPort->begin();
    }
    
    delay(10);  // Wait for sensor to boot
    
    // Read and verify chip ID
    chipID = getChipID();
    
    if (chipID != BME280_CHIPID && chipID != BMP280_CHIPID) {
        setError(ERROR_WRONG_CHIP);
        return false;
    }
    
    // Perform soft reset
    if (!reset()) {
        setError(ERROR_COMM_FAILURE);
        return false;
    }
    
    delay(10);  // Wait for reset to complete
    
    // Read calibration data
    if (!readCalibrationData()) {
        setError(ERROR_COMM_FAILURE);
        return false;
    }
    
    // Configure sensor with default settings
    if (!configure(config)) {
        setError(ERROR_COMM_FAILURE);
        return false;
    }
    
    initialized = true;
    clearError();
    
    return true;
}

bool BME280::reset() {
    write8(BME280_REG_SOFTRESET, BME280_RESET_CMD);
    delay(10);
    
    // Wait for reset to complete
    unsigned long startTime = millis();
    while (millis() - startTime < BME280_TIMEOUT_MS) {
        uint8_t status = read8(BME280_REG_STATUS);
        if ((status & BME280_STATUS_IM_UPDATE) == 0) {
            return true;
        }
        delay(1);
    }
    
    return false;
}

bool BME280::configure(const BME280Config& newConfig) {
    // Put sensor in sleep mode before changing configuration
    write8(BME280_REG_CONTROL, 0x00);
    delay(5);
    
    // Configure humidity oversampling
    uint8_t humidCtrl = static_cast<uint8_t>(newConfig.humidOversampling);
    write8(BME280_REG_CONTROLHUMID, humidCtrl);
    
    // Configure filter and standby time
    uint8_t configReg = (static_cast<uint8_t>(newConfig.standbyTime) << 5) |
                        (static_cast<uint8_t>(newConfig.filter) << 2);
    write8(BME280_REG_CONFIG, configReg);
    
    // Configure temperature, pressure, and mode
    uint8_t controlReg = (static_cast<uint8_t>(newConfig.tempOversampling) << 5) |
                         (static_cast<uint8_t>(newConfig.pressOversampling) << 2) |
                         static_cast<uint8_t>(newConfig.mode);
    write8(BME280_REG_CONTROL, controlReg);
    
    config = newConfig;
    
    return true;
}

void BME280::setWeatherMonitoringSettings() {
    BME280Config weatherConfig;
    weatherConfig.mode = BME280Mode::FORCED;
    weatherConfig.tempOversampling = BME280Oversampling::X1;
    weatherConfig.pressOversampling = BME280Oversampling::X1;
    weatherConfig.humidOversampling = BME280Oversampling::X1;
    weatherConfig.filter = BME280Filter::OFF;
    weatherConfig.standbyTime = BME280Standby::MS_1000;
    
    configure(weatherConfig);
}

void BME280::setIndoorMonitoringSettings() {
    BME280Config indoorConfig;
    indoorConfig.mode = BME280Mode::NORMAL;
    indoorConfig.tempOversampling = BME280Oversampling::X2;
    indoorConfig.pressOversampling = BME280Oversampling::X16;
    indoorConfig.humidOversampling = BME280Oversampling::X1;
    indoorConfig.filter = BME280Filter::COEFF_16;
    indoorConfig.standbyTime = BME280Standby::MS_0_5;
    
    configure(indoorConfig);
}

void BME280::setHighSpeedSettings() {
    BME280Config highSpeedConfig;
    highSpeedConfig.mode = BME280Mode::NORMAL;
    highSpeedConfig.tempOversampling = BME280Oversampling::X1;
    highSpeedConfig.pressOversampling = BME280Oversampling::X2;
    highSpeedConfig.humidOversampling = BME280Oversampling::X1;
    highSpeedConfig.filter = BME280Filter::OFF;
    highSpeedConfig.standbyTime = BME280Standby::MS_0_5;
    
    configure(highSpeedConfig);
}

void BME280::setLowPowerSettings() {
    BME280Config lowPowerConfig;
    lowPowerConfig.mode = BME280Mode::FORCED;
    lowPowerConfig.tempOversampling = BME280Oversampling::X1;
    lowPowerConfig.pressOversampling = BME280Oversampling::X1;
    lowPowerConfig.humidOversampling = BME280Oversampling::X1;
    lowPowerConfig.filter = BME280Filter::OFF;
    lowPowerConfig.standbyTime = BME280Standby::MS_1000;
    
    configure(lowPowerConfig);
}

// ============================================================================
// MEASUREMENT FUNCTIONS
// ============================================================================

bool BME280::readAll(BME280Data& data) {
    if (!initialized) {
        setError(ERROR_NO_DEVICE);
        return false;
    }
    
    // Trigger measurement if in forced mode
    if (config.mode == BME280Mode::FORCED) {
        if (!triggerMeasurement()) {
            return false;
        }
    }
    
    // Read all data in burst mode
    uint8_t rawData[8];
    readBytes(BME280_REG_PRESSUREDATA, rawData, 8);
    
    // Extract raw ADC values
    int32_t adc_P = ((uint32_t)rawData[0] << 12) | ((uint32_t)rawData[1] << 4) | ((rawData[2] >> 4) & 0x0F);
    int32_t adc_T = ((uint32_t)rawData[3] << 12) | ((uint32_t)rawData[4] << 4) | ((rawData[5] >> 4) & 0x0F);
    int32_t adc_H = ((uint32_t)rawData[6] << 8) | rawData[7];
    
    // Compensate temperature (must be done first for t_fine)
    int32_t temp_comp = compensateTemperature(adc_T);
    data.temperature = (temp_comp / 100.0f) + tempOffset;
    
    // Compensate pressure
    uint32_t press_comp = compensatePressure(adc_P);
    data.pressure = press_comp / 25600.0f;  // Convert to hPa
    
    // Compensate humidity (only if BME280, not BMP280)
    if (chipID == BME280_CHIPID) {
        uint32_t humid_comp = compensateHumidity(adc_H);
        data.humidity = humid_comp / 1024.0f;
    } else {
        data.humidity = 0.0f;  // BMP280 doesn't have humidity sensor
    }
    
    // Calculate derived values
    data.dewPoint = calculateDewPoint(data.temperature, data.humidity);
    data.heatIndex = calculateHeatIndex(data.temperature, data.humidity);
    data.altitude = calculateAltitude(data.pressure, seaLevelPressure) + altitudeOffset;
    data.timestamp = millis();
    
    // Validate data
    if (!isDataValid(data)) {
        setError(ERROR_INVALID_DATA);
        return false;
    }
    
    // Update statistics
    if (statisticsEnabled) {
        updateStatistics(data);
    }
    
    lastReadTime = millis();
    clearError();
    
    return true;
}

float BME280::readTemperature() {
    BME280Data data;
    if (readAll(data)) {
        return data.temperature;
    }
    return NAN;
}

float BME280::readPressure() {
    BME280Data data;
    if (readAll(data)) {
        return data.pressure;
    }
    return NAN;
}

float BME280::readHumidity() {
    BME280Data data;
    if (readAll(data)) {
        return data.humidity;
    }
    return NAN;
}

bool BME280::triggerMeasurement() {
    if (config.mode != BME280Mode::FORCED) {
        return false;
    }
    
    // Set forced mode to trigger measurement
    uint8_t controlReg = (static_cast<uint8_t>(config.tempOversampling) << 5) |
                         (static_cast<uint8_t>(config.pressOversampling) << 2) |
                         static_cast<uint8_t>(BME280Mode::FORCED);
    
    write8(BME280_REG_CONTROL, controlReg);
    
    // Wait for measurement to complete
    delay(measurementDelay);
    
    unsigned long startTime = millis();
    while (isMeasuring()) {
        if (millis() - startTime > BME280_TIMEOUT_MS) {
            setError(ERROR_TIMEOUT);
            return false;
        }
        delay(1);
    }
    
    return true;
}

bool BME280::isMeasuring() {
    uint8_t status = read8(BME280_REG_STATUS);
    return (status & BME280_STATUS_MEASURING) != 0;
}

// ============================================================================
// DERIVED CALCULATIONS
// ============================================================================

float BME280::calculateDewPoint(float temperature, float humidity) {
    if (humidity <= 0.0f || humidity > 100.0f) {
        return NAN;
    }
    
    // Magnus-Tetens approximation
    float a = 17.271f;
    float b = 237.7f;
    
    float gamma = ((a * temperature) / (b + temperature)) + logf(humidity / 100.0f);
    float dewPoint = (b * gamma) / (a - gamma);
    
    return dewPoint;
}

float BME280::calculateHeatIndex(float temperature, float humidity) {
    // Convert to Fahrenheit for calculation
    float T = temperature * 9.0f / 5.0f + 32.0f;
    float RH = humidity;
    
    // Steadman's formula
    float HI = -42.379f + 
               2.04901523f * T + 
               10.14333127f * RH - 
               0.22475541f * T * RH - 
               0.00683783f * T * T - 
               0.05481717f * RH * RH + 
               0.00122874f * T * T * RH + 
               0.00085282f * T * RH * RH - 
               0.00000199f * T * T * RH * RH;
    
    // Convert back to Celsius
    float heatIndexC = (HI - 32.0f) * 5.0f / 9.0f;
    
    return heatIndexC;
}

float BME280::calculateAltitude(float pressure, float seaLevelPressure) {
    if (pressure <= 0.0f || seaLevelPressure <= 0.0f) {
        return NAN;
    }
    
    // Barometric formula
    float altitude = 44330.0f * (1.0f - powf(pressure / seaLevelPressure, 0.1903f));
    
    return altitude;
}

float BME280::calculateSeaLevelPressure(float pressure, float altitude) {
    if (pressure <= 0.0f) {
        return NAN;
    }
    
    // Reverse barometric formula
    float seaLevel = pressure / powf(1.0f - (altitude / 44330.0f), 5.255f);
    
    return seaLevel;
}

float BME280::calculateAbsoluteHumidity(float temperature, float relativeHumidity) {
    // Absolute humidity in g/m³
    // Using Magnus formula for saturation vapor pressure
    
    float T = temperature + 273.15f;  // Convert to Kelvin
    
    // Saturation vapor pressure (hPa)
    float es = 6.112f * expf((17.67f * temperature) / (temperature + 243.5f));
    
    // Actual vapor pressure
    float e = (relativeHumidity / 100.0f) * es;
    
    // Absolute humidity
    float AH = (2.16679f * e) / T;
    
    return AH;
}

// ============================================================================
// CALIBRATION & COMPENSATION
// ============================================================================

void BME280::setTemperatureOffset(float offset) {
    tempOffset = offset;
}

void BME280::setAltitudeOffset(float offset) {
    altitudeOffset = offset;
}

void BME280::setSeaLevelPressure(float pressure) {
    seaLevelPressure = pressure;
}

// ============================================================================
// STATISTICS & ANALYSIS
// ============================================================================

void BME280::enableStatistics(bool enable) {
    statisticsEnabled = enable;
    
    if (enable) {
        resetStatistics();
    }
}

BME280Statistics BME280::getStatistics() const {
    return stats;
}

void BME280::resetStatistics() {
    stats.tempMin = 999.0f;
    stats.tempMax = -999.0f;
    stats.tempAvg = 0.0f;
    stats.pressureMin = 9999.0f;
    stats.pressureMax = 0.0f;
    stats.pressureAvg = 0.0f;
    stats.humidityMin = 100.0f;
    stats.humidityMax = 0.0f;
    stats.humidityAvg = 0.0f;
    stats.sampleCount = 0;
    
    bufferIndex = 0;
    memset(dataBuffer, 0, sizeof(dataBuffer));
}

PressureTrend BME280::getPressureTrend() const {
    return pressureTrend;
}

void BME280::updatePressureTrend() {
    unsigned long currentTime = millis();
    
    // Update trend every hour
    if (currentTime - lastTrendUpdate < 3600000) {
        return;
    }
    
    float currentPressure = readPressure();
    
    if (isnan(currentPressure)) {
        return;
    }
    
    // Shift historical data
    pressureTrend.hour_6 = pressureTrend.hour_3;
    pressureTrend.hour_3 = pressureTrend.hour_1;
    pressureTrend.hour_1 = pressureTrend.current;
    pressureTrend.current = currentPressure;
    
    // Calculate rate of change
    if (pressureTrend.hour_3 > 0) {
        pressureTrend.rate = (pressureTrend.current - pressureTrend.hour_3) / 3.0f;
    }
    
    // Predict weather
    pressureTrend.forecast = predictWeather(pressureTrend.rate);
    
    lastTrendUpdate = currentTime;
}

WeatherForecast BME280::getWeatherForecast() const {
    return pressureTrend.forecast;
}

// ============================================================================
// POWER MANAGEMENT
// ============================================================================

bool BME280::setMode(BME280Mode mode) {
    uint8_t controlReg = read8(BME280_REG_CONTROL);
    controlReg = (controlReg & 0xFC) | static_cast<uint8_t>(mode);
    write8(BME280_REG_CONTROL, controlReg);
    
    config.mode = mode;
    
    return true;
}

BME280Mode BME280::getMode() {
    uint8_t controlReg = read8(BME280_REG_CONTROL);
    return static_cast<BME280Mode>(controlReg & 0x03);
}

bool BME280::sleep() {
    return setMode(BME280Mode::SLEEP);
}

bool BME280::wake() {
    return setMode(config.mode);
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

bool BME280::isConnected() {
    uint8_t id = getChipID();
    return (id == BME280_CHIPID || id == BMP280_CHIPID);
}

uint8_t BME280::getChipID() {
    return read8(BME280_REG_CHIPID);
}

bool BME280::selfTest() {
    // Verify chip ID
    if (!isConnected()) {
        return false;
    }
    
    // Read test measurement
    BME280Data testData;
    if (!readAll(testData)) {
        return false;
    }
    
    // Validate test data
    if (!isDataValid(testData)) {
        return false;
    }
    
    // Test reset
    if (!reset()) {
        return false;
    }
    
    // Reconfigure after reset
    if (!configure(config)) {
        return false;
    }
    
    return true;
}

const char* BME280::getErrorDescription(uint8_t errorCode) {
    switch (errorCode) {
        case 0x00: return "No error";
        case 0x01: return "Device not found";
        case 0x02: return "Wrong chip ID";
        case 0x03: return "Communication timeout";
        case 0x04: return "Invalid data";
        case 0x05: return "Communication failure";
        default: return "Unknown error";
    }
}

bool BME280::isDataValid(const BME280Data& data) {
    // Check temperature range
    if (data.temperature < -40.0f || data.temperature > 85.0f) {
        return false;
    }
    
    // Check pressure range
    if (data.pressure < 300.0f || data.pressure > 1100.0f) {
        return false;
    }
    
    // Check humidity range (only for BME280)
    if (data.humidity < 0.0f || data.humidity > 100.0f) {
        return false;
    }
    
    return true;
}

// ============================================================================
// PRIVATE METHODS - HARDWARE INTERFACE
// ============================================================================

uint8_t BME280::read8(uint8_t reg) {
    if (interface == BME280Interface::I2C) {
        wirePort->beginTransmission(i2cAddress);
        wirePort->write(reg);
        wirePort->endTransmission();
        
        wirePort->requestFrom(i2cAddress, (uint8_t)1);
        return wirePort->read();
    } else {
        digitalWrite(csPin, LOW);
        spiPort->transfer(reg | 0x80);  // Set read bit
        uint8_t value = spiPort->transfer(0x00);
        digitalWrite(csPin, HIGH);
        return value;
    }
}

uint16_t BME280::read16(uint8_t reg) {
    uint16_t value = read8(reg);
    value <<= 8;
    value |= read8(reg + 1);
    return value;
}

uint16_t BME280::read16_LE(uint8_t reg) {
    uint16_t value = read8(reg);
    value |= ((uint16_t)read8(reg + 1) << 8);
    return value;
}

int16_t BME280::readS16(uint8_t reg) {
    return (int16_t)read16(reg);
}

int16_t BME280::readS16_LE(uint8_t reg) {
    return (int16_t)read16_LE(reg);
}

uint32_t BME280::read24(uint8_t reg) {
    uint32_t value = read8(reg);
    value <<= 8;
    value |= read8(reg + 1);
    value <<= 8;
    value |= read8(reg + 2);
    return value;
}

void BME280::write8(uint8_t reg, uint8_t value) {
    if (interface == BME280Interface::I2C) {
        wirePort->beginTransmission(i2cAddress);
        wirePort->write(reg);
        wirePort->write(value);
        wirePort->endTransmission();
    } else {
        digitalWrite(csPin, LOW);
        spiPort->transfer(reg & 0x7F);  // Clear read bit
        spiPort->transfer(value);
        digitalWrite(csPin, HIGH);
    }
}

void BME280::readBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
    if (interface == BME280Interface::I2C) {
        wirePort->beginTransmission(i2cAddress);
        wirePort->write(reg);
        wirePort->endTransmission();
        
        wirePort->requestFrom(i2cAddress, length);
        for (uint8_t i = 0; i < length; i++) {
            buffer[i] = wirePort->read();
        }
    } else {
        digitalWrite(csPin, LOW);
        spiPort->transfer(reg | 0x80);
        for (uint8_t i = 0; i < length; i++) {
            buffer[i] = spiPort->transfer(0x00);
        }
        digitalWrite(csPin, HIGH);
    }
}

// ============================================================================
// PRIVATE METHODS - CALIBRATION
// ============================================================================

bool BME280::readCalibrationData() {
    // Read temperature and pressure calibration
    calib.dig_T1 = read16_LE(BME280_REG_DIG_T1);
    calib.dig_T2 = readS16_LE(BME280_REG_DIG_T2);
    calib.dig_T3 = readS16_LE(BME280_REG_DIG_T3);
    
    calib.dig_P1 = read16_LE(BME280_REG_DIG_P1);
    calib.dig_P2 = readS16_LE(BME280_REG_DIG_P2);
    calib.dig_P3 = readS16_LE(BME280_REG_DIG_P3);
    calib.dig_P4 = readS16_LE(BME280_REG_DIG_P4);
    calib.dig_P5 = readS16_LE(BME280_REG_DIG_P5);
    calib.dig_P6 = readS16_LE(BME280_REG_DIG_P6);
    calib.dig_P7 = readS16_LE(BME280_REG_DIG_P7);
    calib.dig_P8 = readS16_LE(BME280_REG_DIG_P8);
    calib.dig_P9 = readS16_LE(BME280_REG_DIG_P9);
    
    // Read humidity calibration (only for BME280)
    if (chipID == BME280_CHIPID) {
        calib.dig_H1 = read8(BME280_REG_DIG_H1);
        calib.dig_H2 = readS16_LE(BME280_REG_DIG_H2);
        calib.dig_H3 = read8(BME280_REG_DIG_H3);
        
        uint8_t e4 = read8(BME280_REG_DIG_H4);
        uint8_t e5 = read8(BME280_REG_DIG_H5);
        uint8_t e6 = read8(BME280_REG_DIG_H5 + 1);
        
        calib.dig_H4 = (int16_t)((e4 << 4) | (e5 & 0x0F));
        calib.dig_H5 = (int16_t)((e6 << 4) | (e5 >> 4));
        calib.dig_H6 = (int8_t)read8(BME280_REG_DIG_H6);
    }
    
    return true;
}

// ============================================================================
// PRIVATE METHODS - COMPENSATION ALGORITHMS
// ============================================================================

int32_t BME280::compensateTemperature(int32_t adc_T) {
    // Bosch compensation algorithm
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * 
                    ((int32_t)calib.dig_T2)) >> 11;
    
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * 
                      ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
                    ((int32_t)calib.dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    
    int32_t T = (t_fine * 5 + 128) >> 8;
    
    return T;
}

uint32_t BME280::compensatePressure(int32_t adc_P) {
    // Bosch compensation algorithm (64-bit)
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0;  // Avoid divide by zero
    }
    
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    
    return (uint32_t)p;
}

uint32_t BME280::compensateHumidity(int32_t adc_H) {
    // Bosch compensation algorithm
    int32_t v_x1_u32r = (t_fine - ((int32_t)76800));
    
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) - 
                    (((int32_t)calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * 
                 (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) * 
                      (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + 
                    ((int32_t)2097152)) * ((int32_t)calib.dig_H2) + 8192) >> 14));
    
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * 
                                ((int32_t)calib.dig_H1)) >> 4));
    
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    
    return (uint32_t)(v_x1_u32r >> 12);
}

// ============================================================================
// PRIVATE METHODS - STATISTICS
// ============================================================================

void BME280::updateStatistics(const BME280Data& data) {
    // Update min/max
    if (data.temperature < stats.tempMin) {
        stats.tempMin = data.temperature;
    }
    if (data.temperature > stats.tempMax) {
        stats.tempMax = data.temperature;
    }
    
    if (data.pressure < stats.pressureMin) {
        stats.pressureMin = data.pressure;
    }
    if (data.pressure > stats.pressureMax) {
        stats.pressureMax = data.pressure;
    }
    
    if (data.humidity < stats.humidityMin) {
        stats.humidityMin = data.humidity;
    }
    if (data.humidity > stats.humidityMax) {
        stats.humidityMax = data.humidity;
    }
    
    // Store in circular buffer
    dataBuffer[bufferIndex][0] = data.temperature;
    dataBuffer[bufferIndex][1] = data.pressure;
    dataBuffer[bufferIndex][2] = data.humidity;
    
    bufferIndex = (bufferIndex + 1) % BME280_DATA_BUFFER_SIZE;
    
    stats.sampleCount++;
    
    // Calculate averages
    calculateStatistics();
}

void BME280::calculateStatistics() {
    uint16_t validSamples = min(stats.sampleCount, (uint32_t)BME280_DATA_BUFFER_SIZE);
    
    if (validSamples == 0) {
        return;
    }
    
    float tempSum = 0.0f;
    float pressSum = 0.0f;
    float humidSum = 0.0f;
    
    for (uint16_t i = 0; i < validSamples; i++) {
        tempSum += dataBuffer[i][0];
        pressSum += dataBuffer[i][1];
        humidSum += dataBuffer[i][2];
    }
    
    stats.tempAvg = tempSum / validSamples;
    stats.pressureAvg = pressSum / validSamples;
    stats.humidityAvg = humidSum / validSamples;
}

// ============================================================================
// PRIVATE METHODS - WEATHER PREDICTION
// ============================================================================

WeatherForecast BME280::predictWeather(float pressureRate) {
    // Weather forecasting based on pressure trend
    
    if (pressureTrend.hour_3 == 0) {
        return WeatherForecast::UNKNOWN;  // Insufficient data
    }
    
    // Rapid pressure drop
    if (pressureRate < -2.0f) {
        return WeatherForecast::STORM_WARNING;
    }
    
    // Moderate pressure drop
    if (pressureRate < -0.5f) {
        return WeatherForecast::DETERIORATING;
    }
    
    // Slow pressure drop
    if (pressureRate < -0.2f) {
        return WeatherForecast::RAIN_LIKELY;
    }
    
    // Pressure rising
    if (pressureRate > 0.5f) {
        return WeatherForecast::IMPROVING;
    }
    
    // Stable pressure
    return WeatherForecast::STABLE;
}

// ============================================================================
// PRIVATE METHODS - VALIDATION
// ============================================================================

bool BME280::validateTemperature(float temp) {
    return (temp >= -40.0f && temp <= 85.0f);
}

bool BME280::validatePressure(float press) {
    return (press >= 300.0f && press <= 1100.0f);
}

bool BME280::validateHumidity(float humid) {
    return (humid >= 0.0f && humid <= 100.0f);
}

// ============================================================================
// PRIVATE METHODS - UTILITY
// ============================================================================

void BME280::setError(uint8_t error) {
    lastError = error;
}

void BME280::clearError() {
    lastError = ERROR_NONE;
}
