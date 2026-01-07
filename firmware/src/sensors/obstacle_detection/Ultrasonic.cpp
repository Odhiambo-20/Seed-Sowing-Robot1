/**
 * @file Ultrasonic.cpp
 * @brief Production-ready Ultrasonic Sensor Array implementation
 * @version 2.0.0
 * @date 2026-01-06
 * 
 * COMPLETE IMPLEMENTATION - NO TODO STATEMENTS
 * READY FOR PRODUCTION TESTING ON ARDUINO/ESP32
 */

#include "sensors/obstacle_detection/Ultrasonic.h"

// ============================================================================
// CONSTRUCTOR & DESTRUCTOR
// ============================================================================

Ultrasonic::Ultrasonic(uint8_t numSensors)
    : _numSensors(constrain(numSensors, 1, ULTRASONIC_MAX_SENSORS)),
      _configuredSensors(0),
      _medianFilterEnabled(false),
      _medianWindowSize(5),
      _movingAverageEnabled(false),
      _movingAverageSize(5),
      _temperature(ULTRASONIC_DEFAULT_TEMP),
      _updateRate(20),
      _lastUpdateTime(0) {
    
    // Allocate memory for sensors
    _configs = new UltrasonicConfig[_numSensors];
    _measurements = new UltrasonicMeasurement[_numSensors];
    _statistics = new SensorStatistics[_numSensors];
    _states = new SensorState[_numSensors];
    _filterBuffers = new uint16_t*[_numSensors];
    _filterIndices = new uint8_t[_numSensors];
    
    // Initialize arrays
    for (uint8_t i = 0; i < _numSensors; i++) {
        memset(&_configs[i], 0, sizeof(UltrasonicConfig));
        memset(&_measurements[i], 0, sizeof(UltrasonicMeasurement));
        memset(&_statistics[i], 0, sizeof(SensorStatistics));
        _states[i] = SENSOR_STATE_UNINITIALIZED;
        
        _configs[i].minDistance = ULTRASONIC_MIN_DISTANCE;
        _configs[i].maxDistance = ULTRASONIC_MAX_DISTANCE;
        _configs[i].enabled = false;
        
        // Allocate filter buffer for each sensor
        _filterBuffers[i] = new uint16_t[10]; // Max filter size
        memset(_filterBuffers[i], 0, 10 * sizeof(uint16_t));
        _filterIndices[i] = 0;
        
        _measurements[i].sensorId = i;
        _statistics[i].minMeasured = ULTRASONIC_MAX_DISTANCE;
        _statistics[i].maxMeasured = 0;
    }
    
    _calculateSpeedOfSound();
    _updateInterval = 1000 / _updateRate;
}

Ultrasonic::~Ultrasonic() {
    // Free allocated memory
    for (uint8_t i = 0; i < _numSensors; i++) {
        delete[] _filterBuffers[i];
    }
    delete[] _filterBuffers;
    delete[] _filterIndices;
    delete[] _configs;
    delete[] _measurements;
    delete[] _statistics;
    delete[] _states;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool Ultrasonic::begin(const UltrasonicConfig* configs, uint8_t configCount) {
    if (configs == nullptr || configCount == 0 || configCount > _numSensors) {
        return false;
    }
    
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < configCount; i++) {
        if (!configureSensor(i, configs[i])) {
            allSuccess = false;
        }
    }
    
    _configuredSensors = configCount;
    
    // Run initial self-test
    delay(100);
    selfTest();
    
    return allSuccess;
}

uint8_t Ultrasonic::addSensor(uint8_t triggerPin, uint8_t echoPin, 
                              uint8_t position, float mountingAngle) {
    if (_configuredSensors >= _numSensors) {
        return 0xFF; // Array full
    }
    
    uint8_t sensorId = _configuredSensors;
    
    UltrasonicConfig config;
    config.triggerPin = triggerPin;
    config.echoPin = echoPin;
    config.position = position;
    config.mountingAngle = mountingAngle;
    config.xOffset = 0.0f;
    config.yOffset = 0.0f;
    config.minDistance = ULTRASONIC_MIN_DISTANCE;
    config.maxDistance = ULTRASONIC_MAX_DISTANCE;
    config.enabled = true;
    
    if (configureSensor(sensorId, config)) {
        _configuredSensors++;
        return sensorId;
    }
    
    return 0xFF;
}

bool Ultrasonic::configureSensor(uint8_t sensorId, const UltrasonicConfig& config) {
    if (sensorId >= _numSensors) {
        return false;
    }
    
    // Validate pin numbers
    if (config.triggerPin >= NUM_DIGITAL_PINS || config.echoPin >= NUM_DIGITAL_PINS) {
        return false;
    }
    
    // Configure pins
    pinMode(config.triggerPin, OUTPUT);
    pinMode(config.echoPin, INPUT);
    digitalWrite(config.triggerPin, LOW);
    
    // Store configuration
    _configs[sensorId] = config;
    _states[sensorId] = SENSOR_STATE_IDLE;
    
    // Reset statistics
    resetStatistics(sensorId);
    
    // Initialize filter buffer with default values
    for (uint8_t i = 0; i < 10; i++) {
        _filterBuffers[sensorId][i] = config.maxDistance;
    }
    _filterIndices[sensorId] = 0;
    
    return true;
}

void Ultrasonic::enableSensor(uint8_t sensorId, bool enable) {
    if (sensorId < _numSensors) {
        _configs[sensorId].enabled = enable;
        if (!enable) {
            _states[sensorId] = SENSOR_STATE_IDLE;
        }
    }
}

// ============================================================================
// MEASUREMENT
// ============================================================================

void Ultrasonic::update() {
    uint32_t currentTime = millis();
    
    // Check if it's time to update
    if (currentTime - _lastUpdateTime < _updateInterval) {
        return;
    }
    
    _lastUpdateTime = currentTime;
    
    // Measure all enabled sensors sequentially
    for (uint8_t i = 0; i < _configuredSensors; i++) {
        if (_configs[i].enabled && _states[i] != SENSOR_STATE_ERROR) {
            measureDistance(i);
            delayMicroseconds(200); // Small delay between sensors to avoid crosstalk
        }
    }
}

uint16_t Ultrasonic::measureDistance(uint8_t sensorId) {
    if (sensorId >= _configuredSensors || !_configs[sensorId].enabled) {
        return 0;
    }
    
    _states[sensorId] = SENSOR_STATE_MEASURING;
    
    // Perform actual measurement
    uint16_t distance = _measureSingleDistance(sensorId);
    
    // Create measurement record
    UltrasonicMeasurement measurement;
    measurement.sensorId = sensorId;
    measurement.distance = distance;
    measurement.timestamp = millis();
    measurement.valid = _validateMeasurement(sensorId, distance);
    
    if (measurement.valid) {
        // Apply filters if enabled
        _applyFilters(sensorId, measurement.distance);
        
        // Calculate signal quality based on consistency with previous measurements
        if (_statistics[sensorId].validCount > 0) {
            float deviation = abs((float)measurement.distance - _statistics[sensorId].averageDistance);
            float normalizedDeviation = deviation / _statistics[sensorId].averageDistance;
            measurement.signalQuality = constrain(1.0f - normalizedDeviation, 0.0f, 1.0f);
        } else {
            measurement.signalQuality = 0.8f; // Default quality for first measurement
        }
        
        _states[sensorId] = SENSOR_STATE_IDLE;
    } else {
        measurement.signalQuality = 0.0f;
        
        // Don't immediately mark as error - allow a few failures
        if (_statistics[sensorId].measurementCount > 10) {
            float failureRate = (float)_statistics[sensorId].invalidCount / 
                               _statistics[sensorId].measurementCount;
            if (failureRate > 0.7f) { // More than 70% failures
                _states[sensorId] = SENSOR_STATE_ERROR;
            } else {
                _states[sensorId] = SENSOR_STATE_IDLE;
            }
        }
    }
    
    // Store measurement
    _measurements[sensorId] = measurement;
    
    // Update statistics
    _updateStatistics(sensorId, measurement);
    
    return measurement.valid ? measurement.distance : 0;
}

UltrasonicMeasurement Ultrasonic::getMeasurement(uint8_t sensorId) const {
    if (sensorId < _configuredSensors) {
        return _measurements[sensorId];
    }
    
    // Return empty measurement if invalid sensor ID
    UltrasonicMeasurement empty;
    memset(&empty, 0, sizeof(UltrasonicMeasurement));
    return empty;
}

uint8_t Ultrasonic::getAllMeasurements(UltrasonicMeasurement* measurements, 
                                       uint8_t maxCount) const {
    if (measurements == nullptr || maxCount == 0) {
        return 0;
    }
    
    uint8_t count = min(_configuredSensors, maxCount);
    for (uint8_t i = 0; i < count; i++) {
        measurements[i] = _measurements[i];
    }
    return count;
}

uint16_t Ultrasonic::getAverageDistance(const uint8_t* sensorIds, uint8_t count) const {
    if (sensorIds == nullptr || count == 0) {
        return 0;
    }
    
    uint32_t sum = 0;
    uint8_t validCount = 0;
    
    for (uint8_t i = 0; i < count; i++) {
        if (sensorIds[i] < _configuredSensors && _measurements[sensorIds[i]].valid) {
            sum += _measurements[sensorIds[i]].distance;
            validCount++;
        }
    }
    
    return validCount > 0 ? (sum / validCount) : 0;
}

uint16_t Ultrasonic::getMinimumDistance() const {
    uint16_t minDist = ULTRASONIC_MAX_DISTANCE;
    bool foundValid = false;
    
    for (uint8_t i = 0; i < _configuredSensors; i++) {
        if (_measurements[i].valid && _measurements[i].distance > 0) {
            if (_measurements[i].distance < minDist) {
                minDist = _measurements[i].distance;
                foundValid = true;
            }
        }
    }
    
    return foundValid ? minDist : 0;
}

uint16_t Ultrasonic::getMinimumDistance(const uint8_t* sensorIds, uint8_t count) const {
    if (sensorIds == nullptr || count == 0) {
        return 0;
    }
    
    uint16_t minDist = ULTRASONIC_MAX_DISTANCE;
    bool foundValid = false;
    
    for (uint8_t i = 0; i < count; i++) {
        if (sensorIds[i] < _configuredSensors && _measurements[sensorIds[i]].valid) {
            if (_measurements[sensorIds[i]].distance > 0 && 
                _measurements[sensorIds[i]].distance < minDist) {
                minDist = _measurements[sensorIds[i]].distance;
                foundValid = true;
            }
        }
    }
    
    return foundValid ? minDist : 0;
}

// ============================================================================
// OBSTACLE DETECTION
// ============================================================================

bool Ultrasonic::detectFrontObstacle(uint16_t threshold) const {
    // Check front-facing sensors (positions 0, 1, 2)
    for (uint8_t i = 0; i < _configuredSensors; i++) {
        if (_configs[i].position >= SENSOR_POSITION_FRONT_LEFT && 
            _configs[i].position <= SENSOR_POSITION_FRONT_RIGHT) {
            if (_measurements[i].valid && 
                _measurements[i].distance > 0 && 
                _measurements[i].distance < threshold) {
                return true;
            }
        }
    }
    return false;
}

bool Ultrasonic::detectSideObstacles(uint16_t leftThreshold, uint16_t rightThreshold) const {
    bool leftObstacle = false;
    bool rightObstacle = false;
    
    for (uint8_t i = 0; i < _configuredSensors; i++) {
        if (_measurements[i].valid && _measurements[i].distance > 0) {
            // Check left side
            if (_configs[i].position == SENSOR_POSITION_LEFT && 
                _measurements[i].distance < leftThreshold) {
                leftObstacle = true;
            }
            // Check right side
            if (_configs[i].position == SENSOR_POSITION_RIGHT && 
                _measurements[i].distance < rightThreshold) {
                rightObstacle = true;
            }
        }
    }
    
    return leftObstacle || rightObstacle;
}

ObstacleInfo Ultrasonic::getClosestObstacle() const {
    ObstacleInfo obstacle;
    obstacle.detected = false;
    obstacle.distance = ULTRASONIC_MAX_DISTANCE;
    obstacle.angle = 0.0f;
    obstacle.sensorId = 0xFF;
    obstacle.timestamp = 0;
    obstacle.confidence = 0;
    
    for (uint8_t i = 0; i < _configuredSensors; i++) {
        if (_measurements[i].valid && 
            _measurements[i].distance > 0 && 
            _measurements[i].distance < obstacle.distance) {
            obstacle.detected = true;
            obstacle.distance = _measurements[i].distance;
            obstacle.angle = _calculateObstacleAngle(i);
            obstacle.sensorId = i;
            obstacle.timestamp = _measurements[i].timestamp;
            obstacle.confidence = (uint8_t)(_measurements[i].signalQuality * 100);
        }
    }
    
    return obstacle;
}

bool Ultrasonic::isPathClear(uint16_t minDistance) const {
    for (uint8_t i = 0; i < _configuredSensors; i++) {
        if (_measurements[i].valid && 
            _measurements[i].distance > 0 && 
            _measurements[i].distance < minDistance) {
            return false;
        }
    }
    return true;
}

bool Ultrasonic::detectObstacleInDirection(float angle, float angleRange, 
                                          uint16_t threshold) const {
    // Normalize target angle to 0-360
    while (angle < 0) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    
    for (uint8_t i = 0; i < _configuredSensors; i++) {
        // Get sensor mounting angle
        float sensorAngle = _configs[i].mountingAngle;
        while (sensorAngle < 0) sensorAngle += 360.0f;
        while (sensorAngle >= 360.0f) sensorAngle -= 360.0f;
        
        // Calculate angular difference
        float angleDiff = abs(sensorAngle - angle);
        if (angleDiff > 180.0f) {
            angleDiff = 360.0f - angleDiff; // Take shorter angle
        }
        
        // Check if sensor is within angle range
        if (angleDiff <= angleRange) {
            if (_measurements[i].valid && 
                _measurements[i].distance > 0 && 
                _measurements[i].distance < threshold) {
                return true;
            }
        }
    }
    
    return false;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void Ultrasonic::setUpdateRate(uint8_t rateHz) {
    _updateRate = constrain(rateHz, 1, 50);
    _updateInterval = 1000 / _updateRate;
}

void Ultrasonic::setMedianFilter(bool enable, uint8_t windowSize) {
    _medianFilterEnabled = enable;
    _medianWindowSize = constrain(windowSize, 3, 7);
    
    // Ensure odd window size for proper median calculation
    if (_medianWindowSize % 2 == 0) {
        _medianWindowSize++;
    }
}

void Ultrasonic::setMovingAverageFilter(bool enable, uint8_t windowSize) {
    _movingAverageEnabled = enable;
    _movingAverageSize = constrain(windowSize, 2, 10);
}

void Ultrasonic::setTemperature(float temperature) {
    _temperature = constrain(temperature, -40.0f, 85.0f);
    _calculateSpeedOfSound();
}

void Ultrasonic::setDistanceThresholds(uint8_t sensorId, uint16_t minDist, uint16_t maxDist) {
    if (sensorId < _numSensors) {
        _configs[sensorId].minDistance = minDist;
        _configs[sensorId].maxDistance = maxDist;
    }
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

SensorState Ultrasonic::getSensorState(uint8_t sensorId) const {
    if (sensorId < _numSensors) {
        return _states[sensorId];
    }
    return SENSOR_STATE_UNINITIALIZED;
}

SensorStatistics Ultrasonic::getStatistics(uint8_t sensorId) const {
    if (sensorId < _numSensors) {
        return _statistics[sensorId];
    }
    
    // Return empty statistics
    SensorStatistics empty;
    memset(&empty, 0, sizeof(SensorStatistics));
    return empty;
}

void Ultrasonic::resetStatistics(uint8_t sensorId) {
    if (sensorId < _numSensors) {
        memset(&_statistics[sensorId], 0, sizeof(SensorStatistics));
        _statistics[sensorId].minMeasured = ULTRASONIC_MAX_DISTANCE;
        _statistics[sensorId].maxMeasured = 0;
        _statistics[sensorId].averageDistance = 0.0f;
    }
}

bool Ultrasonic::isSensorHealthy(uint8_t sensorId) const {
    if (sensorId >= _numSensors) {
        return false;
    }
    
    // Check if sensor is in error state
    if (_states[sensorId] == SENSOR_STATE_ERROR) {
        return false;
    }
    
    // Check if sensor is enabled
    if (!_configs[sensorId].enabled) {
        return false;
    }
    
    // Check if sensor has recent valid measurements
    uint32_t timeSinceUpdate = millis() - _statistics[sensorId].lastUpdateTime;
    if (timeSinceUpdate > 5000) { // No update in 5 seconds
        return false;
    }
    
    // Check validity ratio (need at least 10 measurements to judge)
    if (_statistics[sensorId].measurementCount > 10) {
        float validityRatio = (float)_statistics[sensorId].validCount / 
                             _statistics[sensorId].measurementCount;
        if (validityRatio < 0.5f) { // Less than 50% valid
            return false;
        }
    }
    
    return true;
}

uint8_t Ultrasonic::getSensorCount() const {
    return _configuredSensors;
}

const UltrasonicConfig* Ultrasonic::getSensorConfig(uint8_t sensorId) const {
    if (sensorId < _numSensors) {
        return &_configs[sensorId];
    }
    return nullptr;
}

bool Ultrasonic::selfTest() {
    bool allPassed = true;
    
    for (uint8_t i = 0; i < _configuredSensors; i++) {
        if (!_configs[i].enabled) continue;
        
        // Perform 5 test measurements
        uint8_t validCount = 0;
        uint16_t testDistances[5];
        
        for (uint8_t j = 0; j < 5; j++) {
            uint16_t distance = measureDistance(i);
            testDistances[j] = distance;
            if (distance > 0) {
                validCount++;
            }
            delay(100); // Wait between measurements
        }
        
        // Sensor passes if at least 3 of 5 measurements are valid
        if (validCount < 3) {
            _states[i] = SENSOR_STATE_ERROR;
            allPassed = false;
        } else {
            // Additional check: measurements should be reasonably consistent
            // Calculate standard deviation
            float mean = 0.0f;
            for (uint8_t j = 0; j < validCount; j++) {
                if (testDistances[j] > 0) {
                    mean += testDistances[j];
                }
            }
            mean /= validCount;
            
            float variance = 0.0f;
            for (uint8_t j = 0; j < validCount; j++) {
                if (testDistances[j] > 0) {
                    float diff = testDistances[j] - mean;
                    variance += diff * diff;
                }
            }
            variance /= validCount;
            float stdDev = sqrt(variance);
            
            // If standard deviation is too high relative to mean, sensor may be unreliable
            if (stdDev > mean * 0.5f && mean > 100) { // More than 50% variation
                _states[i] = SENSOR_STATE_ERROR;
                allPassed = false;
            } else {
                _states[i] = SENSOR_STATE_IDLE;
            }
        }
    }
    
    return allPassed;
}

// ============================================================================
// PRIVATE METHODS - MEASUREMENT
// ============================================================================

uint16_t Ultrasonic::_measureSingleDistance(uint8_t sensorId) {
    // Trigger measurement
    if (!_triggerMeasurement(sensorId)) {
        return 0;
    }
    
    // Measure echo pulse width
    uint32_t pulseWidth = _measurePulseWidth(_configs[sensorId].echoPin, 
                                             ULTRASONIC_TIMEOUT_US);
    
    // Check for timeout or no echo
    if (pulseWidth == 0 || pulseWidth >= ULTRASONIC_TIMEOUT_US) {
        return 0;
    }
    
    // Store raw pulse width for diagnostics
    _measurements[sensorId].rawPulseWidth = pulseWidth;
    
    // Convert pulse width to distance
    return _pulseWidthToDistance(pulseWidth);
}

bool Ultrasonic::_triggerMeasurement(uint8_t sensorId) {
    // Ensure trigger is LOW initially
    digitalWrite(_configs[sensorId].triggerPin, LOW);
    delayMicroseconds(2);
    
    // Send 10us HIGH pulse to trigger
    digitalWrite(_configs[sensorId].triggerPin, HIGH);
    delayMicroseconds(ULTRASONIC_TRIGGER_DURATION);
    digitalWrite(_configs[sensorId].triggerPin, LOW);
    
    return true;
}

uint32_t Ultrasonic::_measurePulseWidth(uint8_t echoPin, uint32_t timeout) {
    // Wait for echo to go HIGH (with timeout)
    uint32_t startWait = micros();
    while (digitalRead(echoPin) == LOW) {
        if (micros() - startWait > timeout) {
            return 0; // Timeout waiting for echo start
        }
    }
    
    // Measure HIGH pulse duration
    uint32_t pulseStart = micros();
    while (digitalRead(echoPin) == HIGH) {
        if (micros() - pulseStart > timeout) {
            return timeout; // Timeout during pulse measurement
        }
    }
    uint32_t pulseEnd = micros();
    
    return pulseEnd - pulseStart;
}

uint16_t Ultrasonic::_pulseWidthToDistance(uint32_t pulseWidth) const {
    // Distance = (pulse_width * speed_of_sound) / 2
    // pulse_width is in microseconds
    // speed_of_sound is in m/s
    // Convert to millimeters: multiply by 0.001 to get meters, then by 1000 to get mm
    // Final formula: distance_mm = (pulseWidth_us * speedOfSound_m/s * 0.001) / 2
    
    float distance = (pulseWidth * _speedOfSound * 0.001f) / 2.0f; // Result in mm
    return (uint16_t)distance;
}

bool Ultrasonic::_validateMeasurement(uint8_t sensorId, uint16_t distance) const {
    // Check if distance is zero (measurement failed)
    if (distance == 0) {
        return false;
    }
    
    // Check if distance is within configured range
    if (distance < _configs[sensorId].minDistance || 
        distance > _configs[sensorId].maxDistance) {
        return false;
    }
    
    return true;
}

void Ultrasonic::_applyFilters(uint8_t sensorId, uint16_t& distance) {
    // Apply median filter first if enabled
    if (_medianFilterEnabled) {
        distance = _medianFilter(sensorId, distance);
    }
    
    // Apply moving average filter if enabled
    if (_movingAverageEnabled) {
        distance = _movingAverageFilter(sensorId, distance);
    }
}

uint16_t Ultrasonic::_medianFilter(uint8_t sensorId, uint16_t newValue) {
    // Add new value to circular buffer
    _filterBuffers[sensorId][_filterIndices[sensorId]] = newValue;
    _filterIndices[sensorId] = (_filterIndices[sensorId] + 1) % _medianWindowSize;
    
    // Copy buffer values for sorting (don't sort the original buffer)
    uint16_t sortBuffer[7]; // Max window size is 7
    memcpy(sortBuffer, _filterBuffers[sensorId], _medianWindowSize * sizeof(uint16_t));
    
    // Simple bubble sort (efficient for small arrays)
    for (uint8_t i = 0; i < _medianWindowSize - 1; i++) {
        for (uint8_t j = 0; j < _medianWindowSize - i - 1; j++) {
            if (sortBuffer[j] > sortBuffer[j + 1]) {
                uint16_t temp = sortBuffer[j];
                sortBuffer[j] = sortBuffer[j + 1];
                sortBuffer[j + 1] = temp;
            }
        }
    }
    
    // Return median value (middle element of sorted array)
    return sortBuffer[_medianWindowSize / 2];
}

uint16_t Ultrasonic::_movingAverageFilter(uint8_t sensorId, uint16_t newValue) {
    // Add new value to circular buffer
    _filterBuffers[sensorId][_filterIndices[sensorId]] = newValue;
    _filterIndices[sensorId] = (_filterIndices[sensorId] + 1) % _movingAverageSize;
    
    // Calculate average of all values in buffer
    uint32_t sum = 0;
    for (uint8_t i = 0; i < _movingAverageSize; i++) {
        sum += _filterBuffers[sensorId][i];
    }
    
    return sum / _movingAverageSize;
}

void Ultrasonic::_updateStatistics(uint8_t sensorId, const UltrasonicMeasurement& measurement) {
    SensorStatistics& stats = _statistics[sensorId];
    
    // Increment total measurement count
    stats.measurementCount++;
    stats.lastUpdateTime = measurement.timestamp;
    
    if (measurement.valid) {
        // Increment valid count
        stats.validCount++;
        
        // Update min/max
        if (measurement.distance < stats.minMeasured) {
            stats.minMeasured = measurement.distance;
        }
        if (measurement.distance > stats.maxMeasured) {
            stats.maxMeasured = measurement.distance;
        }
        
        // Update running average using incremental formula
        // new_avg = ((old_avg * (n-1)) + new_value) / n
        if (stats.validCount == 1) {
            stats.averageDistance = measurement.distance;
        } else {
            stats.averageDistance = ((stats.averageDistance * (stats.validCount - 1)) + 
                                    measurement.distance) / stats.validCount;
        }
    } else {
        // Increment invalid count
        stats.invalidCount++;
    }
}

void Ultrasonic::_calculateSpeedOfSound() {
    // Temperature compensation formula for speed of sound in air
    // v = 331.3 + 0.606 * temperature (in m/s)
    // where temperature is in Celsius
    _speedOfSound = 331.3f + (0.606f * _temperature);
}

uint8_t Ultrasonic::_findSensorByPosition(uint8_t position) const {
    for (uint8_t i = 0; i < _configuredSensors; i++) {
        if (_configs[i].position == position) {
            return i;
        }
    }
    return 0xFF; // Not found
}

float Ultrasonic::_calculateObstacleAngle(uint8_t sensorId) const {
    // Return the mounting angle of the sensor as the obstacle angle
    // This assumes obstacle is directly in front of the sensor
    return _configs[sensorId].mountingAngle;
}
