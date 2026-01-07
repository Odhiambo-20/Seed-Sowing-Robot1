/**
 * @file LiDAR.cpp
 * @brief Production-ready RPLIDAR A1/A2 driver implementation
 * @version 2.0.0
 * @date 2026-01-06
 */

#include "sensors/obstacle_detection/LiDAR.h"

// ============================================================================
// CONSTRUCTOR & DESTRUCTOR
// ============================================================================

LiDAR::LiDAR(HardwareSerial& serial, uint8_t motorPin, uint32_t baudRate)
    : _serial(serial),
      _motorPin(motorPin),
      _baudRate(baudRate),
      _state(LIDAR_STATE_IDLE),
      _obstacleZones(nullptr),
      _obstacleZoneCount(0),
      _motorPWM(RPLIDAR_MOTOR_DEFAULT_PWM),
      _minQuality(10),
      _distanceFilterEnabled(true),
      _filterMinDist(LIDAR_MIN_DISTANCE),
      _filterMaxDist(LIDAR_MAX_DISTANCE),
      _medianFilterEnabled(false),
      _medianWindowSize(5),
      _lastScanTime(0),
      _scanCount(0),
      _scanRate(0.0f),
      _rxBufferPos(0),
      _lastRxTime(0) {
    
    memset(&_scanData, 0, sizeof(LiDARScanData));
    memset(&_deviceInfo, 0, sizeof(LiDARDeviceInfo));
    memset(&_healthStatus, 0, sizeof(LiDARHealthStatus));
}

LiDAR::~LiDAR() {
    stopScan();
    motorOff();
    if (_obstacleZones != nullptr) {
        delete[] _obstacleZones;
    }
}

// ============================================================================
// INITIALIZATION AND CONTROL
// ============================================================================

bool LiDAR::begin() {
    pinMode(_motorPin, OUTPUT);
    analogWriteFrequency(_motorPin, RPLIDAR_MOTOR_PWM_FREQ);
    motorOff();
    
    _serial.begin(_baudRate);
    _serial.setTimeout(RPLIDAR_DEFAULT_TIMEOUT);
    
    delay(100);
    
    _state = LIDAR_STATE_INITIALIZING;
    
    if (!reset()) {
        _state = LIDAR_STATE_ERROR;
        return false;
    }
    
    delay(500);
    
    if (!_getDeviceInfo()) {
        _state = LIDAR_STATE_ERROR;
        return false;
    }
    
    if (!updateHealthStatus()) {
        _state = LIDAR_STATE_ERROR;
        return false;
    }
    
    if (_healthStatus.status != 0) {
        _state = LIDAR_STATE_ERROR;
        return false;
    }
    
    _state = LIDAR_STATE_IDLE;
    return true;
}

bool LiDAR::startScan(bool expressMode) {
    if (_state != LIDAR_STATE_IDLE) {
        return false;
    }
    
    motorOn();
    delay(1000);
    
    _clearRxBuffer();
    
    uint8_t cmd = expressMode ? RPLIDAR_CMD_EXPRESS_SCAN : RPLIDAR_CMD_SCAN;
    if (!_sendCommand(cmd)) {
        motorOff();
        return false;
    }
    
    uint8_t descriptor[7];
    if (!_waitResponseHeader(descriptor, 1000)) {
        motorOff();
        return false;
    }
    
    _scanData.pointCount = 0;
    _scanData.scanComplete = false;
    _scanData.scanStartTime = millis();
    _lastScanTime = millis();
    
    _state = LIDAR_STATE_SCANNING;
    return true;
}

bool LiDAR::stopScan() {
    if (_state != LIDAR_STATE_SCANNING) {
        return false;
    }
    
    _state = LIDAR_STATE_STOPPING;
    
    if (!_sendCommand(RPLIDAR_CMD_STOP)) {
        _state = LIDAR_STATE_ERROR;
        return false;
    }
    
    delay(100);
    
    motorOff();
    _clearRxBuffer();
    
    _state = LIDAR_STATE_IDLE;
    return true;
}

bool LiDAR::reset() {
    _sendCommand(RPLIDAR_CMD_RESET);
    delay(100);
    _clearRxBuffer();
    return true;
}

void LiDAR::setMotorSpeed(uint16_t pwmValue) {
    _motorPWM = constrain(pwmValue, 0, RPLIDAR_MOTOR_MAX_PWM);
    if (_healthStatus.motorRunning) {
        analogWrite(_motorPin, _motorPWM);
    }
}

void LiDAR::motorOn() {
    analogWrite(_motorPin, _motorPWM);
    _healthStatus.motorRunning = true;
}

void LiDAR::motorOff() {
    analogWrite(_motorPin, 0);
    _healthStatus.motorRunning = false;
}

// ============================================================================
// DATA ACQUISITION
// ============================================================================

void LiDAR::update() {
    if (_state != LIDAR_STATE_SCANNING) {
        return;
    }
    
    while (_serial.available() > 0) {
        uint8_t byte = _serial.read();
        
        if (_rxBufferPos == 0 && byte == RPLIDAR_SYNC_BYTE) {
            _rxBuffer[_rxBufferPos++] = byte;
        } else if (_rxBufferPos == 1) {
            _rxBuffer[_rxBufferPos++] = byte;
        } else if (_rxBufferPos > 1 && _rxBufferPos < 5) {
            _rxBuffer[_rxBufferPos++] = byte;
            
            if (_rxBufferPos == 5) {
                _parseStandardMeasurement(_rxBuffer);
                _rxBufferPos = 0;
            }
        }
        
        _lastRxTime = millis();
    }
    
    if (_lastRxTime > 0 && (millis() - _lastRxTime) > 1000) {
        _healthStatus.dataValid = false;
    } else {
        _healthStatus.dataValid = true;
    }
    
    if (_scanData.pointCount >= RPLIDAR_MAX_SCAN_POINTS) {
        _scanData.scanEndTime = millis();
        _scanData.scanComplete = true;
        
        uint32_t scanDuration = _scanData.scanEndTime - _scanData.scanStartTime;
        if (scanDuration > 0) {
            _scanRate = 1000.0f / scanDuration;
        }
        
        _scanCount++;
        
        _scanData.pointCount = 0;
        _scanData.scanComplete = false;
        _scanData.scanStartTime = millis();
    }
}

const LiDARScanData* LiDAR::getScanData() const {
    return &_scanData;
}

LiDARMeasurement LiDAR::getMeasurementAtAngle(float angle) const {
    while (angle < 0) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    
    float minDiff = 360.0f;
    LiDARMeasurement closestMeasurement = {0};
    
    for (uint16_t i = 0; i < _scanData.pointCount; i++) {
        if (_scanData.points[i].valid) {
            float diff = abs(_scanData.points[i].angle - angle);
            if (diff > 180.0f) diff = 360.0f - diff;
            
            if (diff < minDiff) {
                minDiff = diff;
                closestMeasurement = _scanData.points[i];
            }
        }
    }
    
    return closestMeasurement;
}

uint16_t LiDAR::getMeasurementsInRange(float startAngle, float endAngle, 
                                       LiDARMeasurement* measurements, 
                                       uint16_t maxCount) const {
    while (startAngle < 0) startAngle += 360.0f;
    while (startAngle >= 360.0f) startAngle -= 360.0f;
    while (endAngle < 0) endAngle += 360.0f;
    while (endAngle >= 360.0f) endAngle -= 360.0f;
    
    uint16_t count = 0;
    
    for (uint16_t i = 0; i < _scanData.pointCount && count < maxCount; i++) {
        if (_scanData.points[i].valid) {
            float angle = _scanData.points[i].angle;
            
            bool inRange = false;
            if (startAngle <= endAngle) {
                inRange = (angle >= startAngle && angle <= endAngle);
            } else {
                inRange = (angle >= startAngle || angle <= endAngle);
            }
            
            if (inRange) {
                measurements[count++] = _scanData.points[i];
            }
        }
    }
    
    return count;
}

// ============================================================================
// OBSTACLE DETECTION
// ============================================================================

bool LiDAR::detectObstacleInZone(float startAngle, float endAngle, uint16_t threshold) {
    LiDARMeasurement measurements[180];
    uint16_t count = getMeasurementsInRange(startAngle, endAngle, measurements, 180);
    
    for (uint16_t i = 0; i < count; i++) {
        if (measurements[i].distance > 0 && measurements[i].distance < threshold) {
            return true;
        }
    }
    
    return false;
}

uint16_t LiDAR::getClosestObstacle() const {
    uint16_t minDistance = LIDAR_MAX_DISTANCE;
    
    for (uint16_t i = 0; i < _scanData.pointCount; i++) {
        if (_scanData.points[i].valid && 
            _scanData.points[i].distance > 0 && 
            _scanData.points[i].distance < minDistance) {
            minDistance = _scanData.points[i].distance;
        }
    }
    
    return (minDistance == LIDAR_MAX_DISTANCE) ? 0 : minDistance;
}

uint16_t LiDAR::getClosestObstacleInDirection(float centerAngle, float angleRange) const {
    float startAngle = centerAngle - angleRange;
    float endAngle = centerAngle + angleRange;
    
    LiDARMeasurement measurements[180];
    uint16_t count = getMeasurementsInRange(startAngle, endAngle, measurements, 180);
    
    uint16_t minDistance = LIDAR_MAX_DISTANCE;
    
    for (uint16_t i = 0; i < count; i++) {
        if (measurements[i].distance > 0 && measurements[i].distance < minDistance) {
            minDistance = measurements[i].distance;
        }
    }
    
    return (minDistance == LIDAR_MAX_DISTANCE) ? 0 : minDistance;
}

void LiDAR::configureObstacleZones(ObstacleZone* zones, uint8_t zoneCount) {
    if (_obstacleZones != nullptr) {
        delete[] _obstacleZones;
    }
    
    _obstacleZoneCount = zoneCount;
    _obstacleZones = new ObstacleZone[zoneCount];
    
    for (uint8_t i = 0; i < zoneCount; i++) {
        _obstacleZones[i] = zones[i];
    }
}

void LiDAR::updateObstacleZones() {
    if (_obstacleZones == nullptr) {
        return;
    }
    
    for (uint8_t i = 0; i < _obstacleZoneCount; i++) {
        ObstacleZone& zone = _obstacleZones[i];
        
        LiDARMeasurement measurements[180];
        uint16_t count = getMeasurementsInRange(zone.startAngle, zone.endAngle, 
                                                measurements, 180);
        
        zone.minDistance = LIDAR_MAX_DISTANCE;
        zone.obstacleDetected = false;
        
        for (uint16_t j = 0; j < count; j++) {
            if (measurements[j].distance > 0) {
                if (measurements[j].distance < zone.minDistance) {
                    zone.minDistance = measurements[j].distance;
                }
                
                if (measurements[j].distance < zone.threshold) {
                    zone.obstacleDetected = true;
                }
            }
        }
    }
}

const ObstacleZone* LiDAR::getObstacleZone(uint8_t zoneIndex) const {
    if (zoneIndex >= _obstacleZoneCount) {
        return nullptr;
    }
    return &_obstacleZones[zoneIndex];
}

// ============================================================================
// DIAGNOSTICS AND STATUS
// ============================================================================

const LiDARDeviceInfo* LiDAR::getDeviceInfo() const {
    return &_deviceInfo;
}

const LiDARHealthStatus* LiDAR::getHealthStatus() const {
    return &_healthStatus;
}

bool LiDAR::updateHealthStatus() {
    return _getHealthStatus();
}

bool LiDAR::isReady() const {
    return (_state == LIDAR_STATE_IDLE || _state == LIDAR_STATE_SCANNING) && 
           _healthStatus.status == 0;
}

bool LiDAR::isScanning() const {
    return _state == LIDAR_STATE_SCANNING;
}

LiDARState LiDAR::getState() const {
    return _state;
}

float LiDAR::getScanRate() const {
    return _scanRate;
}

uint16_t LiDAR::getPointCount() const {
    return _scanData.pointCount;
}

// ============================================================================
// FILTERING AND PROCESSING
// ============================================================================

void LiDAR::setMinQuality(uint8_t quality) {
    _minQuality = constrain(quality, 0, 63);
}

void LiDAR::setDistanceFilter(bool enable, uint16_t minDist, uint16_t maxDist) {
    _distanceFilterEnabled = enable;
    _filterMinDist = minDist;
    _filterMaxDist = maxDist;
}

void LiDAR::setMedianFilter(bool enable, uint8_t windowSize) {
    _medianFilterEnabled = enable;
    _medianWindowSize = constrain(windowSize, 3, 7);
    if (windowSize % 2 == 0) {
        _medianWindowSize = windowSize + 1;
    }
}

// ============================================================================
// PRIVATE METHODS - COMMUNICATION
// ============================================================================

bool LiDAR::_sendCommand(uint8_t cmd, const uint8_t* payload, uint8_t payloadSize) {
    uint8_t packet[128];
    uint8_t packetSize = 0;
    
    packet[packetSize++] = RPLIDAR_SYNC_BYTE;
    packet[packetSize++] = cmd;
    
    if (payload != nullptr && payloadSize > 0) {
        packet[packetSize++] = payloadSize;
        memcpy(&packet[packetSize], payload, payloadSize);
        packetSize += payloadSize;
        
        uint8_t checksum = 0;
        for (uint8_t i = 0; i < packetSize; i++) {
            checksum ^= packet[i];
        }
        packet[packetSize++] = checksum;
    }
    
    _serial.write(packet, packetSize);
    _serial.flush();
    
    return true;
}

bool LiDAR::_waitResponseHeader(uint8_t* descriptor, uint32_t timeout) {
    uint32_t startTime = millis();
    uint8_t recvPos = 0;
    
    while ((millis() - startTime) < timeout) {
        if (_serial.available() > 0) {
            uint8_t byte = _serial.read();
            
            if (recvPos == 0) {
                if (byte == RPLIDAR_SYNC_BYTE) {
                    descriptor[recvPos++] = byte;
                }
            } else if (recvPos == 1) {
                if (byte == RPLIDAR_SYNC_BYTE2) {
                    descriptor[recvPos++] = byte;
                } else {
                    recvPos = 0;
                }
            } else if (recvPos >= 2 && recvPos < 7) {
                descriptor[recvPos++] = byte;
                
                if (recvPos == 7) {
                    return true;
                }
            }
        }
    }
    
    return false;
}

bool LiDAR::_receiveData(uint8_t* data, uint16_t size, uint32_t timeout) {
    uint32_t startTime = millis();
    uint16_t received = 0;
    
    while (received < size && (millis() - startTime) < timeout) {
        if (_serial.available() > 0) {
            data[received++] = _serial.read();
        }
    }
    
    return (received == size);
}

void LiDAR::_clearRxBuffer() {
    while (_serial.available() > 0) {
        _serial.read();
    }
    _rxBufferPos = 0;
}

uint16_t LiDAR::_calculateChecksum(uint8_t* data, uint16_t size) const {
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < size; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// ============================================================================
// PRIVATE METHODS - DEVICE INFO
// ============================================================================

bool LiDAR::_getDeviceInfo() {
    if (!_sendCommand(RPLIDAR_CMD_GET_INFO)) {
        return false;
    }
    
    uint8_t descriptor[7];
    if (!_waitResponseHeader(descriptor, 1000)) {
        return false;
    }
    
    uint8_t infoData[20];
    if (!_receiveData(infoData, 20, 1000)) {
        return false;
    }
    
    _deviceInfo.model = infoData[0];
    _deviceInfo.firmware_minor = infoData[1];
    _deviceInfo.firmware_major = infoData[2];
    _deviceInfo.hardware = infoData[3];
    memcpy(_deviceInfo.serialNumber, &infoData[4], 16);
    
    return true;
}

bool LiDAR::_getHealthStatus() {
    if (!_sendCommand(RPLIDAR_CMD_GET_HEALTH)) {
        return false;
    }
    
    uint8_t descriptor[7];
    if (!_waitResponseHeader(descriptor, 1000)) {
        return false;
    }
    
    uint8_t healthData[3];
    if (!_receiveData(healthData, 3, 1000)) {
        return false;
    }
    
    _healthStatus.status = healthData[0];
    _healthStatus.errorCode = (healthData[2] << 8) | healthData[1];
    
    return true;
}

// ============================================================================
// PRIVATE METHODS - DATA PARSING
// ============================================================================

void LiDAR::_parseStandardMeasurement(uint8_t* data) {
    if (data[0] != RPLIDAR_SYNC_BYTE) {
        return;
    }
    
    LiDARMeasurement measurement;
    
    uint8_t checkBit = (data[1] & 0x01);
    uint8_t inverseCheck = (data[3] & 0x01);
    if (checkBit == inverseCheck) {
        return;
    }
    
    measurement.quality = (data[1] >> 2) & 0x3F;
    
    uint16_t angleQ6 = ((data[3] & 0xFE) << 7) | data[2];
    measurement.angle = (angleQ6 >> 6) + ((angleQ6 & 0x3F) / 64.0f);
    
    measurement.distance = ((data[5] << 8) | data[4]) / 4;
    
    measurement.timestamp = millis();
    
    measurement.valid = _validateMeasurement(measurement);
    
    if (measurement.valid) {
        _applyFilters(measurement);
        
        if (measurement.valid && _scanData.pointCount < RPLIDAR_MAX_SCAN_POINTS) {
            _scanData.points[_scanData.pointCount++] = measurement;
        }
    }
}

void LiDAR::_parseExpressMeasurement(uint8_t* data) {
    if (data[0] != RPLIDAR_SYNC_BYTE || data[1] != RPLIDAR_SYNC_BYTE2) {
        return;
    }
    
    uint8_t cabinCount = 16;
    static float lastAngle = 0;
    static uint16_t lastDistance = 0;
    
    for (uint8_t i = 0; i < cabinCount; i++) {
        uint8_t offset = 4 + (i * 5);
        
        if (offset + 4 >= 84) break;
        
        int16_t distanceDelta1 = (data[offset + 0] >> 2) | ((data[offset + 1] & 0x3F) << 6);
        int16_t distanceDelta2 = (data[offset + 2] >> 2) | ((data[offset + 3] & 0x3F) << 6);
        
        if (distanceDelta1 & 0x2000) distanceDelta1 |= 0xC000;
        if (distanceDelta2 & 0x2000) distanceDelta2 |= 0xC000;
        
        uint8_t angleDelta1 = (data[offset + 0] & 0x03);
        uint8_t angleDelta2 = (data[offset + 2] & 0x03);
        
        LiDARMeasurement measurement1, measurement2;
        
        measurement1.distance = lastDistance + distanceDelta1;
        measurement1.angle = lastAngle + (angleDelta1 * 0.5f);
        measurement1.quality = (data[offset + 1] >> 6) & 0x03;
        measurement1.timestamp = millis();
        measurement1.valid = _validateMeasurement(measurement1);
        
        measurement2.distance = measurement1.distance + distanceDelta2;
        measurement2.angle = measurement1.angle + (angleDelta2 * 0.5f);
        measurement2.quality = (data[offset + 3] >> 6) & 0x03;
        measurement2.timestamp = millis();
        measurement2.valid = _validateMeasurement(measurement2);
        
        if (measurement1.valid) {
            _applyFilters(measurement1);
            if (measurement1.valid && _scanData.pointCount < RPLIDAR_MAX_SCAN_POINTS) {
                _scanData.points[_scanData.pointCount++] = measurement1;
            }
        }
        
        if (measurement2.valid) {
            _applyFilters(measurement2);
            if (measurement2.valid && _scanData.pointCount < RPLIDAR_MAX_SCAN_POINTS) {
                _scanData.points[_scanData.pointCount++] = measurement2;
            }
        }
        
        lastAngle = measurement2.angle;
        lastDistance = measurement2.distance;
    }
}

// ============================================================================
// PRIVATE METHODS - VALIDATION AND FILTERING
// ============================================================================

bool LiDAR::_validateMeasurement(const LiDARMeasurement& measurement) const {
    if (measurement.quality < _minQuality) {
        return false;
    }
    
    if (measurement.distance < LIDAR_MIN_DISTANCE || 
        measurement.distance > LIDAR_MAX_DISTANCE) {
        return false;
    }
    
    if (measurement.angle < 0.0f || measurement.angle >= 360.0f) {
        return false;
    }
    
    return true;
}

void LiDAR::_applyFilters(LiDARMeasurement& measurement) {
    if (_distanceFilterEnabled) {
        if (measurement.distance < _filterMinDist ||
            measurement.distance > _filterMaxDist) {
            measurement.valid = false;
            return;
        }
    }
    
    if (_medianFilterEnabled && _scanData.pointCount >= _medianWindowSize) {
        float angleWindow = 5.0f;
        float values[7];
        uint8_t valueCount = 0;
        
        for (uint16_t i = _scanData.pointCount - _medianWindowSize; 
             i < _scanData.pointCount && valueCount < _medianWindowSize; i++) {
            
            if (_scanData.points[i].valid) {
                float angleDiff = abs(_scanData.points[i].angle - measurement.angle);
                if (angleDiff > 180.0f) angleDiff = 360.0f - angleDiff;
                
                if (angleDiff <= angleWindow) {
                    values[valueCount++] = (float)_scanData.points[i].distance;
                }
            }
        }
        
        if (valueCount >= 3) {
            values[valueCount++] = (float)measurement.distance;
            float medianValue = _medianFilter(values, valueCount);
            
            float deviation = abs((float)measurement.distance - medianValue);
            float threshold = medianValue * 0.3f;
            
            if (deviation > threshold && deviation > 100.0f) {
                measurement.valid = false;
            }
        }
    }
}

float LiDAR::_medianFilter(float* values, uint8_t count) const {
    for (uint8_t i = 0; i < count - 1; i++) {
        for (uint8_t j = 0; j < count - i - 1; j++) {
            if (values[j] > values[j + 1]) {
                float temp = values[j];
                values[j] = values[j + 1];
                values[j + 1] = temp;
            }
        }
    }
    
    return values[count / 2];
}
