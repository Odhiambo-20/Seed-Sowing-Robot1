/**
 * @file LiDAR.h
 * @brief Production-ready RPLIDAR A1/A2 driver for obstacle detection
 * @version 2.0.0
 * @date 2026-01-06
 * 
 * Supports:
 * - RPLIDAR A1 (12m range, 8000 samples/sec)
 * - RPLIDAR A2 (16m range, 8000 samples/sec)
 * - 360° scanning with configurable scan rate
 * - Obstacle detection zones with distance thresholds
 * - Point cloud filtering and processing
 * - Health monitoring and diagnostics
 * 
 * Hardware Connections:
 * - MOTOR_PWM: Pin for motor speed control (PWM)
 * - Serial: Hardware serial port for data communication
 */

#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>
#include <HardwareSerial.h>

// ============================================================================
// RPLIDAR PROTOCOL CONSTANTS
// ============================================================================

// Command Definitions
#define RPLIDAR_CMD_STOP           0x25
#define RPLIDAR_CMD_RESET          0x40
#define RPLIDAR_CMD_SCAN           0x20
#define RPLIDAR_CMD_EXPRESS_SCAN   0x82
#define RPLIDAR_CMD_FORCE_SCAN     0x21
#define RPLIDAR_CMD_GET_INFO       0x50
#define RPLIDAR_CMD_GET_HEALTH     0x52
#define RPLIDAR_CMD_GET_SAMPLERATE 0x59

// Response Definitions
#define RPLIDAR_ANS_TYPE_MEASUREMENT        0x81
#define RPLIDAR_ANS_TYPE_DEVINFO            0x04
#define RPLIDAR_ANS_TYPE_DEVHEALTH          0x06
#define RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED 0x82

// Response Lengths
#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT    (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT   (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

// Protocol Constants
#define RPLIDAR_SYNC_BYTE          0xA5
#define RPLIDAR_SYNC_BYTE2         0x5A
#define RPLIDAR_DEFAULT_TIMEOUT    500
#define RPLIDAR_MAX_SCAN_POINTS    360

// Motor Control
#define RPLIDAR_MOTOR_PWM_FREQ     25000  // 25kHz PWM frequency
#define RPLIDAR_MOTOR_MAX_PWM      1023   // 10-bit PWM resolution
#define RPLIDAR_MOTOR_DEFAULT_PWM  600    // Default motor speed

// Distance Thresholds (in millimeters)
#define LIDAR_MIN_DISTANCE         150    // Minimum valid distance (15cm)
#define LIDAR_MAX_DISTANCE         12000  // Maximum valid distance (12m)
#define LIDAR_OBSTACLE_THRESHOLD   1000   // Obstacle detection threshold (1m)

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @struct LiDARMeasurement
 * @brief Single LiDAR measurement point
 */
struct LiDARMeasurement {
    float angle;           // Angle in degrees (0-360)
    uint16_t distance;     // Distance in millimeters
    uint8_t quality;       // Signal quality (0-63, higher is better)
    bool valid;            // Is this measurement valid?
    uint32_t timestamp;    // Measurement timestamp (milliseconds)
};

/**
 * @struct LiDARScanData
 * @brief Complete 360° scan data
 */
struct LiDARScanData {
    LiDARMeasurement points[RPLIDAR_MAX_SCAN_POINTS];
    uint16_t pointCount;
    uint32_t scanStartTime;
    uint32_t scanEndTime;
    bool scanComplete;
};

/**
 * @struct ObstacleZone
 * @brief Obstacle detection zone configuration
 */
struct ObstacleZone {
    float startAngle;      // Start angle in degrees
    float endAngle;        // End angle in degrees
    uint16_t threshold;    // Distance threshold in mm
    bool obstacleDetected; // Obstacle present in this zone
    uint16_t minDistance;  // Minimum distance in zone
};

/**
 * @struct LiDARDeviceInfo
 * @brief LiDAR device information
 */
struct LiDARDeviceInfo {
    uint8_t model;
    uint8_t firmware_major;
    uint8_t firmware_minor;
    uint8_t hardware;
    uint8_t serialNumber[16];
};

/**
 * @struct LiDARHealthStatus
 * @brief LiDAR health status
 */
struct LiDARHealthStatus {
    uint8_t status;        // 0=Good, 1=Warning, 2=Error
    uint16_t errorCode;
    bool motorRunning;
    bool dataValid;
};

/**
 * @enum LiDARState
 * @brief LiDAR operational state
 */
enum LiDARState {
    LIDAR_STATE_IDLE,
    LIDAR_STATE_INITIALIZING,
    LIDAR_STATE_SCANNING,
    LIDAR_STATE_STOPPING,
    LIDAR_STATE_ERROR
};

// ============================================================================
// LIDAR CLASS DEFINITION
// ============================================================================

class LiDAR {
public:
    /**
     * @brief Constructor
     * @param serial Reference to hardware serial port
     * @param motorPin PWM pin for motor control
     * @param baudRate Serial communication baud rate (default: 115200)
     */
    LiDAR(HardwareSerial& serial, uint8_t motorPin, uint32_t baudRate = 115200);
    
    /**
     * @brief Destructor
     */
    ~LiDAR();
    
    // Initialization and Control
    /**
     * @brief Initialize LiDAR sensor
     * @return true if initialization successful
     */
    bool begin();
    
    /**
     * @brief Start scanning
     * @param expressMode Use express scan mode for higher speed (default: false)
     * @return true if scan started successfully
     */
    bool startScan(bool expressMode = false);
    
    /**
     * @brief Stop scanning
     * @return true if scan stopped successfully
     */
    bool stopScan();
    
    /**
     * @brief Reset LiDAR device
     * @return true if reset successful
     */
    bool reset();
    
    /**
     * @brief Set motor speed
     * @param pwmValue PWM value (0-1023)
     */
    void setMotorSpeed(uint16_t pwmValue);
    
    /**
     * @brief Enable motor
     */
    void motorOn();
    
    /**
     * @brief Disable motor
     */
    void motorOff();
    
    // Data Acquisition
    /**
     * @brief Update - call this in main loop
     * Process incoming LiDAR data
     */
    void update();
    
    /**
     * @brief Get latest scan data
     * @return Pointer to scan data structure
     */
    const LiDARScanData* getScanData() const;
    
    /**
     * @brief Get measurement at specific angle
     * @param angle Angle in degrees (0-360)
     * @return Measurement at specified angle
     */
    LiDARMeasurement getMeasurementAtAngle(float angle) const;
    
    /**
     * @brief Get measurements in angle range
     * @param startAngle Start angle in degrees
     * @param endAngle End angle in degrees
     * @param measurements Output array for measurements
     * @param maxCount Maximum number of measurements to return
     * @return Number of measurements returned
     */
    uint16_t getMeasurementsInRange(float startAngle, float endAngle, 
                                    LiDARMeasurement* measurements, 
                                    uint16_t maxCount) const;
    
    // Obstacle Detection
    /**
     * @brief Check for obstacles in specified zone
     * @param startAngle Zone start angle (degrees)
     * @param endAngle Zone end angle (degrees)
     * @param threshold Distance threshold in mm
     * @return true if obstacle detected
     */
    bool detectObstacleInZone(float startAngle, float endAngle, uint16_t threshold);
    
    /**
     * @brief Get closest obstacle distance
     * @return Distance to closest obstacle in mm (0 if none)
     */
    uint16_t getClosestObstacle() const;
    
    /**
     * @brief Get closest obstacle in specific direction
     * @param centerAngle Center angle to search around
     * @param angleRange Range around center angle (±degrees)
     * @return Distance to closest obstacle in mm (0 if none)
     */
    uint16_t getClosestObstacleInDirection(float centerAngle, float angleRange) const;
    
    /**
     * @brief Configure obstacle detection zones
     * @param zones Array of obstacle zones
     * @param zoneCount Number of zones
     */
    void configureObstacleZones(ObstacleZone* zones, uint8_t zoneCount);
    
    /**
     * @brief Update all configured obstacle zones
     */
    void updateObstacleZones();
    
    /**
     * @brief Get obstacle zone status
     * @param zoneIndex Zone index
     * @return Pointer to zone data (nullptr if invalid index)
     */
    const ObstacleZone* getObstacleZone(uint8_t zoneIndex) const;
    
    // Diagnostics and Status
    /**
     * @brief Get device information
     * @return Pointer to device info structure
     */
    const LiDARDeviceInfo* getDeviceInfo() const;
    
    /**
     * @brief Get health status
     * @return Pointer to health status structure
     */
    const LiDARHealthStatus* getHealthStatus() const;
    
    /**
     * @brief Update health status
     * @return true if health check successful
     */
    bool updateHealthStatus();
    
    /**
     * @brief Check if LiDAR is ready
     * @return true if ready to scan
     */
    bool isReady() const;
    
    /**
     * @brief Check if currently scanning
     * @return true if scanning
     */
    bool isScanning() const;
    
    /**
     * @brief Get current state
     * @return Current LiDAR state
     */
    LiDARState getState() const;
    
    /**
     * @brief Get scan rate (scans per second)
     * @return Scan rate in Hz
     */
    float getScanRate() const;
    
    /**
     * @brief Get data point count in last scan
     * @return Number of valid data points
     */
    uint16_t getPointCount() const;
    
    // Filtering and Processing
    /**
     * @brief Set minimum quality threshold
     * @param quality Minimum quality value (0-63)
     */
    void setMinQuality(uint8_t quality);
    
    /**
     * @brief Enable/disable distance filtering
     * @param enable Enable filtering
     * @param minDist Minimum distance in mm
     * @param maxDist Maximum distance in mm
     */
    void setDistanceFilter(bool enable, uint16_t minDist, uint16_t maxDist);
    
    /**
     * @brief Enable/disable median filtering
     * @param enable Enable median filtering
     * @param windowSize Filter window size (3, 5, or 7)
     */
    void setMedianFilter(bool enable, uint8_t windowSize = 5);

private:
    // Hardware interfaces
    HardwareSerial& _serial;
    uint8_t _motorPin;
    uint32_t _baudRate;
    
    // State variables
    LiDARState _state;
    LiDARDeviceInfo _deviceInfo;
    LiDARHealthStatus _healthStatus;
    LiDARScanData _scanData;
    
    // Obstacle detection
    ObstacleZone* _obstacleZones;
    uint8_t _obstacleZoneCount;
    
    // Configuration
    uint16_t _motorPWM;
    uint8_t _minQuality;
    bool _distanceFilterEnabled;
    uint16_t _filterMinDist;
    uint16_t _filterMaxDist;
    bool _medianFilterEnabled;
    uint8_t _medianWindowSize;
    
    // Statistics
    uint32_t _lastScanTime;
    uint32_t _scanCount;
    float _scanRate;
    
    // Communication
    uint8_t _rxBuffer[512];
    uint16_t _rxBufferPos;
    uint32_t _lastRxTime;
    
    // Internal methods
    bool _sendCommand(uint8_t cmd, const uint8_t* payload = nullptr, uint8_t payloadSize = 0);
    bool _waitResponseHeader(uint8_t* descriptor, uint32_t timeout = RPLIDAR_DEFAULT_TIMEOUT);
    bool _receiveData(uint8_t* data, uint16_t size, uint32_t timeout = RPLIDAR_DEFAULT_TIMEOUT);
    bool _getDeviceInfo();
    bool _getHealthStatus();
    void _processScanData();
    void _parseStandardMeasurement(uint8_t* data);
    void _parseExpressMeasurement(uint8_t* data);
    bool _validateMeasurement(const LiDARMeasurement& measurement) const;
    void _applyFilters(LiDARMeasurement& measurement);
    float _medianFilter(float* values, uint8_t count) const;
    void _clearRxBuffer();
    uint16_t _calculateChecksum(uint8_t* data, uint16_t size) const;
};

#endif // LIDAR_H
