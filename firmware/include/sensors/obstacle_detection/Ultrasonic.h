
/**
 * @file Ultrasonic.h
 * @brief Production-ready Ultrasonic Sensor Array driver (HC-SR04/JSN-SR04T)
 * @version 2.0.0
 * @date 2026-01-06
 * 
 * Supports:
 * - Multiple ultrasonic sensors (up to 8 sensors)
 * - HC-SR04 and JSN-SR04T (waterproof) sensors
 * - Distance measurement 2cm - 400cm
 * - Configurable update rates
 * - Multi-sensor fusion for obstacle detection
 * - Zone-based obstacle detection
 * - Median filtering for noise reduction
 * - Temperature compensation
 * 
 * Hardware Connections:
 * - Each sensor requires 2 pins: Trigger and Echo
 * - VCC: 5V
 * - GND: Ground
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// ============================================================================
// CONSTANTS
// ============================================================================

#define ULTRASONIC_MAX_SENSORS      8      // Maximum number of sensors
#define ULTRASONIC_MIN_DISTANCE     20     // Minimum distance in mm
#define ULTRASONIC_MAX_DISTANCE     4000   // Maximum distance in mm
#define ULTRASONIC_TIMEOUT_US       30000  // Timeout in microseconds (30ms)
#define ULTRASONIC_TRIGGER_DURATION 10     // Trigger pulse duration in microseconds
#define ULTRASONIC_SPEED_OF_SOUND   343.0f // Speed of sound in m/s at 20°C
#define ULTRASONIC_DEFAULT_TEMP     20.0f  // Default temperature in Celsius

// Sensor positions (for array configuration)
#define SENSOR_POSITION_FRONT_LEFT    0
#define SENSOR_POSITION_FRONT_CENTER  1
#define SENSOR_POSITION_FRONT_RIGHT   2
#define SENSOR_POSITION_LEFT          3
#define SENSOR_POSITION_RIGHT         4
#define SENSOR_POSITION_REAR_LEFT     5
#define SENSOR_POSITION_REAR_CENTER   6
#define SENSOR_POSITION_REAR_RIGHT    7

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @struct UltrasonicConfig
 * @brief Configuration for a single ultrasonic sensor
 */
struct UltrasonicConfig {
    uint8_t triggerPin;          // Trigger pin number
    uint8_t echoPin;             // Echo pin number
    uint8_t position;            // Sensor position (0-7)
    float mountingAngle;         // Mounting angle in degrees (0=forward)
    float xOffset;               // X offset from robot center (mm)
    float yOffset;               // Y offset from robot center (mm)
    uint16_t minDistance;        // Minimum valid distance (mm)
    uint16_t maxDistance;        // Maximum valid distance (mm)
    bool enabled;                // Is sensor enabled
};

/**
 * @struct UltrasonicMeasurement
 * @brief Single distance measurement
 */
struct UltrasonicMeasurement {
    uint8_t sensorId;            // Sensor identifier
    uint16_t distance;           // Distance in millimeters
    uint32_t timestamp;          // Measurement timestamp
    bool valid;                  // Is measurement valid
    float signalQuality;         // Signal quality (0.0-1.0)
    uint16_t rawPulseWidth;      // Raw pulse width in microseconds
};

/**
 * @struct ObstacleInfo
 * @brief Detected obstacle information
 */
struct ObstacleInfo {
    bool detected;               // Obstacle detected
    uint16_t distance;           // Distance to obstacle (mm)
    float angle;                 // Angle to obstacle (degrees)
    uint8_t sensorId;            // Sensor that detected obstacle
    uint32_t timestamp;          // Detection timestamp
    uint8_t confidence;          // Detection confidence (0-100)
};

/**
 * @struct SensorStatistics
 * @brief Statistics for a single sensor
 */
struct SensorStatistics {
    uint32_t measurementCount;   // Total measurements
    uint32_t validCount;         // Valid measurements
    uint32_t invalidCount;       // Invalid measurements
    uint16_t minMeasured;        // Minimum distance measured
    uint16_t maxMeasured;        // Maximum distance measured
    float averageDistance;       // Average distance
    uint32_t lastUpdateTime;     // Last update timestamp
};

/**
 * @enum SensorState
 * @brief Sensor operational state
 */
enum SensorState {
    SENSOR_STATE_UNINITIALIZED,
    SENSOR_STATE_IDLE,
    SENSOR_STATE_MEASURING,
    SENSOR_STATE_ERROR
};

// ============================================================================
// ULTRASONIC SENSOR ARRAY CLASS
// ============================================================================

class Ultrasonic {
public:
    /**
     * @brief Constructor
     * @param numSensors Number of sensors in array (1-8)
     */
    Ultrasonic(uint8_t numSensors = 1);
    
    /**
     * @brief Destructor
     */
    ~Ultrasonic();
    
    // Initialization
    /**
     * @brief Initialize sensor array
     * @param configs Array of sensor configurations
     * @param configCount Number of configurations
     * @return true if initialization successful
     */
    bool begin(const UltrasonicConfig* configs, uint8_t configCount);
    
    /**
     * @brief Add single sensor to array
     * @param triggerPin Trigger pin
     * @param echoPin Echo pin
     * @param position Sensor position (0-7)
     * @param mountingAngle Mounting angle in degrees
     * @return Sensor ID (0xFF if failed)
     */
    uint8_t addSensor(uint8_t triggerPin, uint8_t echoPin, 
                      uint8_t position = 0, float mountingAngle = 0.0f);
    
    /**
     * @brief Configure sensor
     * @param sensorId Sensor ID
     * @param config Configuration structure
     * @return true if successful
     */
    bool configureSensor(uint8_t sensorId, const UltrasonicConfig& config);
    
    /**
     * @brief Enable/disable sensor
     * @param sensorId Sensor ID
     * @param enable Enable state
     */
    void enableSensor(uint8_t sensorId, bool enable);
    
    // Measurement
    /**
     * @brief Update all sensors - call in main loop
     */
    void update();
    
    /**
     * @brief Measure distance from specific sensor
     * @param sensorId Sensor ID
     * @return Distance in millimeters (0 if invalid)
     */
    uint16_t measureDistance(uint8_t sensorId);
    
    /**
     * @brief Get last measurement from sensor
     * @param sensorId Sensor ID
     * @return Measurement structure
     */
    UltrasonicMeasurement getMeasurement(uint8_t sensorId) const;
    
    /**
     * @brief Get all measurements
     * @param measurements Output array
     * @param maxCount Maximum number to return
     * @return Number of measurements returned
     */
    uint8_t getAllMeasurements(UltrasonicMeasurement* measurements, 
                               uint8_t maxCount) const;
    
    /**
     * @brief Get average distance from multiple sensors
     * @param sensorIds Array of sensor IDs
     * @param count Number of sensors
     * @return Average distance in mm
     */
    uint16_t getAverageDistance(const uint8_t* sensorIds, uint8_t count) const;
    
    /**
     * @brief Get minimum distance from all sensors
     * @return Minimum distance in mm (0 if none valid)
     */
    uint16_t getMinimumDistance() const;
    
    /**
     * @brief Get minimum distance from specific sensors
     * @param sensorIds Array of sensor IDs
     * @param count Number of sensors
     * @return Minimum distance in mm
     */
    uint16_t getMinimumDistance(const uint8_t* sensorIds, uint8_t count) const;
    
    // Obstacle Detection
    /**
     * @brief Check for obstacles in front
     * @param threshold Distance threshold in mm
     * @return true if obstacle detected
     */
    bool detectFrontObstacle(uint16_t threshold) const;
    
    /**
     * @brief Check for obstacles on sides
     * @param leftThreshold Left side threshold in mm
     * @param rightThreshold Right side threshold in mm
     * @return true if obstacle detected
     */
    bool detectSideObstacles(uint16_t leftThreshold, uint16_t rightThreshold) const;
    
    /**
     * @brief Get detailed obstacle information
     * @return ObstacleInfo structure
     */
    ObstacleInfo getClosestObstacle() const;
    
    /**
     * @brief Check if path is clear
     * @param minDistance Minimum safe distance in mm
     * @return true if path is clear
     */
    bool isPathClear(uint16_t minDistance) const;
    
    /**
     * @brief Get obstacle in specific direction
     * @param angle Direction angle in degrees (0=front)
     * @param angleRange Tolerance range (±degrees)
     * @param threshold Distance threshold in mm
     * @return true if obstacle detected
     */
    bool detectObstacleInDirection(float angle, float angleRange, 
                                   uint16_t threshold) const;
    
    // Configuration
    /**
     * @brief Set update rate
     * @param rateHz Update rate in Hz (1-50)
     */
    void setUpdateRate(uint8_t rateHz);
    
    /**
     * @brief Enable/disable median filtering
     * @param enable Enable filtering
     * @param windowSize Filter window size (3, 5, or 7)
     */
    void setMedianFilter(bool enable, uint8_t windowSize = 5);
    
    /**
     * @brief Enable/disable moving average filter
     * @param enable Enable filtering
     * @param windowSize Window size (2-10)
     */
    void setMovingAverageFilter(bool enable, uint8_t windowSize = 5);
    
    /**
     * @brief Set temperature for speed of sound compensation
     * @param temperature Temperature in Celsius
     */
    void setTemperature(float temperature);
    
    /**
     * @brief Set distance thresholds for sensor
     * @param sensorId Sensor ID
     * @param minDist Minimum valid distance (mm)
     * @param maxDist Maximum valid distance (mm)
     */
    void setDistanceThresholds(uint8_t sensorId, uint16_t minDist, 
                              uint16_t maxDist);
    
    // Diagnostics
    /**
     * @brief Get sensor state
     * @param sensorId Sensor ID
     * @return Sensor state
     */
    SensorState getSensorState(uint8_t sensorId) const;
    
    /**
     * @brief Get sensor statistics
     * @param sensorId Sensor ID
     * @return Statistics structure
     */
    SensorStatistics getStatistics(uint8_t sensorId) const;
    
    /**
     * @brief Reset statistics for sensor
     * @param sensorId Sensor ID
     */
    void resetStatistics(uint8_t sensorId);
    
    /**
     * @brief Check if sensor is working
     * @param sensorId Sensor ID
     * @return true if sensor responding normally
     */
    bool isSensorHealthy(uint8_t sensorId) const;
    
    /**
     * @brief Get number of configured sensors
     * @return Sensor count
     */
    uint8_t getSensorCount() const;
    
    /**
     * @brief Get sensor configuration
     * @param sensorId Sensor ID
     * @return Pointer to configuration (nullptr if invalid)
     */
    const UltrasonicConfig* getSensorConfig(uint8_t sensorId) const;
    
    /**
     * @brief Run self-test on all sensors
     * @return true if all sensors pass
     */
    bool selfTest();

private:
    // Sensor data
    UltrasonicConfig* _configs;
    UltrasonicMeasurement* _measurements;
    SensorStatistics* _statistics;
    SensorState* _states;
    uint8_t _numSensors;
    uint8_t _configuredSensors;
    
    // Filtering
    bool _medianFilterEnabled;
    uint8_t _medianWindowSize;
    bool _movingAverageEnabled;
    uint8_t _movingAverageSize;
    uint16_t** _filterBuffers;
    uint8_t* _filterIndices;
    
    // Configuration
    float _temperature;
    float _speedOfSound;
    uint8_t _updateRate;
    uint32_t _updateInterval;
    uint32_t _lastUpdateTime;
    
    // Internal methods
    uint16_t _measureSingleDistance(uint8_t sensorId);
    bool _triggerMeasurement(uint8_t sensorId);
    uint32_t _measurePulseWidth(uint8_t echoPin, uint32_t timeout);
    uint16_t _pulseWidthToDistance(uint32_t pulseWidth) const;
    bool _validateMeasurement(uint8_t sensorId, uint16_t distance) const;
    void _applyFilters(uint8_t sensorId, uint16_t& distance);
    uint16_t _medianFilter(uint8_t sensorId, uint16_t newValue);
    uint16_t _movingAverageFilter(uint8_t sensorId, uint16_t newValue);
    void _updateStatistics(uint8_t sensorId, const UltrasonicMeasurement& measurement);
    void _calculateSpeedOfSound();
    uint8_t _findSensorByPosition(uint8_t position) const;
    float _calculateObstacleAngle(uint8_t sensorId) const;
};

#endif // ULTRASONIC_H
