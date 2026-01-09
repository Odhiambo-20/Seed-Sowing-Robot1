/**
 * @file DrillMotor.h
 * @brief Production-ready drill motor control system with advanced features
 * @version 3.0.0
 * @date 2026-01-09
 * 
 * Advanced Features:
 * - Closed-loop speed control with PID
 * - Current monitoring and overload protection
 * - Torque estimation and control
 * - Adaptive depth control with soil resistance feedback
 * - Thermal management and protection
 * - Stall detection and recovery
 * - Variable speed profiles for different soil types
 * - Emergency stop with controlled deceleration
 * - Encoder-based position feedback
 * - Statistical performance monitoring
 * - Predictive maintenance indicators
 * - Multi-stage drilling profiles
 * - Vibration analysis for obstacle detection
 * - Power consumption optimization
 * 
 * Hardware Support:
 * - Brushless DC motors (BLDC) 100-500W
 * - Brushed DC motors with encoder feedback
 * - PWM control (10-20kHz)
 * - Current sensing (±50A range)
 * - Temperature monitoring (NTC/PT100)
 * - Position encoders (quadrature/absolute)
 * 
 * Safety Features:
 * - Hardware overcurrent protection
 * - Thermal shutdown
 * - Watchdog timer
 * - Emergency stop circuitry
 * - Fault diagnostics
 * 
 * @author Agricultural Robotics Team
 * @copyright (c) 2026 Agricultural Robotics Inc.
 */

#ifndef DRILL_MOTOR_H
#define DRILL_MOTOR_H

#include <Arduino.h>
#include <stdint.h>
#include <PID_v1.h>

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

#define DRILL_PWM_FREQUENCY         15000    // PWM frequency in Hz
#define DRILL_MIN_SPEED             500      // Minimum RPM
#define DRILL_MAX_SPEED             3000     // Maximum RPM
#define DRILL_DEFAULT_SPEED         1500     // Default operating speed RPM
#define DRILL_RATED_POWER           300      // Rated power in Watts
#define DRILL_MAX_CURRENT           25.0f    // Maximum current in Amps
#define DRILL_STALL_CURRENT         20.0f    // Stall detection current in Amps
#define DRILL_THERMAL_LIMIT         85.0f    // Maximum temperature in °C
#define DRILL_THERMAL_WARNING       75.0f    // Warning temperature in °C
#define DRILL_ENCODER_PPR           1000     // Encoder pulses per revolution
#define DRILL_MAX_DEPTH_MM          200      // Maximum drill depth in mm
#define DRILL_DEPTH_TOLERANCE_MM    2        // Depth accuracy tolerance
#define DRILL_EMERGENCY_DECEL_TIME  500      // Emergency stop time in ms
#define DRILL_NORMAL_DECEL_TIME     2000     // Normal deceleration time in ms
#define DRILL_MAINTENANCE_HOURS     500      // Maintenance interval in hours

// PID tuning parameters
#define DRILL_PID_KP                2.0f
#define DRILL_PID_KI                0.5f
#define DRILL_PID_KD                0.1f
#define DRILL_PID_SAMPLE_TIME       50       // PID sample time in ms

// Fault codes
#define DRILL_FAULT_NONE            0x00
#define DRILL_FAULT_OVERCURRENT     0x01
#define DRILL_FAULT_OVERHEAT        0x02
#define DRILL_FAULT_STALL           0x04
#define DRILL_FAULT_ENCODER         0x08
#define DRILL_FAULT_EMERGENCY       0x10
#define DRILL_FAULT_COMMUNICATION   0x20
#define DRILL_FAULT_POWER           0x40
#define DRILL_FAULT_MECHANICAL      0x80

// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @brief Operating modes for drill motor
 */
enum class DrillMode : uint8_t {
    IDLE = 0,           ///< Motor stopped
    STARTING,           ///< Soft start in progress
    RUNNING,            ///< Normal operation
    DRILLING,           ///< Active drilling with depth control
    STOPPING,           ///< Controlled stop in progress
    EMERGENCY_STOP,     ///< Emergency stop activated
    FAULT,              ///< Fault condition detected
    MAINTENANCE         ///< Maintenance mode
};

/**
 * @brief Soil types for adaptive drilling
 */
enum class SoilType : uint8_t {
    SOFT = 0,           ///< Soft/Sandy soil
    MEDIUM,             ///< Normal agricultural soil
    HARD,               ///< Clay or compacted soil
    ROCKY,              ///< Rocky terrain
    WET,                ///< Wet/Muddy soil
    CUSTOM              ///< Custom profile
};

/**
 * @brief Drill bit types
 */
enum class DrillBitType : uint8_t {
    AUGER_50MM = 0,     ///< 50mm auger bit
    AUGER_75MM,         ///< 75mm auger bit
    AUGER_100MM,        ///< 100mm auger bit
    CONE,               ///< Cone-shaped bit
    FLAT,               ///< Flat drill bit
    CUSTOM              ///< Custom bit
};

/**
 * @brief Drilling profile structure
 */
struct DrillProfile {
    uint16_t initialSpeed;      ///< Initial drilling speed (RPM)
    uint16_t operatingSpeed;    ///< Normal operating speed (RPM)
    uint16_t finalSpeed;        ///< Speed at target depth (RPM)
    uint16_t maxTorque;         ///< Maximum allowed torque (N·cm)
    uint16_t retreatSpeed;      ///< Speed during drill extraction (RPM)
    float maxCurrent;           ///< Maximum current limit (A)
    uint16_t dwellTime;         ///< Dwell time at depth (ms)
};

/**
 * @brief Motor statistics structure
 */
struct DrillStatistics {
    uint32_t totalOperatingTime;    ///< Total operating time (seconds)
    uint32_t totalDrillCycles;      ///< Total number of drill cycles
    uint32_t successfulDrills;      ///< Successful drill operations
    uint32_t failedDrills;          ///< Failed drill operations
    float avgPowerConsumption;      ///< Average power consumption (W)
    float peakCurrent;              ///< Peak current recorded (A)
    float avgTemperature;           ///< Average operating temperature (°C)
    float peakTemperature;          ///< Peak temperature recorded (°C)
    uint16_t faultCount;            ///< Total fault occurrences
    uint32_t maintenanceCountdown;  ///< Hours until maintenance (hours)
};

/**
 * @brief Real-time telemetry data
 */
struct DrillTelemetry {
    uint16_t currentSpeed;          ///< Current motor speed (RPM)
    float currentCurrent;           ///< Current draw (A)
    float currentVoltage;           ///< Supply voltage (V)
    float currentPower;             ///< Power consumption (W)
    float temperature;              ///< Motor temperature (°C)
    int32_t position;               ///< Current position (encoder counts)
    float depth;                    ///< Current drill depth (mm)
    float torque;                   ///< Estimated torque (N·cm)
    uint16_t vibrationLevel;        ///< Vibration amplitude
    DrillMode mode;                 ///< Current operating mode
    uint8_t faultFlags;             ///< Active fault flags
};

// ============================================================================
// MAIN CLASS DEFINITION
// ============================================================================

/**
 * @class DrillMotor
 * @brief Production-ready drill motor controller with advanced features
 * 
 * This class provides comprehensive control of drill motors with:
 * - Closed-loop speed control
 * - Position/depth control
 * - Safety monitoring
 * - Adaptive drilling profiles
 * - Performance analytics
 * - Predictive maintenance
 */
class DrillMotor {
public:
    // ========================================================================
    // CONSTRUCTOR & DESTRUCTOR
    // ========================================================================
    
    /**
     * @brief Construct a new Drill Motor controller
     * 
     * @param pwmPin PWM output pin for motor control
     * @param dirPin Direction control pin
     * @param enablePin Enable pin for motor driver
     * @param currentSensePin Analog pin for current sensing
     * @param tempSensePin Analog pin for temperature sensing
     * @param encoderPinA Encoder channel A
     * @param encoderPinB Encoder channel B
     * @param depthEncoderPin Linear encoder for depth measurement
     */
    DrillMotor(uint8_t pwmPin, 
               uint8_t dirPin, 
               uint8_t enablePin,
               uint8_t currentSensePin,
               uint8_t tempSensePin,
               uint8_t encoderPinA,
               uint8_t encoderPinB,
               uint8_t depthEncoderPin);
    
    /**
     * @brief Destroy the Drill Motor object
     */
    ~DrillMotor();
    
    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    
    /**
     * @brief Initialize the drill motor controller
     * 
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool begin();
    
    /**
     * @brief Calibrate sensors and encoders
     * 
     * @return true if calibration successful
     * @return false if calibration failed
     */
    bool calibrate();
    
    /**
     * @brief Set drill bit type for profile optimization
     * 
     * @param bitType Type of drill bit installed
     * @param diameter Bit diameter in mm
     */
    void setDrillBit(DrillBitType bitType, float diameter);
    
    /**
     * @brief Configure PID controller parameters
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void configurePID(float kp, float ki, float kd);
    
    // ========================================================================
    // MOTOR CONTROL
    // ========================================================================
    
    /**
     * @brief Start motor with soft start profile
     * 
     * @param targetSpeed Target speed in RPM
     * @param rampTime Ramp-up time in milliseconds
     * @return true if start successful
     * @return false if start failed
     */
    bool start(uint16_t targetSpeed, uint16_t rampTime = 2000);
    
    /**
     * @brief Stop motor with controlled deceleration
     * 
     * @param rampTime Deceleration time in milliseconds (0 for emergency)
     * @return true if stop successful
     * @return false if stop failed
     */
    bool stop(uint16_t rampTime = DRILL_NORMAL_DECEL_TIME);
    
    /**
     * @brief Emergency stop - immediate motor shutdown
     */
    void emergencyStop();
    
    /**
     * @brief Set target speed
     * 
     * @param speed Target speed in RPM
     * @return true if speed set successfully
     * @return false if speed out of range or motor not running
     */
    bool setSpeed(uint16_t speed);
    
    /**
     * @brief Get current motor speed
     * 
     * @return uint16_t Current speed in RPM
     */
    uint16_t getSpeed() const;
    
    // ========================================================================
    // DRILLING OPERATIONS
    // ========================================================================
    
    /**
     * @brief Execute drilling operation to specified depth
     * 
     * @param targetDepth Target depth in millimeters
     * @param soilType Type of soil for profile selection
     * @return true if drilling successful
     * @return false if drilling failed
     */
    bool drillToDepth(float targetDepth, SoilType soilType = SoilType::MEDIUM);
    
    /**
     * @brief Execute drilling with custom profile
     * 
     * @param targetDepth Target depth in millimeters
     * @param profile Custom drilling profile
     * @return true if drilling successful
     * @return false if drilling failed
     */
    bool drillWithProfile(float targetDepth, const DrillProfile& profile);
    
    /**
     * @brief Retract drill from hole
     * 
     * @param speed Retraction speed in RPM (0 = use profile speed)
     * @return true if retraction successful
     * @return false if retraction failed
     */
    bool retract(uint16_t speed = 0);
    
    /**
     * @brief Get current drill depth
     * 
     * @return float Current depth in millimeters
     */
    float getDepth() const;
    
    /**
     * @brief Set home position (surface level)
     * 
     * @return true if home position set
     * @return false if failed
     */
    bool setHome();
    
    /**
     * @brief Check if target depth reached
     * 
     * @return true if at target depth
     * @return false if not at target depth
     */
    bool isAtTargetDepth() const;
    
    // ========================================================================
    // MONITORING & DIAGNOSTICS
    // ========================================================================
    
    /**
     * @brief Update motor controller (call in main loop)
     * Must be called at least every 50ms for proper operation
     */
    void update();
    
    /**
     * @brief Get real-time telemetry data
     * 
     * @return DrillTelemetry Current telemetry data
     */
    DrillTelemetry getTelemetry() const;
    
    /**
     * @brief Get operating statistics
     * 
     * @return DrillStatistics Statistical data
     */
    DrillStatistics getStatistics() const;
    
    /**
     * @brief Get current operating mode
     * 
     * @return DrillMode Current mode
     */
    DrillMode getMode() const { return currentMode; }
    
    /**
     * @brief Check if motor is running
     * 
     * @return true if motor is running
     * @return false if motor is stopped
     */
    bool isRunning() const;
    
    /**
     * @brief Check if motor is in fault state
     * 
     * @return true if fault detected
     * @return false if no fault
     */
    bool hasFault() const { return faultFlags != DRILL_FAULT_NONE; }
    
    /**
     * @brief Get active fault flags
     * 
     * @return uint8_t Fault flags bitmask
     */
    uint8_t getFaultFlags() const { return faultFlags; }
    
    /**
     * @brief Get fault description string
     * 
     * @param faultCode Fault code to describe
     * @return const char* Human-readable fault description
     */
    static const char* getFaultDescription(uint8_t faultCode);
    
    /**
     * @brief Clear fault conditions
     * 
     * @return true if faults cleared
     * @return false if faults cannot be cleared
     */
    bool clearFaults();
    
    /**
     * @brief Check if maintenance is due
     * 
     * @return true if maintenance required
     * @return false if no maintenance needed
     */
    bool isMaintenanceDue() const;
    
    /**
     * @brief Reset maintenance counter
     */
    void resetMaintenanceCounter();
    
    // ========================================================================
    // ADVANCED FEATURES
    // ========================================================================
    
    /**
     * @brief Enable/disable thermal protection
     * 
     * @param enable True to enable, false to disable
     */
    void setThermalProtection(bool enable);
    
    /**
     * @brief Enable/disable stall detection
     * 
     * @param enable True to enable, false to disable
     */
    void setStallDetection(bool enable);
    
    /**
     * @brief Set current limit
     * 
     * @param currentLimit Maximum current in Amps
     * @return true if limit set successfully
     * @return false if limit out of range
     */
    bool setCurrentLimit(float currentLimit);
    
    /**
     * @brief Get estimated torque
     * 
     * @return float Estimated torque in N·cm
     */
    float getTorque() const;
    
    /**
     * @brief Get power consumption
     * 
     * @return float Power consumption in Watts
     */
    float getPower() const;
    
    /**
     * @brief Get motor efficiency
     * 
     * @return float Efficiency percentage (0-100)
     */
    float getEfficiency() const;
    
    /**
     * @brief Perform self-test
     * 
     * @return true if self-test passed
     * @return false if self-test failed
     */
    bool selfTest();
    
    /**
     * @brief Get predefined drilling profile for soil type
     * 
     * @param soilType Type of soil
     * @return DrillProfile Optimized drilling profile
     */
    static DrillProfile getProfileForSoil(SoilType soilType);
    
    /**
     * @brief Load drilling profile from EEPROM
     * 
     * @param profileId Profile ID (0-9)
     * @return true if profile loaded
     * @return false if profile not found
     */
    bool loadProfile(uint8_t profileId);
    
    /**
     * @brief Save current profile to EEPROM
     * 
     * @param profileId Profile ID (0-9)
     * @return true if profile saved
     * @return false if save failed
     */
    bool saveProfile(uint8_t profileId);

private:
    // ========================================================================
    // PRIVATE MEMBER VARIABLES
    // ========================================================================
    
    // Pin assignments
    uint8_t pwmPin;
    uint8_t dirPin;
    uint8_t enablePin;
    uint8_t currentSensePin;
    uint8_t tempSensePin;
    uint8_t encoderPinA;
    uint8_t encoderPinB;
    uint8_t depthEncoderPin;
    
    // State variables
    DrillMode currentMode;
    DrillMode previousMode;
    uint8_t faultFlags;
    bool enabled;
    bool thermalProtectionEnabled;
    bool stallDetectionEnabled;
    
    // Speed control
    uint16_t targetSpeed;
    uint16_t currentSpeed;
    uint16_t minSpeed;
    uint16_t maxSpeed;
    
    // Position control
    volatile int32_t encoderPosition;
    int32_t targetPosition;
    int32_t homePosition;
    float currentDepth;
    float targetDepth;
    
    // PID controller
    double pidInput;
    double pidOutput;
    double pidSetpoint;
    PID* speedPID;
    
    // Current profile
    DrillProfile activeProfile;
    DrillBitType currentBitType;
    float bitDiameter;
    
    // Sensor readings
    float currentCurrent;
    float currentVoltage;
    float currentTemperature;
    float currentTorque;
    uint16_t currentVibration;
    
    // Safety limits
    float maxCurrentLimit;
    float stallCurrentThreshold;
    float thermalShutdownTemp;
    float thermalWarningTemp;
    
    // Timing
    unsigned long lastUpdateTime;
    unsigned long operationStartTime;
    unsigned long lastFaultCheckTime;
    unsigned long softStartTime;
    unsigned long decelerationStartTime;
    
    // Statistics
    DrillStatistics stats;
    
    // Buffers for filtering
    float currentBuffer[10];
    uint8_t currentBufferIndex;
    float tempBuffer[5];
    uint8_t tempBufferIndex;
    
    // ========================================================================
    // PRIVATE METHODS
    // ========================================================================
    
    // Hardware interface
    void setPWM(uint16_t duty);
    void setDirection(bool forward);
    void enableMotor(bool enable);
    float readCurrent();
    float readTemperature();
    float readVoltage();
    void updateEncoder();
    float readDepthSensor();
    
    // Control algorithms
    void updateSpeedControl();
    void updatePositionControl();
    void applySoftStart();
    void applyDeceleration();
    float calculateTorque();
    
    // Safety monitoring
    void checkFaults();
    void checkOvercurrent();
    void checkOverheat();
    void checkStall();
    void checkEncoder();
    void handleFault(uint8_t faultCode);
    
    // Profile management
    void applyProfile(const DrillProfile& profile);
    void adaptProfileToLoad();
    
    // Statistics
    void updateStatistics();
    void logOperation(bool success);
    
    // Interrupt handlers (must be static)
    static void encoderISR_A();
    static void encoderISR_B();
    
    // Helper functions
    float filterCurrent(float rawCurrent);
    float filterTemperature(float rawTemp);
    bool validateSpeed(uint16_t speed);
    bool validateDepth(float depth);
};

#endif // DRILL_MOTOR_H
