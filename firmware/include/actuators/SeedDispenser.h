/**
 * @file SeedDispenser.h
 * @brief Production-ready precision seed dispensing system
 * @version 3.0.0
 * @date 2026-01-09
 * 
 * Advanced Features:
 * - Vacuum-based seed singulation
 * - Precision seed metering (±1 seed accuracy)
 * - Real-time seed detection and verification
 * - Adaptive dispensing for different seed types
 * - Multi-crop support (beans, maize, wheat, etc.)
 * - Seed flow monitoring and blockage detection
 * - Automatic seed rate adjustment
 * - Skip compensation and double seed elimination
 * - Statistical quality control
 * - Predictive seed level monitoring
 * - Environmental compensation (humidity, temperature)
 * - Air pressure regulation and monitoring
 * - Seed damage prevention
 * - High-speed operation (up to 10 seeds/second)
 * 
 * Hardware Support:
 * - Vacuum disc metering systems
 * - Mechanical finger-pick systems
 * - Pneumatic singulation
 * - IR break-beam sensors (dual redundant)
 * - Optical seed counters
 * - Pressure sensors (0-100 kPa)
 * - Servo/stepper motor control
 * - Solenoid valve control
 * 
 * Seed Type Support:
 * - Beans (5-20mm)
 * - Maize/Corn (7-12mm)
 * - Wheat (2-4mm)
 * - Sorghum (3-5mm)
 * - Sunflower (8-15mm)
 * - Custom seed profiles
 * 
 * @author Agricultural Robotics Team
 * @copyright (c) 2026 Agricultural Robotics Inc.
 */

#ifndef SEED_DISPENSER_H
#define SEED_DISPENSER_H

#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

#define SEED_MAX_DISPENSE_RATE      10       // Maximum seeds per second
#define SEED_MIN_DISPENSE_INTERVAL  100      // Minimum interval between seeds (ms)
#define SEED_VACUUM_PRESSURE_MIN    20.0f    // Minimum vacuum pressure (kPa)
#define SEED_VACUUM_PRESSURE_MAX    80.0f    // Maximum vacuum pressure (kPa)
#define SEED_VACUUM_PRESSURE_NORMAL 50.0f    // Normal operating pressure (kPa)
#define SEED_HOPPER_CAPACITY        1000     // Hopper capacity in seeds
#define SEED_LOW_LEVEL_THRESHOLD    100      // Low seed warning threshold
#define SEED_CRITICAL_LEVEL         20       // Critical seed level
#define SEED_SENSOR_TIMEOUT         5000     // Sensor timeout in ms
#define SEED_BLOCKAGE_TIMEOUT       3000     // Blockage detection timeout
#define SEED_VERIFICATION_WINDOW    500      // Seed verification time window (ms)
#define SEED_DOUBLE_DETECT_WINDOW   50       // Double seed detection window (ms)
#define SEED_QUALITY_SAMPLE_SIZE    100      // Sample size for quality metrics
#define SEED_MAX_SKIP_COUNT         5        // Max consecutive skips before fault
#define SEED_MAX_DOUBLE_COUNT       3        // Max consecutive doubles before fault
#define SEED_CALIBRATION_SAMPLES    50       // Calibration sample count
#define SEED_TEMPERATURE_NOMINAL    25.0f    // Nominal temperature (°C)
#define SEED_HUMIDITY_NOMINAL       50.0f    // Nominal humidity (%)

// Fault codes
#define SEED_FAULT_NONE             0x00
#define SEED_FAULT_BLOCKAGE         0x01
#define SEED_FAULT_SENSOR           0x02
#define SEED_FAULT_VACUUM           0x04
#define SEED_FAULT_MOTOR            0x08
#define SEED_FAULT_EMPTY            0x10
#define SEED_FAULT_DOUBLE           0x20
#define SEED_FAULT_SKIP             0x40
#define SEED_FAULT_TIMING           0x80

// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @brief Dispenser operating modes
 */
enum class DispenserMode : uint8_t {
    IDLE = 0,           ///< Dispenser idle
    PRIMING,            ///< Priming dispenser
    READY,              ///< Ready to dispense
    DISPENSING,         ///< Active dispensing
    VERIFYING,          ///< Seed verification in progress
    PAUSED,             ///< Temporarily paused
    FAULT,              ///< Fault condition
    CALIBRATION,        ///< Calibration mode
    MAINTENANCE         ///< Maintenance mode
};

/**
 * @brief Seed crop types with characteristics
 */
enum class SeedType : uint8_t {
    BEANS = 0,          ///< Bean seeds (large, 10-15mm)
    MAIZE,              ///< Maize/Corn seeds (medium-large, 8-12mm)
    WHEAT,              ///< Wheat seeds (small, 2-4mm)
    SORGHUM,            ///< Sorghum seeds (small-medium, 3-5mm)
    SUNFLOWER,          ///< Sunflower seeds (large, 10-15mm)
    COTTON,             ///< Cotton seeds (medium, 8-10mm)
    SOYBEAN,            ///< Soybean seeds (medium, 6-9mm)
    CUSTOM              ///< Custom seed profile
};

/**
 * @brief Dispensing mechanism types
 */
enum class MechanismType : uint8_t {
    VACUUM_DISC = 0,    ///< Vacuum disc metering
    FINGER_PICK,        ///< Mechanical finger picker
    PNEUMATIC,          ///< Pneumatic singulation
    BELT_METERING,      ///< Belt-based metering
    CUSTOM              ///< Custom mechanism
};

/**
 * @brief Seed size classification
 */
enum class SeedSize : uint8_t {
    EXTRA_SMALL = 0,    ///< < 3mm
    SMALL,              ///< 3-5mm
    MEDIUM,             ///< 5-8mm
    LARGE,              ///< 8-12mm
    EXTRA_LARGE,        ///< > 12mm
    CUSTOM              ///< Custom size
};

/**
 * @brief Seed characteristics profile
 */
struct SeedProfile {
    SeedType type;                  ///< Seed type
    SeedSize size;                  ///< Seed size classification
    float diameter;                 ///< Average diameter (mm)
    float length;                   ///< Average length (mm)
    float weight;                   ///< Average weight (mg)
    uint16_t optimalVacuum;         ///< Optimal vacuum pressure (kPa)
    uint16_t optimalSpeed;          ///< Optimal disc/motor speed (RPM)
    uint16_t releaseDelay;          ///< Release timing delay (ms)
    uint16_t verificationDelay;     ///< Verification window (ms)
    float singulationAccuracy;      ///< Expected singulation accuracy (%)
    char name[32];                  ///< Profile name
};

/**
 * @brief Dispensing event structure
 */
struct SeedEvent {
    uint32_t timestamp;             ///< Event timestamp (ms)
    bool detected;                  ///< Seed detected
    bool verified;                  ///< Seed verified
    bool isDouble;                  ///< Double seed detected
    bool isSkip;                    ///< Skip detected
    uint16_t transitTime;           ///< Seed transit time (ms)
    float signalStrength;           ///< Sensor signal strength
};

/**
 * @brief Quality metrics structure
 */
struct DispensingQuality {
    uint32_t totalDispensed;        ///< Total seeds dispensed
    uint32_t successfulDispense;    ///< Successful dispenses
    uint32_t doubleSeeds;           ///< Double seed occurrences
    uint32_t skips;                 ///< Skip occurrences
    uint32_t blockages;             ///< Blockage occurrences
    float singulationRate;          ///< Singulation accuracy (%)
    float skipRate;                 ///< Skip rate (%)
    float doubleRate;               ///< Double seed rate (%)
    float overallAccuracy;          ///< Overall accuracy (%)
    uint16_t consecutiveSuccess;    ///< Consecutive successful dispenses
    uint16_t consecutiveFailures;   ///< Consecutive failures
};

/**
 * @brief Dispenser telemetry data
 */
struct DispenserTelemetry {
    DispenserMode mode;             ///< Current operating mode
    uint16_t currentSpeed;          ///< Motor/disc speed (RPM)
    float vacuumPressure;           ///< Current vacuum pressure (kPa)
    uint16_t seedsInHopper;         ///< Estimated seeds remaining
    float temperature;              ///< System temperature (°C)
    float humidity;                 ///< Environmental humidity (%)
    uint32_t seedsDispensed;        ///< Seeds dispensed this session
    float dispensingRate;           ///< Current rate (seeds/sec)
    uint8_t faultFlags;             ///< Active fault flags
    bool sensor1Status;             ///< Primary sensor status
    bool sensor2Status;             ///< Secondary sensor status
    bool vacuumStatus;              ///< Vacuum system status
    bool motorStatus;               ///< Motor status
};

/**
 * @brief Calibration data structure
 */
struct CalibrationData {
    uint16_t sensor1Baseline;       ///< Sensor 1 baseline reading
    uint16_t sensor2Baseline;       ///< Sensor 2 baseline reading
    uint16_t sensor1Threshold;      ///< Sensor 1 detection threshold
    uint16_t sensor2Threshold;      ///< Sensor 2 detection threshold
    float vacuumCalibration;        ///< Vacuum sensor calibration
    uint16_t timingOffset;          ///< Timing calibration offset (ms)
    uint32_t calibrationDate;       ///< Calibration timestamp
    bool isValid;                   ///< Calibration validity flag
};

// ============================================================================
// MAIN CLASS DEFINITION
// ============================================================================

/**
 * @class SeedDispenser
 * @brief Production-ready precision seed dispensing system
 * 
 * This class provides comprehensive seed dispensing control with:
 * - High-precision seed singulation
 * - Real-time verification
 * - Adaptive control algorithms
 * - Quality monitoring
 * - Fault detection and recovery
 * - Multi-seed-type support
 */
class SeedDispenser {
public:
    // ========================================================================
    // CONSTRUCTOR & DESTRUCTOR
    // ========================================================================
    
    /**
     * @brief Construct a new Seed Dispenser controller
     * 
     * @param motorPin Motor control pin (PWM or step)
     * @param dirPin Direction control pin
     * @param enablePin Enable pin
     * @param sensor1Pin Primary IR sensor pin
     * @param sensor2Pin Secondary IR sensor pin
     * @param vacuumValvePin Vacuum solenoid valve pin
     * @param vacuumSensorPin Vacuum pressure sensor (analog)
     * @param levelSensorPin Seed level sensor pin
     * @param tempSensorPin Temperature sensor pin (analog)
     * @param humidityPin Humidity sensor pin
     */
    SeedDispenser(uint8_t motorPin,
                  uint8_t dirPin,
                  uint8_t enablePin,
                  uint8_t sensor1Pin,
                  uint8_t sensor2Pin,
                  uint8_t vacuumValvePin,
                  uint8_t vacuumSensorPin,
                  uint8_t levelSensorPin,
                  uint8_t tempSensorPin,
                  uint8_t humidityPin);
    
    /**
     * @brief Destroy the Seed Dispenser object
     */
    ~SeedDispenser();
    
    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    
    /**
     * @brief Initialize the seed dispenser
     * 
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool begin();
    
    /**
     * @brief Calibrate sensors and system
     * 
     * @param seedType Type of seed for calibration
     * @return true if calibration successful
     * @return false if calibration failed
     */
    bool calibrate(SeedType seedType);
    
    /**
     * @brief Load seed profile
     * 
     * @param profile Seed profile to load
     * @return true if profile loaded successfully
     * @return false if profile invalid
     */
    bool loadSeedProfile(const SeedProfile& profile);
    
    /**
     * @brief Get predefined profile for seed type
     * 
     * @param seedType Type of seed
     * @return SeedProfile Optimized profile for seed type
     */
    static SeedProfile getProfileForSeed(SeedType seedType);
    
    /**
     * @brief Set mechanism type
     * 
     * @param type Mechanism type
     */
    void setMechanismType(MechanismType type);
    
    // ========================================================================
    // DISPENSING OPERATIONS
    // ========================================================================
    
    /**
     * @brief Prime the dispenser system
     * 
     * @return true if priming successful
     * @return false if priming failed
     */
    bool prime();
    
    /**
     * @brief Dispense a single seed
     * 
     * @param verifyDispense Wait for verification
     * @return true if seed dispensed successfully
     * @return false if dispense failed
     */
    bool dispenseSeed(bool verifyDispense = true);
    
    /**
     * @brief Dispense multiple seeds
     * 
     * @param count Number of seeds to dispense
     * @param interval Interval between seeds (ms)
     * @return uint16_t Number of seeds successfully dispensed
     */
    uint16_t dispenseSeeds(uint16_t count, uint16_t interval);
    
    /**
     * @brief Start continuous dispensing mode
     * 
     * @param rate Target rate (seeds per second)
     * @return true if continuous mode started
     * @return false if failed to start
     */
    bool startContinuous(float rate);
    
    /**
     * @brief Stop continuous dispensing
     */
    void stopContinuous();
    
    /**
     * @brief Pause dispensing operation
     */
    void pause();
    
    /**
     * @brief Resume paused operation
     * 
     * @return true if resumed successfully
     * @return false if failed to resume
     */
    bool resume();
    
    // ========================================================================
    // SEED VERIFICATION
    // ========================================================================
    
    /**
     * @brief Wait for seed to be detected
     * 
     * @param timeout Timeout in milliseconds
     * @return true if seed detected
     * @return false if timeout or no seed
     */
    bool waitForSeed(uint16_t timeout = SEED_VERIFICATION_WINDOW);
    
    /**
     * @brief Check if seed was dispensed successfully
     * 
     * @return true if seed detected by sensors
     * @return false if no seed detected
     */
    bool verifySeedDispensed();
    
    /**
     * @brief Check for double seed condition
     * 
     * @return true if double seed detected
     * @return false if single seed or no seed
     */
    bool detectDoubleSeed();
    
    /**
     * @brief Get last seed event data
     * 
     * @return SeedEvent Last dispensing event
     */
    SeedEvent getLastEvent() const { return lastEvent; }
    
    // ========================================================================
    // VACUUM SYSTEM CONTROL
    // ========================================================================
    
    /**
     * @brief Enable/disable vacuum system
     * 
     * @param enable True to enable, false to disable
     */
    void setVacuum(bool enable);
    
    /**
     * @brief Set vacuum pressure
     * 
     * @param pressure Target pressure in kPa
     * @return true if pressure set successfully
     * @return false if pressure out of range
     */
    bool setVacuumPressure(float pressure);
    
    /**
     * @brief Get current vacuum pressure
     * 
     * @return float Current pressure in kPa
     */
    float getVacuumPressure() const;
    
    /**
     * @brief Check vacuum system status
     * 
     * @return true if vacuum system operational
     * @return false if vacuum system fault
     */
    bool isVacuumOK() const;
    
    // ========================================================================
    // MOTOR/DISC CONTROL
    // ========================================================================
    
    /**
     * @brief Set motor speed
     * 
     * @param speed Speed in RPM
     * @return true if speed set successfully
     * @return false if speed out of range
     */
    bool setSpeed(uint16_t speed);
    
    /**
     * @brief Get current motor speed
     * 
     * @return uint16_t Current speed in RPM
     */
    uint16_t getSpeed() const { return currentSpeed; }
    
    /**
     * @brief Enable/disable motor
     * 
     * @param enable True to enable, false to disable
     */
    void enableMotor(bool enable);
    
    // ========================================================================
    // SEED LEVEL MONITORING
    // ========================================================================
    
    /**
     * @brief Get estimated seeds remaining in hopper
     * 
     * @return uint16_t Estimated seed count
     */
    uint16_t getSeedsRemaining() const;
    
    /**
     * @brief Check if seed level is low
     * 
     * @return true if low seed level
     * @return false if adequate seeds
     */
    bool isLowSeedLevel() const;
    
    /**
     * @brief Check if hopper is empty
     * 
     * @return true if hopper empty
     * @return false if seeds present
     */
    bool isEmpty() const;
    
    /**
     * @brief Reset seed count (after refilling)
     * 
     * @param seedCount Number of seeds added
     */
    void resetSeedCount(uint16_t seedCount = SEED_HOPPER_CAPACITY);
    
    // ========================================================================
    // MONITORING & DIAGNOSTICS
    // ========================================================================
    
    /**
     * @brief Update dispenser controller (call in main loop)
     * Must be called frequently (at least every 10ms)
     */
    void update();
    
    /**
     * @brief Get real-time telemetry
     * 
     * @return DispenserTelemetry Current telemetry data
     */
    DispenserTelemetry getTelemetry() const;
    
    /**
     * @brief Get dispensing quality metrics
     * 
     * @return DispensingQuality Quality statistics
     */
    DispensingQuality getQualityMetrics() const;
    
    /**
     * @brief Get current operating mode
     * 
     * @return DispenserMode Current mode
     */
    DispenserMode getMode() const { return currentMode; }
    
    /**
     * @brief Check if dispenser is ready
     * 
     * @return true if ready to dispense
     * @return false if not ready
     */
    bool isReady() const;
    
    /**
     * @brief Check if dispensing in progress
     * 
     * @return true if actively dispensing
     * @return false if idle
     */
    bool isDispensing() const;
    
    /**
     * @brief Check for fault condition
     * 
     * @return true if fault detected
     * @return false if no fault
     */
    bool hasFault() const { return faultFlags != SEED_FAULT_NONE; }
    
    /**
     * @brief Get active fault flags
     * 
     * @return uint8_t Fault flags bitmask
     */
    uint8_t getFaultFlags() const { return faultFlags; }
    
    /**
     * @brief Get fault description
     * 
     * @param faultCode Fault code to describe
     * @return const char* Human-readable description
     */
    static const char* getFaultDescription(uint8_t faultCode);
    
    /**
     * @brief Clear fault conditions
     * 
     * @return true if faults cleared
     * @return false if faults persist
     */
    bool clearFaults();
    
    // ========================================================================
    // ADVANCED FEATURES
    // ========================================================================
    
    /**
     * @brief Enable/disable adaptive control
     * Automatically adjusts parameters based on performance
     * 
     * @param enable True to enable, false to disable
     */
    void setAdaptiveControl(bool enable);
    
    /**
     * @brief Enable/disable skip compensation
     * Automatically dispenses extra seed after skip
     * 
     * @param enable True to enable, false to disable
     */
    void setSkipCompensation(bool enable);
    
    /**
     * @brief Set target singulation accuracy
     * 
     * @param accuracy Target accuracy percentage (90-100)
     */
    void setTargetAccuracy(float accuracy);
    
    /**
     * @brief Perform self-test
     * 
     * @return true if self-test passed
     * @return false if self-test failed
     */
    bool selfTest();
    
    /**
     * @brief Get sensor status
     * 
     * @param sensor1 Reference to store sensor 1 status
     * @param sensor2 Reference to store sensor 2 status
     */
    void getSensorStatus(bool& sensor1, bool& sensor2) const;
    
    /**
     * @brief Load calibration data from EEPROM
     * 
     * @return true if calibration loaded
     * @return false if no valid calibration
     */
    bool loadCalibration();
    
    /**
     * @brief Save calibration data to EEPROM
     * 
     * @return true if calibration saved
     * @return false if save failed
     */
    bool saveCalibration();
    
    /**
     * @brief Reset quality statistics
     */
    void resetQualityMetrics();
    
    /**
     * @brief Get environmental compensation factor
     * 
     * @return float Compensation factor (0.8-1.2)
     */
    float getEnvironmentalFactor() const;

private:
    // ========================================================================
    // PRIVATE MEMBER VARIABLES
    // ========================================================================
    
    // Pin assignments
    uint8_t motorPin;
    uint8_t dirPin;
    uint8_t enablePin;
    uint8_t sensor1Pin;
    uint8_t sensor2Pin;
    uint8_t vacuumValvePin;
    uint8_t vacuumSensorPin;
    uint8_t levelSensorPin;
    uint8_t tempSensorPin;
    uint8_t humidityPin;
    
    // State variables
    DispenserMode currentMode;
    DispenserMode previousMode;
    uint8_t faultFlags;
    bool enabled;
    bool vacuumEnabled;
    bool adaptiveControlEnabled;
    bool skipCompensationEnabled;
    
    // Current profile and mechanism
    SeedProfile activeProfile;
    MechanismType mechanismType;
    
    // Motor/disc control
    uint16_t currentSpeed;
    uint16_t targetSpeed;
    Servo* motorServo;  // For servo-based systems
    
    // Vacuum system
    float currentVacuum;
    float targetVacuum;
    
    // Sensor readings
    bool sensor1State;
    bool sensor2State;
    uint16_t sensor1Reading;
    uint16_t sensor2Reading;
    float temperature;
    float humidity;
    
    // Seed tracking
    uint16_t seedsInHopper;
    uint32_t totalSeedsDispensed;
    SeedEvent lastEvent;
    
    // Quality metrics
    DispensingQuality qualityMetrics;
    
    // Calibration
    CalibrationData calibration;
    
    // Timing
    unsigned long lastDispenseTime;
    unsigned long lastSensorCheckTime;
    unsigned long lastVacuumCheckTime;
    unsigned long verificationStartTime;
    
    // Continuous mode
    bool continuousMode;
    float targetDispenseRate;
    unsigned long lastContinuousDispense;
    
    // Buffers for filtering
    uint16_t sensorBuffer1[5];
    uint16_t sensorBuffer2[5];
    uint8_t sensorBufferIndex;
    
    // ========================================================================
    // PRIVATE METHODS
    // ========================================================================
    
    // Hardware interface
    void setMotorPWM(uint16_t duty);
    void setMotorDirection(bool forward);
    void enableMotorDriver(bool enable);
    bool readSensor1();
    bool readSensor2();
    uint16_t readSensor1Raw();
    uint16_t readSensor2Raw();
    float readVacuumSensor();
    float readTemperatureSensor();
    float readHumiditySensor();
    uint16_t readLevelSensor();
    
    // Control algorithms
    void updateMotorControl();
    void updateVacuumControl();
    void adaptParameters();
    void compensateEnvironment();
    
    // Verification logic
    bool checkSensorTransition();
    bool checkDualSensorAgreement();
    void recordSeedEvent(bool detected, bool verified, bool isDouble, bool isSkip);
    
    // Quality control
    void updateQualityMetrics();
    void detectBlockage();
    void detectSkip();
    void detectDouble();
    bool shouldCompensateSkip();
    
    // Fault handling
    void checkFaults();
    void handleFault(uint8_t faultCode);
    
    // Calibration helpers
    void performSensorCalibration();
    void performVacuumCalibration();
    void performTimingCalibration();
    
    // Helper functions
    uint16_t filterSensorReading(uint16_t reading, uint8_t sensor);
    bool validateProfile(const SeedProfile& profile);
    float calculateEnvironmentalFactor();
};

#endif // SEED_DISPENSER_H
