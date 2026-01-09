/**
 * @file WaterPump.h
 * @brief Production-ready precision water pump control system
 * @version 3.0.0
 * @date 2026-01-09
 * 
 * Advanced Features:
 * - Precision flow rate control (±2% accuracy)
 * - Variable flow profiles for different operations
 * - Real-time flow monitoring and verification
 * - Pressure regulation and monitoring
 * - Multi-zone irrigation support
 * - Pulse-width modulation for flow control
 * - Water consumption tracking and analytics
 * - Leak detection and prevention
 * - Dry-run protection
 * - Cavitation detection and prevention
 * - Self-priming capability monitoring
 * - Filter blockage detection
 * - Adaptive flow compensation
 * - Water level monitoring
 * - Energy-efficient operation modes
 * - Predictive maintenance
 * 
 * Hardware Support:
 * - Peristaltic pumps (50-1000 mL/min)
 * - Diaphragm pumps (100-5000 mL/min)
 * - Centrifugal pumps (500-10000 mL/min)
 * - Solenoid-actuated pumps
 * - Flow meters (Hall-effect, optical)
 * - Pressure sensors (0-1000 kPa)
 * - Water level sensors
 * - Temperature sensors
 * 
 * Application Modes:
 * - Seed soaking/pre-treatment
 * - Furrow irrigation
 * - Spot irrigation
 * - Spray irrigation
 * - Drip irrigation
 * - Cleaning operations
 * 
 * @author Agricultural Robotics Team
 * @copyright (c) 2026 Agricultural Robotics Inc.
 */

#ifndef WATER_PUMP_H
#define WATER_PUMP_H

#include <Arduino.h>
#include <stdint.h>
#include <PID_v1.h>

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

#define PUMP_PWM_FREQUENCY          1000     // PWM frequency in Hz
#define PUMP_MIN_FLOW_RATE          50       // Minimum flow rate (mL/min)
#define PUMP_MAX_FLOW_RATE          2000     // Maximum flow rate (mL/min)
#define PUMP_RATED_FLOW             1000     // Rated flow rate (mL/min)
#define PUMP_MIN_PRESSURE           50.0f    // Minimum pressure (kPa)
#define PUMP_MAX_PRESSURE           500.0f   // Maximum pressure (kPa)
#define PUMP_NORMAL_PRESSURE        200.0f   // Normal operating pressure (kPa)
#define PUMP_TANK_CAPACITY          50000    // Water tank capacity (mL)
#define PUMP_LOW_LEVEL_THRESHOLD    5000     // Low water warning (mL)
#define PUMP_CRITICAL_LEVEL         1000     // Critical water level (mL)
#define PUMP_MAX_POWER              100      // Maximum power (Watts)
#define PUMP_FLOW_SENSOR_PPL        450      // Flow sensor pulses per liter
#define PUMP_FLOW_TIMEOUT           5000     // Flow detection timeout (ms)
#define PUMP_LEAK_THRESHOLD         50       // Leak detection threshold (mL/min)
#define PUMP_PRIME_TIME             10000    // Maximum priming time (ms)
#define PUMP_DRY_RUN_TIMEOUT        3000     // Dry-run detection time (ms)
#define PUMP_FILTER_PRESSURE_DROP   20.0f    // Pressure drop indicating filter blockage
#define PUMP_MAINTENANCE_HOURS      1000     // Maintenance interval (hours)

// PID tuning parameters
#define PUMP_PID_KP                 1.5f
#define PUMP_PID_KI                 0.3f
#define PUMP_PID_KD                 0.05f
#define PUMP_PID_SAMPLE_TIME        100      // PID sample time (ms)

// Fault codes
#define PUMP_FAULT_NONE             0x00
#define PUMP_FAULT_DRY_RUN          0x01
#define PUMP_FAULT_OVERPRESSURE     0x02
#define PUMP_FAULT_UNDERPRESSURE    0x04
#define PUMP_FAULT_NO_FLOW          0x08
#define PUMP_FAULT_LEAK             0x10
#define PUMP_FAULT_CAVITATION       0x20
#define PUMP_FAULT_FILTER_BLOCK     0x40
#define PUMP_FAULT_SENSOR           0x80

// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @brief Pump operating modes
 */
enum class PumpMode : uint8_t {
    OFF = 0,            ///< Pump off
    PRIMING,            ///< Self-priming mode
    IDLE,               ///< Idle/standby
    RUNNING,            ///< Normal operation
    PULSING,            ///< Pulse mode for precision
    FLUSHING,           ///< Flushing mode
    FAULT,              ///< Fault condition
    MAINTENANCE,        ///< Maintenance mode
    CALIBRATION         ///< Calibration mode
};

/**
 * @brief Pump types
 */
enum class PumpType : uint8_t {
    PERISTALTIC = 0,    ///< Peristaltic pump
    DIAPHRAGM,          ///< Diaphragm pump
    CENTRIFUGAL,        ///< Centrifugal pump
    SOLENOID,           ///< Solenoid pump
    GEAR,               ///< Gear pump
    CUSTOM              ///< Custom pump type
};

/**
 * @brief Application profiles
 */
enum class IrrigationMode : uint8_t {
    SEED_SOAK = 0,      ///< Seed soaking
    FURROW,             ///< Furrow irrigation
    SPOT,               ///< Spot irrigation
    SPRAY,              ///< Spray irrigation
    DRIP,               ///< Drip irrigation
    CLEANING,           ///< System cleaning
    CUSTOM              ///< Custom mode
};

/**
 * @brief Flow profile structure
 */
struct FlowProfile {
    uint16_t startupFlow;       ///< Initial flow rate (mL/min)
    uint16_t operatingFlow;     ///< Normal operating flow (mL/min)
    uint16_t shutdownFlow;      ///< Shutdown flow (mL/min)
    uint16_t rampUpTime;        ///< Ramp-up time (ms)
    uint16_t rampDownTime;      ///< Ramp-down time (ms)
    float minPressure;          ///< Minimum acceptable pressure (kPa)
    float maxPressure;          ///< Maximum acceptable pressure (kPa)
    bool pulseModeEnabled;      ///< Enable pulse mode
    uint16_t pulseOnTime;       ///< Pulse on time (ms)
    uint16_t pulseOffTime;      ///< Pulse off time (ms)
    char name[32];              ///< Profile name
};

/**
 * @brief Pump statistics structure
 */
struct PumpStatistics {
    uint32_t totalOperatingTime;    ///< Total operating time (seconds)
    uint32_t totalCycles;           ///< Total on/off cycles
    uint32_t totalVolumeDispensed;  ///< Total water dispensed (mL)
    float avgFlowRate;              ///< Average flow rate (mL/min)
    float avgPressure;              ///< Average pressure (kPa)
    float avgPower;                 ///< Average power consumption (W)
    float peakPower;                ///< Peak power (W)
    uint16_t leakEvents;            ///< Number of leak events
    uint16_t dryRunEvents;          ///< Number of dry-run events
    uint16_t faultCount;            ///< Total fault occurrences
    uint32_t maintenanceCountdown;  ///< Hours until maintenance
};

/**
 * @brief Real-time telemetry
 */
struct PumpTelemetry {
    PumpMode mode;                  ///< Current operating mode
    uint16_t currentFlowRate;       ///< Current flow rate (mL/min)
    float currentPressure;          ///< Current pressure (kPa)
    float inputVoltage;             ///< Input voltage (V)
    float inputCurrent;             ///< Input current (A)
    float powerConsumption;         ///< Power consumption (W)
    float temperature;              ///< Pump temperature (°C)
    uint32_t waterRemaining;        ///< Water remaining in tank (mL)
    uint32_t volumeDispensed;       ///< Volume dispensed this session (mL)
    float efficiency;               ///< Operating efficiency (%)
    uint8_t faultFlags;             ///< Active fault flags
    bool isPrimed;                  ///< Pump primed status
    bool flowDetected;              ///< Flow sensor detection
    bool leakDetected;              ///< Leak detection status
};

/**
 * @brief Calibration data
 */
struct PumpCalibration {
    float flowSensorFactor;         ///< Flow sensor calibration (pulses/L)
    float pressureSensorOffset;     ///< Pressure sensor offset (kPa)
    float pressureSensorGain;       ///< Pressure sensor gain
    uint16_t pwmFlowCurve[10];      ///< PWM to flow mapping points
    uint32_t calibrationDate;       ///< Calibration timestamp
    bool isValid;                   ///< Calibration validity
};

// ============================================================================
// MAIN CLASS DEFINITION
// ============================================================================

/**
 * @class WaterPump
 * @brief Production-ready precision water pump controller
 * 
 * This class provides comprehensive pump control with:
 * - Precision flow rate control
 * - Real-time monitoring
 * - Safety features
 * - Adaptive control
 * - Energy optimization
 * - Predictive maintenance
 */
class WaterPump {
public:
    // ========================================================================
    // CONSTRUCTOR & DESTRUCTOR
    // ========================================================================
    
    /**
     * @brief Construct a new Water Pump controller
     * 
     * @param pwmPin PWM control pin
     * @param enablePin Enable pin
     * @param flowSensorPin Flow meter pulse input
     * @param pressureSensorPin Pressure sensor analog input
     * @param levelSensorPin Water level sensor
     * @param tempSensorPin Temperature sensor
     * @param currentSensePin Current sense pin
     */
    WaterPump(uint8_t pwmPin,
              uint8_t enablePin,
              uint8_t flowSensorPin,
              uint8_t pressureSensorPin,
              uint8_t levelSensorPin,
              uint8_t tempSensorPin,
              uint8_t currentSensePin);
    
    /**
     * @brief Destroy the Water Pump object
     */
    ~WaterPump();
    
    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    
    /**
     * @brief Initialize the pump controller
     * 
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool begin();
    
    /**
     * @brief Calibrate sensors
     * 
     * @return true if calibration successful
     * @return false if calibration failed
     */
    bool calibrate();
    
    /**
     * @brief Set pump type
     * 
     * @param type Pump type
     * @param maxFlow Maximum flow rate (mL/min)
     * @param maxPressure Maximum pressure (kPa)
     */
    void setPumpType(PumpType type, uint16_t maxFlow, float maxPressure);
    
    /**
     * @brief Configure PID controller
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void configurePID(float kp, float ki, float kd);
    
    /**
     * @brief Load flow profile
     * 
     * @param profile Flow profile to load
     * @return true if profile loaded
     * @return false if profile invalid
     */
    bool loadFlowProfile(const FlowProfile& profile);
    
    /**
     * @brief Get predefined profile for irrigation mode
     * 
     * @param mode Irrigation mode
     * @return FlowProfile Optimized flow profile
     */
    static FlowProfile getProfileForMode(IrrigationMode mode);
    
    // ========================================================================
    // PUMP CONTROL
    // ========================================================================
    
    /**
     * @brief Prime the pump
     * 
     * @param timeout Maximum priming time (ms)
     * @return true if priming successful
     * @return false if priming failed
     */
    bool prime(uint16_t timeout = PUMP_PRIME_TIME);
    
    /**
     * @brief Start pump operation
     * 
     * @param flowRate Target flow rate (mL/min)
     * @return true if started successfully
     * @return false if failed to start
     */
    bool start(uint16_t flowRate);
    
    /**
     * @brief Stop pump
     * 
     * @param immediateStop True for immediate stop, false for controlled ramp-down
     * @return true if stopped successfully
     * @return false if stop failed
     */
    bool stop(bool immediateStop = false);
    
    /**
     * @brief Set target flow rate
     * 
     * @param flowRate Target flow rate (mL/min)
     * @return true if flow rate set
     * @return false if out of range or pump not running
     */
    bool setFlowRate(uint16_t flowRate);
    
    /**
     * @brief Get current flow rate
     * 
     * @return uint16_t Current flow rate (mL/min)
     */
    uint16_t getFlowRate() const;
    
    /**
     * @brief Dispense specific volume
     * 
     * @param volume Volume to dispense (mL)
     * @param flowRate Flow rate (mL/min), 0 = use current
     * @return true if dispensing completed
     * @return false if dispensing failed
     */
    bool dispenseVolume(uint32_t volume, uint16_t flowRate = 0);
    
    /**
     * @brief Dispense for specific duration
     * 
     * @param duration Duration in milliseconds
     * @param flowRate Flow rate (mL/min), 0 = use current
     * @return true if dispensing completed
     * @return false if dispensing failed
     */
    bool dispenseForDuration(uint32_t duration, uint16_t flowRate = 0);
    
    // ========================================================================
    // PULSE MODE
    // ========================================================================
    
    /**
     * @brief Enable pulse mode
     * 
     * @param onTime Pulse on time (ms)
     * @param offTime Pulse off time (ms)
     * @return true if pulse mode enabled
     * @return false if parameters invalid
     */
    bool enablePulseMode(uint16_t onTime, uint16_t offTime);
    
    /**
     * @brief Disable pulse mode
     */
    void disablePulseMode();
    
    /**
     * @brief Check if pulse mode active
     * 
     * @return true if pulse mode enabled
     * @return false if continuous mode
     */
    bool isPulseModeActive() const { return pulseModeEnabled; }
    
    // ========================================================================
    // MONITORING
    // ========================================================================
    
    /**
     * @brief Update pump controller (call in main loop)
     * Must be called at least every 100ms
     */
    void update();
    
    /**
     * @brief Get real-time telemetry
     * 
     * @return PumpTelemetry Current telemetry data
     */
    PumpTelemetry getTelemetry() const;
    
    /**
     * @brief Get pump statistics
     * 
     * @return PumpStatistics Statistical data
     */
    PumpStatistics getStatistics() const;
    
    /**
     * @brief Get current operating mode
     * 
     * @return PumpMode Current mode
     */
    PumpMode getMode() const { return currentMode; }
    
    /**
     * @brief Check if pump is running
     * 
     * @return true if pump running
     * @return false if pump stopped
     */
    bool isRunning() const;
    
    /**
     * @brief Check if pump is primed
     * 
     * @return true if primed
     * @return false if not primed
     */
    bool isPrimed() const { return primed; }
    
    /**
     * @brief Get current pressure
     * 
     * @return float Current pressure (kPa)
     */
    float getPressure() const;
    
    /**
     * @brief Get water remaining in tank
     * 
     * @return uint32_t Water remaining (mL)
     */
    uint32_t getWaterRemaining() const;
    
    /**
     * @brief Check if water level is low
     * 
     * @return true if low water level
     * @return false if adequate water
     */
    bool isLowWaterLevel() const;
    
    /**
     * @brief Check if tank is empty
     * 
     * @return true if tank empty
     * @return false if water present
     */
    bool isEmpty() const;
    
    // ========================================================================
    // FAULT MANAGEMENT
    // ========================================================================
    
    /**
     * @brief Check if pump has fault
     * 
     * @return true if fault detected
     * @return false if no fault
     */
    bool hasFault() const { return faultFlags != PUMP_FAULT_NONE; }
    
    /**
     * @brief Get active fault flags
     * 
     * @return uint8_t Fault flags bitmask
     */
    uint8_t getFaultFlags() const { return faultFlags; }
    
    /**
     * @brief Get fault description
     * 
     * @param faultCode Fault code
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
     * @brief Enable/disable dry-run protection
     * 
     * @param enable True to enable, false to disable
     */
    void setDryRunProtection(bool enable);
    
    /**
     * @brief Enable/disable leak detection
     * 
     * @param enable True to enable, false to disable
     */
    void setLeakDetection(bool enable);
    
    /**
     * @brief Enable/disable adaptive flow control
     * 
     * @param enable True to enable, false to disable
     */
    void setAdaptiveControl(bool enable);
    
    /**
     * @brief Get pump efficiency
     * 
     * @return float Efficiency percentage (0-100)
     */
    float getEfficiency() const;
    
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
    
    /**
     * @brief Perform flush operation
     * 
     * @param duration Flush duration (ms)
     * @return true if flush completed
     * @return false if flush failed
     */
    bool flush(uint16_t duration = 30000);
    
    /**
     * @brief Perform self-test
     * 
     * @return true if self-test passed
     * @return false if self-test failed
     */
    bool selfTest();
    
    /**
     * @brief Reset water tank counter
     * 
     * @param capacity Tank capacity (mL)
     */
    void resetWaterTank(uint32_t capacity = PUMP_TANK_CAPACITY);
    
    /**
     * @brief Load calibration from EEPROM
     * 
     * @return true if calibration loaded
     * @return false if no valid calibration
     */
    bool loadCalibration();
    
    /**
     * @brief Save calibration to EEPROM
     * 
     * @return true if calibration saved
     * @return false if save failed
     */
    bool saveCalibration();

private:
    // ========================================================================
    // PRIVATE MEMBER VARIABLES
    // ========================================================================
    
    // Pin assignments
    uint8_t pwmPin;
    uint8_t enablePin;
    uint8_t flowSensorPin;
    uint8_t pressureSensorPin;
    uint8_t levelSensorPin;
    uint8_t tempSensorPin;
    uint8_t currentSensePin;
    
    // State variables
    PumpMode currentMode;
    PumpMode previousMode;
    PumpType pumpType;
    uint8_t faultFlags;
    bool enabled;
    bool primed;
    bool dryRunProtectionEnabled;
    bool leakDetectionEnabled;
    bool adaptiveControlEnabled;
    
    // Flow control
    uint16_t targetFlowRate;
    uint16_t currentFlowRate;
    uint16_t maxFlowRate;
    uint16_t minFlowRate;
    
    // Pressure
    float currentPressure;
    float maxPressureLimit;
    float minPressureLimit;
    
    // Volume tracking
    uint32_t waterInTank;
    uint32_t totalVolumeDispensed;
    uint32_t targetVolume;
    bool volumeDispenseActive;
    
    // PID controller
    double pidInput;
    double pidOutput;
    double pidSetpoint;
    PID* flowPID;
    
    // Current profile
    FlowProfile activeProfile;
    
    // Pulse mode
    bool pulseModeEnabled;
    uint16_t pulseOnTime;
    uint16_t pulseOffTime;
    unsigned long lastPulseToggle;
    bool pulseState;
    
    // Sensor readings
    volatile uint32_t flowPulseCount;
    unsigned long lastFlowPulse;
    float inputVoltage;
    float inputCurrent;
    float temperature;
    
    // Calibration
    PumpCalibration calibration;
    
    // Timing
    unsigned long lastUpdateTime;
    unsigned long operationStartTime;
    unsigned long primingStartTime;
    unsigned long lastFlowCheckTime;
    unsigned long dispenseDuration;
    
    // Statistics
    PumpStatistics stats;
    
    // Filtering buffers
    float pressureBuffer[5];
    uint8_t pressureBufferIndex;
    uint16_t flowBuffer[10];
    uint8_t flowBufferIndex;
    
    // ========================================================================
    // PRIVATE METHODS
    // ========================================================================
    
    // Hardware interface
    void setPWM(uint16_t duty);
    void enablePump(bool enable);
    float readPressureSensor();
    float readTemperatureSensor();
    float readCurrentSensor();
    float readVoltageSensor();
    uint32_t readLevelSensor();
    void updateFlowSensor();
    
    // Control algorithms
    void updateFlowControl();
    void updatePulseMode();
    void applyRampUp();
    void applyRampDown();
    void adaptFlowControl();
    
    // Monitoring
    void checkFaults();
    void checkDryRun();
    void checkPressure();
    void checkLeak();
    void checkCavitation();
    void checkFilterBlockage();
    void handleFault(uint8_t faultCode);
    
    // Volume management
    void updateVolumeTracking();
    void checkVolumeTarget();
    
    // Statistics
    void updateStatistics();
    
    // Interrupt handler (static)
    static void flowPulseISR();
    
    // Helper functions
    float filterPressure(float rawPressure);
    uint16_t filterFlowRate(uint16_t rawFlow);
    uint16_t calculateFlowFromPulses();
    float calculateEfficiency();
    bool validateFlowRate(uint16_t flowRate);
    uint16_t pwmFromFlowRate(uint16_t flowRate);
};

#endif // WATER_PUMP_H
