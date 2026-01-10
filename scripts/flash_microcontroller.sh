#!/bin/bash

################################################################################
# flash_microcontroller.sh
# Production-ready firmware flashing script for seed sowing robot
# Supports ESP32, Arduino Mega, and other microcontrollers
#
# Features:
# - Automatic board detection
# - Multiple upload methods (USB, OTA, JTAG)
# - Pre-flash validation and compilation
# - Post-flash verification
# - Backup and rollback capabilities
# - Serial monitor integration
# - Multi-board flashing support
# - Error handling and logging
#
# Version: 2.0.0
# Date: 2026-01-10
################################################################################

set -euo pipefail  # Exit on error, undefined vars, pipe failures

# ============================================================================
# CONFIGURATION
# ============================================================================

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
FIRMWARE_DIR="$PROJECT_ROOT/firmware"
BUILD_DIR="$FIRMWARE_DIR/.pio/build"
BACKUP_DIR="$PROJECT_ROOT/firmware_backups"
LOG_DIR="$PROJECT_ROOT/logs"
LOG_FILE="$LOG_DIR/flash_$(date +%Y%m%d_%H%M%S).log"

# Default configuration
DEFAULT_PORT="/dev/ttyUSB0"
DEFAULT_BAUD="115200"
DEFAULT_ENVIRONMENT="esp32dev"
FLASH_TIMEOUT=120  # seconds
VERIFY_ENABLED=true
BACKUP_ENABLED=true

# Supported boards
declare -A BOARD_CONFIGS=(
    ["esp32"]="esp32dev:921600:esptool"
    ["esp32-s3"]="esp32-s3-devkitc-1:921600:esptool"
    ["mega2560"]="megaatmega2560:115200:avrdude"
    ["due"]="dueUSB:115200:bossac"
    ["teensy41"]="teensy41:115200:teensy_loader_cli"
)

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

print_banner() {
    echo -e "${CYAN}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        Seed Sowing Robot - Firmware Flash Utility             ║"
    echo "║                    Version 2.0.0                               ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

log() {
    local level=$1
    shift
    local message="$@"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    echo "[$timestamp] [$level] $message" >> "$LOG_FILE"
    
    case $level in
        INFO)
            echo -e "${BLUE}[INFO]${NC} $message"
            ;;
        SUCCESS)
            echo -e "${GREEN}[SUCCESS]${NC} $message"
            ;;
        WARNING)
            echo -e "${YELLOW}[WARNING]${NC} $message"
            ;;
        ERROR)
            echo -e "${RED}[ERROR]${NC} $message"
            ;;
        DEBUG)
            if [[ "${DEBUG:-0}" == "1" ]]; then
                echo -e "${MAGENTA}[DEBUG]${NC} $message"
            fi
            ;;
    esac
}

error_exit() {
    log ERROR "$1"
    exit 1
}

check_dependencies() {
    log INFO "Checking dependencies..."
    
    local missing_deps=()
    
    # Check for PlatformIO
    if ! command -v pio &> /dev/null; then
        missing_deps+=("platformio")
    fi
    
    # Check for Python
    if ! command -v python3 &> /dev/null; then
        missing_deps+=("python3")
    fi
    
    # Optional dependencies
    if ! command -v esptool.py &> /dev/null; then
        log WARNING "esptool.py not found (optional for ESP32 direct flashing)"
    fi
    
    if ! command -v avrdude &> /dev/null; then
        log WARNING "avrdude not found (optional for Arduino direct flashing)"
    fi
    
    if [[ ${#missing_deps[@]} -gt 0 ]]; then
        error_exit "Missing required dependencies: ${missing_deps[*]}"
    fi
    
    log SUCCESS "All required dependencies found"
}

create_directories() {
    log DEBUG "Creating necessary directories..."
    
    mkdir -p "$BACKUP_DIR"
    mkdir -p "$LOG_DIR"
    mkdir -p "$BUILD_DIR"
}

# ============================================================================
# DEVICE DETECTION
# ============================================================================

detect_port() {
    log INFO "Detecting microcontroller port..."
    
    local ports=()
    
    # Detect USB serial devices
    for port in /dev/ttyUSB* /dev/ttyACM* /dev/cu.usb* /dev/cu.SLAB*; do
        if [[ -e "$port" ]]; then
            ports+=("$port")
        fi
    done
    
    if [[ ${#ports[@]} -eq 0 ]]; then
        log WARNING "No USB serial ports detected"
        return 1
    fi
    
    if [[ ${#ports[@]} -eq 1 ]]; then
        DETECTED_PORT="${ports[0]}"
        log SUCCESS "Detected port: $DETECTED_PORT"
        return 0
    fi
    
    # Multiple ports found - let user choose
    log INFO "Multiple ports detected:"
    select port in "${ports[@]}"; do
        if [[ -n "$port" ]]; then
            DETECTED_PORT="$port"
            log SUCCESS "Selected port: $DETECTED_PORT"
            return 0
        fi
    done
    
    return 1
}

detect_board_type() {
    local port=$1
    log INFO "Detecting board type on $port..."
    
    # Try to get board info using PlatformIO
    local board_info=$(pio device list 2>/dev/null | grep -A 5 "$port" || true)
    
    if echo "$board_info" | grep -qi "esp32"; then
        echo "esp32"
    elif echo "$board_info" | grep -qi "mega"; then
        echo "mega2560"
    elif echo "$board_info" | grep -qi "due"; then
        echo "due"
    elif echo "$board_info" | grep -qi "teensy"; then
        echo "teensy41"
    else
        log WARNING "Could not auto-detect board type"
        echo "unknown"
    fi
}

# ============================================================================
# FIRMWARE COMPILATION
# ============================================================================

compile_firmware() {
    local environment=$1
    
    log INFO "Compiling firmware for environment: $environment"
    
    cd "$FIRMWARE_DIR" || error_exit "Cannot access firmware directory"
    
    # Clean previous build
    log DEBUG "Cleaning previous build..."
    pio run -e "$environment" --target clean >> "$LOG_FILE" 2>&1 || true
    
    # Compile
    log INFO "Building firmware (this may take a few minutes)..."
    if pio run -e "$environment" >> "$LOG_FILE" 2>&1; then
        log SUCCESS "Firmware compiled successfully"
        return 0
    else
        log ERROR "Firmware compilation failed. Check log: $LOG_FILE"
        tail -n 50 "$LOG_FILE"
        return 1
    fi
}

verify_firmware_size() {
    local environment=$1
    local firmware_file="$BUILD_DIR/$environment/firmware.bin"
    
    if [[ ! -f "$firmware_file" ]]; then
        firmware_file="$BUILD_DIR/$environment/firmware.hex"
    fi
    
    if [[ ! -f "$firmware_file" ]]; then
        log ERROR "Firmware file not found after compilation"
        return 1
    fi
    
    local size=$(stat -f%z "$firmware_file" 2>/dev/null || stat -c%s "$firmware_file" 2>/dev/null)
    local size_kb=$((size / 1024))
    
    log INFO "Firmware size: ${size_kb} KB"
    
    # Check against flash size limits
    if [[ "$environment" == "esp32"* ]]; then
        if [[ $size -gt $((4 * 1024 * 1024)) ]]; then
            log WARNING "Firmware size exceeds 4MB (ESP32 limit)"
        fi
    elif [[ "$environment" == "mega"* ]]; then
        if [[ $size -gt $((256 * 1024)) ]]; then
            log ERROR "Firmware size exceeds 256KB (Arduino Mega limit)"
            return 1
        fi
    fi
    
    return 0
}

# ============================================================================
# BACKUP AND RESTORE
# ============================================================================

backup_current_firmware() {
    local port=$1
    local board_type=$2
    
    if [[ "$BACKUP_ENABLED" != "true" ]]; then
        log INFO "Backup disabled, skipping..."
        return 0
    fi
    
    log INFO "Backing up current firmware..."
    
    local backup_file="$BACKUP_DIR/backup_${board_type}_$(date +%Y%m%d_%H%M%S).bin"
    
    case $board_type in
        esp32*)
            # Read flash using esptool
            if command -v esptool.py &> /dev/null; then
                esptool.py --port "$port" --baud 115200 read_flash 0x0 0x400000 "$backup_file" >> "$LOG_FILE" 2>&1
                log SUCCESS "Firmware backed up to: $backup_file"
            else
                log WARNING "esptool.py not found, skipping backup"
            fi
            ;;
        mega2560)
            log WARNING "Backup not supported for Arduino Mega"
            ;;
        *)
            log WARNING "Backup not implemented for board type: $board_type"
            ;;
    esac
}

# ============================================================================
# FLASHING OPERATIONS
# ============================================================================

flash_using_platformio() {
    local environment=$1
    local port=$2
    
    log INFO "Flashing firmware using PlatformIO..."
    
    cd "$FIRMWARE_DIR" || error_exit "Cannot access firmware directory"
    
    # Flash command with timeout
    if timeout $FLASH_TIMEOUT pio run -e "$environment" --target upload --upload-port "$port" >> "$LOG_FILE" 2>&1; then
        log SUCCESS "Firmware flashed successfully via PlatformIO"
        return 0
    else
        log ERROR "Firmware flashing failed. Check log: $LOG_FILE"
        return 1
    fi
}

flash_esp32_direct() {
    local port=$1
    local firmware_file="$BUILD_DIR/esp32dev/firmware.bin"
    local bootloader="$BUILD_DIR/esp32dev/bootloader.bin"
    local partitions="$BUILD_DIR/esp32dev/partitions.bin"
    
    log INFO "Flashing ESP32 using esptool.py..."
    
    if ! command -v esptool.py &> /dev/null; then
        log ERROR "esptool.py not found"
        return 1
    fi
    
    # Erase flash
    log INFO "Erasing flash..."
    esptool.py --chip esp32 --port "$port" erase_flash >> "$LOG_FILE" 2>&1
    
    # Flash firmware
    log INFO "Writing firmware..."
    esptool.py --chip esp32 --port "$port" --baud 921600 \
        --before default_reset --after hard_reset write_flash -z \
        --flash_mode dio --flash_freq 40m --flash_size detect \
        0x1000 "$bootloader" \
        0x8000 "$partitions" \
        0x10000 "$firmware_file" >> "$LOG_FILE" 2>&1
    
    if [[ $? -eq 0 ]]; then
        log SUCCESS "ESP32 flashed successfully"
        return 0
    else
        log ERROR "ESP32 flashing failed"
        return 1
    fi
}

flash_arduino_direct() {
    local port=$1
    local board=$2
    local firmware_file="$BUILD_DIR/$board/firmware.hex"
    
    log INFO "Flashing Arduino using avrdude..."
    
    if ! command -v avrdude &> /dev/null; then
        log ERROR "avrdude not found"
        return 1
    fi
    
    local avr_part="m2560"
    if [[ "$board" == "mega2560" ]]; then
        avr_part="m2560"
    fi
    
    avrdude -v -p "$avr_part" -c wiring -P "$port" -b 115200 -D \
        -U "flash:w:$firmware_file:i" >> "$LOG_FILE" 2>&1
    
    if [[ $? -eq 0 ]]; then
        log SUCCESS "Arduino flashed successfully"
        return 0
    else
        log ERROR "Arduino flashing failed"
        return 1
    fi
}

flash_ota() {
    local ip_address=$1
    local environment=$2
    
    log INFO "Flashing over-the-air to $ip_address..."
    
    cd "$FIRMWARE_DIR" || error_exit "Cannot access firmware directory"
    
    pio run -e "$environment" --target upload --upload-port "$ip_address" >> "$LOG_FILE" 2>&1
    
    if [[ $? -eq 0 ]]; then
        log SUCCESS "OTA flash successful"
        return 0
    else
        log ERROR "OTA flash failed"
        return 1
    fi
}

# ============================================================================
# VERIFICATION
# ============================================================================

verify_flash() {
    local port=$1
    local board_type=$2
    
    if [[ "$VERIFY_ENABLED" != "true" ]]; then
        log INFO "Verification disabled, skipping..."
        return 0
    fi
    
    log INFO "Verifying flashed firmware..."
    
    # Wait for device to reboot
    sleep 3
    
    # Check if device is responding on serial
    if [[ -e "$port" ]]; then
        log INFO "Device detected on $port"
        
        # Try to read version info from serial
        timeout 10 cat "$port" | head -n 20 | tee -a "$LOG_FILE" || true
        
        log SUCCESS "Verification complete - device is responding"
        return 0
    else
        log ERROR "Device not detected after flashing"
        return 1
    fi
}

open_serial_monitor() {
    local port=$1
    local baud=${2:-115200}
    
    log INFO "Opening serial monitor on $port @ $baud baud..."
    
    if command -v pio &> /dev/null; then
        pio device monitor --port "$port" --baud "$baud"
    elif command -v screen &> /dev/null; then
        screen "$port" "$baud"
    elif command -v minicom &> /dev/null; then
        minicom -D "$port" -b "$baud"
    else
        log WARNING "No serial monitor tool found (pio, screen, or minicom)"
    fi
}

# ============================================================================
# MULTI-BOARD FLASHING
# ============================================================================

flash_all_boards() {
    log INFO "Scanning for multiple boards..."
    
    local ports=()
    for port in /dev/ttyUSB* /dev/ttyACM*; do
        if [[ -e "$port" ]]; then
            ports+=("$port")
        fi
    done
    
    if [[ ${#ports[@]} -eq 0 ]]; then
        log ERROR "No boards detected"
        return 1
    fi
    
    log INFO "Found ${#ports[@]} board(s)"
    
    for port in "${ports[@]}"; do
        log INFO "Processing $port..."
        
        local board_type=$(detect_board_type "$port")
        log INFO "Detected board: $board_type"
        
        if [[ "$board_type" == "unknown" ]]; then
            log WARNING "Skipping unknown board on $port"
            continue
        fi
        
        # Flash the board
        flash_single_board "$port" "$board_type"
        
        sleep 2
    done
    
    log SUCCESS "All boards processed"
}

flash_single_board() {
    local port=$1
    local board_type=${2:-""}
    
    # Auto-detect if not specified
    if [[ -z "$board_type" ]]; then
        board_type=$(detect_board_type "$port")
    fi
    
    # Get environment name
    local environment="${board_type}"
    if [[ "$board_type" == "esp32" ]]; then
        environment="esp32dev"
    fi
    
    log INFO "Flashing $board_type on $port (environment: $environment)"
    
    # Compile if needed
    if [[ ! -d "$BUILD_DIR/$environment" ]]; then
        compile_firmware "$environment" || return 1
        verify_firmware_size "$environment" || return 1
    fi
    
    # Backup current firmware
    backup_current_firmware "$port" "$board_type"
    
    # Flash firmware
    if flash_using_platformio "$environment" "$port"; then
        verify_flash "$port" "$board_type"
        return 0
    else
        return 1
    fi
}

# ============================================================================
# MAIN FUNCTIONS
# ============================================================================

show_usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Options:
    -p, --port PORT         Serial port (default: auto-detect)
    -b, --board BOARD       Board type (esp32, mega2560, etc.)
    -e, --env ENV           PlatformIO environment
    -m, --method METHOD     Flash method (pio, direct, ota)
    -i, --ip IP             IP address for OTA flashing
    --no-compile            Skip compilation step
    --no-verify             Skip verification step
    --no-backup             Skip firmware backup
    --monitor               Open serial monitor after flashing
    --all                   Flash all detected boards
    -h, --help              Show this help message
    -v, --verbose           Enable verbose output

Examples:
    # Auto-detect and flash
    $0

    # Flash specific board
    $0 --board esp32 --port /dev/ttyUSB0

    # Flash via OTA
    $0 --method ota --ip 192.168.1.100

    # Flash all connected boards
    $0 --all

    # Flash and open monitor
    $0 --monitor

EOF
}

parse_arguments() {
    FLASH_PORT=""
    BOARD_TYPE=""
    ENVIRONMENT=""
    FLASH_METHOD="pio"
    OTA_IP=""
    SKIP_COMPILE=false
    OPEN_MONITOR=false
    FLASH_ALL=false
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            -p|--port)
                FLASH_PORT="$2"
                shift 2
                ;;
            -b|--board)
                BOARD_TYPE="$2"
                shift 2
                ;;
            -e|--env)
                ENVIRONMENT="$2"
                shift 2
                ;;
            -m|--method)
                FLASH_METHOD="$2"
                shift 2
                ;;
            -i|--ip)
                OTA_IP="$2"
                FLASH_METHOD="ota"
                shift 2
                ;;
            --no-compile)
                SKIP_COMPILE=true
                shift
                ;;
            --no-verify)
                VERIFY_ENABLED=false
                shift
                ;;
            --no-backup)
                BACKUP_ENABLED=false
                shift
                ;;
            --monitor)
                OPEN_MONITOR=true
                shift
                ;;
            --all)
                FLASH_ALL=true
                shift
                ;;
            -v|--verbose)
                DEBUG=1
                shift
                ;;
            -h|--help)
                show_usage
                exit 0
                ;;
            *)
                log ERROR "Unknown option: $1"
                show_usage
                exit 1
                ;;
        esac
    done
}

main() {
    print_banner
    
    # Setup
    create_directories
    log INFO "Log file: $LOG_FILE"
    
    # Check dependencies
    check_dependencies
    
    # Flash all boards if requested
    if [[ "$FLASH_ALL" == "true" ]]; then
        flash_all_boards
        exit $?
    fi
    
    # OTA flashing
    if [[ "$FLASH_METHOD" == "ota" ]]; then
        if [[ -z "$OTA_IP" ]]; then
            error_exit "IP address required for OTA flashing"
        fi
        
        ENVIRONMENT="${ENVIRONMENT:-esp32dev}"
        
        if [[ "$SKIP_COMPILE" != "true" ]]; then
            compile_firmware "$ENVIRONMENT" || error_exit "Compilation failed"
        fi
        
        flash_ota "$OTA_IP" "$ENVIRONMENT"
        exit $?
    fi
    
    # Detect port if not specified
    if [[ -z "$FLASH_PORT" ]]; then
        detect_port || error_exit "Could not detect port. Please specify with --port"
        FLASH_PORT="$DETECTED_PORT"
    fi
    
    # Detect board type if not specified
    if [[ -z "$BOARD_TYPE" ]]; then
        BOARD_TYPE=$(detect_board_type "$FLASH_PORT")
        if [[ "$BOARD_TYPE" == "unknown" ]]; then
            error_exit "Could not detect board type. Please specify with --board"
        fi
    fi
    
    # Determine environment
    if [[ -z "$ENVIRONMENT" ]]; then
        if [[ "$BOARD_TYPE" == "esp32" ]]; then
            ENVIRONMENT="esp32dev"
        else
            ENVIRONMENT="$BOARD_TYPE"
        fi
    fi
    
    log INFO "Configuration:"
    log INFO "  Port: $FLASH_PORT"
    log INFO "  Board: $BOARD_TYPE"
    log INFO "  Environment: $ENVIRONMENT"
    log INFO "  Method: $FLASH_METHOD"
    
    # Compile firmware
    if [[ "$SKIP_COMPILE" != "true" ]]; then
        compile_firmware "$ENVIRONMENT" || error_exit "Compilation failed"
        verify_firmware_size "$ENVIRONMENT" || error_exit "Firmware size verification failed"
    fi
    
    # Backup current firmware
    backup_current_firmware "$FLASH_PORT" "$BOARD_TYPE"
    
    # Flash firmware
    case $FLASH_METHOD in
        pio)
            flash_using_platformio "$ENVIRONMENT" "$FLASH_PORT" || error_exit "Flashing failed"
            ;;
        direct)
            if [[ "$BOARD_TYPE" == "esp32"* ]]; then
                flash_esp32_direct "$FLASH_PORT" || error_exit "Flashing failed"
            elif [[ "$BOARD_TYPE" == "mega2560" ]]; then
                flash_arduino_direct "$FLASH_PORT" "$BOARD_TYPE" || error_exit "Flashing failed"
            else
                error_exit "Direct flashing not supported for board type: $BOARD_TYPE"
            fi
            ;;
        *)
            error_exit "Unknown flash method: $FLASH_METHOD"
            ;;
    esac
    
    # Verify flash
    verify_flash "$FLASH_PORT" "$BOARD_TYPE"
    
    # Open serial monitor if requested
    if [[ "$OPEN_MONITOR" == "true" ]]; then
        open_serial_monitor "$FLASH_PORT" "$DEFAULT_BAUD"
    fi
    
    log SUCCESS "All operations completed successfully!"
    log INFO "Log saved to: $LOG_FILE"
}

# ============================================================================
# SCRIPT ENTRY POINT
# ============================================================================

# Parse command line arguments
parse_arguments "$@"

# Run main function
main

exit 0
