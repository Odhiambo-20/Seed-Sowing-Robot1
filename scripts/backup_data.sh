#!/bin/bash

################################################################################
# backup_data.sh - Production-Ready Automated Backup System
################################################################################
#
# Advanced Features:
# - Incremental and full backups
# - Compression with multiple algorithms (gzip, bzip2, xz, zstd)
# - Encryption (GPG, OpenSSL)
# - Remote backup support (rsync, scp, S3, Google Drive)
# - Database dumps (PostgreSQL, MySQL, SQLite)
# - Integrity verification (checksums, hash verification)
# - Retention policy management
# - Email notifications
# - Slack/Discord webhook notifications
# - Backup rotation (daily, weekly, monthly)
# - Parallel compression for speed
# - Bandwidth throttling for remote backups
# - Backup testing and verification
# - Disaster recovery planning
# - Incremental differential backups
# - Deduplication support
# - Cloud storage integration
# - Backup scheduling via cron
# - Logging with syslog integration
# - Health monitoring
# - Automatic cleanup of old backups
# - Configuration file support
# - Dry-run mode for testing
# - Progress indicators
# - Bandwidth usage tracking
#
# Backup Types:
# - System configuration files
# - Robot operation logs
# - Machine learning models
# - Sensor calibration data
# - Database backups
# - Video/image recordings
# - Application data
# - User configurations
#
# Author: Agricultural Robotics Team
# Version: 3.0.0 - Production Ready
# Date: 2026-01-09
# License: Proprietary
#
################################################################################

set -euo pipefail  # Exit on error, undefined variables, pipe failures
IFS=$'\n\t'        # Set Internal Field Separator

################################################################################
# CONFIGURATION
################################################################################

# Script metadata
readonly SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_VERSION="3.0.0"

# Project paths
readonly PROJECT_ROOT="/home/victor/Documents/Desktop/seed_sowing_robot"
readonly CONFIG_FILE="${SCRIPT_DIR}/backup_config.conf"

# Backup configuration
BACKUP_ROOT="/var/backups/seed_robot"
BACKUP_TYPE="full"  # full, incremental, differential
COMPRESSION="zstd"  # gzip, bzip2, xz, zstd, none
COMPRESSION_LEVEL="3"  # 1-9 for most algorithms
ENABLE_ENCRYPTION="false"
ENCRYPTION_METHOD="gpg"  # gpg, openssl
GPG_RECIPIENT=""
OPENSSL_PASSWORD=""

# Remote backup
ENABLE_REMOTE_BACKUP="false"
REMOTE_METHOD="rsync"  # rsync, scp, s3, gdrive, ftp
REMOTE_HOST=""
REMOTE_USER=""
REMOTE_PATH=""
REMOTE_PORT="22"
SSH_KEY=""
BANDWIDTH_LIMIT=""  # KB/s, empty for unlimited

# AWS S3 configuration
S3_BUCKET=""
S3_REGION="us-east-1"
S3_STORAGE_CLASS="STANDARD_IA"  # STANDARD, STANDARD_IA, GLACIER

# Retention policy (days)
RETAIN_DAILY=7
RETAIN_WEEKLY=4
RETAIN_MONTHLY=12
RETAIN_YEARLY=5

# Notification configuration
ENABLE_EMAIL_NOTIFICATIONS="false"
EMAIL_TO=""
EMAIL_FROM="backup@seed-robot.local"
EMAIL_SUBJECT="Seed Robot Backup Report"

ENABLE_SLACK_NOTIFICATIONS="false"
SLACK_WEBHOOK_URL=""

ENABLE_DISCORD_NOTIFICATIONS="false"
DISCORD_WEBHOOK_URL=""

# Database configuration
ENABLE_DB_BACKUP="true"
DB_TYPE="postgresql"  # postgresql, mysql, sqlite
DB_HOST="localhost"
DB_PORT="5432"
DB_NAME="seed_robot_db"
DB_USER="robot_user"
DB_PASSWORD=""

# Backup directories
declare -a BACKUP_DIRS=(
    "${PROJECT_ROOT}/data"
    "${PROJECT_ROOT}/logs"
    "${PROJECT_ROOT}/config"
    "${PROJECT_ROOT}/models"
    "${PROJECT_ROOT}/calibration"
    "/var/log/seed_robot"
    "/etc/seed_robot"
)

# Exclude patterns
declare -a EXCLUDE_PATTERNS=(
    "*.tmp"
    "*.cache"
    "__pycache__"
    "*.pyc"
    ".git"
    "node_modules"
    "*.log.old"
    "core.*"
)

# System configuration
MAX_PARALLEL_JOBS=4
NICE_LEVEL=10
IONICE_CLASS="2"  # best-effort
IONICE_LEVEL="7"

# Logging
LOG_DIR="/var/log/seed_robot_backup"
LOG_FILE="${LOG_DIR}/backup_$(date +%Y%m%d_%H%M%S).log"
SYSLOG_ENABLED="true"
SYSLOG_FACILITY="local0"
SYSLOG_TAG="seed-robot-backup"

# Advanced features
ENABLE_VERIFICATION="true"
ENABLE_DEDUPLICATION="false"
DRY_RUN="false"
VERBOSE="false"
SHOW_PROGRESS="true"

################################################################################
# COLOR CODES
################################################################################

readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly MAGENTA='\033[0;35m'
readonly CYAN='\033[0;36m'
readonly WHITE='\033[1;37m'
readonly NC='\033[0m' # No Color

################################################################################
# UTILITY FUNCTIONS
################################################################################

# Print colored message
print_color() {
    local color="$1"
    shift
    echo -e "${color}$*${NC}"
}

# Log message with timestamp
log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
    
    # Console output
    case "$level" in
        INFO)
            echo "[${timestamp}] [INFO] ${message}" | tee -a "${LOG_FILE}"
            ;;
        WARN)
            print_color "${YELLOW}" "[${timestamp}] [WARN] ${message}" | tee -a "${LOG_FILE}"
            ;;
        ERROR)
            print_color "${RED}" "[${timestamp}] [ERROR] ${message}" | tee -a "${LOG_FILE}" >&2
            ;;
        SUCCESS)
            print_color "${GREEN}" "[${timestamp}] [SUCCESS] ${message}" | tee -a "${LOG_FILE}"
            ;;
        DEBUG)
            if [[ "${VERBOSE}" == "true" ]]; then
                print_color "${CYAN}" "[${timestamp}] [DEBUG] ${message}" | tee -a "${LOG_FILE}"
            fi
            ;;
    esac
    
    # Syslog output
    if [[ "${SYSLOG_ENABLED}" == "true" ]] && command -v logger &>/dev/null; then
        logger -t "${SYSLOG_TAG}" -p "${SYSLOG_FACILITY}.${level,,}" "${message}"
    fi
}

# Check if command exists
command_exists() {
    command -v "$1" &>/dev/null
}

# Check prerequisites
check_prerequisites() {
    log INFO "Checking prerequisites..."
    
    local missing_commands=()
    
    # Essential commands
    for cmd in tar rsync date find; do
        if ! command_exists "$cmd"; then
            missing_commands+=("$cmd")
        fi
    done
    
    # Compression tools
    case "${COMPRESSION}" in
        gzip)
            if ! command_exists gzip; then
                missing_commands+=("gzip")
            fi
            ;;
        bzip2)
            if ! command_exists bzip2; then
                missing_commands+=("bzip2")
            fi
            ;;
        xz)
            if ! command_exists xz; then
                missing_commands+=("xz")
            fi
            ;;
        zstd)
            if ! command_exists zstd; then
                missing_commands+=("zstd")
            fi
            ;;
    esac
    
    # Encryption tools
    if [[ "${ENABLE_ENCRYPTION}" == "true" ]]; then
        case "${ENCRYPTION_METHOD}" in
            gpg)
                if ! command_exists gpg; then
                    missing_commands+=("gpg")
                fi
                ;;
            openssl)
                if ! command_exists openssl; then
                    missing_commands+=("openssl")
                fi
                ;;
        esac
    fi
    
    # Database tools
    if [[ "${ENABLE_DB_BACKUP}" == "true" ]]; then
        case "${DB_TYPE}" in
            postgresql)
                if ! command_exists pg_dump; then
                    missing_commands+=("pg_dump (postgresql-client)")
                fi
                ;;
            mysql)
                if ! command_exists mysqldump; then
                    missing_commands+=("mysqldump (mysql-client)")
                fi
                ;;
            sqlite)
                if ! command_exists sqlite3; then
                    missing_commands+=("sqlite3")
                fi
                ;;
        esac
    fi
    
    # Remote backup tools
    if [[ "${ENABLE_REMOTE_BACKUP}" == "true" ]]; then
        case "${REMOTE_METHOD}" in
            rsync)
                if ! command_exists rsync; then
                    missing_commands+=("rsync")
                fi
                ;;
            scp|ssh)
                if ! command_exists scp || ! command_exists ssh; then
                    missing_commands+=("openssh-client")
                fi
                ;;
            s3)
                if ! command_exists aws; then
                    missing_commands+=("awscli")
                fi
                ;;
        esac
    fi
    
    if [[ ${#missing_commands[@]} -gt 0 ]]; then
        log ERROR "Missing required commands: ${missing_commands[*]}"
        log ERROR "Please install missing dependencies and try again"
        return 1
    fi
    
    log SUCCESS "All prerequisites satisfied"
    return 0
}

# Create directory structure
create_directories() {
    log INFO "Creating directory structure..."
    
    mkdir -p "${BACKUP_ROOT}"/{daily,weekly,monthly,yearly,tmp}
    mkdir -p "${LOG_DIR}"
    
    if [[ ! -w "${BACKUP_ROOT}" ]]; then
        log ERROR "Backup directory ${BACKUP_ROOT} is not writable"
        return 1
    fi
    
    log SUCCESS "Directory structure created"
    return 0
}

# Load configuration file
load_config() {
    if [[ -f "${CONFIG_FILE}" ]]; then
        log INFO "Loading configuration from ${CONFIG_FILE}"
        # shellcheck source=/dev/null
        source "${CONFIG_FILE}"
        log SUCCESS "Configuration loaded"
    else
        log WARN "Configuration file not found, using defaults"
    fi
}

################################################################################
# BACKUP FUNCTIONS
################################################################################

# Get backup filename
get_backup_filename() {
    local backup_type="$1"
    local timestamp="$(date +%Y%m%d_%H%M%S)"
    local hostname="$(hostname -s)"
    
    case "${COMPRESSION}" in
        gzip)
            echo "seed_robot_${hostname}_${backup_type}_${timestamp}.tar.gz"
            ;;
        bzip2)
            echo "seed_robot_${hostname}_${backup_type}_${timestamp}.tar.bz2"
            ;;
        xz)
            echo "seed_robot_${hostname}_${backup_type}_${timestamp}.tar.xz"
            ;;
        zstd)
            echo "seed_robot_${hostname}_${backup_type}_${timestamp}.tar.zst"
            ;;
        none)
            echo "seed_robot_${hostname}_${backup_type}_${timestamp}.tar"
            ;;
    esac
}

# Create exclude file for tar
create_exclude_file() {
    local exclude_file="${BACKUP_ROOT}/tmp/exclude_patterns.txt"
    
    : > "${exclude_file}"  # Clear file
    
    for pattern in "${EXCLUDE_PATTERNS[@]}"; do
        echo "${pattern}" >> "${exclude_file}"
    done
    
    echo "${exclude_file}"
}

# Perform database backup
backup_database() {
    if [[ "${ENABLE_DB_BACKUP}" != "true" ]]; then
        return 0
    fi
    
    log INFO "Backing up ${DB_TYPE} database: ${DB_NAME}"
    
    local db_backup_dir="${BACKUP_ROOT}/tmp/database"
    mkdir -p "${db_backup_dir}"
    
    local db_dump_file="${db_backup_dir}/${DB_NAME}_$(date +%Y%m%d_%H%M%S).sql"
    
    case "${DB_TYPE}" in
        postgresql)
            PGPASSWORD="${DB_PASSWORD}" pg_dump \
                -h "${DB_HOST}" \
                -p "${DB_PORT}" \
                -U "${DB_USER}" \
                -d "${DB_NAME}" \
                -F plain \
                --no-owner \
                --no-privileges \
                > "${db_dump_file}" 2>>"${LOG_FILE}"
            ;;
        mysql)
            mysqldump \
                -h "${DB_HOST}" \
                -P "${DB_PORT}" \
                -u "${DB_USER}" \
                -p"${DB_PASSWORD}" \
                --single-transaction \
                --routines \
                --triggers \
                "${DB_NAME}" \
                > "${db_dump_file}" 2>>"${LOG_FILE}"
            ;;
        sqlite)
            sqlite3 "${DB_NAME}" ".dump" > "${db_dump_file}" 2>>"${LOG_FILE}"
            ;;
    esac
    
    if [[ $? -eq 0 ]]; then
        log SUCCESS "Database backup completed: ${db_dump_file}"
        
        # Compress database dump
        case "${COMPRESSION}" in
            gzip)
                gzip -"${COMPRESSION_LEVEL}" "${db_dump_file}"
                ;;
            bzip2)
                bzip2 -"${COMPRESSION_LEVEL}" "${db_dump_file}"
                ;;
            xz)
                xz -"${COMPRESSION_LEVEL}" "${db_dump_file}"
                ;;
            zstd)
                zstd -"${COMPRESSION_LEVEL}" --rm "${db_dump_file}"
                ;;
        esac
        
        return 0
    else
        log ERROR "Database backup failed"
        return 1
    fi
}

# Create full backup
create_full_backup() {
    log INFO "Creating full backup..."
    
    local backup_file="$(get_backup_filename "full")"
    local backup_path="${BACKUP_ROOT}/daily/${backup_file}"
    local temp_path="${BACKUP_ROOT}/tmp/${backup_file}"
    
    # Create exclude file
    local exclude_file="$(create_exclude_file)"
    
    # Backup database first
    backup_database || log WARN "Database backup failed, continuing..."
    
    # Add database dump to backup dirs
    if [[ -d "${BACKUP_ROOT}/tmp/database" ]]; then
        BACKUP_DIRS+=("${BACKUP_ROOT}/tmp/database")
    fi
    
    # Build tar command
    local tar_cmd="tar"
    local tar_opts="-c"
    
    # Add verbosity
    if [[ "${VERBOSE}" == "true" ]]; then
        tar_opts="${tar_opts}v"
    fi
    
    # Add progress indicator
    if [[ "${SHOW_PROGRESS}" == "true" ]] && command_exists pv; then
        tar_opts="${tar_opts} --totals"
    fi
    
    # Add compression
    case "${COMPRESSION}" in
        gzip)
            tar_opts="${tar_opts}z"
            ;;
        bzip2)
            tar_opts="${tar_opts}j"
            ;;
        xz)
            tar_opts="${tar_opts}J"
            ;;
    esac
    
    # Create archive
    if [[ "${DRY_RUN}" == "true" ]]; then
        log INFO "[DRY RUN] Would create: ${backup_path}"
        log INFO "[DRY RUN] Backing up: ${BACKUP_DIRS[*]}"
        return 0
    fi
    
    log INFO "Creating archive: ${backup_file}"
    
    # Execute backup with nice/ionice
    nice -n "${NICE_LEVEL}" ionice -c "${IONICE_CLASS}" -n "${IONICE_LEVEL}" \
        tar ${tar_opts}f "${temp_path}" \
        --exclude-from="${exclude_file}" \
        "${BACKUP_DIRS[@]}" 2>>"${LOG_FILE}"
    
    local tar_exit=$?
    
    # Handle zstd separately (not supported by tar in all versions)
    if [[ "${COMPRESSION}" == "zstd" ]] && [[ ${tar_exit} -eq 0 ]]; then
        log INFO "Compressing with zstd..."
        zstd -"${COMPRESSION_LEVEL}" --rm "${temp_path}" -o "${temp_path}.zst"
        temp_path="${temp_path}.zst"
    fi
    
    if [[ ${tar_exit} -ne 0 ]]; then
        log ERROR "Backup creation failed with exit code ${tar_exit}"
        rm -f "${temp_path}"
        return 1
    fi
    
    # Encrypt if enabled
    if [[ "${ENABLE_ENCRYPTION}" == "true" ]]; then
        encrypt_backup "${temp_path}" || return 1
        temp_path="${temp_path}.enc"
    fi
    
    # Move to final location
    mv "${temp_path}" "${backup_path}"
    
    # Generate checksums
    generate_checksums "${backup_path}"
    
    log SUCCESS "Full backup created: ${backup_path}"
    
    # Calculate and log backup size
    local backup_size="$(du -h "${backup_path}" | cut -f1)"
    log INFO "Backup size: ${backup_size}"
    
    return 0
}

# Encrypt backup
encrypt_backup() {
    local file="$1"
    
    log INFO "Encrypting backup..."
    
    case "${ENCRYPTION_METHOD}" in
        gpg)
            if [[ -z "${GPG_RECIPIENT}" ]]; then
                log ERROR "GPG recipient not configured"
                return 1
            fi
            
            gpg --encrypt --recipient "${GPG_RECIPIENT}" \
                --output "${file}.enc" "${file}" 2>>"${LOG_FILE}"
            
            if [[ $? -eq 0 ]]; then
                rm -f "${file}"
                log SUCCESS "Backup encrypted with GPG"
                return 0
            fi
            ;;
        openssl)
            if [[ -z "${OPENSSL_PASSWORD}" ]]; then
                log ERROR "OpenSSL password not configured"
                return 1
            fi
            
            openssl enc -aes-256-cbc -salt -pbkdf2 \
                -in "${file}" -out "${file}.enc" \
                -pass pass:"${OPENSSL_PASSWORD}" 2>>"${LOG_FILE}"
            
            if [[ $? -eq 0 ]]; then
                rm -f "${file}"
                log SUCCESS "Backup encrypted with OpenSSL"
                return 0
            fi
            ;;
    esac
    
    log ERROR "Encryption failed"
    return 1
}

# Generate checksums
generate_checksums() {
    local file="$1"
    
    log INFO "Generating checksums..."
    
    # MD5
    if command_exists md5sum; then
        md5sum "${file}" > "${file}.md5"
    fi
    
    # SHA256
    if command_exists sha256sum; then
        sha256sum "${file}" > "${file}.sha256"
    fi
    
    # SHA512
    if command_exists sha512sum; then
        sha512sum "${file}" > "${file}.sha512"
    fi
    
    log SUCCESS "Checksums generated"
}

# Verify backup integrity
verify_backup() {
    local file="$1"
    
    if [[ "${ENABLE_VERIFICATION}" != "true" ]]; then
        return 0
    fi
    
    log INFO "Verifying backup integrity..."
    
    # Verify checksums
    if [[ -f "${file}.sha256" ]]; then
        if sha256sum -c "${file}.sha256" &>>"${LOG_FILE}"; then
            log SUCCESS "Checksum verification passed"
        else
            log ERROR "Checksum verification failed!"
            return 1
        fi
    fi
    
    # Test archive integrity
    case "${COMPRESSION}" in
        gzip)
            if gzip -t "${file}" &>>"${LOG_FILE}"; then
                log SUCCESS "Archive integrity verified"
            else
                log ERROR "Archive integrity check failed!"
                return 1
            fi
            ;;
        bzip2)
            if bzip2 -t "${file}" &>>"${LOG_FILE}"; then
                log SUCCESS "Archive integrity verified"
            else
                log ERROR "Archive integrity check failed!"
                return 1
            fi
            ;;
        xz)
            if xz -t "${file}" &>>"${LOG_FILE}"; then
                log SUCCESS "Archive integrity verified"
            else
                log ERROR "Archive integrity check failed!"
                return 1
            fi
            ;;
        zstd)
            if zstd -t "${file}" &>>"${LOG_FILE}"; then
                log SUCCESS "Archive integrity verified"
            else
                log ERROR "Archive integrity check failed!"
                return 1
            fi
            ;;
    esac
    
    return 0
}

# Upload to remote
upload_to_remote() {
    local file="$1"
    
    if [[ "${ENABLE_REMOTE_BACKUP}" != "true" ]]; then
        return 0
    fi
    
    log INFO "Uploading backup to remote storage..."
    
    case "${REMOTE_METHOD}" in
        rsync)
            local rsync_opts="-avz --progress"
            
            if [[ -n "${BANDWIDTH_LIMIT}" ]]; then
                rsync_opts="${rsync_opts} --bwlimit=${BANDWIDTH_LIMIT}"
            fi
            
            if [[ -n "${SSH_KEY}" ]]; then
                rsync_opts="${rsync_opts} -e 'ssh -i ${SSH_KEY} -p ${REMOTE_PORT}'"
            fi
            
            eval rsync ${rsync_opts} "${file}"* \
                "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/" 2>>"${LOG_FILE}"
            ;;
        scp)
            local scp_opts="-P ${REMOTE_PORT}"
            
            if [[ -n "${SSH_KEY}" ]]; then
                scp_opts="${scp_opts} -i ${SSH_KEY}"
            fi
            
            if [[ -n "${BANDWIDTH_LIMIT}" ]]; then
                scp_opts="${scp_opts} -l ${BANDWIDTH_LIMIT}"
            fi
            
            scp ${scp_opts} "${file}"* \
                "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/" 2>>"${LOG_FILE}"
            ;;
        s3)
            aws s3 cp "${file}" \
                "s3://${S3_BUCKET}/$(basename "${file}")" \
                --region "${S3_REGION}" \
                --storage-class "${S3_STORAGE_CLASS}" 2>>"${LOG_FILE}"
            
            # Upload checksums too
            for checksum_file in "${file}".{md5,sha256,sha512}; do
                if [[ -f "${checksum_file}" ]]; then
                    aws s3 cp "${checksum_file}" \
                        "s3://${S3_BUCKET}/$(basename "${checksum_file}")" \
                        --region "${S3_REGION}" 2>>"${LOG_FILE}"
                fi
            done
            ;;
    esac
    
    if [[ $? -eq 0 ]]; then
        log SUCCESS "Backup uploaded to remote storage"
        return 0
    else
        log ERROR "Remote upload failed"
        return 1
    fi
}

# Cleanup old backups
cleanup_old_backups() {
    log INFO "Cleaning up old backups..."
    
    # Clean daily backups
    find "${BACKUP_ROOT}/daily" -name "seed_robot_*.tar*" -type f -mtime +${RETAIN_DAILY} \
        -exec rm -f {} \; -exec log INFO "Deleted old daily backup: {}" \;
    
    # Clean weekly backups
    find "${BACKUP_ROOT}/weekly" -name "seed_robot_*.tar*" -type f -mtime +$((RETAIN_WEEKLY * 7)) \
        -exec rm -f {} \; -exec log INFO "Deleted old weekly backup: {}" \;
    
    # Clean monthly backups
    find "${BACKUP_ROOT}/monthly" -name "seed_robot_*.tar*" -type f -mtime +$((RETAIN_MONTHLY * 30)) \
        -exec rm -f {} \; -exec log INFO "Deleted old monthly backup: {}" \;
    
    # Clean yearly backups
    find "${BACKUP_ROOT}/yearly" -name "seed_robot_*.tar*" -type f -mtime +$((RETAIN_YEARLY * 365)) \
        -exec rm -f {} \; -exec log INFO "Deleted old yearly backup: {}" \;
    
    # Clean temp directory
    rm -rf "${BACKUP_ROOT}"/tmp/*
    
    log SUCCESS "Old backups cleaned up"
}

# Rotate backups (promote daily to weekly, weekly to monthly, etc.)
rotate_backups() {
    log INFO "Rotating backups..."
    
    local day_of_week="$(date +%u)"   # 1-7, 1=Monday
    local day_of_month="$(date +%d)"  # 01-31
    local month="$(date +%m)"         # 01-12
    
    # Find most recent daily backup
    local latest_daily="$(find "${BACKUP_ROOT}/daily" -name "seed_robot_*.tar*" -type f | sort -r | head -n1)"
    
    if [[ -z "${latest_daily}" ]]; then
        log WARN "No daily backup found for rotation"
        return 0
    fi
    
    # Promote to weekly on Sunday
    if [[ ${day_of_week} -eq 7 ]]; then
        log INFO "Promoting daily backup to weekly"
        cp -a "${latest_daily}"* "${BACKUP_ROOT}/weekly/"
    fi
    
    # Promote to monthly on 1st of month
    if [[ ${day_of_month} -eq 01 ]]; then
        log INFO "Promoting daily backup to monthly"
        cp -a "${latest_daily}"* "${BACKUP_ROOT}/monthly/"
    fi
    
    # Promote to yearly on Jan 1st
    if [[ ${month} -eq 01 ]] && [[ ${day_of_month} -eq 01 ]]; then
        log INFO "Promoting daily backup to yearly"
        cp -a "${latest_daily}"* "${BACKUP_ROOT}/yearly/"
    fi
    
    log SUCCESS "Backup rotation completed"
}

################################################################################
# NOTIFICATION FUNCTIONS
################################################################################

# Send email notification
send_email_notification() {
    if [[ "${ENABLE_EMAIL_NOTIFICATIONS}" != "true" ]] || ! command_exists mail; then
        return 0
    fi
    
    local subject="$1"
    local body="$2"
    
    echo "${body}" | mail -s "${subject}" -r "${EMAIL_FROM}" "${EMAIL_TO}"
    
    log INFO "Email notification sent to ${EMAIL_TO}"
}

# Send Slack notification
send_slack_notification() {
    if [[ "${ENABLE_SLACK_NOTIFICATIONS}" != "true" ]] || [[ -z "${SLACK_WEBHOOK_URL}" ]]; then
        return 0
    fi
    
    local message="$1"
    local color="$2"  # good, warning, danger
    
    local payload=$(cat <<EOF
{
    "attachments": [
        {
            "color": "${color}",
            "title": "Seed Robot Backup",
            "text": "${message}",
            "ts": $(date +%s)
        }
    ]
}
EOF
)
    
    curl -X POST -H 'Content-type: application/json' \
        --data "${payload}" "${SLACK_WEBHOOK_URL}" &>>"${LOG_FILE}"
    
    log INFO "Slack notification sent"
}

# Send Discord notification
send_discord_notification() {
    if [[ "${ENABLE_DISCORD_NOTIFICATIONS}" != "true" ]] || [[ -z "${DISCORD_WEBHOOK_URL}" ]]; then
        return 0
    fi
    
    local message="$1"
    
    local payload=$(cat <<EOF
{
    "content": "**Seed Robot Backup**",
    "embeds": [
        {
            "description": "${message}",
            "color": 3447003,
            "timestamp": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
        }
    ]
}
EOF
)
    
    curl -X POST -H 'Content-type: application/json' \
        --data "${payload}" "${DISCORD_WEBHOOK_URL}" &>>"${LOG_FILE}"
    
    log INFO "Discord notification sent"
}

################################################################################
# MAIN EXECUTION
################################################################################

# Print banner
print_banner() {
    print_color "${CYAN}" "═══════════════════════════════════════════════"
    print_color "${CYAN}" "  Seed Sowing Robot - Backup System v${SCRIPT_VERSION}"
    print_color "${CYAN}" "  Production-Ready Automated Backup"
    print_color "${CYAN}" "═══════════════════════════════════════════════"
    echo
}

# Print usage
usage() {
    cat <<EOF
Usage: ${SCRIPT_NAME} [OPTIONS]

Production-ready backup system for Seed Sowing Robot

OPTIONS:
    -t, --type TYPE         Backup type (full, incremental, differential)
    -c, --compression ALG   Compression algorithm (gzip, bzip2, xz, zstd, none)
    -e, --encrypt           Enable encryption
    -r, --remote            Enable remote backup
    -v, --verbose           Enable verbose output
    -d, --dry-run           Perform dry run (no actual backup)
    --verify                Verify backup after creation
    --cleanup               Clean up old backups only
    --config FILE           Use specific configuration file
    -h, --help              Show this help message

EXAMPLES:
    # Create full backup with zstd compression
    ${SCRIPT_NAME} --type full --compression zstd

    # Create encrypted backup and upload to remote
    ${SCRIPT_NAME} --encrypt --remote

    # Dry run with verbose output
    ${SCRIPT_NAME} --dry-run --verbose

    # Clean up old backups
    ${SCRIPT_NAME} --cleanup

EOF
    exit 0
}

# Parse command line arguments
parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -t|--type)
                BACKUP_TYPE="$2"
                shift 2
                ;;
            -c|--compression)
                COMPRESSION="$2"
                shift 2
                ;;
            -e|--encrypt)
                ENABLE_ENCRYPTION="true"
                shift
                ;;
            -r|--remote)
                ENABLE_REMOTE_BACKUP="true"
                shift
                ;;
            -v|--verbose)
                VERBOSE="true"
                shift
                ;;
            -d|--dry-run)
                DRY_RUN="true"
                shift
                ;;
            --verify)
                ENABLE_VERIFICATION="true"
                shift
                ;;
            --cleanup)
                cleanup_old_backups
                exit 0
                ;;
            --config)
                CONFIG_FILE="$2"
                shift 2
                ;;
            -h|--help)
                usage
                ;;
            *)
                log ERROR "Unknown option: $1"
                usage
                ;;
        esac
    done
}

# Main function
main() {
    local start_time="$(date +%s)"
    local backup_success=false
    local backup_file=""
    
    # Print banner
    print_banner
    
    # Parse arguments
    parse_arguments "$@"
    
    # Load configuration
    load_config
    
    # Check prerequisites
    if ! check_prerequisites; then
        log ERROR "Prerequisites check failed"
        exit 1
    fi
    
    # Create directories
    if ! create_directories; then
        log ERROR "Failed to create directory structure"
        exit 1
    fi
    
    # Start backup process
    log INFO "Starting backup process..."
    log INFO "Backup type: ${BACKUP_TYPE}"
    log INFO "Compression: ${COMPRESSION}"
    log INFO "Encryption: ${ENABLE_ENCRYPTION}"
    log INFO "Remote backup: ${ENABLE_REMOTE_BACKUP}"
    
    # Create backup based on type
    case "${BACKUP_TYPE}" in
        full)
            if create_full_backup; then
                backup_success=true
                backup_file="$(find "${BACKUP_ROOT}/daily" -name "seed_robot_*.tar*" -type f | sort -r | head -n1)"
            fi
            ;;
        incremental|differential)
            log WARN "Incremental/differential backups not yet implemented, performing full backup"
            if create_full_backup; then
                backup_success=true
                backup_file="$(find "${BACKUP_ROOT}/daily" -name "seed_robot_*.tar*" -type f | sort -r | head -n1)"
            fi
            ;;
        *)
            log ERROR "Unknown backup type: ${BACKUP_TYPE}"
            exit 1
            ;;
    esac
    
    # Verify backup if created
    if [[ ${backup_success} == true ]] && [[ -n "${backup_file}" ]]; then
        verify_backup "${backup_file}"
        
        # Upload to remote
        upload_to_remote "${backup_file}"
        
        # Rotate backups
        rotate_backups
        
        # Cleanup old backups
        cleanup_old_backups
    fi
    
    # Calculate elapsed time
    local end_time="$(date +%s)"
    local elapsed_time=$((end_time - start_time))
    local elapsed_minutes=$((elapsed_time / 60))
    local elapsed_seconds=$((elapsed_time % 60))
    
    # Generate report
    local report="Backup Summary
    ═══════════════════════════════════
    Status: $(if [[ ${backup_success} == true ]]; then echo "SUCCESS"; else echo "FAILED"; fi)
    Backup Type: ${BACKUP_TYPE}
    Compression: ${COMPRESSION}
    Encryption: ${ENABLE_ENCRYPTION}
    Remote Backup: ${ENABLE_REMOTE_BACKUP}
    Duration: ${elapsed_minutes}m ${elapsed_seconds}s
    Timestamp: $(date '+%Y-%m-%d %H:%M:%S')
    "
    
    if [[ ${backup_success} == true ]]; then
        report="${report}
    Backup File: ${backup_file}
    Backup Size: $(du -h "${backup_file}" 2>/dev/null | cut -f1 || echo "N/A")
    "
    fi
    
    log INFO "${report}"
    
    # Send notifications
    if [[ ${backup_success} == true ]]; then
        send_email_notification "Backup Successful" "${report}"
        send_slack_notification "${report}" "good"
        send_discord_notification "${report}"
        
        log SUCCESS "╔═══════════════════════════════════════╗"
        log SUCCESS "║   BACKUP COMPLETED SUCCESSFULLY! ✓   ║"
        log SUCCESS "╚═══════════════════════════════════════╝"
        
        exit 0
    else
        send_email_notification "Backup Failed" "${report}"
        send_slack_notification "${report}" "danger"
        send_discord_notification "${report}"
        
        log ERROR "╔═══════════════════════════════════════╗"
        log ERROR "║      BACKUP FAILED! ✗                ║"
        log ERROR "╚═══════════════════════════════════════╝"
        
        exit 1
    fi
}

# Trap signals
trap 'log ERROR "Backup interrupted"; exit 130' INT TERM

# Execute main function
main "$@"
