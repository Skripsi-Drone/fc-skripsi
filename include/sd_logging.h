#ifndef SD_LOGGING_H
#define SD_LOGGING_H

#include "bno_qt.h"
#include "control_flip_3.h"
#include "fufufuzizizi.h"
#include "radio.h"
#include <SD.h>

#define SD_BUFFER_SIZE 100
#define SD_CHIPSELECT BUILTIN_SDCARD
#define SD_FLUSH_INTERVAL_MS 500

// extern FlipGains flipgain;

struct LogEntry {
    uint32_t timestamp_us;
    uint8_t arming;
    uint8_t flip_switch;
    uint8_t flip_phase;

    float setpoint_flip_roll;
    float roll_relative;
    float roll_absolute;
    float roll;
    float pitch;
    float yaw;
    
    float setpoint_flip_roll_rate;
    float gxrs;
    float gyrs;
    float gzrs;

    float error_roll;
    float error_roll_rate;

    float kr_eff;
    float kp_eff;
    
    uint16_t motor1_pwm;
    uint16_t motor2_pwm;
    uint16_t motor3_pwm;
    uint16_t motor4_pwm;
};

File dataFile;
char filename[32];
bool sd_initialized = false;
bool sd_logging_active = false;

LogEntry log_buffer[SD_BUFFER_SIZE];
volatile int buffer_write_index = 0;
volatile int buffer_read_index = 0;
volatile int buffer_count = 0;

uint32_t last_flush_time = 0;
uint32_t total_logs_written = 0;
uint32_t sd_write_errors = 0;

bool is_buffer_full() {
    return buffer_count >= SD_BUFFER_SIZE;
}

bool is_buffer_empty() {
    return buffer_count == 0;
}

int get_buffer_free_space() {
    return SD_BUFFER_SIZE - buffer_count;
}

bool setup_sd() {
    Serial.println("Initializing SD card...");
    
    if (!SD.begin(SD_CHIPSELECT)) {
        Serial.println("ERROR: SD card init failed!");
        Serial.println("  - Check SD card inserted");
        Serial.println("  - Check FAT32 format");
        Serial.println("  - Try different SD card");
        sd_initialized = false;
        return false;
    }
    
    Serial.println("SD card initialized successfully.");
    
    // Find next available filename
    int fileIndex = 1;
    while (fileIndex < 1000) { 
        sprintf(filename, "flip2_%03d.csv", fileIndex);
        
        if (!SD.exists(filename)) {
            break;  
        }
        fileIndex++;
    }
    
    if (fileIndex >= 1000) {
        Serial.println("ERROR: Too many log files! Clean SD card.");
        sd_initialized = false;
        return false;
    }
    
    // Open file for writing
    dataFile = SD.open(filename, FILE_WRITE);
    if (!dataFile) {
        Serial.print("ERROR: Cannot create file: ");
        Serial.println(filename);
        sd_initialized = false;
        return false;
    }
    
    Serial.print("Logging to: ");
    Serial.println(filename);
    
    dataFile.println("micros,arm,flip_sw,phase,spf_roll,roll_rlv,roll_abs,roll,pitch,yaw,spf_rate,gxrs,gyrs,gzrs,e_roll,e_rrate,kr_eff,kp_eff,m1,m2,m3,m4");
    dataFile.flush();
    
    sd_initialized = true;
    sd_logging_active = true;
    
    Serial.println("SD logging ready!");
    return true;
}

void log_to_buffer() {
    if (!sd_initialized || !sd_logging_active) return;
    
    // Check buffer overflow
    if (is_buffer_full()) {
        sd_write_errors++;
        Serial.println("WARNING: SD buffer overflow! Data loss!");
        return;
    }
    
    // Store data in RAM buffer (FAST - no SD access)
    LogEntry* entry = &log_buffer[buffer_write_index];
    
    entry->timestamp_us = micros();
    entry->arming = arming;
    entry->flip_switch = flip_switch_on;
    entry->flip_phase = fp;
    
    entry->setpoint_flip_roll = setpoint_roll_now;
    entry->roll_relative = roll_relative;
    entry->roll_absolute = roll_absolute;
    entry->roll = roll;
    entry->pitch = pitch;
    entry->yaw = yaw;
    
    entry->setpoint_flip_roll_rate = setpoint_roll_rate_now;
    entry->gxrs = (-gxrs);
    entry->gyrs = gyrs;
    entry->gzrs = gzrs;

    entry->error_roll = error_roll;
    entry->error_roll_rate = error_roll_rate;

    entry->kr_eff = get_kroll_eff();
    entry->kp_eff = get_kp_eff();
    
    entry->motor1_pwm = motor1_pwm;
    entry->motor2_pwm = motor2_pwm;
    entry->motor3_pwm = motor3_pwm;
    entry->motor4_pwm = motor4_pwm;
    
    // Advance buffer index (circular)
    buffer_write_index = (buffer_write_index + 1) % SD_BUFFER_SIZE;
    buffer_count++;
}

// WRITE BUFFER TO SD CARD (Call when buffer nearly full or periodically)
bool flush_buffer_to_sd() {
    if (!sd_initialized || is_buffer_empty()) return true;
    
    uint32_t flush_start = micros();
    int entries_written = 0;
    
    // Write all buffered entries to SD
    while (!is_buffer_empty()) {
        LogEntry* entry = &log_buffer[buffer_read_index];
        
        // Write CSV row (single print for speed)
        char line[256];
        sprintf(line, "%lu,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d",
                entry->timestamp_us,
                entry->arming,
                entry->flip_switch,
                entry->flip_phase,
                entry->setpoint_flip_roll,
                entry->roll_relative,
                entry->roll_absolute,
                entry->roll,
                entry->pitch,
                entry->yaw,
                entry->setpoint_flip_roll_rate,
                entry->gxrs,
                entry->gyrs,
                entry->gzrs,
                entry->error_roll,
                entry->error_roll_rate,
                entry->kr_eff,
                entry->kp_eff,
                entry->motor1_pwm,
                entry->motor2_pwm,
                entry->motor3_pwm,
                entry->motor4_pwm);
        
        if (dataFile.println(line) <= 0) {
            Serial.println("ERROR: SD write failed!");
            sd_write_errors++;
            return false;
        }
        
        // Advance read index
        buffer_read_index = (buffer_read_index + 1) % SD_BUFFER_SIZE;
        buffer_count--;
        entries_written++;
        total_logs_written++;
    }
    
    dataFile.flush();
    
    uint32_t flush_duration = micros() - flush_start;
    
    // Debug info (only print occasionally to avoid spam)
    static uint32_t last_debug = 0;
    if (millis() - last_debug > 5000) {  // Every 5s
        Serial.print("SD: Wrote ");
        Serial.print(entries_written);
        Serial.print(" entries in ");
        Serial.print(flush_duration);
        Serial.print(" us. Total logs: ");
        Serial.print(total_logs_written);
        Serial.print(", Errors: ");
        Serial.println(sd_write_errors);
        last_debug = millis();
    }
    
    return true;
}

// PERIODIC SD MAINTENANCE (Call in main loop)
void sd_logging_update() {
    if (!sd_initialized) return;
    
    uint32_t now = millis();
    
    // Strategy 1: Flush when buffer is 80% full
    if (buffer_count >= (SD_BUFFER_SIZE * 4 / 5)) {
        flush_buffer_to_sd();
    }
    
    // Strategy 2: Periodic flush for safety (prevent data loss in crash)
    if (now - last_flush_time >= SD_FLUSH_INTERVAL_MS) {
        flush_buffer_to_sd();
        last_flush_time = now;
    }
}

// SAFE SHUTDOWN (Call before landing or in emergency)
void close_sd_log() {
    if (!sd_initialized) return;
    
    Serial.println("Closing SD log file...");
    
    // Flush remaining buffer
    flush_buffer_to_sd();
    
    // Write footer with statistics
    dataFile.println();
    dataFile.print("# Total entries: ");
    dataFile.println(total_logs_written);
    dataFile.print("# Write errors: ");
    dataFile.println(sd_write_errors);
    dataFile.print("# Flight duration: ");
    dataFile.print((micros() - log_buffer[0].timestamp_us) / 1000000.0);
    dataFile.println(" seconds");
    
    dataFile.flush();
    dataFile.close();
    
    sd_logging_active = false;
    
    Serial.print("SD log closed: ");
    Serial.println(filename);
    Serial.print("Total logs: ");
    Serial.println(total_logs_written);
}

// START/STOP LOGGING (For selective logging)
void start_sd_logging() {
    if (sd_initialized) {
        sd_logging_active = true;
        Serial.println("SD logging STARTED");
    }
}

void stop_sd_logging() {
    sd_logging_active = false;
    flush_buffer_to_sd(); 
    Serial.println("SD logging STOPPED");
}

// DIAGNOSTIC FUNCTIONS
void print_sd_status() {
    Serial.println("=== SD LOGGING STATUS ===");
    Serial.print("Initialized: ");
    Serial.println(sd_initialized ? "YES" : "NO");
    Serial.print("Active: ");
    Serial.println(sd_logging_active ? "YES" : "NO");
    Serial.print("Filename: ");
    Serial.println(filename);
    Serial.print("Buffer usage: ");
    Serial.print(buffer_count);
    Serial.print(" / ");
    Serial.println(SD_BUFFER_SIZE);
    Serial.print("Total logs: ");
    Serial.println(total_logs_written);
    Serial.print("Errors: ");
    Serial.println(sd_write_errors);
    Serial.println("========================");
}

#endif