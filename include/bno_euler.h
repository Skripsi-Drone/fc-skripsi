// #pragma once
// #include <Arduino.h>
// #include <Wire.h>
// #include <EEPROM.h>
// #include "BNO055_support.h"  //Contains the bridge code between the API and Arduino


// struct bno055_t myBNO;

// unsigned char accelCalibStatus = 0;  // Variable t
// unsigned char magCalibStatus = 0;    // Variable to hold the calibration status of the Magnetometer
// unsigned char gyroCalibStatus = 0;   // Variable to hold the calibration status of the Gyroscope
// unsigned char sysCalibStatus = 0;    // Variable to hold the calibration status of the System (BNO055's MCU)

// unsigned long last_Time = 0;

// struct bno055_euler myEulerData;
// struct bno055_gyro gyroData;
// struct bno055_accel accelData;

// unsigned char sys_calib_status = 0;
// unsigned char mag_calib_status = 0;
// unsigned long last_time = 0;
// float gxrs = 0.0, gyrs = 0.0, gzrs = 0.0;
// float axrs = 0.0, ayrs = 0.0, azrs = 0.0;

// float yaw_sp = 0.0, last_yaw = 0.0;
// float roll = 0.0, pitch = 0.0, yaw = 0.0;
// float roll_error = -0.5, pitch_error = 0.0;

// void check_imu_calibration_NDOF() {
//     while (mag_calib_status != 3 || sys_calib_status != 3) {
//         if ((millis() - last_time) > 200) {
//             bno055_get_magcalib_status(&mag_calib_status);
//             bno055_get_syscalib_status(&sys_calib_status);
//             Serial7.print("Time Stamp: ");
//             Serial7.println(last_time);
//             Serial7.print("Magnetometer Calibration Status: ");
//             Serial7.println(mag_calib_status);
//             Serial7.print("System Calibration Status: ");
//             Serial7.println(sys_calib_status);
//             last_time = millis();
//         }
//     }
// }

// void check_imu_calibration() {
//     while (gyroCalibStatus != 3 || accelCalibStatus != 3) {
//         if ((millis() - last_time) > 200) {
//             bno055_get_gyrocalib_status(&gyroCalibStatus);
//             bno055_get_accelcalib_status(&accelCalibStatus);
//             Serial7.println("--- IMU Calibration Status ---");
//             Serial7.print("Gyroscope: "); Serial7.println(gyroCalibStatus);
//             Serial7.print("Accelerometer: "); Serial7.println(accelCalibStatus);
//             last_time = millis();
//         }
//     }
//     Serial.println("IMU fully calibrated!");
// }

// void bno055_update() {
//     static unsigned long lastUpdate = 0;
//     if (millis() - lastUpdate < 10) return;  // Limit update rate to 100 Hz
//     lastUpdate = millis();

//     bno055_read_gyro_xyz(&gyroData);
//     // Apply simple low-pass filter to gyro data (α = 0.1)
//     gxrs = (float(gyroData.x) / 16.0);
//     gyrs = (float(gyroData.y) / 16.0);
//     gzrs = (float(gyroData.z) / 16.0);

//     // bno055_read_accel_xyz(&accelData);
//     // axrs = float(accelData.x) / 100.0;
//     // ayrs = float(accelData.y) / 100.0;
//     // azrs = float(accelData.z) / 100.0;

//     // x jadi y, y jadi z, yaw z diminus

//     bno055_read_euler_hrp(&myEulerData);
//     roll = (float(myEulerData.r) / 16.00) - roll_error;
//     pitch = (float(myEulerData.p) / 16.00) - pitch_error;
//     yaw = (float(myEulerData.h) / 16.00);
//     if (yaw > 180.0) yaw -= 360.0;
//     // Additional Low-Pass Filter for Yaw
//     yaw = 0.95 * last_yaw + 0.05 * yaw;
//     last_yaw = yaw;
//     yaw_sp = yaw - last_yaw;

  
// }

// void bno055_init() {
//     Wire.begin();
//     BNO_Init(&myBNO);  // Assigning the structure to hold information about the device
//     bno055_set_operation_mode(OPERATION_MODE_NDOF);
//     delay(50);
//     // check_imu_calibration_NDOF();
// }

