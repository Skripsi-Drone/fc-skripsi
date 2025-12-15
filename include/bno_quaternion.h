

// pas dicoba di output kontrol rada ancur y wkwkw
#pragma once
#include <Arduino.h>
#include <Wire.h>
// #include <imu.h>
#include <EEPROM.h>
#include "BNO055_support.h"  //Contains the bridge code between the API and Arduino

float vertical_velocity = 0.1;
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float gxrs = 0.0, gyrs = 0.0, gzrs = 0.0;
float accx = 0.0, accy = 0.0, accz = 0.0;
float yaw_sp = 0.0, last_yaw = 0.0;
float roll_error = 0.0, pitch_error = 0.0;
float qw = 0.0, qx = 0.0, qy = 0.0, qz = 0.0;
// float roll_error = 0.00, pitch_error = 0.00;
//  r:-1.19 p:2.56 y:360.00
struct bno055_t myBNO;
unsigned char accelCalibStatus = 0;  // Variable t
//o hold the calibration status of the Accelerometer
unsigned char magCalibStatus = 0;    // Variable to hold the calibration status of the Magnetometer
unsigned char gyroCalibStatus = 0;   // Variable to hold the calibration status of the Gyroscope
unsigned char sysCalibStatus = 0;    // Variable to hold the calibration status of the System (BNO055's MCU)

unsigned long last_Time = 0;

struct bno055_euler myEulerData;
struct bno055_gyro gyroData;
struct bno055_accel accelData;
struct bno055_quaternion quatData;
#define QUAT_SCALING_FACTOR 16384.0f

unsigned char sys_calib_status = 0;
unsigned char mag_calib_status = 0;
unsigned long last_time = 0;

void check_imu_calibration_NDOF() {
    while (mag_calib_status != 3 || sys_calib_status != 3) {
        if ((millis() - last_time) > 200) {
            bno055_get_magcalib_status(&mag_calib_status);
            bno055_get_syscalib_status(&sys_calib_status);
            Serial2.print("Time Stamp: ");
            Serial2.println(last_time);
            Serial2.print("Magnetometer Calibration Status: ");
            Serial2.println(mag_calib_status);
            Serial2.print("System Calibration Status: ");
            Serial2.println(sys_calib_status);
            last_time = millis();
        }
    }
}

// Function to calibrate BNO055
void check_imu_calibration() {
    while (gyroCalibStatus != 3 || accelCalibStatus != 3) {
        if ((millis() - last_time) > 200) {
            bno055_get_gyrocalib_status(&gyroCalibStatus);
            bno055_get_accelcalib_status(&accelCalibStatus);
            Serial2.println("--- IMU Calibration Status ---");
            Serial2.print("Gyroscope: "); Serial2.println(gyroCalibStatus);
            Serial2.print("Accelerometer: "); Serial2.println(accelCalibStatus);
            last_time = millis();
        }
    }
    Serial.println("IMU fully calibrated!");
}

void bno055_update() {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate < 10) return;  // Limit update rate to 100 Hz
    lastUpdate = millis();

    // 1. PEMBACAAN GYRO & QUATERNION (Tetap)
    bno055_read_gyro_xyz(&gyroData);
    gxrs = -1 * (float(gyroData.x) / 16.0); // Kecepatan Roll (p)
    gyrs = (float(gyroData.y) / 16.0); // Kecepatan Pitch (q)
    gzrs = (float(gyroData.z) / 16.0); // Kecepatan Yaw (r)

    // bno055_read_accel_xyz(&accelData);
    // accx = (float)accelData.x;
    // accy = (float)accelData.y;
    // accz = (float)accelData.z;

    bno055_read_quaternion_wxyz(&quatData);
    qw = (float(quatData.w) / QUAT_SCALING_FACTOR);
    qx = (float(quatData.x) / QUAT_SCALING_FACTOR);
    qy = (float(quatData.y) / QUAT_SCALING_FACTOR);
    qz = (float(quatData.z) / QUAT_SCALING_FACTOR);

    float qx_in = qy;
    float qy_in = qx;
    float qz_in = qz;

    // 2. PERHITUNGAN QUATERNION (Tetap)
    float qw2 = qw * qw;
    float qx2 = qx_in * qx_in;
    float qy2 = qy_in * qy_in;
    float qz2 = qz_in * qz_in;
    float qwx = qw * qx;
    float qwy = qw * qy;
    float qwz = qw * qz;
    float qxy = qx * qy;
    float qxz = qx * qz;
    float qyz = qy * qz;

    // 4. Hitung Euler Angles (Rumus Z-Y-X)
    // Roll (x-axis rotation)
    float t0 = +2.0f * (qw * qx_in + qy_in * qz_in);
    float t1 = +1.0f - 2.0f * (qx2 + qy2);
    float roll_rad = atan2(t0, t1);

    // Pitch (y-axis rotation)
    float t2 = +2.0f * (qw * qy_in - qz_in * qx_in);
    // Safety clamp untuk asin (menghindari NaN jika t2 > 1 akibat noise)
    if (t2 > 1.0f) t2 = 1.0f;
    if (t2 < -1.0f) t2 = -1.0f;
    float pitch_rad = asin(t2);

    // Yaw (z-axis rotation)
    float t3 = +2.0f * (qw * qz_in + qx_in * qy_in);
    float t4 = +1.0f - 2.0f * (qy2 + qz2);
    float yaw_rad = atan2(t3, t4);

    // 5. Konversi ke Derajat & Update Variabel Global
    roll  = roll_rad * RAD_TO_DEG;
    pitch = pitch_rad * RAD_TO_DEG;
    yaw   = yaw_rad * RAD_TO_DEG;

    // // --- 3. KONVERSI KE RADIAN (Standard ZYX Sequence) ---
    // // Pitch (arcsin) paling rentan terhadap Gimbal Lock
    // float pitch_rad_calc = asin(2.0f * (qwy - qxz)); 
    // float roll_rad_calc = atan2(2.0f * (qwx + qyz), 1.0f - 2.0f * (qx2 + qy2));
    // float yaw_rad_calc = atan2(2.0f * (qwz + qxy), 1.0f - 2.0f * (qy2 + qz2));

    // // --- 4. PENYESUAIAN SUMBU FISIK (AXIS REMAPPING) ---
    // // Simpan hasil konversi ke variabel sementara
    // float roll_deg_temp = roll_rad_calc * (180.0f / PI);
    // float pitch_deg_temp = pitch_rad_calc * (180.0f / PI);
    // float yaw_deg_temp = yaw_rad_calc * (180.0f / PI);

    // roll = pitch_deg_temp;  
    // pitch = roll_deg_temp;
    // yaw = yaw_deg_temp;
    
    // --- 5. NORMALISASI YAW (Opsional) ---
    // if (roll < -180.0f) roll += 360.0f;
    // Karena Yaw sudah dari Quaternion, ia sudah kontinu. Kita hanya membatasi display ke +/- 180.
    // if (yaw > 180.0f) yaw -= 360.0f;
    // if (yaw < -180.0f) yaw += 360.0f;

    // --- 6. REMAPPING KECEPATAN SUDUT (WAJIB JIKA SUMBU DITUKAR) ---
    // Jika roll dan pitch ditukar di atas, maka gxrs dan gyrs harus ditukar:
    // float gxrs_temp = gxrs;
    // gxrs = gyrs; // gxrs (Roll dot) mengambil gyrs (Pitch dot)
    // gyrs = gxrs_temp; // gyrs (Pitch dot) mengambil gxrs (Roll dot) lama
}

void bno055_init() {
    // Initialize I2C communication
    Wire.begin();
    // Initialization of the BNO055
    BNO_Init(&myBNO);  // Assigning the structure to hold information about the device
    // Configuration to IMUPLUS mode
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    delay(50);
    // check_imu_calibration_NDOF();
}


// /*
// // PROGRAMNYA FC kalo yg atas jelek
// #pragma once

// #include <Arduino.h>
// #include <AP_Math.h>
// #include <BNO055_support.h>
// #include <TeensyThreads.h>
// #include "I2CDev.h"

// // Representation Unit Settings
// #define ACCEL_TO_MS2        100
// #define ACCEL_TO_MG         1
// #define MAG_TO_MIKROTESLA   16
// #define GYRO_TO_DPS         16
// #define GYRO_TO_RPS         900
// #define EULER_TO_DEG        16
// #define EULER_TO_RAD        900
// #define QUAT_UNIT_LESS      16384
// #define GRAVITY_TO_MS2      100
// #define GRAVITY_TO_MG       1
// #define TEMP_TO_CELC        1
// #define TEMP_TO_FAHR        2


// class IMU{
//     public:
//         Vector3f _accel;
//         Vector3f _linear_acc;
//         float roll, pitch, heading, yaw;
//         float rad_roll, rad_pitch, rad_yaw;
//         float last_roll, last_pitch, last_yaw;
//         float delta_roll, delta_pitch, delta_yaw;
//         float gyro_x, gyro_y, gyro_z;
//         int pitch_mode;

//         void init_imu();
//         void update_imu();
//         void check_imu_calibration();
//         void imu_thread();
//         void imu_task();

//     private:
//         struct bno055_t bno_data;              
//         struct bno055_euler bno_euler;     
//         struct bno055_gyro bno_gyro;      
//         struct bno055_accel bno_accel;
//         struct bno055_quaternion bno_quat;
//         struct bno055_linear_accel bno_linear;

//         unsigned char sys_calib_status = 0;
//         unsigned char mag_calib_status = 0;
//         unsigned long last_time = 0;

//         float invert_heading(float raw_h);
//         float calc_yaw(float real_h);
//         void extract_euler();
//         void extract_gyroscope();
//         void extract_accelerometer();
//         void extract_linear_accelerometer();
// };

// void IMU::check_imu_calibration() {
//     while (mag_calib_status != 3 || sys_calib_status != 3) {
//         if ((millis() - last_time) > 200) {
//             bno055_get_magcalib_status(&mag_calib_status);
//             bno055_get_syscalib_status(&sys_calib_status);
//             Serial2.print("Time Stamp: ");
//             Serial2.println(last_time);
//             Serial2.print("Magnetometer Calibration Status: ");
//             Serial2.println(mag_calib_status);
//             Serial2.print("System Calibration Status: ");
//             Serial2.println(sys_calib_status);
//             last_time = millis();
//         }
//     }
// }

// float IMU::invert_heading(float raw_h) {
//     float real_h = raw_h - 180.0;
//     if (real_h < 0) real_h += 360;
//     return real_h;
// }

// float IMU::calc_yaw(float real_h) {
//     return real_h > 180 ? real_h - 360.0f : real_h;
// }

// void IMU::extract_euler() {
//     bno055_read_euler_hrp(&bno_euler);
    
//     //? Convert to degree
//     roll = (float)bno_euler.r / EULER_TO_DEG;
//     pitch = ((float)bno_euler.p / EULER_TO_DEG);

//     // if (pitch_mode == 1) {
//     //     pitch = (float)bno_euler.p / EULER_TO_DEG;
//     // } else if (pitch_mode == 2 ) {
//     //     pitch = ((float)bno_euler.p / EULER_TO_DEG) - 90;
//     // }

//     heading = invert_heading((float)bno_euler.h / EULER_TO_DEG);
//     yaw = calc_yaw(heading);

//     //? Convert to radians
//     rad_roll = (float)bno_euler.r / EULER_TO_RAD;
//     rad_pitch = (float)bno_euler.p / EULER_TO_RAD;
//     rad_yaw = radians(yaw);
// }

// void IMU::extract_gyroscope() {
//     bno055_read_gyro_xyz(&bno_gyro);
//     gyro_x = ((float)bno_gyro.x / GYRO_TO_DPS);
//     gyro_y = -1 * ((float)bno_gyro.y / GYRO_TO_DPS);
//     gyro_z = ((float)bno_gyro.z / GYRO_TO_DPS);
// }

// void IMU::extract_accelerometer() {
//     bno055_read_accel_xyz(&bno_accel);
//     _accel.x = (float)bno_accel.x;
//     _accel.y = (float)bno_accel.y;
//     _accel.z = (float)bno_accel.z;
// }

// void IMU::extract_linear_accelerometer() {
//     bno055_read_linear_accel_xyz(&bno_linear);
//     _linear_acc.x = (float)bno_linear.x / ACCEL_TO_MS2;
//     _linear_acc.y = (float)bno_linear.y / ACCEL_TO_MS2;
//     _linear_acc.z = (float)bno_linear.z / ACCEL_TO_MS2;
// }

// void IMU::init_imu() {
//     _I2C.init();
//     BNO_Init(&bno_data);
//     bno055_set_operation_mode(OPERATION_MODE_NDOF);
//     delay(1 << 9);
// }

// void IMU::update_imu() {
//     extract_linear_accelerometer();
//     extract_euler();
//     extract_gyroscope();
//     extract_accelerometer();

//     delta_roll = abs(roll - last_roll);
//     delta_pitch = abs(pitch - last_pitch);
//     delta_yaw = abs(yaw - last_yaw);

//     last_roll = roll;
//     last_pitch = pitch;
//     last_yaw = yaw;
// }
// */