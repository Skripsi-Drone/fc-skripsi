#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

Adafruit_BNO055 bno;
#define BNO055_SAMPLERATE_DELAY_MS 20 
#define RAD_TO_DEG 57.295779513082320876798154814105

uint8_t sys, gyro, accel, mag;
float gxrs, gyrs, gzrs;
float roll, pitch, yaw;
float yaw_sp, last_yaw;
float yaw_offset = 0; 
bool initial_tare_done = false;

void bno_init() {
    Serial.begin(115200);
    
    // PERBAIKAN 1: Hapus while(!Serial) atau beri timeout
    // Tunggu Serial max 2 detik, kalau tidak ada (pake baterai) lanjut jalan.
    unsigned long start_wait = millis();
    while (!Serial && (millis() - start_wait < 2000)) {
        delay(10);
    }
    
    // Debug print
    Serial.println("Initializing BNO055...");

    // PERBAIKAN 2: Handling jika BNO tidak terdeteksi
    if (!bno.begin())
    {
      Serial.print("No BNO055 detected!");
      // Beri tanda error fisik (misal LED kedip cepat) supaya tau ini error hardware
      while (1) {
          digitalWrite(13, HIGH); // Asumsi LED Builtin di pin 13 (Teensy 4.1)
          delay(100);
          digitalWrite(13, LOW);
          delay(100);
      }
    }   
    
    bno.setExtCrystalUse(true);
    Serial.println("System Started. Waiting for Calibration...");
}

void bno_update() {
    sensors_event_t gyroEvent;
    bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE); 
    imu::Quaternion quat = bno.getQuat();
    // sensors_event_t magEvent;
    // bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);  
    //read gyro & swap
    float gyro_x = gyroEvent.gyro.x;
    float gyro_y = gyroEvent.gyro.y;
    float gyro_z = gyroEvent.gyro.z;    
    gxrs = gyro_y;
    gyrs = -gyro_x;
    gzrs = gyro_z;  
    //read qt & swap
    float qw = quat.w();
    float qx = quat.x();
    float qy = quat.y();
    float qz = quat.z();
    // AXIS SWAPPING Rotasi 90 derajat CW: Sensor X -> Body Y, Sensor Y -> Body -X
    float qx_b =  qy; 
    float qy_b = -qx;
    float qz_b =  qz;
    float qw_b =  qw;   
    // read magneto & swap
    // float magx_b =  magEvent.magnetic.y;  
    // float magy_b = -magEvent.magnetic.x;  
    // float magz_b =  magEvent.magnetic.z;
    
    // konversi ke euler radian
    float roll_rad  = atan2(2*(qw_b*qx_b + qy_b*qz_b), 1 - 2*(qx_b*qx_b + qy_b*qy_b));
    float pitch_rad = asin(constrain(2*(qw_b*qy_b - qz_b*qx_b), -1, 1));
    float yaw_rad = atan2(2 * (qw_b * qz_b + qx_b * qy_b), 1 - 2 * (qy_b * qy_b + qz_b * qz_b));

    //Yaw dengan magnetometer
    // float magy_horiz = magy_b * cos(roll_rad) - magz_b * sin(roll_rad);
    // float magz_horiz = magy_b * sin(roll_rad) + magz_b * cos(roll_rad);
    // float magx_horiz = magx_b * cos(pitch_rad) + magz_horiz * sin(pitch_rad);
    // float yaw_rad = atan2(magy_horiz, magx_horiz);  

    // konversi euler radian ke derajat
    roll  = -1 * (roll_rad * RAD_TO_DEG); //krn kanan kiri kebalik pos/neg nya
    pitch = pitch_rad * RAD_TO_DEG; 
    float yaw_raw = yaw_rad * RAD_TO_DEG;
    if(yaw_raw < 0) yaw_raw += 360; // Normalisasi 0-360

    //Auto Tare (Set 0 saat kalibrasi selesai)
    sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);  
    if (mag == 3 && accel == 3 && !initial_tare_done) {
       yaw_offset = yaw_raw; 
       initial_tare_done = true;
    }
    yaw = yaw_raw - yaw_offset;
    while(yaw < 0) yaw += 360;
    while(yaw >= 360) yaw -= 360;

    // 2. Potong range: Jika lebih dari 180, kurangi 360 agar jadi negatif
    if (yaw > 180) {
        yaw -= 360;
    }
    yaw = 0.95 * last_yaw + 0.05 * yaw; // low pass filter biar bacanya ga responsif bgt
    last_yaw = yaw;
    yaw_sp = yaw - last_yaw;
}