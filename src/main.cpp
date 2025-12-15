#include <Arduino.h>
// #include <control_lqr.h>
#include <fc_euler.h>
#include <actu_setup.h>
#include <TeensyThreads.h>
#include <bno_euler.h>
// #include "bno_quaternion.h"

#define TELEMETRY Serial2
#define USB Serial
#define LED 2
#define M_CALIB false

uint32_t debug_timer = 0;
uint32_t timer_now, timer_last;

void debug_data() {
    // USB.print ("motor1_pwm:");
    // USB.print(motor1_pwm);
    // USB.print (",");
    // USB.print ("motor2_pwm:");
    // USB.print(motor2_pwm);
    // USB.print (",");
    // USB.print ("motor3_pwm:");
    // USB.print(motor3_pwm);
    // USB.print (",");
    // USB.print ("motor4_pwm:");
    // USB.print(motor4_pwm);
    // USB.print (", ");
    // USB.print("TS: ");
    // USB.print(data.ch[i]);
    // USB.print(" Mode: ");
    // USB.print(flight_mode);
    // USB.print(" SPR: ");
    // USB.print(setpoint_roll);
    USB.print("Roll:");
    USB.print(roll);
    USB.print(",");
    // USB.print(" Rdot: ");
    // USB.print(gxrs);
    // USB.print(" SPP: ");
    // USB.print(setpoint_pitch);
    USB.print("Pitch:");
    USB.print(pitch);
    USB.print(",");
    // USB.print(" SPY: ");
    // USB.print(setpoint_yaw);
    // USB.print("Yaw:");
    // USB.print(yaw);
    // USB.print(",");

    // USB.print(" qw: ");
    // USB.print(qw, 4); // Cetak dengan 4 desimal
    // USB.print(" qx: ");
    // USB.print(qx, 4);
    // USB.print(" qy: ");
    // USB.print(qy, 4);
    // USB.print(" qz: ");
    // USB.print(qz, 4);
    // USB.print("gxrs:");
    // USB.print(gxrs);
    // USB.print(",");
    // USB.print("gyrs:");
    // USB.print(gyrs);
    // USB.print(",");
    // USB.print("gzrs:");
    // USB.print(gzrs);
    // USB.print(",");

    // cek dulu bedanya kalau u sama mx_pwm apa
    // USB.print(" u1: ");
    // USB.print(u1);
    // USB.print(" u2: ");
    // USB.print(u2);
    // USB.print(" u3: ");
    // USB.print(u3);
    // USB.print(" u4: ");
    // USB.println(u4);
    USB.println();
}

void telemetry_data() {
        timer_now = millis();
        dt = timer_now - timer_last;
        if (dt >= 50) {
        // TELEMETRY.print(millis());
        // TELEMETRY.print(" ; ");
        // if (arming) {
        //     TELEMETRY.print("Armed");
        // } else {
        //     TELEMETRY.print("Disarmed");
        // }
        // TELEMETRY.print(",");
        // TELEMETRY.print(" Mod: ");
        //blm di define ini
        // TELEMETRY.print(flight_mode);
        // TELEMETRY.print(" ; ");
        // TELEMETRY.print(" SPR: ");
        TELEMETRY.print(setpoint_roll);
        TELEMETRY.print(",");
        // TELEMETRY.print("R:");
        TELEMETRY.print(roll);
        TELEMETRY.print(",");
        // TELEMETRY.print(",");
        // TELEMETRY.print(" SPP: ");
        TELEMETRY.print(setpoint_pitch);
        TELEMETRY.print(",");
        // TELEMETRY.print("P:");
        TELEMETRY.print(pitch);
        TELEMETRY.print(",");
        // TELEMETRY.print("Gx:");
        // TELEMETRY.print("Gy:");
        // TELEMETRY.print(" SPY: ");
        TELEMETRY.print(setpoint_yaw);
        TELEMETRY.print(",");
        // TELEMETRY.print("Y:");
        TELEMETRY.print(yaw);
        TELEMETRY.print(",");
        TELEMETRY.print(gyrs);
        TELEMETRY.print(",");
        TELEMETRY.print(gxrs);
        TELEMETRY.print(",");
        TELEMETRY.print(gzrs);
        TELEMETRY.print(",");
        // TELEMETRY.print("Gz:");
        // TELEMETRY.print ("m1_pwm:");
        TELEMETRY.print(motor1_pwm);
        TELEMETRY.print(",");
        // // TELEMETRY.print ("m2_pwm:");
        TELEMETRY.print(motor2_pwm);
        TELEMETRY.print(",");
        // // TELEMETRY.print ("m3_pwm:");
        TELEMETRY.print(motor3_pwm);
        TELEMETRY.print(",");
        // // TELEMETRY.print ("m4_pwm:");
        TELEMETRY.println(motor4_pwm);
        // TELEMETRY.println("");
        timer_last = timer_now;
    }

}

void debug_wireless_com(){
    Serial.print("Package : <");
    Serial.print(arming);
    Serial.print(",");
    Serial.print(ch_roll);
    Serial.print(",");
    Serial.print(ch_pitch);
    Serial.print(",");
    Serial.print(ch_throttle);
    Serial.print(",");
    Serial.print(ch_yaw);
    Serial.print(">");
    Serial.print("  Latency: ");
    Serial.print(arrival_time);
    Serial.println(" ms");
}
void telemetry_thread() {
    while (true) {
        telemetry_data();
        // debug_data();
        threads.yield();
    }
    
}

void imu_thread() {
    while (true) {
        bno055_update();
        threads.yield();
    }
}

void remote_thread() {
    while (true) {
        remote_loop();
        threads.yield();
    }
}

void control_thread() {
    while (true) {
        set_control_reference();
        drone_controller();
        threads.yield();
    }
}

void drone_setup() {
    bno055_init();
    init_motors();
    remote_setup();
}

void setup() {
    TELEMETRY.begin(57600);
    USB.begin(115200);

    drone_setup();

    if (M_CALIB) {
        motor_calibration();
    }

    // threads.addThread(imu_thread, 1);
    // threads.addThread(remote_thread, 2);
    // threads.addThread(telemetry_thread, 1);
    
    pinMode(LED, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    remote_loop();
    bno055_update();
    debug_wireless_com();
    telemetry_data();
    set_control_reference();
    drone_controller();
    // thrust_check(ch_throttle);
    writeMotors(motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm);
    // if (millis() - debug_timer >= 100) { 
    //     debug_data(); // Mencetak ke USB (Serial Monitor)
        // debug_timer = millis(); // Reset timer
    // }
    // if (millis() % 100 == 0){
    // if ((micros - debug_timer) > 10000){
    //     telemetry_data();
    //     debug_timer = micros();
    // }
    // telemetry_data();
    // }
    
    // writeMotors(motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm);

}