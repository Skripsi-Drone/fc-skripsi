#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <bno_qt.h>
#include <control_lqr.h>
#include <actu_setup.h>
#include <TeensyThreads.h>

#define TELEMETRY Serial2
#define USB Serial
#define LED 2
#define M_CALIB false

uint32_t debug_timer = 0;
uint32_t timer_now, timer_last;

void debug_data() {
    // USB.print(millis());
    // USB.print(" ");
    // USB.print("Clb:");
    // USB.print(sys); 
    // USB.print("/"); 
    // USB.print(gyro); 
    // USB.print("/"); 
    // USB.print(accel); 
    // USB.print("/"); 
    // USB.print(mag);
    // USB.print(" ");
    // if (arming) {
    //     USB.print("Arm");
    // } else {
    //     USB.print("Dsm");
    // }
    // USB.print(" Phs:");
    // USB.print(flip_phase);
    USB.print(" R:"); 
    USB.print(roll);
    USB.print(" P:"); 
    USB.print(pitch);
    USB.print(" Y:"); 
    USB.print(yaw);
    // USB.print(" gx:");
    // USB.print(gxrs); //r
    // USB.print(" gy:");
    // USB.print(gyrs); //p
    // USB.print(" gz:");
    // USB.print(gzrs); //y

    // //aksi
    USB.print(" m1:");
    USB.print(motor1_pwm);
    USB.print(",");
    USB.print("m2:");
    USB.print(motor2_pwm);
    USB.print(",");
    USB.print("m3:");
    USB.print(motor3_pwm);
    USB.print(",");
    USB.print("m4:");
    USB.print(motor4_pwm);

    // USB.print(" ch: ");
    // USB.print(ch_throttle);
    // USB.print(" ");
    // USB.print(ch_roll);
    // USB.print(" ");
    // USB.print(ch_pitch);
    // USB.print(" ");
    // USB.print(ch_yaw);
    USB.print(" err:");
    USB.print(error_roll); 
    USB.print(" erp");
    USB.print(error_pitch); 
    USB.print(" ery");
    USB.print(error_yaw);
    USB.print(" spr:");
    USB.print(setpoint_roll);
    USB.print(" spp:");
    USB.print(setpoint_pitch);
    USB.print(" spy:");
    USB.print(setpoint_yaw);
    //motor mixer
    // USB.print("mix:");
    // USB.print(motor_speed_squared[0]); 
    // USB.print(" ");
    // USB.print(motor_speed_squared[1]); 
    // USB.print(" ");
    // USB.print(motor_speed_squared[2]); 
    // USB.print(" ");
    // USB.print(motor_speed_squared[3]);
    USB.println();
}

void telemetry_data() {
        // timer_now = millis();
        // dt = timer_now - timer_last;
        // if (dt >= 100) {
        // TELEMETRY.print(millis());
        // TELEMETRY.print(" ");
        // TELEMETRY.print("Clb:");
        // TELEMETRY.print(sys); 
        // TELEMETRY.print("/"); 
        // TELEMETRY.print(gyro); 
        // TELEMETRY.print("/"); 
        // TELEMETRY.print(accel); 
        // TELEMETRY.print("/"); 
        // TELEMETRY.print(mag);
        // if (arming) {
        //     TELEMETRY.print("Arm");
        // } else {
        //     TELEMETRY.print("Dsm");
        // }
        // TELEMETRY.print(" Phs:");
        // TELEMETRY.print(flip_phase);
        // TELEMETRY.print(" R:"); 
        // TELEMETRY.print(roll);
        // TELEMETRY.print(" P:"); 
        // TELEMETRY.print(pitch);
        // TELEMETRY.print(" Y:"); 
        // TELEMETRY.print(yaw);
        // TELEMETRY.print(" gx:");
        // TELEMETRY.print(gxrs); //r
        // TELEMETRY.print(" gy:");
        // TELEMETRY.print(gyrs); //p
        // TELEMETRY.print(" gz:");
        // TELEMETRY.print(gzrs); //y

        //aksi
        TELEMETRY.print("m1:");
        TELEMETRY.print(motor1_pwm);
        TELEMETRY.print(",");
        TELEMETRY.print("m2:");
        TELEMETRY.print(motor2_pwm);
        TELEMETRY.print(",");
        TELEMETRY.print("m3:");
        TELEMETRY.print(motor3_pwm);
        TELEMETRY.print(",");
        TELEMETRY.print("m4:");
        TELEMETRY.print(motor4_pwm);

        // TELEMETRY.print(" ch: ");
        // TELEMETRY.print(ch_throttle);
        // TELEMETRY.print(" ");
        // TELEMETRY.print(ch_roll);
        // TELEMETRY.print(" ");
        // TELEMETRY.print(ch_pitch);
        // TELEMETRY.print(" ");
        // TELEMETRY.print(ch_yaw);
        // TELEMETRY.print(" err:");
        // TELEMETRY.print(error_roll); 
        // TELEMETRY.print(" erp");
        // TELEMETRY.print(error_pitch); 
        // TELEMETRY.print(" ery");
        // TELEMETRY.print(error_yaw);
        TELEMETRY.println();
        // timer_last = timer_now;
    // }
}

void telemetry_thread() {
    while (true) {
        telemetry_data();
        threads.yield();
    }
    
}

void imu_thread() {
    while (true) {
        bno_update();
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
        // set_control_reference();
        drone_controller();
        threads.yield();
    }
}

void drone_setup() {
    bno_init();
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
    bno_update();
    debug_data();
    // debug_wireless_com();
    // telemetry_data();
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
    

}