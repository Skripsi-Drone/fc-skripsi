#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <bno_qt.h>
#include <control_lqr.h>
#include <actu_setup.h>
#include <TeensyThreads.h>
#include <tuningtelem.h>

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
    USB.print(" gx:");
    USB.print(gxrs); //r
    USB.print(" gy:");
    USB.print(gyrs); //p
    USB.print(" gz:");
    USB.print(gzrs); //y

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
    static bool last_arming = false;

    // Logika Event ARM/DISARM biarkan terpisah atau ikut streaming (opsional)
    // Di sini kita biarkan event dikirim terpisah seperti kode asli Anda
    if (arming != last_arming) {
        if (arming) {
            TELEMETRY.println("ARM");
        } else {
            TELEMETRY.println("DISARM");
        }
        last_arming = arming;
    }

    // --- MULAI PAKET DATA ---
    // HEAD (Tanda Awal)
    TELEMETRY.print("<"); 

    TELEMETRY.print(millis()); TELEMETRY.print(" ");  
    TELEMETRY.print(arming); TELEMETRY.print(" ");

    TELEMETRY.print(roll); TELEMETRY.print(" ");
    TELEMETRY.print(pitch); TELEMETRY.print(" ");
    TELEMETRY.print(yaw); TELEMETRY.print(" ");

    TELEMETRY.print(gxrs); TELEMETRY.print(" ");
    TELEMETRY.print(gyrs); TELEMETRY.print(" ");
    TELEMETRY.print(gzrs); TELEMETRY.print(" ");

    TELEMETRY.print(error_roll); TELEMETRY.print(" ");
    TELEMETRY.print(error_pitch); TELEMETRY.print(" ");
    TELEMETRY.print(error_yaw); TELEMETRY.print(" ");

    TELEMETRY.print(motor1_pwm); TELEMETRY.print(" ");
    TELEMETRY.print(motor2_pwm); TELEMETRY.print(" ");
    TELEMETRY.print(motor3_pwm); TELEMETRY.print(" ");
    TELEMETRY.print(motor4_pwm); // Hapus spasi terakhir agar rapi

    // TAIL (Tanda Akhir) + Newline
    TELEMETRY.println(">"); 
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
    pinMode(LED, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    remote_loop();
    bno_update();
    // debug_data();
    // telemetry_data();
    telemetry_gain_tuning();
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

}