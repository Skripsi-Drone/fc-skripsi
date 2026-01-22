#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <bno_qt.h>
#include <actu_setup.h>
#include <radio.h>
// #include <control_lqr.h>
#include <control_flip_2.h>
#include <TeensyThreads.h>
#include <tuningtelem.h>

#define TELEMETRY Serial2
#define USB Serial
#define LED 2
#define M_CALIB false

uint32_t debug_timer = 0;
uint32_t timer_now, timer_last;

void debug_data() {
    USB.print(millis());
    USB.print(" ");
    USB.print(rategain.P, 2);
    USB.print(" ");
    USB.print(rategain.D, 2); 
    USB.print(" "); 
    USB.print(rategain.I, 3); 
    USB.print(" "); 
    USB.print(rategain.IMAX); 
    USB.print(" "); 
    USB.print(rategain.max_rate);
    USB.print(" R:");
    USB.print(roll);
    USB.print(" "); 
    USB.print(gxrs);
    USB.print(" "); 
    USB.print(setpoint_roll_rate);
    USB.print(" "); 
    USB.print(er);
    USB.print(" "); 

    // USB.print(flip_pulse_delta, 3);
    // USB.print(" ");
    // USB.print(flip_climb_ms, 3);
    // USB.print(" ");
    // USB.print(U2_MAX, 5);

    USB.println();
}

void telemetry_data() {
    TELEMETRY.print("<"); 
    TELEMETRY.print(millis());  TELEMETRY.print(" ");  
    TELEMETRY.print(arming);    TELEMETRY.print(" ");
    // TELEMETRY.print(fp);        TELEMETRY.print(" ");
    
    // TELEMETRY.print(desired_roll_rate);      TELEMETRY.print(" "); //flip
    TELEMETRY.print(setpoint_roll_rate);      TELEMETRY.print(" "); //hover

    // TELEMETRY.print(roll);   TELEMETRY.print(" ");
    // TELEMETRY.print(pitch);  TELEMETRY.print(" ");
    // TELEMETRY.print(yaw);    TELEMETRY.print(" ");

    TELEMETRY.print(gxrs);      TELEMETRY.print(" ");
    // TELEMETRY.print(gyrs);   TELEMETRY.print(" ");
    // TELEMETRY.print(gzrs);   TELEMETRY.print(" ");

    TELEMETRY.print(u2,8);      TELEMETRY.print(" ");
    TELEMETRY.print(roll);      TELEMETRY.print(" ");

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
    // Flip
    if (!arming) {
        ctrl_mode = MODE_HOVER;
        fp = IDLE;
        flip_phase_start = 0;
        desired_roll_rate = 0.0f;
    }
    if (flip_event && ctrl_mode == MODE_HOVER && ch_throttle > 1100) {
        ctrl_mode = MODE_FLIP;
        fp = IDLE;
        flip_event = false;   // WAJIB reset event
    }
    if (ctrl_mode == MODE_FLIP) {
        if (!flip_switch_on) {
            ctrl_mode = MODE_HOVER;
            fp = IDLE;
            flip_pulse_active = false;
            desired_roll_rate = 0.0f;
            Serial.println("FLIP ABORTED BY SWITCH");
        }
        else {
            update_flip();
        }
    }
    // set_control_reference(); //angle control
    drone_controller();
    // thrust_check(ch_throttle);
    writeMotors(motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm);
    // static uint32_t telemetry_timer = 0;
    // if (millis() - telemetry_timer >= 10) {   // 100 Hz
    //     telemetry_data();
    //     telemetry_timer = millis();
    // }

}