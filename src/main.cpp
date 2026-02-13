#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <bno_qt.h>
#include <actu_setup.h>
#include <radio.h>
#include <control_flip_3.h>
#include <TeensyThreads.h>
#include <tuningtelem.h>
#include <sd_logging.h>

#define TELEMETRY Serial2
#define USB Serial
#define LED 2
#define M_CALIB false

#define SD_LOGGING_ENABLED true
#define TELEM_LOGGING_ENABLED false //telemdata biasa
#define LIVE_TUNING_ENABLED false

#define TELEM_INTVL_MS 50   //20Hz
#define DEBUG_INTVL_MS 100  //10Hz

uint32_t debug_timer = 0;
uint32_t timer_now, timer_last;
uint32_t loop_timer_start = 0;
uint32_t loop_time_max = 0;
uint32_t loop_count = 0;

// for sd card logging
uint32_t timer_telemetry = 0;
const int TELEM_INTVL = 100;    //100ms, 10Hz

// execution time calc
uint32_t t_profile_start;
uint32_t dur_radio, dur_sensor, dur_control, dur_logging, dur_loop_total;
uint32_t max_dur_radio, max_dur_sensor, max_dur_control, max_dur_logging, max_dur_loop;
uint32_t timer_profile_print = 0;
uint32_t loop_start_time;

void reset_profile_max() {
    max_dur_radio = 0;
    max_dur_sensor = 0;
    max_dur_control = 0;
    max_dur_logging = 0;
    max_dur_loop = 0;
}

void debug_data() {
    USB.print(millis());            USB.print(" ");
    // USB.print(rategain.P, 2);    USB.print(" ");
    // USB.print(rategain.D, 2);    USB.print(" "); 
    // USB.print(rategain.I, 3);    USB.print(" "); 
    // USB.print(rategain.IMAX);    USB.print(" "); 
    // USB.print(rategain.max_rate);
    USB.print(" R:");               USB.print(roll);
    USB.print(" ");                 USB.print(gxrs);
    USB.print(" "); 
    // USB.print(setpoint_roll_rate);   USB.print(" "); 
    // USB.print(er);               USB.print(" "); 

    // USB.print(flip_pulse_delta, 3);  USB.print(" ");
    // USB.print(flip_climb_ms, 3);     USB.print(" ");
    // USB.print(U2_MAX, 5);
    USB.println();
}

void telemetry_data() {
    TELEMETRY.print("<"); 
    TELEMETRY.print(micros());                  TELEMETRY.print(",");  
    TELEMETRY.print(flip_switch_on);            TELEMETRY.print(",");
    TELEMETRY.print(fp);                        TELEMETRY.print(",");
    TELEMETRY.print(setpoint_flip_roll, 2);     TELEMETRY.print(",");
    TELEMETRY.print(roll_relative, 2);          TELEMETRY.print(",");
    TELEMETRY.print(roll_absolute, 2);          TELEMETRY.print(",");
    TELEMETRY.print(roll, 2);                   TELEMETRY.print(",");
    TELEMETRY.print(setpoint_flip_roll_rate, 2);TELEMETRY.print(",");
    TELEMETRY.print(-gxrs, 2);                  TELEMETRY.print(",");
    TELEMETRY.print(motor1_pwm);                TELEMETRY.print(",");
    TELEMETRY.print(motor2_pwm);                TELEMETRY.print(",");
    TELEMETRY.print(motor3_pwm);                TELEMETRY.print(",");
    TELEMETRY.print(motor4_pwm);
    TELEMETRY.println(">"); 
}

void manuver_flip_roll() {
    if (!arming) {
        ctrl_mode = MODE_HOVER;
        fp = IDLE;
        flip_start_time = 0;
    }
    if (ctrl_mode != MODE_FLIP_LQR && ctrl_mode != MODE_FLIP_FUZZY_LPV) {
         setpoint_flip_roll = 0.0f; // Biar di serial monitor 0
         setpoint_flip_roll_rate = 0.0f; 
         roll_relative = 0.0f;
         roll_absolute = 0.0f;
    }
    if (flip_event && ctrl_mode == MODE_HOVER && ch_throttle > 1100) {
        flip_start_time = micros();
        flip_start_angle = roll_absolute;
        error_roll = 0;
        error_roll_rate = 0;
        enter_flip();
        flip_event = false;   // WAJIB reset event
    }
    if (ctrl_mode == MODE_FLIP_LQR || ctrl_mode == MODE_FLIP_FUZZY_LPV || ctrl_mode == MODE_RECOVERY) {
        if (!flip_switch_on) {
            ctrl_mode = MODE_HOVER;
            fp = IDLE;
            Serial.println("FLIP ABORTED BY SWITCH");
        }
        else {
        }
    }
}

void drone_setup() {
    bno_init();
    init_motors();
    remote_setup();

    #if SD_LOGGING_ENABLED
    if (setup_sd()) {
        USB.println(" SD Card OK");
    }
    else {
        USB.println(" SD Card Failed");
    }
    #endif
}

void setup() {
    TELEMETRY.begin(57600);
    USB.begin(115200);
    drone_setup();
    // fuzzy_roll();
    setup_fuzzy_lpv();

    if (M_CALIB) {
        motor_calibration();
    }
    pinMode(LED, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    loop_start_time = micros(); //punya execution time

    // Radio
    t_profile_start = micros();
    remote_loop();
    dur_radio = micros() - t_profile_start;
    if (dur_radio > max_dur_radio) max_dur_radio = dur_radio;

    // Sensor
    t_profile_start = micros();
    bno_update();
    dur_sensor = micros() - t_profile_start;
    if (dur_sensor > max_dur_sensor) max_dur_sensor = dur_sensor;

    manuver_flip_roll();
    // Control
    t_profile_start = micros();
    // set_control_reference(); //angle control
    drone_controller();
    dur_control = micros() - t_profile_start;
    if (dur_control > max_dur_control) max_dur_control = dur_control;

    writeMotors(motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm);

    // Logging
    t_profile_start = micros();
    #if SD_LOGGING_ENABLED
    log_to_buffer();
    #endif
    #if TELEM_LOGGING_ENABLED
    if (millis() - timer_telemetry >= TELEM_INTVL) {
        timer_telemetry = millis();
        telemetry_data(); 

        #if LIVE_TUNING_ENABLED
        telemetry_gain_tuning();
        #endif
    }
    #endif

    dur_logging = micros() - t_profile_start;
    if (dur_logging > max_dur_logging) max_dur_logging = dur_logging;

    #if SD_LOGGING_ENABLED
    sd_logging_update();
    #endif

    dur_loop_total = micros() - loop_start_time;
    if (dur_loop_total > max_dur_loop) max_dur_loop = dur_loop_total;

    // print execution time (µs)
    if (millis() - timer_profile_print > 1000) {
        timer_profile_print = millis();
        TELEMETRY.print("#exct,"); 
        TELEMETRY.print(max_dur_radio); TELEMETRY.print(","); 
        TELEMETRY.print(max_dur_sensor); TELEMETRY.print(",");
        TELEMETRY.print(max_dur_control); TELEMETRY.print(",");
        TELEMETRY.print(max_dur_logging); TELEMETRY.print(",");
        TELEMETRY.print(max_dur_loop); TELEMETRY.print(",");

        // Calculate loop frequency
        float loop_freq = 1000000.0 / max_dur_loop;
        TELEMETRY.print(loop_freq, 0);     TELEMETRY.print(" Hz");
        TELEMETRY.println();
        reset_profile_max();

         #if SD_LOGGING_ENABLED
        if (sd_initialized) {
            USB.print("SD: ");
            USB.print(buffer_count);
            USB.print("/");
            USB.print(SD_BUFFER_SIZE);
            USB.print(" buffered, ");
            USB.print(total_logs_written);
            USB.println(" total");
        }
        #endif
    }
}

void finish_flight() {
    #if SD_LOGGING_ENABLED
    USB.println("\nClosing SD log...");
    close_sd_log();
    USB.println("SD log saved successfully!");
    #endif
}