#ifndef CONTROL_LQR_H
#define CONTROL_LQR_H

#include <Arduino.h>
#include <math.h>
#include "Radio.h"
// #include "bno_euler.h"
#include "bno_quaternion.h"
#include "actu_setup.h"

// #define INTEGRAL_LIMIT 10.0f
// #define MAX_VALUE 50.0f
// #define MIN_VALUE -50.0f
// #define MIN_THRUST 30.0f
// #define MAX_MOTOR_SPEED 1025.38f
// #define MIN_MOTOR_SPEED 0.0f
#define MAX_VALUE_YAW 50.0f
#define MIN_VALUE_YAW -50.0f
// hover
#define MAX_ROLL 35.0f 
#define MAX_PITCH 35.0f
#define MAX_YAW 25.0f
#define MIN_PWM 1000 //hover
#define MAX_PWM 1800 //hover
#define MAX_PWM_FLIP 2000
// #define MAX_THRUST 18.24f

float u1, u2, u3, u4;
// float roll_integrator, pitch_integrator, yaw_integrator, altitude_integrator;
float setpoint_roll, setpoint_pitch, setpoint_yaw, target_alt;
// float setpoint_roll_last, setpoint_pitch_last, setpoint_yaw_last, target_alt_last;
// float setpoint_roll_now, setpoint_pitch_now, setpoint_yaw_now, target_alt_now;
// float setpoint_roll_rate, setpoint_pitch_rate, setpoint_yaw_rate;
float p_roll, d_roll, p_pitch, d_pitch, p_yaw, d_yaw;
float roll_cmd, pitch_cmd, yaw_cmd;
float trim_roll = 0.0f;
float trim_pitch = 0.0f; 
float min_roll  =  0.0f;
float max_roll  =  0.0f;
float min_pitch = -35.0f;
float max_pitch =  35.0f;
float min_yaw   = -20.0f;
float max_yaw   =  20.0f;
const int YAW_DEADZONE_RC = 15;

float state_yaw;
float altitude;
float alt_last, alt_now, alt_vel;
float ki_roll = 0.0f;
float ki_pitch = 0.0f;
float motor_speed[4];
float motor_thrust[4];
float motor_speed_rpm[4];
float motor_speed_squared[4];
float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
uint8_t t_now, t_last, dt;

//buat flip pakai LQR
bool flip_LQR_mode = false;
unsigned long flip_start_time = 0;
// const float FLIP_TARGET_ANGLE_DEG =180.0;

bool test_setpoint_variation = false;

//masih F450
// const double A_invers[4][4] = {{292600, -1300300, -1300300,  6283300},
//                                {292600,  1300300, -1300300, -6283300},
//                                {292600,  1300300,  1300300,  6283300},
//                                {292600, -1300300,  1300300, -6283300}};

//zmr
const double A_invers[4][4] = {{207484.62209327, 2634725.3599145,  2044183.46889918,  11421606.57957169},
                               {207484.62209327,  -2634725.3599145,  2044183.46889918, -11421606.57957169},
                               {207484.62209327,  -2634725.3599145, -2044183.46889918,  11421606.57957169},
                               {207484.62209327, 2634725.3599145, -2044183.46889918, -11421606.57957169}};
// atas u, bawah motor
// udah update
struct gains {
    float k_alt        = 5.0f;
    float k_z_velocity = 3.0f;
    float k_z_vel      = 1.0f;
    float k_pos        = 1.0f;
    int16_t roll_rmt, pitch_rmt, yaw_rmt;

    // Stabilize (angle -> rate), P-only (
    float stab_roll_P  = 4.5f;
    float stab_pitch_P = 4.5f;
    float stab_yaw_P   = 4.5f;

    // Rate PID 
    float rate_roll_P   = 0.150f;
    float rate_roll_D   = 0.0040f;

    float rate_pitch_P  = 0.150f;
    float rate_pitch_D  = 0.0040f;

    float rate_yaw_P    = 0.200f;
    float rate_yaw_D    = 0.0000f;

    // batas rate target (deg/s)
    float max_rate_r = 220.0f;
    float max_rate_p = 220.0f;
    float max_rate_y = 150.0f;
};
gains gain;

float roll_rate_sp  = 0.0f;
float pitch_rate_sp = 0.0f;
float yaw_rate_sp   = 0.0f;

static float gy_prev = 0.0f, gx_prev = 0.0f, gz_prev = 0.0f;
static uint32_t last_us_rate = 0;

// float constrain_value(float value, float min, float max) {
//     if (value < min) {
//         return min;
//     }
//     if (value > max) {
//         return max;
//     }
//     return value;
// }

void drone_controller() {
    uint32_t now_us = micros();
    float dt = (last_us_rate==0) ? 0.0f : (now_us - last_us_rate) * 1e-6f;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.0f;  // lock I/D jika timing jelek
    last_us_rate = now_us;
    // tesdata = last_us_rate;

    // ===== RC → angle command =====
    roll_cmd  = 1.3f*(map(ch_roll - 1500, min_roll_corr,  max_roll_corr,  min_roll,  max_roll));
    pitch_cmd = 1.1f*(map(ch_pitch - 1500, min_pitch_corr, max_pitch_corr, min_pitch, max_pitch));
    yaw_cmd   = 1.3f*(map(ch_yaw - 1500, min_yaw_corr,   max_yaw_corr,   min_yaw,   max_yaw));

    // ===== error sudut (cmd - meas) =====
    const float e_roll  = (roll_cmd  + trim_roll)  - roll;
    const float e_pitch = (pitch_cmd + trim_pitch) - (-pitch); // sesuai definisimu

    // ===== Stabilize (angle→rate) P-only =====
    roll_rate_sp  = constrain(e_roll  * gain.stab_roll_P,  -gain.max_rate_r,  gain.max_rate_r);
    pitch_rate_sp = constrain(e_pitch * gain.stab_pitch_P, -gain.max_rate_p,  gain.max_rate_p);

    // Yaw: rate dari stick atau heading-hold
    const bool yaw_stick = (abs(ch_yaw - 1500) > YAW_DEADZONE_RC);
    static float heading_target_deg = 0.0f;
    if (yaw_stick) {
        heading_target_deg = yaw; // update target ketika pilot gerak yaw
        yaw_rate_sp = constrain(map(ch_yaw, 1000, 2000, -gain.max_rate_y, gain.max_rate_y), -gain.max_rate_y, gain.max_rate_y);
    } else {
        float e_yaw = heading_target_deg - yaw;
        if (e_yaw > 180.0f)  e_yaw -= 360.0f;
        if (e_yaw < -180.0f) e_yaw += 360.0f;
        yaw_rate_sp = constrain(e_yaw * gain.stab_yaw_P, -gain.max_rate_y, gain.max_rate_y);
    }

    // ===== Rate PID (P + D_on_meas) =====
    const float er = (roll_rate_sp  - gyrs);  // NOTE: kamu memang pakai gy utk roll
    const float ep = (pitch_rate_sp - gxrs);   // gx utk pitch
    const float ey = (yaw_rate_sp   - gzrs);   // gz utk yaw

    const float dgy = (dt>0) ? (gyrs - gy_prev)/dt : 0.0f; gy_prev = gyrs;
    const float dgx = (dt>0) ? (gxrs - gx_prev)/dt : 0.0f; gx_prev = gxrs;
    const float dgz = (dt>0) ? (gzrs - gz_prev)/dt : 0.0f; gz_prev = gzrs;

    u1 = 0.0; //(gain.alt * error_altitude) + (gain.vz * alt_vel);
    u2 = ( gain.rate_roll_P  * er  - gain.rate_roll_D  * dgy ); // 10'000'000.0f; // roll via gy
    u3 = ( gain.rate_pitch_P * ep  - gain.rate_pitch_D * dgx ); // 10'000'000.0f; // pitch via gx
    u4 = ( gain.rate_yaw_P   * ey  - gain.rate_yaw_D   * dgz );// 10'000'000.0f; // yaw

    motor_speed_squared[0] = ((A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4));
    motor_speed_squared[1] = ((A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4));
    motor_speed_squared[2] = ((A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4));
    motor_speed_squared[3] = ((A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4));

    motor1_pwm = ch_throttle + (int)(motor_speed_squared[0]);
    motor2_pwm = ch_throttle + (int)(motor_speed_squared[1]);
    motor3_pwm = ch_throttle + (int)(motor_speed_squared[2]);
    motor4_pwm = ch_throttle + (int)(motor_speed_squared[3]);

    if (flip_LQR_mode) {
        motor1_pwm = constrain(motor1_pwm, MIN_PWM, MAX_PWM_FLIP);
        motor2_pwm = constrain(motor2_pwm, MIN_PWM, MAX_PWM_FLIP);
        motor3_pwm = constrain(motor3_pwm, MIN_PWM, MAX_PWM_FLIP);
        motor4_pwm = constrain(motor4_pwm, MIN_PWM, MAX_PWM_FLIP);
    } else {
        motor1_pwm = constrain(motor1_pwm, MIN_PWM, MAX_PWM);
        motor2_pwm = constrain(motor2_pwm, MIN_PWM, MAX_PWM);
        motor3_pwm = constrain(motor3_pwm, MIN_PWM, MAX_PWM);
        motor4_pwm = constrain(motor4_pwm, MIN_PWM, MAX_PWM);
    }
}

void set_control_reference() {
    // blm diupdate sesuai kendali yg baru
    if (test_setpoint_variation) {
        setpoint_roll = 22.5f;
        setpoint_pitch = 0.0f;
        setpoint_yaw = 0.0f;
        if (roll == 22.5) { //blm mempertimbangkan toleransi response kayak di bab 4
            setpoint_roll = 45.0f;
            setpoint_pitch = 0.0f;
            setpoint_yaw = 0.0f;
            if (roll == 45.0) {
                setpoint_roll == 0.0f;
                setpoint_pitch = 0.0f;
                setpoint_yaw = 0.0f;
            }
        }
    }
    else if (flip_LQR_mode) {
        setpoint_roll = 90.0f;
        if (roll == 90.0) { //blm mempertimbangkan toleransi response sesuai kyk di bab 4
            setpoint_roll = 180.0f;
            if (roll == 180.0) {
                setpoint_roll = -90.0f;
                if (roll == -90.0) {
                    setpoint_roll = 0.0f;
                }
            }
        }
        setpoint_pitch = 0.0f;
        setpoint_yaw = 0.0f;
    } 
    else {
        setpoint_roll = roll_scaler() * MAX_ROLL;
        setpoint_pitch = pitch_scaler() * MAX_PITCH;
        setpoint_yaw = yaw_scaler() * MAX_YAW;
        // setpoint_yaw = yaw_sp;
        // if (setpoint_yaw > 180.0f) { setpoint_yaw -= 360.0f; }
        // if (setpoint_yaw < -180.0f) { setpoint_yaw += 360.0f; }
    }
}

#endif