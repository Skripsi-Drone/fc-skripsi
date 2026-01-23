#ifndef CONTROL_FLIP_2_H
#define CONTROL_FLIP_2_H

#include <Arduino.h>
#include <math.h>
#include "bno_qt.h"
#include "actu_setup.h"
#include "radio.h"

#define MAX_ROLL     35.0f
#define MAX_PITCH    35.0f
#define MAX_YAW      35.0f
#define MIN_PWM      1000
#define MAX_PWM      1800
#define MAX_PWM_FLIP 2000
#define I_YAW_MAX    30.0f 

float setpoint_roll, setpoint_pitch, setpoint_yaw, target_alt;
float setpoint_roll_last, setpoint_pitch_last, setpoint_yaw_last, target_alt_last;
float error_roll, error_pitch, error_yaw;
float error_roll_rate, error_pitch_rate, error_yaw_rate;
float setpoint_roll_now, setpoint_pitch_now, setpoint_yaw_now, target_alt_now;
float setpoint_roll_rate;
float u1, u2, u3, u4;
int raw_pwm1, raw_pwm2, raw_pwm3, raw_pwm4;
float motor_speed_squared[4];
float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
uint32_t t_now, t_last, dt;
float dt_sec;
float p_roll, d_roll, p_pitch, d_pitch, p_yaw, d_yaw, i_yaw;
float yaw_integrator = 0.0f;

enum ControlMode {
    MODE_HOVER = 0,
    MODE_FLIP,
    MODE_RECOVERY
};
ControlMode ctrl_mode = MODE_HOVER;

enum FlipPhase : uint8_t {
    IDLE = 0,
    CLIMB,
    SWING,
    RECOVER,
    FINISH
}; 
FlipPhase fp = IDLE;
uint32_t flip_phase_start   = 0;
bool flip_pulse_active      = false;
int flip_pulse_delta        = 100; //350 25% 300 30% 250 35% 200 45% 150 55% 100 65% OK | tambahan pwm
uint32_t flip_climb_ms      = 150; //100 110 | ms, kalo kekecilan gagal flip, kegedean over flip
uint32_t flip_swing_ms      = 80; // 100 | ms, kalo kekecilan nanti recovery pas di tengah2 flip 
uint32_t flip_recover_ms    = 250; // | timeout buat recovery sblm pindah ke hover lagi
float gx_prev, er;
float i_rate_roll               = 0.0f;
float desired_roll_rate         = 0.0f;
float desired_roll_rate_climb   = 250.0f; //400
float desired_roll_rate_recover = 300.0f;
float U2_MAX = 0.0002; //0.0002

//zmr250 (roll pake arm length x)
const double A_invers[4][4] = {{206611.57024793,   -2035581.97288605,  2623638.98727535, -17730496.45390071},
                               {206611.57024793,   2035581.97288605,  2623638.98727535,  17730496.45390071}, 
                               {206611.57024793,   2035581.97288605, -2623638.98727535,  -17730496.45390071},
                               {206611.57024793,   -2035581.97288605, -2623638.98727535, 17730496.45390071}};

struct AngleGains {
    float alt   = 0.0f;
    float vz    = 0.0f;
    float roll  = 2.76; //2.65
    float p     = 1.94; //1.20 1.53 1.72 | 0.8 kurang msh osilasi jd gedein sampe 1 lebih. kl kurang naikin dikit aja
    //gain damping segini cm cukup buat flip di thr 70
    float pitch = 0.0; //2.80; //2.45
    float q     = 0.0; //1.85; //0.98 1.28 | pake lpf gyro rangenya 0.98 sampe 1.0, tanpa lpf 1.0 sampe 1.1 atau tambah dikit lg
    float yaw   = 0.0; //0.141; // 0.140
    float r     = 0.0; //0.043; // 0.040
    float iy    = 0.0; //0.001; //trial
} anglegain;

struct RollRateGains {
    float P    = 1.91;      //1.21; 3.83 2.00 |tuningan terakhir 21 jan
    float D    = 0.0101;    //0.0101
    float I    = 0.49;      //0.63 0.50 |bisa dikecilin lg kalo ga ngerem2
    float IMAX = 500.0;
    float max_rate = 500.0; //220 deg/s 
} rategain;

float constrain_value(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

void enter_flip() {
    ctrl_mode = MODE_FLIP;
    fp = CLIMB;
    flip_phase_start = millis();
    flip_pulse_active = true;
}

void update_flip() {
    if (ctrl_mode != MODE_FLIP) return;

    uint32_t now = millis();
    uint32_t elapsed = now - flip_phase_start;

    switch (fp) {
        case CLIMB:
            flip_pulse_active = true;
            desired_roll_rate = desired_roll_rate_climb;
            if (elapsed > flip_climb_ms) {
                fp = SWING;
                flip_phase_start = now;
                flip_pulse_active = false;
                // optionally cut throttle to let body tumble
                // you can temporarily reduce ch_throttle here if safe
            }
            break;

        case SWING:
            // remove active forcing, let inertia act
            desired_roll_rate = 0.0f; // let tumble naturally
            flip_pulse_active = false;
            if (elapsed > flip_swing_ms) {
                fp = RECOVER;
                flip_phase_start = now;
                desired_roll_rate = -desired_roll_rate_recover;
            }
            break;

        case RECOVER:
            flip_pulse_active = false;
            if (elapsed > flip_recover_ms) {
                fp = FINISH;
                flip_phase_start = now;
            }
            break;

        case FINISH:
            ctrl_mode = MODE_RECOVERY; 
            fp = IDLE;
            flip_pulse_active = false;
            break;

        default:
            fp = IDLE;
            flip_pulse_active = false;
            break;
    }
}

void roll_control() {
    static bool was_flip = false;
    if (ctrl_mode == MODE_FLIP) {
        if (!was_flip) {
            i_rate_roll = 0.0f;
            gx_prev = gxrs;
            was_flip = true;
        }

        float gx = gxrs;
        er = desired_roll_rate - gx;
        
        float dgx = 0.0f;
        if (dt_sec > 0.0f) {
            dgx = (gx - gx_prev) / dt_sec;
        }
        gx_prev = gx;

        if (dt_sec > 0.0f && ch_throttle > 1100) {
            i_rate_roll += rategain.I * er * dt_sec;
            i_rate_roll = constrain(i_rate_roll, -rategain.IMAX, rategain.IMAX);
        }

        float u2_raw = (rategain.P * er + i_rate_roll - rategain.D * dgx)/10'000'000;
        u2 = constrain(u2_raw, -U2_MAX, U2_MAX);
    }
    else { 
        if (was_flip) {
            was_flip = false;
            i_rate_roll = 0.0;
        }
        /* Roll control (rate based)
        //pure rate mapping
        float roll_cmd = map(ch_roll, 1000, 2000, -rategain.max_rate, rategain.max_rate);
        setpoint_roll_rate = constrain(roll_cmd, -rategain.max_rate, rategain.max_rate);
        //mix angle mapping
        // float roll_cmd = map(ch_roll - 1500, min_roll_corr, max_roll_corr, -MAX_ROLL, MAX_ROLL); //mix angle
        // const float e_roll = roll_cmd - roll; //mix angle
        // setpoint_roll_rate = constrain(e_roll * anglegain.roll, -rategain.max_rate, rategain.max_rate); //mix angle 

        float gx = gxrs;
        er = setpoint_roll_rate - gx;

        float dgx = 0.0f;
        if (dt_sec > 0.0f) {
            dgx = (gx - gx_prev) / dt_sec;
        }
        gx_prev = gx;
        
        //reset integrator
        if (ch_throttle < 1100) {
            i_rate_roll = 0.0f;
        }
        else if (dt_sec > 0.0f) {
            i_rate_roll += rategain.I * er * dt_sec;
            i_rate_roll = constrain(i_rate_roll, -rategain.IMAX, rategain.IMAX);
        }

        float u2_raw = (rategain.P * er + i_rate_roll - rategain.D * dgx) / 10'000'000.0f;
        u2 = constrain(u2_raw, -U2_MAX, U2_MAX);
        */

        //Roll control (angle based)
        setpoint_roll_last = setpoint_roll_now;
        setpoint_roll_now = setpoint_roll;

        error_roll = roll - setpoint_roll;
        error_roll_rate = -gxrs;

        p_roll = -anglegain.roll * error_roll;
        d_roll = -anglegain.p * error_roll_rate;

        u2 = (p_roll + d_roll)  /10'000'000.0f;
    }
}

void pitch_yaw_control() {
    setpoint_pitch_last = setpoint_pitch_now;
    setpoint_pitch_now = setpoint_pitch;
    setpoint_yaw_last = setpoint_yaw_now;
    setpoint_yaw_now = setpoint_yaw;

    error_pitch = pitch - setpoint_pitch;
    error_pitch_rate = gyrs;
    error_yaw = yaw - setpoint_yaw;
    if (error_yaw > 180.0f) error_yaw -= 360.0f;
    if (error_yaw < -180.0f) error_yaw += 360.0f;
    error_yaw_rate = -gzrs;

    p_pitch = -anglegain.pitch * error_pitch;
    d_pitch = -anglegain.q * error_pitch_rate;
    p_yaw = -anglegain.yaw * error_yaw;
    d_yaw = -anglegain.r * error_yaw_rate;

    yaw_integrator += error_yaw * dt_sec;
    yaw_integrator = constrain (yaw_integrator, -I_YAW_MAX, I_YAW_MAX);
    if (ch_throttle < 1100) {
        yaw_integrator = 0.0f;
    }
    i_yaw = -anglegain.iy * yaw_integrator;

    u1 = 0.0;
    u3 = (p_pitch + d_pitch)    /10'000'000.0f;
    u4 = (p_yaw + d_yaw + i_yaw)/10'000'000.0f;
}

void motor_mixer() {
    motor_speed_squared[0] = ((A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4));
    motor_speed_squared[1] = ((A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4));
    motor_speed_squared[2] = ((A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4));
    motor_speed_squared[3] = ((A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4));

    raw_pwm1 = ch_throttle + (int)(motor_speed_squared[0]);
    raw_pwm2 = ch_throttle + (int)(motor_speed_squared[1]);
    raw_pwm3 = ch_throttle + (int)(motor_speed_squared[2]);
    raw_pwm4 = ch_throttle + (int)(motor_speed_squared[3]);

    if (flip_pulse_active) {
        int delta = flip_pulse_delta;
        raw_pwm1 -= delta;
        raw_pwm4 -= delta;
        raw_pwm2 += delta;
        raw_pwm3 += delta;
    }

    int pwm_max = (ctrl_mode == MODE_FLIP) ? MAX_PWM_FLIP : MAX_PWM;
    motor1_pwm = constrain_value(raw_pwm1, MIN_PWM, pwm_max);
    motor2_pwm = constrain_value(raw_pwm2, MIN_PWM, pwm_max);
    motor3_pwm = constrain_value(raw_pwm3, MIN_PWM, pwm_max);
    motor4_pwm = constrain_value(raw_pwm4, MIN_PWM, pwm_max);
}

void set_control_reference() {
    setpoint_roll = roll_scaler() * MAX_ROLL;
    setpoint_pitch = -pitch_scaler() * MAX_PITCH;
    setpoint_yaw = yaw_scaler() * MAX_YAW;
}

void drone_controller(){
    t_now = micros();
    dt = t_now - t_last;
    t_last = t_now;

    dt_sec = dt * 1e-6f;
    if (dt_sec < 0.0005f) dt_sec = 0.0005f;
    if (dt_sec > 0.01f)   dt_sec = 0.01f;

    set_control_reference();
    roll_control();
    // pitch_yaw_control(); //di testbench off
    motor_mixer();
}

#endif