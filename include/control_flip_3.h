#ifndef CONTROL_FLIP_3_H
#define CONTROL_FLIP_3_H

#include <Arduino.h>
#include <math.h>
#include "bno_qt.h"
#include "actu_setup.h"
#include "radio.h"
#include "fufufuzizizi.h"

#define MAX_ROLL_HOVER  35.0f
#define MAX_PITCH_HOVER 35.0f
#define MAX_YAW_HOVER   35.0f
// #define MAX_YAW_RATE    200.0f //deg/s
#define MIN_PWM         1000
#define MAX_PWM         1800
#define MAX_PWM_FLIP    2000
#define I_YAW_MAX       30.0f
#define FLIP_TRAJECTORY_LINEAR  1
#define FLIP_TRAJECTORY_COSINE  2 //S curve katanya teh

float setpoint_roll, setpoint_pitch, setpoint_yaw;
float setpoint_roll_now, setpoint_pitch_now, setpoint_yaw_now;
float setpoint_roll_last, setpoint_pitch_last, setpoint_yaw_last;
float setpoint_roll_rate, setpoint_roll_rate_now, setpoint_roll_rate_last;
float error_roll, error_pitch, error_yaw;
float error_roll_rate, error_pitch_rate, error_yaw_rate;
float p_roll, d_roll, p_pitch, d_pitch, p_yaw, d_yaw, i_yaw;
float yaw_integrator = 0.0f;
float u1, u2, u3, u4;
int raw_pwm1, raw_pwm2, raw_pwm3, raw_pwm4;
float motor_speed_squared[4];
float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
uint32_t t_now, t_last, dt;
float dt_sec;

int flip_trajectory_type = FLIP_TRAJECTORY_COSINE;
float setpoint_flip_roll = 0.0f;
float setpoint_flip_roll_rate = 0.0f;
float roll_absolute = 0.0f;
float roll_relative;
float yaw_at_flip_end = 0.0f;
uint32_t flip_start_time = 0;
float flip_start_angle = 0.0f;
float flip_duration_sec = 0.8f; //0.75 0.5 1.0 0.8
float K_roll_effective, K_p_effective;
float accumulated_roll, last_roll_raw;

enum ControlMode {
    MODE_HOVER = 0,
    MODE_FLIP_LQR,
    MODE_FLIP_FUZZY_LPV,
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

//zmr250 (roll pake arm length x)
const double A_invers[4][4] = {{206611.57024793,   -2035581.97288605,  2623638.98727535, -17730496.45390071},
                               {206611.57024793,   2035581.97288605,  2623638.98727535,  17730496.45390071}, 
                               {206611.57024793,   2035581.97288605, -2623638.98727535,  -17730496.45390071},
                               {206611.57024793,   -2035581.97288605, -2623638.98727535, 17730496.45390071}};

struct HoverGains {
    float alt   = 0.0f;
    float vz    = 0.0f;
    float roll  = 2.84; //2.65
    float p     = 2.44; //1.20 1.53 1.72 | 0.8 kurang msh osilasi jd gedein sampe 1 lebih. kl kurang naikin dikit aja
    float pitch = 2.87; //2.45
    float q     = 2.20; //0.98 1.28 | pake lpf gyro rangenya 0.98 sampe 1.0, tanpa lpf 1.0 sampe 1.1 atau tambah dikit lg
    float yaw   = 0.1443; // 0.140
    float r     = 0.0430; // 0.040 0429
    float iy    = 0.0006; //trial
} hovergain; 

struct FlipGains { //samain kek base gain
    float roll  = 3.4; //2.76 3.76 4.6 5.5 7 8 10 13 15 17 20 22 18 12 
    float p     = 2.98; //1.94 3.5
} flipgain;

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
    // ctrl_mode = MODE_FLIP_LQR;
    ctrl_mode = MODE_FLIP_FUZZY_LPV;
    flip_start_time = micros();
    // flip_start_angle = roll_absolute;

    setpoint_flip_roll = 0.0f;
    setpoint_flip_roll_rate = 0.0f;
    // roll_absolute = 0.0f;
    yaw_integrator = 0.0f;
    // yaw_at_flip_end = yaw;
}

void update_flip_trajectory() {
    if (ctrl_mode != MODE_FLIP_LQR && ctrl_mode != MODE_FLIP_FUZZY_LPV) return;

    float t_now_sec = (micros() - flip_start_time) / 1'000'000.0f;
    if (t_now_sec >= flip_duration_sec) {
        setpoint_flip_roll = 360.0f;
        setpoint_flip_roll_rate = 0.0f;
        ctrl_mode = MODE_RECOVERY; //ini masuk recovery nya
        fp = IDLE;

        // tambahan baru biar gak yawing2 pas recovery krn disorient
        yaw_at_flip_end = yaw;
        setpoint_yaw = yaw; //tambahan
        yaw_integrator = 0.0f;

        return;
    }

    if (flip_trajectory_type == FLIP_TRAJECTORY_LINEAR) {
        float movement = t_now_sec / flip_duration_sec;
        setpoint_flip_roll = movement * 360.0f; //sudut = (waktu / durasi) * 360
        setpoint_flip_roll_rate = 360.0f / flip_duration_sec;   //kecepatan = jarak / waktu
    }
    else if (flip_trajectory_type == FLIP_TRAJECTORY_COSINE) {
        float t_norm = t_now_sec / flip_duration_sec;
        float s_curve = 0.5f * (1.0f - cos(t_norm * PI));   //0.5*(1-cos(pi * t))
        setpoint_flip_roll = s_curve * 360.0f;
        float max_vel = (360.0f * PI) / (2.0f * flip_duration_sec);
        setpoint_flip_roll_rate = max_vel * sin(t_norm * PI); //(360*pi) / (2*durasi)
    }

    if (setpoint_flip_roll < 90.0f) {
        fp = CLIMB;
    }
    else if (setpoint_flip_roll < 270.0f) {
        fp = SWING;
    }
    else if (setpoint_flip_roll < 360.0f) {
        fp = RECOVER;
    }
    else {
        fp = FINISH;
    }
}

void roll_control() {
    static bool was_flip = false;
    update_flip_trajectory();

    if (ctrl_mode == MODE_FLIP_LQR) {
        if (!was_flip) {
            // roll_absolute = 0.0f;
            was_flip = true;
        }

        roll_absolute += ((-gxrs) * dt_sec);
        roll_relative = roll_absolute - flip_start_angle;

        setpoint_roll_last      = setpoint_roll_now;
        setpoint_roll_now       = setpoint_flip_roll;
        setpoint_roll_rate_last = setpoint_roll_rate_now;
        setpoint_roll_rate_now  = setpoint_flip_roll_rate;

        error_roll      = roll_relative - setpoint_roll_now;
        error_roll_rate = (-gxrs) - setpoint_roll_rate_now;

        p_roll = -flipgain.roll * error_roll;
        d_roll = -flipgain.p    * error_roll_rate;

        u2 = (p_roll + d_roll) / 10'000'000.0f;
        u2 = constrain(u2, -MAX_PWM_FLIP, MAX_PWM_FLIP);
    }
    else  if (ctrl_mode == MODE_FLIP_FUZZY_LPV) {
        if (!was_flip) {
        was_flip = true;
        // roll_absolute = 0.0;
        // flip_start_angle = 0.0;
        // last_roll_raw = roll;
        // accumulated_roll = 0.0;
        }

        // float roll_delta = roll - last_roll_raw;

        // if (roll_delta > 180.0) {
        //     roll_delta -= 360.0;
        // } else if (roll_delta < -180.0) {
        //     roll_delta += 360.0;
        // }
        
        // accumulated_roll += roll_delta;
        // last_roll_raw = roll;
        // roll_relative = accumulated_roll;
        roll_absolute += ((-gxrs) * dt_sec);
        roll_relative = roll_absolute - flip_start_angle;

        setpoint_roll_last      = setpoint_roll_now;
        setpoint_roll_now       = setpoint_flip_roll;
        setpoint_roll_rate_last = setpoint_roll_rate_now;
        setpoint_roll_rate_now  = setpoint_flip_roll_rate;

        error_roll      = roll_relative - setpoint_roll_now;
        error_roll_rate = (-gxrs) - setpoint_roll_rate_now;

        // lookup table
        // fuzzy_lpv_controller(error_roll, error_roll_rate, fp, K_roll_effective, K_p_effective);
        // p_roll = -K_roll_effective * error_roll;
        // d_roll = -K_p_effective * error_roll_rate;

        // fuzzy real
        update_fuzzy_gain();

        p_roll = -get_kroll_eff() * error_roll;
        d_roll = -get_kp_eff() * error_roll_rate;

        u2 = (p_roll + d_roll) / 10'000'000.0f;
        u2 = constrain(u2, -MAX_PWM_FLIP, MAX_PWM_FLIP);    
    }
    else {
        if (was_flip) {
            was_flip = false;
        }

        setpoint_roll_last  = setpoint_roll_now;
        setpoint_roll_now   = setpoint_roll;

        error_roll       = roll - setpoint_roll;
        error_roll_rate  = -gxrs;

        p_roll  = -hovergain.roll * error_roll;
        d_roll  = -hovergain.p * error_roll_rate;

        u2 = (p_roll + d_roll)  /10'000'000.0f;
    }
}

void pitch_yaw_control() {
    setpoint_pitch_last = setpoint_pitch_now;
    setpoint_pitch_now = setpoint_pitch;
    setpoint_yaw_last = setpoint_yaw_now;
    setpoint_yaw_now = setpoint_yaw;

    // hold/lock setpoint recovery yaw sesuai nilai setelah flip fase 3 selesai
    // if (ctrl_mode == MODE_FLIP_LQR || ctrl_mode == MODE_FLIP_FUZZY_LPV) {
    //     setpoint_yaw_now = yaw_at_flip_end;
    // }
    // else {
    //     setpoint_yaw_now = setpoint_yaw;
    // }

    error_pitch = pitch - setpoint_pitch;
    error_pitch_rate = gyrs;
    error_yaw = yaw - setpoint_yaw;
    if (error_yaw > 180.0f) error_yaw -= 360.0f;
    if (error_yaw < -180.0f) error_yaw += 360.0f;
    error_yaw_rate = -gzrs;

    p_pitch = -hovergain.pitch * error_pitch;
    d_pitch = -hovergain.q * error_pitch_rate;
    p_yaw = -hovergain.yaw * error_yaw;
    d_yaw = -hovergain.r * error_yaw_rate;

    yaw_integrator += error_yaw * dt_sec;
    yaw_integrator = constrain (yaw_integrator, -I_YAW_MAX, I_YAW_MAX);
    if (ch_throttle < 1100) {
        yaw_integrator = 0.0f;
    }
    i_yaw = -hovergain.iy * yaw_integrator;

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

    int pwm_max = (ctrl_mode == MODE_FLIP_LQR || ctrl_mode == MODE_FLIP_FUZZY_LPV) ? MAX_PWM_FLIP : MAX_PWM;
    motor1_pwm = constrain_value(raw_pwm1, MIN_PWM, pwm_max);
    motor2_pwm = constrain_value(raw_pwm2, MIN_PWM, pwm_max);
    motor3_pwm = constrain_value(raw_pwm3, MIN_PWM, pwm_max);
    motor4_pwm = constrain_value(raw_pwm4, MIN_PWM, pwm_max);
}

void set_control_reference() {
    setpoint_roll = roll_scaler() * MAX_ROLL_HOVER;
    setpoint_pitch = -pitch_scaler() * MAX_PITCH_HOVER;
    setpoint_yaw = yaw_scaler() * MAX_YAW_HOVER; //v1, default
    // yaw rate based (RC)
    // float yaw_input = yaw_scaler();
    // if (abs(yaw_input) > 0.05) {
    //     setpoint_yaw += yaw_input * MAX_YAW_RATE * dt;
    // }
    // if (setpoint_yaw > 180) setpoint_yaw -= 360;
    // if (setpoint_yaw < -180) setpoint_yaw += 360;
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
    pitch_yaw_control(); //di testbench off
    motor_mixer();
}


#endif