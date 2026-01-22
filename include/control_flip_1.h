#ifndef CONTROL_LQR_H
#define CONTROL_LQR_H

#include <Arduino.h>
#include <math.h>
#include "Radio.h"
#include "bno_qt.h"

#define MAX_ROLL 45.0f
#define MAX_PITCH 35.0f
#define MAX_YAW 35.0f
#define I_YAW_MAX 30.0f //trial
#define MIN_PWM 1000
#define MAX_PWM 1800

#define MAX_PWM_FLIP 2000
#define FLIP_PWM_START 1650
#define FLIP_PWM_CUT 1150
#define FLIP_PWM_RECOVERY 1600

#define U2_MAX 0.000995 //acuan: -0.0000826

float u1, u2, u3, u4;
float yaw_integrator = 0.0f;
float setpoint_roll, setpoint_pitch, setpoint_yaw, target_alt;
float setpoint_roll_last, setpoint_pitch_last, setpoint_yaw_last, target_alt_last;
float setpoint_roll_now, setpoint_pitch_now, setpoint_yaw_now, target_alt_now;
float setpoint_roll_rate, setpoint_pitch_rate, setpoint_yaw_rate;
float p_roll, d_roll, p_pitch, d_pitch, p_yaw, d_yaw, i_yaw;
float error_roll, error_pitch, error_yaw;
float error_roll_rate, error_pitch_rate, error_yaw_rate;
float altitude, alt_last, alt_now, alt_vel;
float motor_speed_squared[4];
int pwm_max;
float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
uint32_t t_now, t_last, dt;
uint32_t flip_start_time = 0;
float max_flip_rate = 0.0f;

enum FlightMode {
    HOVER,
    FLIP_ROLL,
    FAILSAFE
};

enum FlipPhase : uint8_t {
    FLIP_IDLE       = 0,
    FLIP_PHASE_1    = 1, //0 ke 90
    FLIP_PHASE_2    = 2, //90 ke 180
    FLIP_PHASE_3    = 3, //180 ke -90
    FLIP_PHASE_4    = 4, //-90 ke 0
    FLIP_FINISH     = 5,
    FLIP_ABORT      = 255
};

FlightMode fm = HOVER;
FlipPhase fp = FLIP_IDLE;

float flip_target_roll(FlipPhase phase) {
    switch(phase) {
        case FLIP_PHASE_1: return 50.0f;
        case FLIP_PHASE_2: return 177.0f;
        case FLIP_PHASE_3: return -90.0f;
        case FLIP_PHASE_4: return 0.0f;
        default: return setpoint_roll;
    }
}

int flip_target_throttle(FlipPhase phase, int throttle) {
    switch(phase) {
        case FLIP_PHASE_1: return FLIP_PWM_START;
        case FLIP_PHASE_2: return FLIP_PWM_CUT;
        case FLIP_PHASE_3: return FLIP_PWM_CUT;
        case FLIP_PHASE_4: return FLIP_PWM_RECOVERY;
        default: return throttle;
    }
}

//zmr250 (roll pake arm length x)
const double A_invers[4][4] = {{206611.57024793,   -2035581.97288605,  2623638.98727535, -17730496.45390071},
                               {206611.57024793,   2035581.97288605,  2623638.98727535,  17730496.45390071}, 
                               {206611.57024793,   2035581.97288605, -2623638.98727535,  -17730496.45390071},
                               {206611.57024793,   -2035581.97288605, -2623638.98727535, 17730496.45390071}};
struct Gains {
    float alt   = 0.0f;
    float vz    = 0.0f;
    float roll  = 2.77; //2.65
    float p     = 1.84; //1.20 1.53 1.72 | 0.8 kurang msh osilasi jd gedein sampe 1 lebih. kl kurang naikin dikit aja
    float pitch = 0.0; //2.80; //2.45
    float q     = 0.0; //1.85; //0.98 1.28 | pake lpf gyro rangenya 0.98 sampe 1.0, tanpa lpf 1.0 sampe 1.1 atau tambah dikit lg
    float yaw   = 0.0; //0.141; // 0.140
    float r     = 0.0; //0.043; // 0.040
    float iy    = 0.0; //0.001; //trial
} gain;

float constrain_value(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void drone_controller() {
    t_now = micros();
    dt = t_now - t_last;
    t_last = t_now;

    float dt_sec = dt * 1e-6f;
    // safety clamp 
    if (dt_sec < 0.0005f) dt_sec = 0.0005f;
    if (dt_sec > 0.01f)   dt_sec = 0.01f;

    setpoint_roll_last = setpoint_roll_now;
    setpoint_roll_now = setpoint_roll;
    setpoint_pitch_last = setpoint_pitch_now;
    setpoint_pitch_now = setpoint_pitch;
    setpoint_yaw_last = setpoint_yaw_now;
    setpoint_yaw_now = setpoint_yaw;

    // alt_last = alt_now;
    // alt_now = altitude;
    // alt_vel = (alt_now - alt_last) / dt;

    error_roll = roll - setpoint_roll;
    error_roll = normalize_angle(error_roll);
    error_roll_rate = -gxrs;
    error_pitch = pitch - setpoint_pitch;
    error_pitch_rate = gyrs;
    error_yaw = yaw - setpoint_yaw;
    error_yaw = normalize_angle(error_yaw);
    error_yaw_rate = -gzrs;
    // float error_altitude = target_alt - altitude;

    /* u = -k * (state - setpoint) */
    p_roll = -gain.roll * error_roll;
    d_roll = -gain.p * error_roll_rate;
    p_pitch = -gain.pitch * error_pitch;
    d_pitch = -gain.q * error_pitch_rate;
    p_yaw = -gain.yaw * error_yaw;
    d_yaw = -gain.r * error_yaw_rate;

    if (fm == FLIP_ROLL) {
        yaw_integrator = 0.0;
        i_yaw = 0.0;

        if (abs(gxrs) > max_flip_rate) {
            max_flip_rate = abs(gxrs);
        }
    } else {
        yaw_integrator += error_yaw * dt_sec;
        yaw_integrator = constrain (yaw_integrator, -I_YAW_MAX, I_YAW_MAX);
        if (ch_throttle < 1100) {
            yaw_integrator = 0.0f;
        }
        i_yaw = -gain.iy * yaw_integrator;
    }

    u1 = 0.0; // maximum thrust of x2216 skywalker x 4 using 1047 prop and 4s battery in newton //0.0;//(gain.alt * error_altitude) + (gain.vz * alt_vel);
    u2 = (p_roll + d_roll)/10'000'000.0f;
    if (fm == FLIP_ROLL) {
        u2 = constrain(u2, -U2_MAX * 0.8, U2_MAX * 0.8);
    }
    u3 = (p_pitch + d_pitch)/10'000'000.0f;
    u4 = (p_yaw + d_yaw + i_yaw)/10'000'000.0f;

    motor_speed_squared[0] = ((A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4));
    motor_speed_squared[1] = ((A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4));
    motor_speed_squared[2] = ((A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4));
    motor_speed_squared[3] = ((A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4));

    int master_throttle;
    if (fm == FLIP_ROLL) {
        master_throttle = flip_target_throttle(fp, ch_throttle);
        if (fp == FLIP_ABORT) {
            master_throttle = 1000; // cukup buat stabil di rig
        }
    } else if (fm == FAILSAFE) {
        master_throttle = ch_throttle;
    } else {
        master_throttle = ch_throttle;
    }
    motor1_pwm = master_throttle + (int)(motor_speed_squared[0]);
    motor2_pwm = master_throttle + (int)(motor_speed_squared[1]);
    motor3_pwm = master_throttle + (int)(motor_speed_squared[2]);
    motor4_pwm = master_throttle + (int)(motor_speed_squared[3]);

    pwm_max = MAX_PWM;
    if (fm == FLIP_ROLL) {
        pwm_max = MAX_PWM_FLIP;
    }

    motor1_pwm = constrain_value(motor1_pwm, MIN_PWM, pwm_max);
    motor2_pwm = constrain_value(motor2_pwm, MIN_PWM, pwm_max);
    motor3_pwm = constrain_value(motor3_pwm, MIN_PWM, pwm_max);
    motor4_pwm = constrain_value(motor4_pwm, MIN_PWM, pwm_max);
}

void flip_lqr_control() {
    if (flip_start_time > 0 && millis() - flip_start_time > 8000) {
        fm = FAILSAFE;
    }
    if (abs(gxrs) > 800.0f) {
        fm = FAILSAFE;
    }
    static bool flip_started = false;
    if (fm == FLIP_ROLL && !flip_started) {
        flip_start_time = millis();
        max_flip_rate = 0.0f;
        flip_started = true;
    }
    if (fm != FLIP_ROLL) {
        flip_started = false;
        flip_start_time = 0;
    }

    if (fm != FLIP_ROLL) {
        fp = FLIP_IDLE;
        return;
    }
    switch (fp) {
        case FLIP_IDLE:
        fp = FLIP_PHASE_1;
        break;

        case FLIP_PHASE_1:
        if (roll > 70.0) fp = FLIP_PHASE_2;
        break;

        case FLIP_PHASE_2:
        if (roll > 160.0 || roll < -170.0) fp = FLIP_PHASE_3;
        break;

        case FLIP_PHASE_3:
        if (roll > -70.0 && roll < 0.0) fp = FLIP_PHASE_4;
        break;

        case FLIP_PHASE_4:
        if (abs(roll) < 10.0) fp = FLIP_FINISH;
        break;

        case FLIP_FINISH:
        Serial.print("Max Rate: ");
        Serial.print(max_flip_rate);
        Serial.println(" deg/s");

        fm = HOVER;
        fp = FLIP_IDLE;
        break;
    }
} 

void set_control_reference() {
    if (fm == FAILSAFE) {
        setpoint_roll  = 0.0f;
        setpoint_pitch = 0.0f;
        setpoint_yaw   = 0.0f;
        return;
    }

    if (fm == FLIP_ROLL) {
        setpoint_roll = flip_target_roll(fp);
        setpoint_pitch = 0.0;
        setpoint_yaw = 0.0;
        return;
    }
    
    setpoint_roll = roll_scaler() * MAX_ROLL;
    setpoint_pitch = -pitch_scaler() * MAX_PITCH;
    setpoint_yaw = yaw_scaler() * MAX_YAW;
}

#endif