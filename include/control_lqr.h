#ifndef CONTROL_LQR_H
#define CONTROL_LQR_H

#include <Arduino.h>
#include <math.h>
#include "Radio.h"
#include "bno_qt.h"

#define MAX_ROLL 35.0f
#define MAX_PITCH 35.0f
#define MAX_YAW 35.0f
#define MIN_PWM 1000
#define MAX_PWM 1800
#define MAX_PWM_FLIP 2000
#define I_YAW_MAX 30.0f //trial

float u1, u2, u3, u4;
float yaw_integrator = 0.0f;
float setpoint_roll, setpoint_pitch, setpoint_yaw, target_alt;
float setpoint_roll_last, setpoint_pitch_last, setpoint_yaw_last, target_alt_last;
float setpoint_roll_now, setpoint_pitch_now, setpoint_yaw_now, target_alt_now;
float setpoint_roll_rate, setpoint_pitch_rate, setpoint_yaw_rate;
float p_roll, d_roll, p_pitch, d_pitch, p_yaw, d_yaw, i_yaw;
float error_roll, error_pitch, error_yaw;
float error_roll_rate, error_pitch_rate, error_yaw_rate;
float altitude;
float alt_last, alt_now, alt_vel;
float motor_speed_squared[4];
float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
uint32_t t_now, t_last, dt;

//zmr250 (roll pake arm length x)
const double A_invers[4][4] = {{206611.57024793,   -2035581.97288605,  2623638.98727535, -17730496.45390071},
                               {206611.57024793,   2035581.97288605,  2623638.98727535,  17730496.45390071}, 
                               {206611.57024793,   2035581.97288605, -2623638.98727535,  -17730496.45390071},
                               {206611.57024793,   -2035581.97288605, -2623638.98727535, 17730496.45390071}};
struct Gains {
    float alt   = 0.0f;
    float vz    = 0.0f;
    float roll  = 2.76; //2.65
    float p     = 1.94; //1.20 1.53 1.72 | 0.8 kurang msh osilasi jd gedein sampe 1 lebih. kl kurang naikin dikit aja
    float pitch = 2.85; //2.45
    float q     = 1.97; //0.98 1.28 | pake lpf gyro rangenya 0.98 sampe 1.0, tanpa lpf 1.0 sampe 1.1 atau tambah dikit lg
    float yaw   = 0.14; // 0.140
    float r     = 0.043; // 0.040
    float iy    = 0.0004; //trial
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

void drone_controller() {
    t_now = micros();
    dt = t_now - t_last;
    t_last = t_now;

    float dt_sec = dt * 1e-6f;
    // safety clamp (WAJIB)
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
    error_roll_rate = -gxrs;
    error_pitch = pitch - setpoint_pitch;
    error_pitch_rate = gyrs;
    error_yaw = yaw - setpoint_yaw;
    if (error_yaw > 180.0f) error_yaw -= 360.0f;
    if (error_yaw < -180.0f) error_yaw += 360.0f;
    error_yaw_rate = -gzrs;
    // float error_altitude = target_alt - altitude;

    /* u = -k * (state - setpoint) */
    p_roll = -gain.roll * error_roll;
    d_roll = -gain.p * error_roll_rate;
    p_pitch = -gain.pitch * error_pitch;
    d_pitch = -gain.q * error_pitch_rate;
    p_yaw = -gain.yaw * error_yaw;
    d_yaw = -gain.r * error_yaw_rate;

    yaw_integrator += error_yaw * dt_sec;
    yaw_integrator = constrain (yaw_integrator, -I_YAW_MAX, I_YAW_MAX);
    if (ch_throttle < 1100) {
        yaw_integrator = 0.0f;
    }
    i_yaw = -gain.iy * yaw_integrator;

    u1 = 0.0; // maximum thrust of x2216 skywalker x 4 using 1047 prop and 4s battery in newton //0.0;//(gain.alt * error_altitude) + (gain.vz * alt_vel);
    u2 = (p_roll + d_roll)/10'000'000.0f;
    //roll kiri positif, roll kanan negatif
    u3 = (p_pitch + d_pitch)/10'000'000.0f;
    u4 = (p_yaw + d_yaw + i_yaw)/10'000'000.0f;

    motor_speed_squared[0] = ((A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4));
    motor_speed_squared[1] = ((A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4));
    motor_speed_squared[2] = ((A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4));
    motor_speed_squared[3] = ((A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4));

    motor1_pwm = ch_throttle + (int)(motor_speed_squared[0]);
    motor2_pwm = ch_throttle + (int)(motor_speed_squared[1]);
    motor3_pwm = ch_throttle + (int)(motor_speed_squared[2]);
    motor4_pwm = ch_throttle + (int)(motor_speed_squared[3]);

    motor1_pwm = constrain_value(motor1_pwm, MIN_PWM, MAX_PWM);
    motor2_pwm = constrain_value(motor2_pwm, MIN_PWM, MAX_PWM);
    motor3_pwm = constrain_value(motor3_pwm, MIN_PWM, MAX_PWM);
    motor4_pwm = constrain_value(motor4_pwm, MIN_PWM, MAX_PWM);
}

void set_control_reference() {
    setpoint_roll = roll_scaler() * MAX_ROLL;
    setpoint_pitch = -pitch_scaler() * MAX_PITCH;
    setpoint_yaw = yaw_scaler() * MAX_YAW;
}
#endif