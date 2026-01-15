#ifndef TUNINGTELEM_H
#define TUNINGTELEM_H

#include <Arduino.h>
#include "bno_qt.h"
#include "control_lqr.h"
#include "radio.h"

void modifygain(float &prev_gain, float new_gain) {
    prev_gain += new_gain;
}

void telemetry_gain_tuning() {
    if (Serial2.available()) {
        char selector = Serial2.read();

        if (true) {
            switch (selector) {
                case 'Q':
                case 'q':
                    modifygain(gain.roll, 0.01);
                    break;
                case 'A':
                case 'a':
                    modifygain(gain.roll, -0.01);
                    break;
                case 'W':
                case 'w':
                    modifygain(gain.p, 0.01);
                    break;
                case 'S':
                case 's':
                    modifygain(gain.p, -0.01);
                    break;
                case 'E':
                case 'e':
                    modifygain(gain.pitch, 0.01);
                    break;
                case 'D':
                case 'd':
                    modifygain(gain.pitch, -0.01);
                    break;
                case 'R':
                case 'r':
                    modifygain(gain.q, 0.01);
                    break;
                case 'F':
                case 'f':
                    modifygain(gain.q, -0.01);
                    break;
                case 'T':
                case 't':
                    modifygain(gain.yaw, 0.001);
                    break;
                case 'G':
                case 'g':
                    modifygain(gain.yaw, -0.001);
                    break;
                case 'Y':
                case 'y':
                    modifygain(gain.r, 0.001);
                    break;
                case 'H':
                case 'h':
                    modifygain(gain.r, -0.001);
                    break;
                default:
                    break;
            }
        }
    }
    Serial2.print(millis());
    Serial2.print(" ");
    Serial2.print(gain.roll);
    Serial2.print(" ");
    Serial2.print(gain.p);
    Serial2.print(" ");
    Serial2.print(gain.pitch);
    Serial2.print(" ");
    Serial2.print(gain.q);
    Serial2.print(" ");
    Serial2.print(gain.yaw, 4);
    Serial2.print(" ");
    Serial2.print(gain.r, 4);
    Serial2.print(" R:");
    Serial2.print(roll);
    Serial2.print(" P:");
    Serial2.print(pitch);
    Serial2.print(" Y:");
    Serial2.print(yaw);
    Serial2.print(" gx:");
    Serial2.print(gxrs);
    Serial2.print(" gy:");
    Serial2.print(gyrs);
    Serial2.print(" gz:");
    Serial2.print(gzrs);
    // Serial2.print(" pyaw:");
    // Serial2.print(p_yaw);
    // Serial2.print(" dyaw:");
    // Serial2.print(d_yaw);
    // Serial2.print(" Error:");
    // Serial2.print(error_roll);
    // Serial2.print(" ");
    // Serial2.print(error_pitch);
    // Serial2.print(" ");
    // Serial2.print(error_yaw);
    // Serial2.print(" pwm:");
    // Serial2.print(motor1_pwm);
    // Serial2.print(" ");
    // Serial2.print(motor2_pwm);
    // Serial2.print(" ");
    // Serial2.print(motor3_pwm);
    // Serial2.print(" ");
    // Serial2.print(motor4_pwm);
    Serial2.println(" ");
}

#endif