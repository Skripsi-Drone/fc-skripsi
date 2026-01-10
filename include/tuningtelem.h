#ifndef TUNINGTELEM_H
#define TUNINGTELEM_H

#include <Arduino.h>
#include "bno_qt.h"
#include "control_lqr.h"
#include "radio.h"


void printTimestamp() {
    unsigned long now = micros();
    
    // Hitung unit waktu
    unsigned long hours = now / 3600000000;
    unsigned long minutes = (now / 60000000) % 60;
    unsigned long seconds = (now / 1000000) % 60;
    unsigned long millisecs = (now / 1000) % 1000;
    unsigned long microsecs = now % 1000;

    Serial2.print("Time:");
    
    // Jam (XX:)
    if (hours < 10) Serial2.print("0");
    Serial2.print(hours);
    Serial2.print(":");

    // Menit (XX:)
    if (minutes < 10) Serial2.print("0");
    Serial2.print(minutes);
    Serial2.print(":");

    // Detik (XX.)
    if (seconds < 10) Serial2.print("0");
    Serial2.print(seconds);
    Serial2.print(".");

    // Milidetik (XXX.)
    if (millisecs < 100) Serial2.print("0"); // Biar rata kanan (0xx)
    if (millisecs < 10) Serial2.print("0");  // Biar rata kanan (00x)
    Serial2.print(millisecs);
    Serial2.print(".");

    // Mikrodetik (XXX)
    if (microsecs < 100) Serial2.print("0");
    if (microsecs < 10) Serial2.print("0");
    Serial2.print(microsecs);
    
    Serial2.print(" "); // Spasi pemisah dengan data berikutnya
}

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
                    modifygain(gain.yaw, 0.01);
                    break;
                case 'G':
                case 'g':
                    modifygain(gain.yaw, -0.01);
                    break;
                case 'Y':
                case 'y':
                    modifygain(gain.r, 0.01);
                    break;
                case 'H':
                case 'h':
                    modifygain(gain.r, -0.01);
                    break;
                default:
                    break;
            }
        }
    }
    printTimestamp();
    Serial2.print("GRoll:");
    Serial2.print(gain.roll);
    Serial2.print(" Gp:");
    Serial2.print(gain.p);
    Serial2.print(" GPitch:");
    Serial2.print(gain.pitch);
    Serial2.print(" Gq:");
    Serial2.print(gain.q);
    Serial2.print("GYaw:");
    Serial2.print(gain.yaw);
    Serial2.print(" Gr:");
    Serial2.print(gain.r);
    Serial2.print(" Roll:");
    Serial2.print(roll);
    Serial2.print( " gxrs:");
    Serial2.print(gxrs);
    Serial2.print(" Pitch:");
    Serial2.print(pitch);
    Serial2.print(" Gy:");
    Serial2.print(gyrs);
    Serial2.print(" Yaw:");
    Serial2.print(yaw);
    Serial2.print(" Error:");
    Serial2.print(error_roll);
    Serial2.print(" ");
    Serial2.print(error_pitch);
    Serial2.print(" ");
    Serial2.print(error_yaw);
    Serial2.print(" pwm:");
    Serial2.print(motor1_pwm);
    Serial2.print(" ");
    Serial2.print(motor2_pwm);
    Serial2.print(" ");
    Serial2.print(motor3_pwm);
    Serial2.print(" ");
    Serial2.print(motor4_pwm);
    Serial2.println(" ");
}

#endif