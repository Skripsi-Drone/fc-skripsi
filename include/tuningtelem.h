#ifndef TUNINGTELEM_H
#define TUNINGTELEM_H

#include <Arduino.h>
#include "bno_qt.h"
// #include "control_lqr.h"
#include "control_flip_2.h"
#include "radio.h"

void modifygain(float &prev_gain, float new_gain) {
    prev_gain += new_gain;
}

void modifygain(int &prev_gain, int new_gain) {
    prev_gain += new_gain;
}

void modifygain(uint32_t &prev_gain, int new_gain) {
    if (new_gain < 0 && prev_gain < (uint32_t)(-new_gain)) {
        prev_gain = 0; // Prevent underflow
    } else {
        prev_gain += new_gain;
    }
}

void telemetry_gain_tuning() {
    if (Serial2.available()) {
        char selector = tolower(Serial2.read()); 

        switch (selector) {
            // Roll Rate Gains
            case 'q': modifygain(rategain.P,   0.01); break;
            case 'a': modifygain(rategain.P,  -0.01); break;
            case 'w': modifygain(rategain.D,   0.0001); break;
            case 's': modifygain(rategain.D,  -0.0001); break;
            case 'e': modifygain(rategain.I,   0.01); break;
            case 'd': modifygain(rategain.I,  -0.01); break;
            // case 'r': modifygain(rategain.IMAX,      10.0); break;
            // case 'f': modifygain(rategain.IMAX,     -10.0); break;
            // case 't': modifygain(rategain.max_rate,  10.0); break;
            // case 'g': modifygain(rategain.max_rate, -10.0); break;

            /*desired roll rate < 600: naikin flip pulse delta / gain P
              desired roll rate >= 600: naikin gain D / U2 max
              flip < 360: gedein flip climb ms / flip pulse delta */
            // Flip Params
            
            case 'r': modifygain(flip_pulse_delta,   20); break;
            case 'f': modifygain(flip_pulse_delta,  -20); break;
            case 't': modifygain(flip_climb_ms,      10); break;
            case 'g': modifygain(flip_climb_ms,     -10); break;
            case 'y': modifygain(desired_roll_rate,  20); break;
            case 'h': modifygain(desired_roll_rate, -20); break;
            // case 'e': modifygain(rategain.P,      0.01); break;
            // case 'd': modifygain(rategain.P,     -0.01); break;
            // case 'r': modifygain(rategain.D,      0.0001); break;
            // case 'f': modifygain(rategain.D,     -0.0001); break;
            case 'u': modifygain(U2_MAX,          0.0003); break;
            case 'j': modifygain(U2_MAX,         -0.0003); break;
            /* 
            // Angle Gains
            case 'q': modifygain(gain.roll,   0.01); break;
            case 'a': modifygain(gain.roll,  -0.01); break;
            case 'w': modifygain(gain.p,      0.01); break;
            case 's': modifygain(gain.p,     -0.01); break;
            case 'e': modifygain(gain.pitch,  0.01); break;
            case 'd': modifygain(gain.pitch, -0.01); break;
            case 'r': modifygain(gain.q,      0.01); break;
            case 'f': modifygain(gain.q,     -0.01); break;
            case 't': modifygain(gain.yaw,    0.001); break;
            case 'g': modifygain(gain.yaw,   -0.001); break;
            case 'y': modifygain(gain.r,      0.0001); break;
            case 'h': modifygain(gain.r,     -0.0001); break;
            case 'u': modifygain(gain.iy,     0.0001); break;
            case 'j': modifygain(gain.iy,    -0.0001); break; 
            */
        }
    }

    // Roll Rate gains
    Serial2.print(millis());            Serial2.print(" ");
    Serial2.print(fp);                  Serial2.print(" ");
    Serial2.print(flip_switch_on);      Serial2.print(" ");
    Serial2.print(rategain.P, 2);       Serial2.print(" ");
    Serial2.print(rategain.D, 5);       Serial2.print(" ");
    Serial2.print(rategain.I, 2);       Serial2.print(" ");
    Serial2.print(rategain.IMAX);       Serial2.print(" ");
    Serial2.print(rategain.max_rate);   Serial2.print(" ");

    Serial2.print(" fpd:");             Serial2.print(flip_pulse_delta);
    Serial2.print(" fc:");              Serial2.print(flip_climb_ms);
    Serial2.print(" u2:");              Serial2.print(U2_MAX, 5);

    Serial2.print(" gx:");              Serial2.print(gxrs);
    Serial2.print(" drr:");             Serial2.print(desired_roll_rate);
    Serial2.print(" er:");              Serial2.print(er);
    Serial2.print(" R:");               Serial2.print(roll);

    // Flip Params

    // Angle Gains
    // Serial2.print(millis());            Serial2.print(" ");
    // Serial2.print(anglegain.roll);      Serial2.print(" ");
    // Serial2.print(anglegain.p);         Serial2.print(" ");
    // Serial2.print(anglegain.pitch);     Serial2.print(" ");
    // Serial2.print(anglegain.q);         Serial2.print(" ");
    // Serial2.print(anglegain.yaw);       Serial2.print(" ");
    // Serial2.print(anglegain.r);         Serial2.print(" ");
    // Serial2.print(anglegain.iy);        Serial2.print(" ");
    // Serial2.print("R:");                Serial2.print(roll);
    // Serial2.print("gx:");               Serial2.print(gxrs);

    Serial2.println(" ");
}

#endif