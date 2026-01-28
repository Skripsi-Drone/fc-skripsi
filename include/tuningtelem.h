#ifndef TUNINGTELEM_H
#define TUNINGTELEM_H

#include <Arduino.h>
#include "bno_qt.h"
// #include "control_lqr.h"
#include "control_flip_3.h"
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
            /*
            // Roll Rate Gains
            case 'q': modifygain(rategain.P,   0.01); break;
            case 'a': modifygain(rategain.P,  -0.01); break;
            case 'w': modifygain(rategain.D,   0.0001); break;
            case 's': modifygain(rategain.D,  -0.0001); break;
            case 'e': modifygain(rategain.I,   0.01); break;
            case 'd': modifygain(rategain.I,  -0.01); break;
            */
            // case 'r': modifygain(rategain.IMAX,      10.0); break;
            // case 'f': modifygain(rategain.IMAX,     -10.0); break;
            // case 't': modifygain(rategain.max_rate,  10.0); break;
            // case 'g': modifygain(rategain.max_rate, -10.0); break;
            /*desired roll rate < 600: naikin flip pulse delta / gain P
              desired roll rate >= 600: naikin gain D / U2 max
              flip < 360: gedein flip climb ms / flip pulse delta */
              // Flip Params (edisi 2)
            /*  
            case 'r': modifygain(flip_pulse_delta,   10); break;
            case 'f': modifygain(flip_pulse_delta,  -10); break;
            case 't': modifygain(flip_climb_ms,      10); break;
            case 'g': modifygain(flip_climb_ms,     -10); break;
            case 'y': modifygain(desired_roll_rate_climb,  10); break;
            case 'h': modifygain(desired_roll_rate_climb, -10); break;
            case 'u': modifygain(desired_roll_rate_recover,  10); break;
            case 'j': modifygain(desired_roll_rate_recover, -10); break;
            case 'i': modifygain(U2_MAX,          0.0003); break;
            case 'k': modifygain(U2_MAX,         -0.0003); break;
            */

            // Flip Params (edisi 3)
            // case 'q': modifygain(flipgain.roll,   0.01); break;
            // case 'a': modifygain(flipgain.roll,  -0.01); break;
            // case 'w': modifygain(flipgain.p,      0.01); break;
            // case 's': modifygain(flipgain.p,     -0.01); break;
            // case 'e': modifygain(flip_duration_sec,   0.05); break;
            // case 'd': modifygain(flip_duration_sec,  -0.05); break;

            // Angle Gains
            case 'q': modifygain(hovergain.roll,   0.01); break;
            case 'a': modifygain(hovergain.roll,  -0.01); break;
            case 'w': modifygain(hovergain.p,      0.01); break;
            case 's': modifygain(hovergain.p,     -0.01); break;
            case 'e': modifygain(hovergain.pitch,  0.01); break;
            case 'd': modifygain(hovergain.pitch, -0.01); break;
            case 'r': modifygain(hovergain.q,      0.01); break;
            case 'f': modifygain(hovergain.q,     -0.01); break;
            case 't': modifygain(hovergain.yaw,    0.001); break;
            case 'g': modifygain(hovergain.yaw,   -0.001); break;
            case 'y': modifygain(hovergain.r,      0.0001); break;
            case 'h': modifygain(hovergain.r,     -0.0001); break;
            case 'u': modifygain(hovergain.iy,     0.0001); break;
            case 'j': modifygain(hovergain.iy,    -0.0001); break; 
            case 'i': modifygain(flipgain.roll,   0.05); break;
            case 'k': modifygain(flipgain.roll,  -0.05); break;
            case 'o': modifygain(flipgain.p,      0.01); break;
            case 'l': modifygain(flipgain.p,     -0.01); break;
            case 'm': modifygain(flip_duration_sec,   0.05); break;
            case 'n': modifygain(flip_duration_sec,  -0.05); break;

        }
    }

    // Roll Rate gains
    // Serial2.print("<");
    // Serial2.print(millis());            Serial2.print(",");
    // Serial2.print(flip_switch_on);      Serial2.print(",");
    // Serial2.print(fp);                  Serial2.print(",");
    // Serial2.print(rategain.P, 2);       Serial2.print(",");
    // Serial2.print(rategain.D, 5);       Serial2.print(",");
    // Serial2.print(rategain.I, 2);       Serial2.print(",");
    // Serial2.print(rategain.IMAX);       Serial2.print(",");
    // Serial2.print(rategain.max_rate);   Serial2.print(",");

    // Flip Params (edisi 2)
    // Serial2.print(",fpd:");             Serial2.print(flip_pulse_delta);
    // Serial2.print(",fc:");              Serial2.print(flip_climb_ms);
    // Serial2.print(",dc:");              Serial2.print(desired_roll_rate_climb);
    // Serial2.print(",dr:");              Serial2.print(desired_roll_rate_recover);
    // Serial2.print(",u2:");              Serial2.print(U2_MAX, 5);

    // Serial2.print(",gx:");              Serial2.print(gxrs);
    // Serial2.print(",er:");              Serial2.print(er);
    // Serial2.print(",R:");               Serial2.print(roll);

    // Flip Params (edisi 3)
    // Serial2.print("<");
    // Serial2.print(micros());            Serial2.print(",");
    // Serial2.print(flip_switch_on);      Serial2.print(",");
    // Serial2.print(fp);                  Serial2.print(",");
    // Serial2.print(flipgain.roll);       Serial2.print(",");
    // Serial2.print(flipgain.p);          Serial2.print(",");
    // Serial2.print(flip_duration_sec);   Serial2.print(",");
    // Serial2.print(",SFR:");             Serial2.print(setpoint_flip_roll);
    // Serial2.print(",Rabs:");            Serial2.print(roll_absolute);
    // Serial2.print(",R:");               Serial2.print(roll);
    // Serial2.print(",SFG:");             Serial2.print(setpoint_flip_roll_rate);
    // Serial2.print(",gx:");              Serial2.print(-gxrs);
    // Serial2.print(",ERR:");             Serial2.print(error_roll);
    // Serial2.print(",ERG:");             Serial2.print(error_roll_rate);
    // Serial2.print(",u2:");              Serial2.print(u2, 5);
    // Serial.print(" ");                  Serial2.print(ch_throttle);

    // Angle Gains
    Serial2.print(millis());            Serial2.print(" ");
    Serial2.print(arming);              Serial2.print(",");
    Serial2.print(flip_switch_on);      Serial2.print(",");
    Serial2.print(fp);                  Serial2.print(" ");
    Serial2.print(hovergain.roll);      Serial2.print(" ");
    Serial2.print(hovergain.p);         Serial2.print(" ");
    Serial2.print(hovergain.pitch);     Serial2.print(" ");
    Serial2.print(hovergain.q);         Serial2.print(" ");
    Serial2.print(hovergain.yaw, 4);       Serial2.print(" ");
    Serial2.print(hovergain.r, 4);         Serial2.print(" ");
    Serial2.print(hovergain.iy, 4);        Serial2.print(" ");
    Serial2.print(flipgain.roll);       Serial2.print(" ");
    Serial2.print(flipgain.p);          Serial2.print(" ");
    Serial2.print(flip_duration_sec);   Serial2.print(" ");
    Serial2.print("R:");                Serial2.print(roll);
    Serial2.print(" gx:");               Serial2.print(-gxrs);
    Serial2.print(" P:");                Serial2.print(pitch);
    Serial2.print(" gy:");               Serial2.print(gyrs);
    Serial2.print(" Y:");                Serial2.print(yaw);
    Serial2.print(" gz:");               Serial2.print(gzrs);
    Serial2.print(" ");                  Serial2.print(ch_throttle);

    Serial2.println(">");
}

#endif