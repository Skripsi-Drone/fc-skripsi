#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <sbus.h>

bfs::SbusRx sbus_rx(&Serial8);

bfs::SbusData data;

bool signal_lost = false;
int16_t ch_roll, ch_pitch, ch_throttle, ch_yaw;
bool arming;
bool alt_hold_mode = false;
unsigned long arrival_time;

void failsafe() {
    ch_roll = 1500;
    ch_pitch = 1500;
    ch_throttle = 1000;
    ch_yaw = 1500;
    arming = false;
}

void remote_setup() {
    sbus_rx.Begin();

    if (sbus_rx.Read()) {
        data = sbus_rx.data();

        arming = data.ch[4] > 1500 ? 1 : 0;
        signal_lost = data.lost_frame;

        if (arming) {
            Serial.println("Please disarm the remote for safety");
            while (arming) {
                if (sbus_rx.Read()) {
                    data = sbus_rx.data();
                    arming = data.ch[4] > 1500 ? 1 : 0;
                    signal_lost = data.lost_frame;
                }
            }
        }

        if (signal_lost) {
            Serial.println("Signal lost, please reconnect");
            while (signal_lost) {
                if (sbus_rx.Read()) {
                    data = sbus_rx.data();
                    arming = data.ch[4] > 1500 ? 1 : 0;
                    signal_lost = data.lost_frame;
                }
            }
        }
    }

    Serial.println("Remote setup complete");
}

void remote_loop() {
  if (sbus_rx.Read()) {
    arrival_time = millis();
    data = sbus_rx.data();
    signal_lost = data.lost_frame;

    if (signal_lost) {
      failsafe();
      return;
    }
    for (int i=0;i<5;i++) {
        Serial.print(data.ch[i]);
        Serial.print(" ");
    }
    Serial.println();

    ch_roll = data.ch[0] * 0.611f + 895;
    ch_pitch = data.ch[1] * 0.611f + 895;
    ch_throttle = data.ch[2] * 0.611f + 895;
    ch_yaw = data.ch[3] * 0.611f + 895;

    ch_roll = constrain(ch_roll, 1000, 2000);
    ch_pitch = constrain(ch_pitch, 1000, 2000);
    ch_throttle = constrain(ch_throttle, 1000, 2000);
    ch_yaw = constrain(ch_yaw, 1000, 2000);

    arming = data.ch[4] > 1500 ? true : false;
    if (arming) { digitalWrite(2, HIGH); }
    else { digitalWrite(2, LOW); }
    alt_hold_mode = data.ch[5] > 1500 ? true : false;
    //tambah trigger/switch buat ganti mode, pakai SC aja
  }
}

float roll_scaler() {
    return (ch_roll - 1500) / 500.0f;
}

float throttle_scaler() {
    return (ch_throttle - 1000) / 1000.0f;
}

float pitch_scaler() {
    return (ch_pitch - 1500) / 500.0f;
}

float yaw_scaler() {
    return (ch_yaw - 1500) / 500.0f;
}

#endif
