#ifndef FUZZYLPV_H
#define FUZZYLPV_H

#include <Fuzzy.h>
#include "bno_qt.h"
#include "actu_setup.h"
#include "control_flip_3.h"
#include "radio.h"

const int op_points = 3;
const float op_roll[op_points] = {0, 90, 270}; //deg
const float op_rate[op_points] = {200, 620, 507}; //deg/s 

const float gain_k_roll[3] = {5.50, 4.00, 3.00};
const float gain_k_rate[3] = {2.50, 3.00, 4.00}; // naik ke akhir untuk braking

float weight[op_points] = {0.0, 0.0, 0.0};
//bacanya: weight 1 untuk op 1 dengan gain kroll 3.58 dan krate 3.07

extern float K_roll_effective;
extern float K_p_effective;

FuzzySet *RL, *RM, *RH;
FuzzySet *GL, *GM, *GH;
Fuzzy *fuzzyLPV = new Fuzzy();

void setup_fuzzy_lpv() {
    FuzzyInput *roll_angle = new FuzzyInput(1);
    RL = new FuzzySet(0,  0,   60, 120);   // dominan di 0-60°
    RM = new FuzzySet(60, 90,  90, 180);   // dominan di 90°
    RH = new FuzzySet(150, 270, 270, 360); // dominan di 270°
    roll_angle -> addFuzzySet(RL);
    roll_angle -> addFuzzySet(RM);
    roll_angle -> addFuzzySet(RH);
    fuzzyLPV   -> addFuzzyInput(roll_angle);

    FuzzyInput *roll_rate = new FuzzyInput(2);
    GL = new FuzzySet(0,   0,   200, 450);  // dominan di rate rendah
    GM = new FuzzySet(350, 620, 620, 720);  // dominan di sekitar 620
    GH = new FuzzySet(620, 720, 800, 800);  // zona transisi braking
    roll_rate -> addFuzzySet(GL);
    roll_rate -> addFuzzySet(GM);
    roll_rate -> addFuzzySet(GH);
    fuzzyLPV  -> addFuzzyInput(roll_rate);
}

void fuzzy_lpv_gain_sched(float roll_relative, float gxrs) {
    fuzzyLPV -> setInput(1, (fabs(roll_relative)));
    fuzzyLPV -> setInput(2, (fabs(-gxrs)));
    fuzzyLPV -> fuzzify();
    
    float weight1 = RL -> getPertinence() * GL -> getPertinence();
    float weight2 = RM -> getPertinence() * GM -> getPertinence();
    float weight3 = RH -> getPertinence() * GH -> getPertinence();

    float total_weight = weight1 + weight2 + weight3;

    if (total_weight > 0.01) {
        weight[0] = weight1 / total_weight;
        weight[1] = weight2 / total_weight;
        weight[2] = weight3 / total_weight;
    }

    K_roll_effective = (weight[0] * gain_k_roll[0]) + (weight[1] * gain_k_roll[1]) + (weight[2] * gain_k_roll[2]);
    K_p_effective    = (weight[0] * gain_k_rate[0]) + (weight[1] * gain_k_rate[1]) + (weight[2] * gain_k_rate[2]);
}

#endif