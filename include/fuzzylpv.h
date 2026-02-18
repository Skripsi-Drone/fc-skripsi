#ifndef FUZZYLPV_H
#define FUZZYLPV_H

#include <Fuzzy.h>
#include "bno_qt.h"
#include "actu_setup.h"
#include "control_flip_3.h"
#include "radio.h"

const int op_points = 3;
const float op_roll[op_points] = {0, 90, 180}; //deg
const float op_rate[op_points] = {200, 450, 650}; //deg/s 

//dituning
const float gain_k_roll[op_points] = {2.27, 3.20, 4.50};
const float gain_k_rate[op_points] = {2.00, 2.80, 3.80};

float weight[op_points] = {0.0, 0.0, 0.0};
//bacanya: weight 1 untuk op 1 dengan gain kroll 3.58 dan krate 3.07

extern float K_roll_effective;
extern float K_p_effective;

FuzzySet *RL, *RM, *RH;
FuzzySet *GL, *GM, *GH;
Fuzzy *fuzzyLPV = new Fuzzy();

void setup_fuzzy_lpv() {
    FuzzyInput *roll_angle = new FuzzyInput(1);

    //dituning
    RL = new FuzzySet(0, 0, 45, 90);
    RM = new FuzzySet(67.5, 120, 240, 300);
    RH = new FuzzySet(225, 325, 360, 360);
    // FuzzySet *RL = new FuzzySet(0, 0, 30, 60);
    // FuzzySet *RM = new FuzzySet(45, 90, 90, 135);
    // FuzzySet *RH = new FuzzySet(135, 180, 180, 225);
    roll_angle -> addFuzzySet(RL);
    roll_angle -> addFuzzySet(RM);
    roll_angle -> addFuzzySet(RH);
    fuzzyLPV   -> addFuzzyInput(roll_angle);

    FuzzyInput *roll_rate = new FuzzyInput(2);
    GL = new FuzzySet(0, 0, 200, 400);   
    GM = new FuzzySet(300, 500, 500, 850);
    GH = new FuzzySet(650, 775, 1000, 1000);  
    // FuzzySet *GL = new FuzzySet(0, 0, 150, 300);  
    // FuzzySet *GM = new FuzzySet(200, 450, 450, 700);  
    // FuzzySet *GH = new FuzzySet(600, 850, 1000, 1000);
    roll_rate -> addFuzzySet(GL);
    roll_rate -> addFuzzySet(GM);
    roll_rate -> addFuzzySet(GH);
    fuzzyLPV  -> addFuzzyInput(roll_rate);
}

void fuzzy_lpv_gain_sched(float roll_relative, float gxrs) {
    fuzzyLPV -> setInput(1, (fabs(roll_relative)));
    fuzzyLPV -> setInput(2, (fabs(-gxrs)));
    fuzzyLPV -> fuzzify();

    // baru, bisa dituning juga kasih persentase kepercayaan
    float weight1 = RL -> getPertinence() + GL -> getPertinence();
    float weight2 = RM -> getPertinence() + GM -> getPertinence();
    float weight3 = RH -> getPertinence() + GH -> getPertinence();

    float total_weight = weight1 + weight2 + weight3;

    if (total_weight > 0.01) {
        weight[0] = weight1 / total_weight;
        weight[1] = weight2 / total_weight;
        weight[2] = weight3 / total_weight;
    }

    K_roll_effective = (weight[0] * gain_k_roll[0]) + (weight[1] * gain_k_roll[1]) + (weight[2] * gain_k_roll[2]);
    K_p_effective    = (weight[0] * gain_k_rate[0]) + (weight[1] * gain_k_rate[1]) + (weight[2] * gain_k_rate[2]);
    // sampe sini barunya
}

#endif