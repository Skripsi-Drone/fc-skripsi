#ifndef FUZZYLPV_H
#define FUZZYLPV_H

#include <Fuzzy.h>
#include "bno_qt.h"
#include "actu_setup.h"
#include "control_flip_3.h"
#include "radio.h"

const int op_points = 3;
const float op_roll[op_points] = {0, 180, 270}; //deg
const float op_rate[op_points] = {200, 660, 450}; //deg/s 

//dituning
// const float gain_k_roll[op_points] = {4.50, 5.50, 6.50};
// const float gain_k_roll[op_points] = {3.70, 2.90, 2.60}; //claude
// const float gain_k_rate[op_points] = {3.10, 2.40, 2.50}; //claude
// const float gain_k_rate[op_points] = {3.00, 3.40, 3.80};
// const float gain_k_roll[op_points] = {3.70, 3.50, 3.20};
// const float gain_k_rate[op_points] = {3.10, 2.80, 2.60};
//lumayan
// const float gain_k_roll[op_points] = {3.70, 4.20, 3.50};
// const float gain_k_rate[op_points] = {3.10, 3.00, 2.60};
//smooth lgsg climb swing tp tacking roll jelek
// const float gain_k_roll[op_points] = {4.70, 3.70, 3.00};
// const float gain_k_rate[op_points] = {2.30, 2.70, 3.00};
// const float gain_k_roll[op_points] = {4.90, 3.70, 4.50}; // better
//keep
// const float gain_k_roll[op_points] = {5.50, 4.00, 3.00};
// const float gain_k_rate[op_points] = {3.50, 3.00, 2.70}; // dibalik! besar di awal
const float gain_k_roll[op_points] = {4.40, 2.90, 3.30}; // jelek
// const float gain_k_roll[op_points] = {4.00, 5.50, 6.50}; // Naik perlahan
const float gain_k_rate[op_points] = {2.60, 3.20, 3.80}; // Damping makin kuat di akhir

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
    // RL = new FuzzySet(0, 5, 45, 90);
    // RM = new FuzzySet(67.5, 120, 240, 300);
    // RH = new FuzzySet(225, 325, 360, 360);
    // RL = new FuzzySet(-10.0, 5,  25,  50);   // cepat turun setelah 25°
    // RM = new FuzzySet(30,    60, 120, 160);  // mulai aktif lebih awal
    // RH = new FuzzySet(120,  160, 180, 180);  // puncak flip
    // bgus tp blm sampe 270
    // RL = new FuzzySet(0, 0, 30, 60);
    // RM = new FuzzySet(45, 80, 100, 140);
    // RH = new FuzzySet(120, 160, 180, 180);
    // di trial berkali2 di bawahnya komen ini 
    // RL = new FuzzySet(0, 0, 50, 90);
    // RM = new FuzzySet(60, 100, 160, 200);
    // RH = new FuzzySet(170, 200, 270, 270);
    RL = new FuzzySet(-180, -180, 0, 90);
    RM = new FuzzySet(60, 120, 240, 300);
    RH = new FuzzySet(270, 330, 360, 360);
    roll_angle -> addFuzzySet(RL);
    roll_angle -> addFuzzySet(RM);
    roll_angle -> addFuzzySet(RH);
    fuzzyLPV   -> addFuzzyInput(roll_angle);

    FuzzyInput *roll_rate = new FuzzyInput(2);
    GL = new FuzzySet(0, 0, 200, 400);
    GM = new FuzzySet(300, 500, 500, 700);
    GH = new FuzzySet(600, 800, 1000, 1000); 
    // GL = new FuzzySet(0, 0, 100, 200);
    // GM = new FuzzySet(150, 300, 450, 600);
    // GH = new FuzzySet(500, 700, 800, 800); 
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