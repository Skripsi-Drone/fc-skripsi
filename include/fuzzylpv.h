#ifndef FUZZYLPV_H
#define FUZZYLPV_H

#include <Fuzzy.h>
#include "bno_qt.h"
#include "actu_setup.h"
#include "control_flip_3.h"
#include "radio.h"

const int op_points = 3;
const float op_roll[op_points] = {12.3, 151.0, 331.4}; //deg
const float op_rate[op_points] = {201.4, 694.9, 492.0}; //deg/s 

const float gain_k_roll[op_points] = {3.58, 2.62, 2.60};
const float gain_k_rate[op_points] = {3.07, 2.24, 2.40};

float weight[op_points] = {0.0, 0.0, 0.0};

//bacanya: weight 1 untuk op 1 dengan gain kroll 3.58 dan krate 3.07

extern float K_roll_effective;
extern float K_p_effective;

Fuzzy *fuzzyLPV = new Fuzzy();

void setup_fuzzy_lpv() {
    FuzzyInput *roll_angle = new FuzzyInput(1);
    FuzzySet *RL = new FuzzySet(0, 0, 50, 100);
    FuzzySet *RM = new FuzzySet(80, 130, 180, 230);
    FuzzySet *RH = new FuzzySet(280, 330, 360, 360);
    roll_angle -> addFuzzySet(RL);
    roll_angle -> addFuzzySet(RM);
    roll_angle -> addFuzzySet(RH);
    fuzzyLPV   -> addFuzzyInput(roll_angle);

    FuzzyInput *roll_rate = new FuzzyInput(2);
    FuzzySet *GL = new FuzzySet(0, 0, 150, 350);        
    FuzzySet *GM = new FuzzySet(300, 500, 600, 750);     
    FuzzySet *GH = new FuzzySet(650, 800, 1000, 1000);  
    roll_rate -> addFuzzySet(GL);
    roll_rate -> addFuzzySet(GM);
    roll_rate -> addFuzzySet(GH);
    fuzzyLPV  -> addFuzzyInput(roll_rate);

    FuzzyOutput *weight_op1 = new FuzzyOutput(1);
    FuzzySet *W1_Z = new FuzzySet(0.0, 0.0, 0.0, 0.2);
    FuzzySet *W1_L = new FuzzySet(0.1, 0.3, 0.4, 0.6);
    FuzzySet *W1_H = new FuzzySet(0.5, 0.7, 1.0, 1.0);
    weight_op1->addFuzzySet(W1_Z);
    weight_op1->addFuzzySet(W1_L);
    weight_op1->addFuzzySet(W1_H);
    fuzzyLPV->addFuzzyOutput(weight_op1);

    FuzzyOutput *weight_op2 = new FuzzyOutput(2);
    FuzzySet *W2_Z = new FuzzySet(0.0, 0.0, 0.0, 0.2);
    FuzzySet *W2_L = new FuzzySet(0.1, 0.3, 0.4, 0.6);
    FuzzySet *W2_H = new FuzzySet(0.5, 0.7, 1.0, 1.0);
    weight_op2->addFuzzySet(W2_Z);
    weight_op2->addFuzzySet(W2_L);
    weight_op2->addFuzzySet(W2_H);
    fuzzyLPV->addFuzzyOutput(weight_op2);

    FuzzyOutput *weight_op3 = new FuzzyOutput(3);
    FuzzySet *W3_Z = new FuzzySet(0.0, 0.0, 0.0, 0.2);
    FuzzySet *W3_L = new FuzzySet(0.1, 0.3, 0.4, 0.6);
    FuzzySet *W3_H = new FuzzySet(0.5, 0.7, 1.0, 1.0);
    // membership w1-3 Z, L, H sama semua karena semua output adalah weight
    weight_op3->addFuzzySet(W3_Z);
    weight_op3->addFuzzySet(W3_L);
    weight_op3->addFuzzySet(W3_H);
    fuzzyLPV->addFuzzyOutput(weight_op3);

    // RULES
    {
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(RL, GL);
    FuzzyRuleConsequent *cons = new FuzzyRuleConsequent();
    cons->addOutput(W1_H);
    cons->addOutput(W2_Z);
    cons->addOutput(W3_Z);
    fuzzyLPV->addFuzzyRule(new FuzzyRule(1, ant, cons));
    }
    {
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(RL, GM);
    FuzzyRuleConsequent *cons = new FuzzyRuleConsequent();
    cons->addOutput(W1_H);
    cons->addOutput(W2_L);
    cons->addOutput(W3_Z);
    fuzzyLPV->addFuzzyRule(new FuzzyRule(2, ant, cons));
    }
    {
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(RM, GH);
    FuzzyRuleConsequent *cons = new FuzzyRuleConsequent();
    cons->addOutput(W1_Z);
    cons->addOutput(W2_H);
    cons->addOutput(W3_Z);
    fuzzyLPV->addFuzzyRule(new FuzzyRule(3, ant, cons));
    }
    {
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(RM, GM);
    FuzzyRuleConsequent *cons = new FuzzyRuleConsequent();
    cons->addOutput(W1_L);
    cons->addOutput(W2_H);
    cons->addOutput(W3_L);
    fuzzyLPV->addFuzzyRule(new FuzzyRule(4, ant, cons));
    }
    {
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(RH, GM);
    FuzzyRuleConsequent *cons = new FuzzyRuleConsequent();
    cons->addOutput(W1_Z);
    cons->addOutput(W2_L);
    cons->addOutput(W3_H);
    fuzzyLPV->addFuzzyRule(new FuzzyRule(5, ant, cons));
    }
    {
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(RH, GL);
    FuzzyRuleConsequent *cons = new FuzzyRuleConsequent();
    cons->addOutput(W1_Z);
    cons->addOutput(W2_Z);
    cons->addOutput(W3_H);
    fuzzyLPV->addFuzzyRule(new FuzzyRule(6, ant, cons));
    }
    {
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(RL, GH);
    FuzzyRuleConsequent *cons = new FuzzyRuleConsequent();
    cons->addOutput(W1_H);
    cons->addOutput(W2_H);
    cons->addOutput(W3_Z);
    fuzzyLPV->addFuzzyRule(new FuzzyRule(7, ant, cons));
    }
    {
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(RM, GL);
    FuzzyRuleConsequent *cons = new FuzzyRuleConsequent();
    cons->addOutput(W1_L);
    cons->addOutput(W2_H);
    cons->addOutput(W3_L);
    fuzzyLPV->addFuzzyRule(new FuzzyRule(8, ant, cons));
    }
    {
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(RH, GH);
    FuzzyRuleConsequent *cons = new FuzzyRuleConsequent();
    cons->addOutput(W1_Z);
    cons->addOutput(W2_H);
    cons->addOutput(W3_H);
    fuzzyLPV->addFuzzyRule(new FuzzyRule(9, ant, cons));
    }
}

void fuzzy_lpv_gain_sched(float roll_relative, float gxrs) {
    fuzzyLPV -> setInput(1, roll_relative);
    fuzzyLPV -> setInput(2, (fabs(-gxrs)));
    fuzzyLPV -> fuzzify();

    weight[0] = fuzzyLPV -> defuzzify(1);
    weight[1] = fuzzyLPV -> defuzzify(2);
    weight[2] = fuzzyLPV -> defuzzify(3);

    //normalize weights (sum=1)
    //misal 0.6 + 0.3 + 0.1, weight 0 maka 0.6/1 = 0.6 (60%)dst
    float sum_weight = weight[0] + weight[1] + weight[2];
    if (sum_weight > 0.01) {
        //threshold 0.01 krn kalo sum terlalu kecil, fuzzy gagal fire. avoid division by zero.
        weight[0] /= sum_weight;
        weight[1] /= sum_weight;
        weight[2] /= sum_weight;
    } else {
        weight[0] = weight[1] = weight[2] = 0.333; //equal weights
    }

    //polytopic interpolation - gain blending [K(ρ) = Σ μᵢ(ρ) × Kᵢ]
    K_roll_effective = 0.0;
    K_p_effective    = 0.0;

    for (int i=0; i<op_points; i++) {
        // K = weight1 * K + weight2 * K + weight3 * K
        K_roll_effective += weight[i] * gain_k_roll[i]; 
        K_p_effective    += weight[i] * gain_k_rate[i];
    }
}

#endif