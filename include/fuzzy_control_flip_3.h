#pragma once
#include "bno_qt.h"
#include "actu_setup.h"
#include "radio.h"
#include "control_flip_3.h"
#include "Fuzzy.h"

Fuzzy *fuzzyfliproll = new Fuzzy();

float outputgain;
float adaptive_roll_gain = 5.0; //sesuai gain di file sblh
float adaptive_p_gain = 3.5; 

void update_fuzzy_gain() {
    // set the input
    fuzzyfliproll->setInput(1, error_roll);
    fuzzyfliproll->setInput(2, error_roll_rate);
    // running the fuzzification
    fuzzyfliproll->fuzzify();
    // running the defuzzification
    float raw_gain_roll = fuzzyfliproll->defuzzify(1);
    float raw_gain_p = fuzzyfliproll->defuzzify(2);
    set_gain_roll(raw_gain_roll, raw_gain_p); 
}

void set_gain_roll(float fuzzy_P, float fuzzy_D) {
    adaptive_roll_gain = fuzzy_P;
    adaptive_p_gain = fuzzy_D;
}

void setup_fuzzy_roll() {
    // Err Roll -31 sampe 25, paling padat di -15, -10 s.d. 0 s.d. 10
    FuzzyInput *errorRoll_fuzy = new FuzzyInput(1);
    FuzzySet *RollN = new FuzzySet(-31.0, -31.0, -26.33, -7.67); 
    errorRoll_fuzy->addFuzzySet(RollN);
    FuzzySet *RollZ = new FuzzySet(-14.67, -5.33, -0.67, 8.67); 
    errorRoll_fuzy->addFuzzySet(RollZ);
    FuzzySet *RollP = new FuzzySet(1.67, 20.33, 25.0, 25.0); 
    errorRoll_fuzy->addFuzzySet(RollP);
    fuzzyfliproll->addFuzzyInput(errorRoll_fuzy);

    // Err Rate -175 sampe 500, padat di -35 s.d. 35, -175 s.d. -100, 100 s.d. 350 (coba dulu)
    FuzzyInput *errorRollRate_fuzy = new FuzzyInput(2);
    FuzzySet *RateNB = new FuzzySet(-200, -175, -120, -60);
    errorRollRate_fuzy->addFuzzySet(RateNB);
    FuzzySet *RateNS = new FuzzySet(-100, -60, -35, 0);
    errorRollRate_fuzy->addFuzzySet(RateNS);
    FuzzySet *RateZ = new FuzzySet(-35, -10, 10, 35);
    errorRollRate_fuzy->addFuzzySet(RateZ);
    FuzzySet *RatePS = new FuzzySet(0, 35, 80, 120);
    errorRollRate_fuzy->addFuzzySet(RatePS);
    FuzzySet *RatePB = new FuzzySet(80, 150, 500, 500);
    errorRollRate_fuzy->addFuzzySet(RatePB);
    fuzzyfliproll->addFuzzyInput(errorRollRate_fuzy);

    // Instantiating a FuzzyOutput objects
    // ni harusnya 2 gksi buat gain roll sama P
    FuzzyOutput *GainRoll = new FuzzyOutput(1);
    FuzzySet *RL = new FuzzySet(3.0, 3.0, 3.27, 5.69);
    GainRoll->addFuzzySet(RL);
    FuzzySet *RM = new FuzzySet(5.15, 6.37, 6.63, 7.85);
    GainRoll->addFuzzySet(RM);
    FuzzySet *RH = new FuzzySet(7.31, 9.73, 10.0, 10.0); 
    GainRoll->addFuzzySet(RH);
    fuzzyfliproll->addFuzzyOutput(GainRoll);

    FuzzyOutput *GainP = new FuzzyOutput(2);
    FuzzySet *PL = new FuzzySet(1.0, 1.0, 1.42, 3.8);
    GainP->addFuzzySet(PL);
    FuzzySet *PM = new FuzzySet(3.1, 4.29, 4.71, 5.9);
    GainP->addFuzzySet(PM);
    FuzzySet *PH = new FuzzySet(5.2, 7.58, 8.0, 8.0); 
    GainP->addFuzzySet(PH);
    fuzzyfliproll->addFuzzyOutput(GainP);

    // Instantiating a FuzzyRuleAntecedent objects
    // ini rule yg bener buat 2 I/O, buat 15x hehe, tp isi gainnya blm ya sama harus kayak gimana jg belum
    FuzzyRuleAntecedent *if_RollN_RateNB = new FuzzyRuleAntecedent();
    if_RollN_RateNB->joinWithAND(RollN, RateNB);
    FuzzyRuleConsequent *thenOP1 = new FuzzyRuleConsequent();
    thenOP1->addOutput(RL);
    thenOP1->addOutput(PH);
    FuzzyRule *fuzzyRule01 = new FuzzyRule(1, if_RollN_RateNB, thenOP1);
    fuzzyfliproll->addFuzzyRule(fuzzyRule01);

    FuzzyRuleAntecedent *if_RollN_RateNS = new FuzzyRuleAntecedent();
    if_RollN_RateNS->joinWithAND(RollN, RateNS);
    FuzzyRuleConsequent *thenOP2 = new FuzzyRuleConsequent();
    thenOP2->addOutput(RL);
    thenOP2->addOutput(PM);
    FuzzyRule *fuzzyRule02 = new FuzzyRule(2, if_RollN_RateNS, thenOP2);
    fuzzyfliproll->addFuzzyRule(fuzzyRule02);

    FuzzyRuleAntecedent *if_RollN_RateZ = new FuzzyRuleAntecedent();
    if_RollN_RateZ->joinWithAND(RollN, RateZ);
    FuzzyRuleConsequent *thenOP3 = new FuzzyRuleConsequent();
    thenOP3->addOutput(RL);
    thenOP3->addOutput(PM);
    FuzzyRule *fuzzyRule03 = new FuzzyRule(3, if_RollN_RateZ, thenOP3);
    fuzzyfliproll->addFuzzyRule(fuzzyRule03);

    FuzzyRuleAntecedent *if_RollN_RatePS = new FuzzyRuleAntecedent();
    if_RollN_RatePS->joinWithAND(RollN, RatePS);
    FuzzyRuleConsequent *thenOP4 = new FuzzyRuleConsequent();
    thenOP4->addOutput(RM);
    thenOP4->addOutput(PM);
    FuzzyRule *fuzzyRule04 = new FuzzyRule(4, if_RollN_RatePS, thenOP4);
    fuzzyfliproll->addFuzzyRule(fuzzyRule04);

    FuzzyRuleAntecedent *if_RollN_RatePB = new FuzzyRuleAntecedent();
    if_RollN_RatePB->joinWithAND(RollN, RatePB);
    FuzzyRuleConsequent *thenOP5 = new FuzzyRuleConsequent();
    thenOP5->addOutput(RM);
    thenOP5->addOutput(PM);
    FuzzyRule *fuzzyRule05 = new FuzzyRule(5, if_RollN_RatePB, thenOP5);
    fuzzyfliproll->addFuzzyRule(fuzzyRule05);

    FuzzyRuleAntecedent *if_RollZ_RateNB = new FuzzyRuleAntecedent();
    if_RollZ_RateNB->joinWithAND(RollZ, RateNB);
    FuzzyRuleConsequent *thenOP6 = new FuzzyRuleConsequent();
    thenOP6->addOutput(RL);
    thenOP6->addOutput(PH);
    FuzzyRule *fuzzyRule06 = new FuzzyRule(6, if_RollZ_RateNB, thenOP6);
    fuzzyfliproll->addFuzzyRule(fuzzyRule06);

    FuzzyRuleAntecedent *if_RollZ_RateNS = new FuzzyRuleAntecedent();
    if_RollZ_RateNS->joinWithAND(RollZ, RateNS);
    FuzzyRuleConsequent *thenOP7 = new FuzzyRuleConsequent();
    thenOP7->addOutput(RL);
    thenOP7->addOutput(PM);
    FuzzyRule *fuzzyRule07 = new FuzzyRule(7, if_RollZ_RateNS, thenOP7);
    fuzzyfliproll->addFuzzyRule(fuzzyRule07);

    FuzzyRuleAntecedent *if_RollZ_RateZ = new FuzzyRuleAntecedent();
    if_RollZ_RateZ->joinWithAND(RollZ, RateZ);
    FuzzyRuleConsequent *thenOP8 = new FuzzyRuleConsequent();
    thenOP8->addOutput(RL);
    thenOP8->addOutput(PM);
    FuzzyRule *fuzzyRule08 = new FuzzyRule(8, if_RollZ_RateZ, thenOP8);
    fuzzyfliproll->addFuzzyRule(fuzzyRule08);

    FuzzyRuleAntecedent *if_RollZ_RatePS = new FuzzyRuleAntecedent();
    if_RollZ_RatePS->joinWithAND(RollZ, RatePS);
    FuzzyRuleConsequent *thenOP9 = new FuzzyRuleConsequent();
    thenOP9->addOutput(RM);
    thenOP9->addOutput(PM);
    FuzzyRule *fuzzyRule09 = new FuzzyRule(9, if_RollZ_RatePS, thenOP9);
    fuzzyfliproll->addFuzzyRule(fuzzyRule09);

    FuzzyRuleAntecedent *if_RollZ_RatePB = new FuzzyRuleAntecedent();
    if_RollZ_RatePB->joinWithAND(RollZ, RatePB);
    FuzzyRuleConsequent *thenOP10 = new FuzzyRuleConsequent();
    thenOP10->addOutput(RM);
    thenOP10->addOutput(PL);
    FuzzyRule *fuzzyRule10 = new FuzzyRule(10, if_RollZ_RatePB, thenOP10);
    fuzzyfliproll->addFuzzyRule(fuzzyRule10);

    FuzzyRuleAntecedent *if_RollP_RateNB = new FuzzyRuleAntecedent();
    if_RollP_RateNB->joinWithAND(RollP, RateNB);
    FuzzyRuleConsequent *thenOP11 = new FuzzyRuleConsequent();
    thenOP11->addOutput(RM);
    thenOP11->addOutput(PM);
    FuzzyRule *fuzzyRule11 = new FuzzyRule(11, if_RollP_RateNB, thenOP11);
    fuzzyfliproll->addFuzzyRule(fuzzyRule11);

    FuzzyRuleAntecedent *if_RollP_RateNS = new FuzzyRuleAntecedent();
    if_RollP_RateNS->joinWithAND(RollP, RateNS);
    FuzzyRuleConsequent *thenOP12 = new FuzzyRuleConsequent();
    thenOP12->addOutput(RM);
    thenOP12->addOutput(PM);
    FuzzyRule *fuzzyRule12 = new FuzzyRule(12, if_RollP_RateNS, thenOP12);
    fuzzyfliproll->addFuzzyRule(fuzzyRule12);

    FuzzyRuleAntecedent *if_RollP_RateZ = new FuzzyRuleAntecedent();
    if_RollP_RateZ->joinWithAND(RollP, RateZ);
    FuzzyRuleConsequent *thenOP13 = new FuzzyRuleConsequent();
    thenOP13->addOutput(RH);
    thenOP13->addOutput(PM);
    FuzzyRule *fuzzyRule13 = new FuzzyRule(13, if_RollP_RateZ, thenOP13);
    fuzzyfliproll->addFuzzyRule(fuzzyRule13);

    FuzzyRuleAntecedent *if_RollP_RatePS = new FuzzyRuleAntecedent();
    if_RollP_RatePS->joinWithAND(RollP, RatePS);
    FuzzyRuleConsequent *thenOP14 = new FuzzyRuleConsequent();
    thenOP14->addOutput(RH);
    thenOP14->addOutput(PL);
    FuzzyRule *fuzzyRule14 = new FuzzyRule(14, if_RollP_RatePS, thenOP14);
    fuzzyfliproll->addFuzzyRule(fuzzyRule14);

    FuzzyRuleAntecedent *if_RollP_RatePB = new FuzzyRuleAntecedent();
    if_RollP_RatePB->joinWithAND(RollP, RatePB);
    FuzzyRuleConsequent *thenOP15 = new FuzzyRuleConsequent();
    thenOP15->addOutput(RH);
    thenOP15->addOutput(PL);
    FuzzyRule *fuzzyRule15 = new FuzzyRule(15, if_RollP_RatePB, thenOP15);
    fuzzyfliproll->addFuzzyRule(fuzzyRule15);
}

// void threadFWRoll() {
//     while (true) {
//         test_fuzzy_roll();
//         threads.yield();
//     }
// }