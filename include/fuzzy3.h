#pragma once
#include "bno_qt.h"
#include "actu_setup.h"
#include "radio.h"
#include "control_flip_3.h"
#include "Fuzzy.h"

Fuzzy *fuzzyfliproll = new Fuzzy();

float outputgain;
float adaptive_roll_gain = 5.0; // Akan di-update dari baseline + adjustment
float adaptive_p_gain = 3.5;    // Akan di-update dari baseline + adjustment

float baseline_roll_gain = 5.0;  
float baseline_p_gain = 3.5;     

void update_fuzzy_gain() {
    // Set the inputs
    fuzzyfliproll->setInput(1, error_roll);
    fuzzyfliproll->setInput(2, error_roll_rate);
    
    // Running the fuzzification
    fuzzyfliproll->fuzzify();
    
    // Running the defuzzification - OUTPUT ADALAH ADJUSTMENT!
    float roll_adjustment = fuzzyfliproll->defuzzify(1);  // Range: ±1.0
    float p_adjustment = fuzzyfliproll->defuzzify(2);     // Range: ±1.0
    
    // Apply adjustment ke baseline
    set_gain_roll(roll_adjustment, p_adjustment); 
}

void set_gain_roll(float roll_adj, float p_adj) {
    adaptive_roll_gain = baseline_roll_gain + roll_adj;
    adaptive_p_gain = baseline_p_gain + p_adj;
    
    // Safety constraints
    adaptive_roll_gain = constrain(adaptive_roll_gain, 3.5, 7.0);  // ±40% dari baseline
    adaptive_p_gain = constrain(adaptive_p_gain, 2.5, 5.0);        // Bisa turun juga
}

void setup_fuzzy_roll() {
    FuzzyInput *errorRoll_fuzy = new FuzzyInput(1);
    FuzzySet *RollN = new FuzzySet(-31.0, -31.0, -26.33, -7.67); 
    errorRoll_fuzy->addFuzzySet(RollN);
    FuzzySet *RollZ = new FuzzySet(-14.67, -5.33, -0.67, 8.67); 
    errorRoll_fuzy->addFuzzySet(RollZ);
    FuzzySet *RollP = new FuzzySet(1.67, 20.33, 25.0, 25.0); 
    errorRoll_fuzy->addFuzzySet(RollP);
    fuzzyfliproll->addFuzzyInput(errorRoll_fuzy);
    
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

    FuzzyOutput *GainRoll = new FuzzyOutput(1);
    // LOW: Reduce gain (saat overshooting)
    FuzzySet *RL = new FuzzySet(-1.0, -1.0, -0.7, -0.4);
    GainRoll->addFuzzySet(RL);
    // MEDIUM: Baseline/slight adjustment
    FuzzySet *RM = new FuzzySet(-0.5, -0.2, 0.2, 0.5);
    GainRoll->addFuzzySet(RM);
    // HIGH: Increase gain (saat lagging)
    FuzzySet *RH = new FuzzySet(0.4, 0.7, 1.0, 1.0);
    GainRoll->addFuzzySet(RH);
    fuzzyfliproll->addFuzzyOutput(GainRoll);

    FuzzyOutput *GainP = new FuzzyOutput(2);
    // LOW: Reduce damping (butuh speed, lagging badly)
    FuzzySet *PL = new FuzzySet(-1.0, -1.0, -0.6, -0.3);
    GainP->addFuzzySet(PL);
    // MEDIUM: Baseline damping
    FuzzySet *PM = new FuzzySet(-0.4, -0.1, 0.1, 0.4);
    GainP->addFuzzySet(PM);
    // HIGH: Increase damping (prevent overshoot!)
    FuzzySet *PH = new FuzzySet(0.3, 0.6, 1.0, 1.0);
    GainP->addFuzzySet(PH);
    fuzzyfliproll->addFuzzyOutput(GainP);

    // Logic:
    // - Lagging (NEG roll, NEG rate) → Increase Kr, Reduce Kp (speed!)
    // - Overshooting (POS roll, POS rate) → Reduce Kr, Increase Kp (brake!)
    // - Good tracking (ZERO roll, ZERO rate) → Baseline
    
    // Rule 1: NEG roll, NB rate → Butuh speed maksimal!
    FuzzyRuleAntecedent *if_RollN_RateNB = new FuzzyRuleAntecedent();
    if_RollN_RateNB->joinWithAND(RollN, RateNB);
    FuzzyRuleConsequent *thenOP1 = new FuzzyRuleConsequent();
    thenOP1->addOutput(RH);  // Increase Kr HIGH (+0.8)
    thenOP1->addOutput(PL);  // Reduce Kp (need speed, not damping)
    FuzzyRule *fuzzyRule01 = new FuzzyRule(1, if_RollN_RateNB, thenOP1);
    fuzzyfliproll->addFuzzyRule(fuzzyRule01);

    // Rule 2: NEG roll, NS rate → Still lagging
    FuzzyRuleAntecedent *if_RollN_RateNS = new FuzzyRuleAntecedent();
    if_RollN_RateNS->joinWithAND(RollN, RateNS);
    FuzzyRuleConsequent *thenOP2 = new FuzzyRuleConsequent();
    thenOP2->addOutput(RH);  // Increase Kr HIGH
    thenOP2->addOutput(PM);  // Baseline damping
    FuzzyRule *fuzzyRule02 = new FuzzyRule(2, if_RollN_RateNS, thenOP2);
    fuzzyfliproll->addFuzzyRule(fuzzyRule02);

    // Rule 3: NEG roll, ZERO rate → Rate ok, position lagging
    FuzzyRuleAntecedent *if_RollN_RateZ = new FuzzyRuleAntecedent();
    if_RollN_RateZ->joinWithAND(RollN, RateZ);
    FuzzyRuleConsequent *thenOP3 = new FuzzyRuleConsequent();
    thenOP3->addOutput(RM);  // Medium increase
    thenOP3->addOutput(PM);  // Baseline damping
    FuzzyRule *fuzzyRule03 = new FuzzyRule(3, if_RollN_RateZ, thenOP3);
    fuzzyfliproll->addFuzzyRule(fuzzyRule03);

    // Rule 4: NEG roll, PS rate → Rate catching up
    FuzzyRuleAntecedent *if_RollN_RatePS = new FuzzyRuleAntecedent();
    if_RollN_RatePS->joinWithAND(RollN, RatePS);
    FuzzyRuleConsequent *thenOP4 = new FuzzyRuleConsequent();
    thenOP4->addOutput(RM);  // Baseline (rate will close the gap)
    thenOP4->addOutput(PM);  // Baseline damping
    FuzzyRule *fuzzyRule04 = new FuzzyRule(4, if_RollN_RatePS, thenOP4);
    fuzzyfliproll->addFuzzyRule(fuzzyRule04);

    // Rule 5: NEG roll, PB rate → Rate too fast! Will overshoot!
    FuzzyRuleAntecedent *if_RollN_RatePB = new FuzzyRuleAntecedent();
    if_RollN_RatePB->joinWithAND(RollN, RatePB);
    FuzzyRuleConsequent *thenOP5 = new FuzzyRuleConsequent();
    thenOP5->addOutput(RL);  // Reduce Kr (prevent overshoot)
    thenOP5->addOutput(PH);  // Increase damping HIGH
    FuzzyRule *fuzzyRule05 = new FuzzyRule(5, if_RollN_RatePB, thenOP5);
    fuzzyfliproll->addFuzzyRule(fuzzyRule05);
    
    // Rule 6: ZERO roll, NB rate → Rate lagging
    FuzzyRuleAntecedent *if_RollZ_RateNB = new FuzzyRuleAntecedent();
    if_RollZ_RateNB->joinWithAND(RollZ, RateNB);
    FuzzyRuleConsequent *thenOP6 = new FuzzyRuleConsequent();
    thenOP6->addOutput(RM);  // Slight increase
    thenOP6->addOutput(PM);  // Baseline damping
    FuzzyRule *fuzzyRule06 = new FuzzyRule(6, if_RollZ_RateNB, thenOP6);
    fuzzyfliproll->addFuzzyRule(fuzzyRule06);

    // Rule 7: ZERO roll, NS rate → Slight lag
    FuzzyRuleAntecedent *if_RollZ_RateNS = new FuzzyRuleAntecedent();
    if_RollZ_RateNS->joinWithAND(RollZ, RateNS);
    FuzzyRuleConsequent *thenOP7 = new FuzzyRuleConsequent();
    thenOP7->addOutput(RM);  // Baseline+
    thenOP7->addOutput(PM);  // Baseline damping
    FuzzyRule *fuzzyRule07 = new FuzzyRule(7, if_RollZ_RateNS, thenOP7);
    fuzzyfliproll->addFuzzyRule(fuzzyRule07);

    // Rule 8: ZERO roll, ZERO rate → PERFECT TRACKING!
    FuzzyRuleAntecedent *if_RollZ_RateZ = new FuzzyRuleAntecedent();
    if_RollZ_RateZ->joinWithAND(RollZ, RateZ);
    FuzzyRuleConsequent *thenOP8 = new FuzzyRuleConsequent();
    thenOP8->addOutput(RM);  // Keep baseline
    thenOP8->addOutput(PM);  // Keep baseline damping
    FuzzyRule *fuzzyRule08 = new FuzzyRule(8, if_RollZ_RateZ, thenOP8);
    fuzzyfliproll->addFuzzyRule(fuzzyRule08);

    // Rule 9: ZERO roll, PS rate → Slight lead
    FuzzyRuleAntecedent *if_RollZ_RatePS = new FuzzyRuleAntecedent();
    if_RollZ_RatePS->joinWithAND(RollZ, RatePS);
    FuzzyRuleConsequent *thenOP9 = new FuzzyRuleConsequent();
    thenOP9->addOutput(RM);  // Baseline-
    thenOP9->addOutput(PM);  // Baseline damping
    FuzzyRule *fuzzyRule09 = new FuzzyRule(9, if_RollZ_RatePS, thenOP9);
    fuzzyfliproll->addFuzzyRule(fuzzyRule09);

    // Rule 10: ZERO roll, PB rate → Rate too fast!
    FuzzyRuleAntecedent *if_RollZ_RatePB = new FuzzyRuleAntecedent();
    if_RollZ_RatePB->joinWithAND(RollZ, RatePB);
    FuzzyRuleConsequent *thenOP10 = new FuzzyRuleConsequent();
    thenOP10->addOutput(RL);  // Reduce Kr
    thenOP10->addOutput(PH);  // Increase damping (prevent overshoot)
    FuzzyRule *fuzzyRule10 = new FuzzyRule(10, if_RollZ_RatePB, thenOP10);
    fuzzyfliproll->addFuzzyRule(fuzzyRule10);
    
    // Rule 11: POS roll, NB rate → Overshoot but rate correcting
    FuzzyRuleAntecedent *if_RollP_RateNB = new FuzzyRuleAntecedent();
    if_RollP_RateNB->joinWithAND(RollP, RateNB);
    FuzzyRuleConsequent *thenOP11 = new FuzzyRuleConsequent();
    thenOP11->addOutput(RM);  // Baseline (will self-correct)
    thenOP11->addOutput(PM);  // Baseline damping
    FuzzyRule *fuzzyRule11 = new FuzzyRule(11, if_RollP_RateNB, thenOP11);
    fuzzyfliproll->addFuzzyRule(fuzzyRule11);

    // Rule 12: POS roll, NS rate → Still overshooting
    FuzzyRuleAntecedent *if_RollP_RateNS = new FuzzyRuleAntecedent();
    if_RollP_RateNS->joinWithAND(RollP, RateNS);
    FuzzyRuleConsequent *thenOP12 = new FuzzyRuleConsequent();
    thenOP12->addOutput(RM);  // Slight reduce
    thenOP12->addOutput(PM);  // Baseline damping
    FuzzyRule *fuzzyRule12 = new FuzzyRule(12, if_RollP_RateNS, thenOP12);
    fuzzyfliproll->addFuzzyRule(fuzzyRule12);

    // Rule 13: POS roll, ZERO rate → Overshoot stabilizing
    FuzzyRuleAntecedent *if_RollP_RateZ = new FuzzyRuleAntecedent();
    if_RollP_RateZ->joinWithAND(RollP, RateZ);
    FuzzyRuleConsequent *thenOP13 = new FuzzyRuleConsequent();
    thenOP13->addOutput(RL);  // Reduce Kr
    thenOP13->addOutput(PM);  // Baseline damping
    FuzzyRule *fuzzyRule13 = new FuzzyRule(13, if_RollP_RateZ, thenOP13);
    fuzzyfliproll->addFuzzyRule(fuzzyRule13);

    // Rule 14: POS roll, PS rate → Overshooting moderately!
    FuzzyRuleAntecedent *if_RollP_RatePS = new FuzzyRuleAntecedent();
    if_RollP_RatePS->joinWithAND(RollP, RatePS);
    FuzzyRuleConsequent *thenOP14 = new FuzzyRuleConsequent();
    thenOP14->addOutput(RL);  // Reduce Kr HIGH
    thenOP14->addOutput(PH);  // Increase damping HIGH
    FuzzyRule *fuzzyRule14 = new FuzzyRule(14, if_RollP_RatePS, thenOP14);
    fuzzyfliproll->addFuzzyRule(fuzzyRule14);

    // Rule 15: POS roll, PB rate → CRITICAL OVERSHOOT!
    FuzzyRuleAntecedent *if_RollP_RatePB = new FuzzyRuleAntecedent();
    if_RollP_RatePB->joinWithAND(RollP, RatePB);
    FuzzyRuleConsequent *thenOP15 = new FuzzyRuleConsequent();
    thenOP15->addOutput(RL);  // Reduce Kr MAX (-0.8 to -1.0)
    thenOP15->addOutput(PH);  // Increase damping MAX (+0.6 to +1.0)
    FuzzyRule *fuzzyRule15 = new FuzzyRule(15, if_RollP_RatePB, thenOP15);
    fuzzyfliproll->addFuzzyRule(fuzzyRule15);
}

// HELPER FUNCTION: Update baseline gains dari flipgain struct
// Panggil ini di setup() setelah flipgain di-set
void sync_fuzzy_baseline_gains(float roll_baseline, float p_baseline) {
    baseline_roll_gain = roll_baseline;
    baseline_p_gain = p_baseline;
    
    // Reset adaptive gains ke baseline
    adaptive_roll_gain = baseline_roll_gain;
    adaptive_p_gain = baseline_p_gain;
}

void print_fuzzy_debug() {
    Serial.print("Baseline Kr=");
    Serial.print(baseline_roll_gain, 2);
    Serial.print(" Kp=");
    Serial.print(baseline_p_gain, 2);
    Serial.print(" | Adaptive Kr=");
    Serial.print(adaptive_roll_gain, 2);
    Serial.print(" Kp=");
    Serial.print(adaptive_p_gain, 2);
    Serial.print(" | Adj Kr=");
    Serial.print(adaptive_roll_gain - baseline_roll_gain, 3);
    Serial.print(" Kp=");
    Serial.println(adaptive_p_gain - baseline_p_gain, 3);
}