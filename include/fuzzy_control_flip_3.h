#ifndef FUZZY_CONTROL_FLIP_3_H
#define FUZZY_CONTROL_FLIP_3_H

#include "Fuzzy.h"
#include "bno_qt.h"
#include "actu_setup.h"
#include "radio.h"
#include "control_flip_3.h"

// =======================
// EXTERNAL STATES
// =======================
extern float error_roll;
extern float error_roll_rate;

// =======================
// FUZZY OBJECT
// =======================
Fuzzy *fuzzyfliproll = new Fuzzy();

// =======================
// GAINS
// =======================
float adaptive_roll_gain = 5.0;   // FIXED
float adaptive_p_gain    = 3.5;   // FUZZY OUTPUT

// =======================
// UPDATE FUZZY GAIN
// =======================
void update_fuzzy_gain() {
    fuzzyfliproll->setInput(1, error_roll);
    fuzzyfliproll->setInput(2, error_roll_rate);

    fuzzyfliproll->fuzzify();

    adaptive_p_gain = fuzzyfliproll->defuzzify(1);

    // optional clamp (boleh hapus kalau mau raw)
    adaptive_p_gain = constrain(adaptive_p_gain, 2.0, 8.0);
}

// =======================
// SETUP FUZZY SYSTEM
// =======================
void setup_fuzzy_roll() {

    // -------------------------------------------------
    // INPUT 1: ERROR ROLL
    // -------------------------------------------------
    FuzzyInput *errorRoll = new FuzzyInput(1);

    FuzzySet *RollN = new FuzzySet(-31.0, -31.0, -26.0, -7.0);
    FuzzySet *RollZ = new FuzzySet(-10.0, -3.0, 3.0, 10.0);
    FuzzySet *RollP = new FuzzySet(7.0, 20.0, 25.0, 25.0);

    errorRoll->addFuzzySet(RollN);
    errorRoll->addFuzzySet(RollZ);
    errorRoll->addFuzzySet(RollP);

    fuzzyfliproll->addFuzzyInput(errorRoll);

    // -------------------------------------------------
    // INPUT 2: ERROR ROLL RATE
    // -------------------------------------------------
    FuzzyInput *errorRate = new FuzzyInput(2);

    FuzzySet *RateNB = new FuzzySet(-600, -450, -300, -150);
    FuzzySet *RateNS = new FuzzySet(-200, -120, -60, 0);
    FuzzySet *RateZ  = new FuzzySet(-30, -10, 10, 30);
    FuzzySet *RatePS = new FuzzySet(0, 60, 120, 200);
    FuzzySet *RatePB = new FuzzySet(150, 300, 450, 600);

    errorRate->addFuzzySet(RateNB);
    errorRate->addFuzzySet(RateNS);
    errorRate->addFuzzySet(RateZ);
    errorRate->addFuzzySet(RatePS);
    errorRate->addFuzzySet(RatePB);

    fuzzyfliproll->addFuzzyInput(errorRate);

    // -------------------------------------------------
    // OUTPUT: GAIN P (RATE / D GAIN)
    // -------------------------------------------------
    FuzzyOutput *GainP = new FuzzyOutput(1);

    // RAW
    // FuzzySet *PL = new FuzzySet(2.0, 2.5, 3.0, 3.5);   // brake / coast
    // FuzzySet *PM = new FuzzySet(3.5, 4.5, 5.5, 6.5);   // sustain
    // FuzzySet *PH = new FuzzySet(6.0, 7.0, 8.0, 8.0);   // kick

    // Modif V1 (202)
    // FuzzySet *PL = new FuzzySet(0.8, 1.2, 1.6, 2.2);   // very soft
    // FuzzySet *PM = new FuzzySet(2.0, 2.8, 3.6, 4.5);   // normal
    // FuzzySet *PH = new FuzzySet(4.0, 5.0, 6.0, 6.8);   // HARD BRAKE

    // V2 (204)
    // FuzzySet *PL = new FuzzySet(0.8, 1.2, 1.6, 2.2);
    // FuzzySet *PM = new FuzzySet(2.0, 2.8, 3.6, 4.5);
    // FuzzySet *PH = new FuzzySet(4.5, 5.2, 5.8, 6.2);  // DIPERHALUS

    // V3 (206)
    // FuzzySet *PL = new FuzzySet(0.6, 0.9, 1.3, 1.8);
    // FuzzySet *PM = new FuzzySet(1.8, 2.6, 3.6, 4.6);
    // FuzzySet *PH = new FuzzySet(4.6, 5.2, 5.8, 6.2);

    // V4
    // STRATEGI: Pisahkan jelas antara BUILD - SUSTAIN - BRAKE
    // FuzzySet *PL = new FuzzySet(1.5, 2.0, 2.5, 3.0);   // Coast (jangan lawan)
    // FuzzySet *PM = new FuzzySet(3.2, 3.8, 4.5, 5.0);   // Sustain
    // FuzzySet *PH = new FuzzySet(5.2, 6.0, 7.0, 8.0);   // Build + Brake (TINGGI!)

    // V5
    // TODO 1: besok 07/02/2026: habis coba settingan terakhir pakai roll abs sama roll rlv biasanya, coba fuzzyset ini 
    // FuzzySet *PL = new FuzzySet(1.2, 1.8, 2.4, 3.2);   // Coast (naikin dikit)
    // FuzzySet *PM = new FuzzySet(3.5, 4.2, 5.0, 5.8);   // Sustain (turunin sedikit)
    // FuzzySet *PH = new FuzzySet(6.0, 6.8, 7.5, 8.0);   // Aggressive
    // TODO 3: kalo msh jelek, ganti jadi: FuzzySet *PH = new FuzzySet(5.5, 6.2, 7.0, 7.5);

    // TODO 1: ini gain yg lbh bener dr aturan mamdani
    FuzzySet *PL = new FuzzySet(1.2, 1.8, 2.4, 3.2);
    FuzzySet *PM = new FuzzySet(3.5, 4.2, 5.0, 5.8);
    FuzzySet *PH = new FuzzySet(6.0, 6.8, 7.5, 8.0);

    GainP->addFuzzySet(PL);
    GainP->addFuzzySet(PM);
    GainP->addFuzzySet(PH);

    fuzzyfliproll->addFuzzyOutput(GainP);
    // -------- BUILD MOMENTUM (AWAL FLIP) --------
    // Roll masih positif, rate kecil → D BESAR
    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollP, RateZ);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PH); //PM LOCK
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(1, a, c));
    }

    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollP, RateNS);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PH); //PH PM PH
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(2, a, c));
    }

    // -------- COAST (JANGAN LAWAN MOMENTUM) --------
    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollP, RatePS);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PM);
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(3, a, c));
    }

    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollP, RatePB);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PL);
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(4, a, c));
    }

    // -------- BRAKE SETELAH LEWAT 180° --------
    // Roll negatif → HARUS REM
    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollN, RatePS);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PM); //TODO 2: PH, TODO 3: PM LOCK
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(5, a, c));
    }

    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollN, RateZ);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PL); //TODO 2: PM
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(6, a, c));
    }

    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollN, RateNS);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PL); //TODO 2: PL, kalo malah overshoot parah, balikin ke PM (TODO 3) LOCK 
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(7, a, c));
    }

    // -------- STABILISASI DEKET SETPOINT --------
    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollZ, RateZ);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PH);
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(8, a, c));
    }

    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollZ, RatePS);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PM); //PL
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(9, a, c));
    }

    // BARU (kamis)
     {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollN, RatePB);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PM); //TODO 2: PH sebenernya, tp PH bikin jelek/peak di akhir, worst case aja ini, gausa diganti sek LOCK
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(10, a, c));
    }

    // BARU lg (kamis tp lbh akhir)
    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollZ, RateNS);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PH); //kl jelek PB
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(11, a, c));
    }

    // BARU hr jumat, rules yg blm
    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollN, RateNB);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PM); //TODO 2: PL, kalo malah osilasi parah balikin ke PM (TODO 3) LOCK
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(12, a, c));
    }

    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollZ, RateNB);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PH); //TODO 4: PM (buat kurangi undershoot negatif)
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(13, a, c));
    }

    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollP, RateNB);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PL); // antara ini atau high PM
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(14, a, c));
    }

    {
        FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
        a->joinWithAND(RollZ, RatePB);
        FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
        c->addOutput(PH); //harusnya bukan low sih PH
        fuzzyfliproll->addFuzzyRule(new FuzzyRule(15, a, c));
    }
}

#endif