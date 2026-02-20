#ifndef FUFUZIZI_H
#define FUFUZIZI_H

#include <Fuzzy.h>
#include "bno_qt.h"
#include "actu_setup.h"
#include "control_flip_3.h"
#include "radio.h"

extern float roll_relative;
extern float gxrs;

Fuzzy *flipcuy = new Fuzzy();

#define KROLL_BASE 3.30f
#define KP_BASE 2.50f
float delta_kroll = 0.0f;
float delta_kp    = 0.0f;

void update_fuzzy_gain() {
    flipcuy -> setInput(1, roll_relative);
    flipcuy -> setInput(2, (fabs(-gxrs)));
    flipcuy -> fuzzify();
    delta_kroll = flipcuy -> defuzzify(1);
    delta_kp    = flipcuy -> defuzzify(2);
}

float get_kroll_eff() {
    return KROLL_BASE + delta_kroll;
}

float get_kp_eff() {
    return KP_BASE + delta_kp;
}

void fuzzy_roll() {
    FuzzyInput *roll_rlv = new FuzzyInput(1);
    FuzzySet *RS = new FuzzySet(-180, -180, 0, 90);
    FuzzySet *RM = new FuzzySet(60, 120, 240, 300);
    FuzzySet *RB = new FuzzySet(270, 330, 360, 360);
    roll_rlv -> addFuzzySet(RS);
    roll_rlv -> addFuzzySet(RM);
    roll_rlv -> addFuzzySet(RB);
    flipcuy -> addFuzzyInput(roll_rlv);

    FuzzyInput *roll_rate = new FuzzyInput(2);
    FuzzySet *GL = new FuzzySet(0, 0, 200, 400);
    FuzzySet *GM = new FuzzySet(300, 500, 500, 700);
    FuzzySet *GH = new FuzzySet(600, 800, 1000, 1000);
    roll_rate -> addFuzzySet(GL);
    roll_rate -> addFuzzySet(GM);
    roll_rate -> addFuzzySet(GH);
    flipcuy -> addFuzzyInput(roll_rate);

    FuzzyOutput *gainRoll = new FuzzyOutput(1);
    FuzzySet *Slow  = new FuzzySet(-1.45, -1.20, -0.80, 0.00);
    FuzzySet *RNorm = new FuzzySet(-0.40,  0.00,  0.00, 0.80);
    FuzzySet *Aggresive = new FuzzySet( 0.00,  0.60,  1.20, 1.45);
    gainRoll -> addFuzzySet(Slow);
    gainRoll -> addFuzzySet(RNorm);
    gainRoll -> addFuzzySet(Aggresive);
    flipcuy -> addFuzzyOutput(gainRoll);

    FuzzyOutput *gainP = new FuzzyOutput(2);
    FuzzySet *Damped = new FuzzySet(-1.40, -1.00, -0.70, 0.00);    
    FuzzySet *PNorm = new FuzzySet(-0.50,  0.00,  0.00, 0.80);     
    FuzzySet *Responsive = new FuzzySet(0.00,  0.60,  1.00, 1.40);
    gainP -> addFuzzySet(Damped);
    gainP -> addFuzzySet(PNorm);
    gainP -> addFuzzySet(Responsive);
    flipcuy -> addFuzzyOutput(gainP);

    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RS, GL);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(RNorm);
    c->addOutput(PNorm);
    flipcuy->addFuzzyRule(new FuzzyRule(1, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RS, GM);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Aggresive);
    c->addOutput(Responsive);
    flipcuy->addFuzzyRule(new FuzzyRule(2, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RS, GH);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Aggresive);
    c->addOutput(Responsive);
    flipcuy->addFuzzyRule(new FuzzyRule(3, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RM, GL);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Aggresive); //slow
    c->addOutput(Damped); //damped
    flipcuy->addFuzzyRule(new FuzzyRule(4, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RM, GM);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(RNorm); 
    c->addOutput(PNorm); 
    flipcuy->addFuzzyRule(new FuzzyRule(5, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RM, GH);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Slow);
    c->addOutput(Damped);
    flipcuy->addFuzzyRule(new FuzzyRule(6, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RB, GL);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Slow); 
    c->addOutput(Damped); 
    flipcuy->addFuzzyRule(new FuzzyRule(7, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RB, GM);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(RNorm); //norm
    c->addOutput(PNorm); //norm
    flipcuy->addFuzzyRule(new FuzzyRule(8, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RB, GH);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Slow);
    c->addOutput(PNorm); //norm
    flipcuy->addFuzzyRule(new FuzzyRule(9, a, c));
    }
}

#endif