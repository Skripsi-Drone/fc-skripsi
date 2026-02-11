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

float outputgain_roll = 5.0;
float outputgain_p = 3.3;

void update_fuzzy_gain() {
    flipcuy -> setInput(1, roll_relative);
    flipcuy -> setInput(2, (fabs(-gxrs)));
    flipcuy -> fuzzify();
    outputgain_roll = flipcuy -> defuzzify(1);
    outputgain_p    = flipcuy -> defuzzify(2);
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
    FuzzySet *Slow = new FuzzySet(1.8, 2.2, 2.2, 2.8);       // was 2.8-4.5
    FuzzySet *RNorm = new FuzzySet(2.5, 3.2, 3.2, 4.0);      // was 4-6
    FuzzySet *Aggresive = new FuzzySet(3.5, 4.5, 4.5, 5.5);  // was 5.5-8.5
    gainRoll -> addFuzzySet(Slow);
    gainRoll -> addFuzzySet(RNorm);
    gainRoll -> addFuzzySet(Aggresive);
    flipcuy -> addFuzzyOutput(gainRoll);

    FuzzyOutput *gainP = new FuzzyOutput(2);
    FuzzySet *Damped = new FuzzySet(1.5, 2.0, 2.0, 2.5);     // was 2.2-3.2, lebih damped!
    FuzzySet *PNorm = new FuzzySet(2.3, 2.8, 2.8, 3.3);      // was 3-4
    FuzzySet *Responsive = new FuzzySet(3.0, 3.8, 3.8, 4.5);
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
    c->addOutput(Slow);
    c->addOutput(Damped);
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
    c->addOutput(RNorm);
    c->addOutput(PNorm);
    flipcuy->addFuzzyRule(new FuzzyRule(8, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RB, GH);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Slow);
    c->addOutput(PNorm);
    flipcuy->addFuzzyRule(new FuzzyRule(9, a, c));
    }
}

#endif