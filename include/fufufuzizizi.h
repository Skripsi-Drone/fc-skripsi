#ifndef FUFUZIZI_H
#define FUFUZIZI_H

#include <Fuzzy.h>
#include "bno_qt.h"
#include "actu_setup.h"
#include "control_flip_3.h"
#include "radio.h"

extern float roll_relative;
extern float gxrs;

Fuzzy *fliproll = new Fuzzy();

#define KROLL_BASE 3.4f //3.38 3.76
#define KP_BASE 2.98f //2.88
float delta_kroll = 0.0f;
float delta_kp    = 0.0f;

void update_fuzzy_gain() {
    fliproll -> setInput(1, roll_relative);
    fliproll -> setInput(2, (fabs(-gxrs)));
    fliproll -> fuzzify();
    delta_kroll = fliproll -> defuzzify(1);
    delta_kp    = fliproll -> defuzzify(2);
}

float get_kroll_eff() {
    return KROLL_BASE + delta_kroll;
}

float get_kp_eff() {
    return KP_BASE + delta_kp;
}

void fuzzy_roll() {
    FuzzyInput *roll_rlv = new FuzzyInput(1);
    FuzzySet *RS = new FuzzySet(0, 30, 60, 110);
    FuzzySet *RM = new FuzzySet(80, 150, 220, 310);
    // FuzzySet *RS = new FuzzySet(0, 20, 50, 80); 
    // FuzzySet *RM = new FuzzySet(60, 120, 200, 280);
    FuzzySet *RB = new FuzzySet(280, 330, 360, 360);

    roll_rlv -> addFuzzySet(RS);
    roll_rlv -> addFuzzySet(RM);
    roll_rlv -> addFuzzySet(RB);
    fliproll -> addFuzzyInput(roll_rlv);

    FuzzyInput *roll_rate = new FuzzyInput(2);
    FuzzySet *GL = new FuzzySet(0, 0, 100, 300);
    FuzzySet *GM = new FuzzySet(200, 400, 400, 600);
    FuzzySet *GH = new FuzzySet(500, 700, 900, 900);
    roll_rate -> addFuzzySet(GL);
    roll_rate -> addFuzzySet(GM);
    roll_rate -> addFuzzySet(GH);
    fliproll -> addFuzzyInput(roll_rate);

    FuzzyOutput *gainRoll = new FuzzyOutput(1);
    FuzzySet *Slow  = new FuzzySet(-1.50, -1.00, -0.70, -0.30);
    FuzzySet *RNorm = new FuzzySet(-0.60, 0.00, 0.00, 0.90);
    FuzzySet *Aggresive = new FuzzySet(0.6, 1.2, 2.2, 3.4); //uji1
    // FuzzySet *Aggresive = new FuzzySet(0.6, 1.1, 1.6, 2.1); //uji2
    // FuzzySet *Aggressive = new FuzzySet(0.60, 0.90, 1.30, 1.80);
    gainRoll -> addFuzzySet(Slow);
    gainRoll -> addFuzzySet(RNorm);
    gainRoll -> addFuzzySet(Aggresive);
    fliproll -> addFuzzyOutput(gainRoll);

    FuzzyOutput *gainP = new FuzzyOutput(2);
    FuzzySet *Damped = new FuzzySet(-1.00, -0.80, -0.60, -0.20);    
    FuzzySet *PNorm = new FuzzySet(-0.50, 0.00, 0.00, 0.60);     
    FuzzySet *Responsive = new FuzzySet(0.30, 0.50, 0.80, 1.00);
    gainP -> addFuzzySet(Damped);
    gainP -> addFuzzySet(PNorm);
    gainP -> addFuzzySet(Responsive);
    fliproll -> addFuzzyOutput(gainP);

    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RS, GL);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Aggresive); //RNorm
    c->addOutput(PNorm);
    fliproll->addFuzzyRule(new FuzzyRule(1, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RS, GM);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Aggresive);
    c->addOutput(Responsive); //pnorm
    fliproll->addFuzzyRule(new FuzzyRule(2, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RS, GH);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Aggresive); //norm
    c->addOutput(Responsive); //damped, bisa jadi pnorm
    fliproll->addFuzzyRule(new FuzzyRule(3, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RM, GL);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    // cek ini bs jd ga cocok jd balikin ke norm semua
    c->addOutput(Aggresive); //norm
    c->addOutput(Responsive); //norm
    fliproll->addFuzzyRule(new FuzzyRule(4, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RM, GM);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(RNorm); //aggr
    c->addOutput(PNorm); 
    fliproll->addFuzzyRule(new FuzzyRule(5, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RM, GH);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Aggresive); //nprm
    c->addOutput(Damped); //norm
    fliproll->addFuzzyRule(new FuzzyRule(6, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RB, GL);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Slow); //slow
    c->addOutput(Responsive); //norm 
    fliproll->addFuzzyRule(new FuzzyRule(7, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RB, GM);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Slow); //norm
    c->addOutput(Responsive); //norm
    fliproll->addFuzzyRule(new FuzzyRule(8, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RB, GH);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Slow);
    c->addOutput(Responsive); //norm resp
    fliproll->addFuzzyRule(new FuzzyRule(9, a, c));
    }
}

#endif