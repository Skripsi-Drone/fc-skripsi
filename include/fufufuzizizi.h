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

#define KROLL_BASE 3.52f //3.38 3.76
#define KP_BASE 2.98f //2.88
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
    FuzzySet *RS = new FuzzySet(0, 30, 60, 100);
    // FuzzySet *RS = new FuzzySet(-10, 30, 60, 100);
    FuzzySet *RM = new FuzzySet(80, 150, 220, 300);
    FuzzySet *RB = new FuzzySet(270, 330, 360, 360);

    //opsi 2
    // FuzzySet *RS = new FuzzySet(  0,  20,  60, 100);  // ~100° wide
    // FuzzySet *RM = new FuzzySet( 80, 130, 210, 270);  // ~190° → ~170° wide
    // FuzzySet *RB = new FuzzySet(240, 290, 340, 360);  // ~120° wide

    roll_rlv -> addFuzzySet(RS);
    roll_rlv -> addFuzzySet(RM);
    roll_rlv -> addFuzzySet(RB);
    flipcuy -> addFuzzyInput(roll_rlv);

    FuzzyInput *roll_rate = new FuzzyInput(2);
    FuzzySet *GL = new FuzzySet(0, 0, 100, 300);
    FuzzySet *GM = new FuzzySet(200, 400, 400, 600);
    FuzzySet *GH = new FuzzySet(500, 700, 800, 900);
    roll_rate -> addFuzzySet(GL);
    roll_rate -> addFuzzySet(GM);
    roll_rate -> addFuzzySet(GH);
    flipcuy -> addFuzzyInput(roll_rate);

    FuzzyOutput *gainRoll = new FuzzyOutput(1);
    FuzzySet *Slow  = new FuzzySet(-2.00, -1.50, -1.00, -0.50);
    // -2.00, -1.80, -1.30, -0.50 atau -0.70
    FuzzySet *RNorm = new FuzzySet(-0.80, 0.00, 0.00, 0.90);
    // -0.50, 0, 0, 0.60
    FuzzySet *Aggresive = new FuzzySet( 0.50, 1.00, 1.50, 2.00);
    // 0.80, 1.30, 1.70, 1.90
    gainRoll -> addFuzzySet(Slow);
    gainRoll -> addFuzzySet(RNorm);
    gainRoll -> addFuzzySet(Aggresive);
    flipcuy -> addFuzzyOutput(gainRoll);

    FuzzyOutput *gainP = new FuzzyOutput(2);
    FuzzySet *Damped = new FuzzySet(-1.00, -0.80, -0.60, -0.30);    
    FuzzySet *PNorm = new FuzzySet(-0.50, 0.00, 0.00, 0.60);     
    FuzzySet *Responsive = new FuzzySet(0.30, 0.50, 0.80, 1.00);
    gainP -> addFuzzySet(Damped);
    gainP -> addFuzzySet(PNorm);
    gainP -> addFuzzySet(Responsive);
    flipcuy -> addFuzzyOutput(gainP);

    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RS, GL);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(RNorm); //RNorm
    c->addOutput(Damped);
    flipcuy->addFuzzyRule(new FuzzyRule(1, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RS, GM);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Aggresive);
    c->addOutput(Responsive); //pnorm
    flipcuy->addFuzzyRule(new FuzzyRule(2, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RS, GH);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Aggresive); //norm
    c->addOutput(Responsive); //damped, bisa jadi pnorm
    flipcuy->addFuzzyRule(new FuzzyRule(3, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RM, GL);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    // cek ini bs jd ga cocok jd balikin ke norm semua
    c->addOutput(Aggresive); //norm
    c->addOutput(Responsive); //norm
    flipcuy->addFuzzyRule(new FuzzyRule(4, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RM, GM);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(RNorm); //aggr
    c->addOutput(PNorm); 
    flipcuy->addFuzzyRule(new FuzzyRule(5, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RM, GH);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Slow); //nprm
    c->addOutput(Damped); //norm
    flipcuy->addFuzzyRule(new FuzzyRule(6, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RB, GL);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(RNorm); //slow
    c->addOutput(PNorm); 
    flipcuy->addFuzzyRule(new FuzzyRule(7, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RB, GM);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Slow); //norm
    c->addOutput(Responsive); //responsive
    flipcuy->addFuzzyRule(new FuzzyRule(8, a, c));
    }
    {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(RB, GH);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(Slow);
    c->addOutput(Responsive); //norm resp
    flipcuy->addFuzzyRule(new FuzzyRule(9, a, c));
    }
}

#endif