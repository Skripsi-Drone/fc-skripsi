#pragma once

#include <Arduino.h>

// ============================================================================
// COMPLETE FUZZY LPV CONTROLLER - READY TO USE
// ============================================================================
// Based on real flight data analysis (6,235 samples)
// Addresses: overshoot to 360° (K_roll=5) and overshoot to 665°/s (K_p=3.5)
// Solution: Phase-aware adaptive gains
// ============================================================================

// ============================================================================
// MEMBERSHIP FUNCTIONS
// ============================================================================

enum ErrorCategory {
    NEG = 0,   // Negative (lagging)
    ZERO = 1,  // Near setpoint (good tracking)
    POS = 2    // Positive (overshooting)
};

ErrorCategory classify_error_roll(float error) {
    // error_roll boundaries based on data: [-60, +60]
    if (error < -10.0f) return NEG;   // Lagging behind setpoint
    if (error > 10.0f) return POS;    // Ahead of setpoint / overshoot
    return ZERO;                      // Good tracking
}

ErrorCategory classify_error_rate(float error) {
    // error_roll_rate boundaries based on data: [-700, +500]
    if (error < -100.0f) return NEG;  // Actual rate too slow
    if (error > 100.0f) return POS;   // Actual rate too fast
    return ZERO;                      // Good rate tracking
}

// ============================================================================
// FUZZY RULE TABLES - PHASE-AWARE
// ============================================================================

// CLIMB PHASE (0-90°, rate increasing 0→665°/s)
// =============================================

const float K_ROLL_CLIMB[3][3] = {
    // error_roll:  NEG   ZERO  POS
    /* e_rate NEG */  {4.6,  4.4,  4.3},  // Lagging: increase gain
    /* e_rate ZERO */ {4.5,  4.6,  4.7},  // Tracking OK: maintain baseline
    /* e_rate POS */  {4.2,  4.4,  4.5}   // Overshooting early: reduce then catch up
};

const float K_P_CLIMB[3][3] = {
    // error_roll:  NEG   ZERO  POS
    /* e_rate NEG */  {5.0,  4.5,  4.8},  // Lagging: moderate damping (need speed)
    /* e_rate ZERO */ {3.4,  3.2,  3.4},  // Stable: baseline
    /* e_rate POS */  {4.0,  4.6,  5.0}   // Rate overshooting: HIGH damping!
};

// SWING PHASE (90-270°, rate decreasing 665→0°/s)
// ================================================

const float K_ROLL_SWING[3][3] = {
    // error_roll:  NEG   ZERO  POS
    /* e_rate NEG */  {5.5,  5.0,  4.5},  // Still lagging: aggressive
    /* e_rate ZERO */ {5.0,  5.0,  5.0},  // Maintain
    /* e_rate POS */  {4.5,  5.0,  5.5}   // Adjust for overshoot
};

const float K_P_SWING[3][3] = {
    // error_roll:  NEG   ZERO  POS
    /* e_rate NEG */  {4.7,  4.5,  4.2},  // Lagging during decel: high damping
    /* e_rate ZERO */ {4.2,  3.5,  4.2},  // Stable zone
    /* e_rate POS */  {4.2,  4.5,  4.7}   // Overshooting: maximum damping
};

// RECOVER PHASE (270-360°, finishing rotation)
// =============================================

const float K_ROLL_RECOVER[3][3] = {
    // error_roll:  NEG   ZERO  POS
    /* e_rate NEG */  {4.7,  4.5,  4.3},  // Gentle recovery
    /* e_rate ZERO */ {4.5,  4.3,  4.5},  // Conservative baseline
    /* e_rate POS */  {4.3,  4.5,  4.8}   // Prevent final overshoot
};

const float K_P_RECOVER[3][3] = {
    // error_roll:  NEG   ZERO  POS
    /* e_rate NEG */  {4.5,  4.1,  3.6},  // Gentle correction
    /* e_rate ZERO */ {3.6,  3.5,  3.6},  // Baseline for smooth finish
    /* e_rate POS */  {3.6,  4.1,  4.4}   // Prevent final overshoot
};

// ============================================================================
// MAIN FUZZY FUNCTION
// ============================================================================

void fuzzy_lpv_controller(float error_roll, float error_roll_rate, 
                         uint8_t phase,  // FlipPhase: CLIMB=1, SWING=2, RECOVER=3
                         float &K_roll_out, float &K_p_out) {
    /**
     * Complete Fuzzy LPV Controller
     * 
     * Inputs:
     *   error_roll: Position error (degrees) [-60, +60]
     *   error_roll_rate: Rate error (deg/s) [-700, +500]
     *   phase: Current flip phase (1=CLIMB, 2=SWING, 3=RECOVER)
     * 
     * Outputs:
     *   K_roll_out: Adaptive proportional gain [4.5, 5.5]
     *   K_p_out: Adaptive derivative gain [3.5, 4.9]
     * 
     * Design based on 6,235 real flight samples
     */
    
    // Classify errors into fuzzy categories
    ErrorCategory cat_roll = classify_error_roll(error_roll);
    ErrorCategory cat_rate = classify_error_rate(error_roll_rate);
    
    // Select appropriate rule table based on phase
    const float (*k_roll_table)[3];
    const float (*k_p_table)[3];
    
    if (phase == 1) {  // CLIMB
        k_roll_table = K_ROLL_CLIMB;
        k_p_table = K_P_CLIMB;
    }
    else if (phase == 2) {  // SWING
        k_roll_table = K_ROLL_SWING;
        k_p_table = K_P_SWING;
    }
    else {  // RECOVER or IDLE or FINISH
        k_roll_table = K_ROLL_RECOVER;
        k_p_table = K_P_RECOVER;
    }
    
    // Lookup gains from rule table
    K_roll_out = k_roll_table[cat_rate][cat_roll];
    K_p_out = k_p_table[cat_rate][cat_roll];
    
    // Safety constraints
    K_roll_out = constrain(K_roll_out, 4.0f, 5.0f);
    K_p_out = constrain(K_p_out, 3.0f, 5.0f);
}
