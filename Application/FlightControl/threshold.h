#ifndef THRESHOLDS_H
#define THRESHOLDS_H


#define PRESSURE_UPPER 100.0f
#define PRESSURE_LOWER 80.0f

// --- General Parameters ---
#define N2_FILL_PRESSURE_LIMIT           100.0f  // Bar
#define N2_CRITICAL_PRESSURE_LIMIT       50.0f   // Bar
#define OX_TEMPERATURE_OFFSET            10.0f   // °C
#define FUEL_MASS_LIMIT                  500.0f  // kg
#define MAX_HOLD_LIMIT                   60.0f   // s

// --- Avionics Parameters ---
#define LAUNCH_DELAY                     5.0f    // s
#define PRESSURIZATION_HOLD              10.0f   // s
#define PRESSURIZATION_OX_SET_PRESSURE   25.0f   // Bar
#define PRESSURIZATION_FUEL_SET_PRESSURE 25.0f   // Bar
#define PRESSURIZATION_RAMP_UP_DURATION  2.0f    // s
#define PRESSURIZATION_CHECK_PRESSURE    20.0f   // Bar
#define PRECHILL_DURATION                30.0f   // s
#define IGNITER_DURATION                 1.5f    // s
#define IGNITION_DELAY                   0.5f    // s
#define RAMP_UP_DURATION                 3.0f    // s
#define ACCEL_LIFTOFF_DURATION_MS        100.0f  // s
#define ACCEL_LIFTOFF                    15.0f   // m/s²
#define RAMP_UP_CHECK_PRESSURE           22.0f   // bar
#define ISP                              280.0f  // s
#define C_STAR                           1500.0f // m/s
#define THROAT_AREA                      0.005f  // m2
#define CUTOFF_DELAY                     0.2f    // s
#define MIN_BURN_DURATION                10.0f   // s
#define BURN_IMPULSE                     50000.0f // N.s
#define BURN_MAX_DURATION                120.0f  // s
#define ASCENT_MAX_DURATION              300.0f  // s
#define DESCENT_THRESHOLD_SPEED          -3.0f   // m/s
#define DESCENT_THRESHOLD_DURATION       0.3f    // s
#define PASSIVATION_DELAY_PRB            5.0f    // s
#define PASSIVATION_DELAY_DPR            5.0f    // s
#define PASSIVATION_DURATION_PRB         10.0f   // s
#define PASSIVATION_DURATION_DPR         10.0f   // s
#define PASSIVATION_DELAY_NO_COM_PRB     60.0f   // s
#define PASSIVATION_DELAY_NO_COM_DPR     60.0f   // s
#define DECENT_MAX_DURATION              200.0f  // s

#endif //THRESHOLDS_H
