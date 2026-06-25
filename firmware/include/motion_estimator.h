// motion_estimator.h
//
// Source-agnostic movement/speed estimator core. Pure C (only <stdint.h>) so it
// compiles + unit-tests under native gcc with no Xilinx/lwIP dependency. The PS
// firmware (motion.c) DMAs decimated [x,y,z] accelerometer triplets out of the
// PL accel BRAM and feeds them here one triplet at a time; a future BNO055 IMU
// (Zynq PS I2C) feeds the SAME core via motion_est_push_imu(), so both sources
// drive one movement estimate (the fusion point is the PS).
//
// Output semantics (decided with Caleb): the PRIMARY output is a velocity/SPEED
// PROXY (leaky-integrated dynamic acceleration with zero-velocity updates). A
// single triaxial accelerometer CANNOT observe metric velocity -- the proxy is
// drift-bounded but NOT calibrated speed; it becomes meaningful once the IMU's
// orientation/linear-accel are fused. The drift-free ACTIVITY INDEX (RMS of
// band-passed dynamic-accel magnitude) is computed alongside and gates the
// integrator (ZUPT) -- it is the honest companion measure.
#ifndef MOTION_ESTIMATOR_H
#define MOTION_ESTIMATOR_H

#include <stdint.h>

typedef enum {
    MOTION_SRC_ACCEL = 0,   // analog accel only (own gravity removal)
    MOTION_SRC_IMU   = 1,   // BNO055 linear-accel (gravity already removed) -- future
    MOTION_SRC_FUSED = 2    // future
} motion_source_t;

typedef struct {
    // ---- configuration (set by motion_est_configure / firmware CMD_MOTION_*) ----
    float dt;             // seconds between triplets (= decim_M / 30000)
    float gravity_alpha;  // per-sample gravity EMA coeff (~ 2*pi*fc*dt, fc~0.5 Hz)
    float activity_alpha; // per-sample activity RMS-EMA coeff (~ dt/tau_act)
    float leak;           // per-sample velocity leak (drift bound), 0..1
    float zupt_thresh;    // activity (g) below which velocity is forced toward 0
    float gain[3];        // counts -> g, per axis
    float bias[3];        // zero-g offset in centered counts, per axis
    motion_source_t source;

    // ---- state ----
    float g_lp[3];        // gravity vector estimate (g)
    float vel[3];         // velocity proxy (integrated dynamic accel, arbitrary units)
    float act_ms;         // mean-square of dynamic-accel magnitude (EMA)
    float activity;       // sqrt(act_ms) -- the activity index (g)
    float speed;          // |vel| -- the speed proxy
    uint32_t n;           // triplets processed since init (settling counter)
} motion_est_t;

// Initialize with the triplet period dt (s) and sane default coefficients.
void  motion_est_init(motion_est_t *m, float dt);

// Override coefficients (firmware maps CMD_MOTION_* params here). Pass <0 to keep.
void  motion_est_configure(motion_est_t *m, float gravity_fc_hz, float activity_tau_s,
                           float leak, float zupt_thresh);

// Per-axis calibration (counts->g). axis 0..2.
void  motion_est_set_calib(motion_est_t *m, int axis, float gain, float bias);

// Feed one decimated accelerometer triplet (signed centered counts).
void  motion_est_push_accel(motion_est_t *m, int16_t x, int16_t y, int16_t z);

// Future IMU path: gravity-removed linear accel in g (BNO055 LINEAR_ACCEL).
void  motion_est_push_imu(motion_est_t *m, float lx, float ly, float lz);

// Outputs.
float motion_est_speed(const motion_est_t *m);     // primary: speed proxy
float motion_est_activity(const motion_est_t *m);  // supporting: activity index (g)
int   motion_est_settled(const motion_est_t *m);   // gravity EMA past startup transient

#endif // MOTION_ESTIMATOR_H
