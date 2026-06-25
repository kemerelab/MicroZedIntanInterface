// motion_estimator.c -- see motion_estimator.h. Pure C (stdint + math), no
// Xilinx/lwIP deps, so it builds + unit-tests under native gcc.
#include "motion_estimator.h"
#include <math.h>

// Nominal ADXL335 on the RHD aux ADC: ~300 mV/g, the aux LSB is small. The real
// counts->g must be characterized on hardware (6-position static cal); this is a
// placeholder so the units are roughly g. ~8000 counts/g => 1.25e-4 g/count.
#define MOTION_DEFAULT_GAIN   1.25e-4f
#define MOTION_SETTLE_TRIPLETS 64u   // ~ a few gravity time constants

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void motion_est_init(motion_est_t *m, float dt) {
    int k;
    for (k = 0; k < 3; k++) { m->g_lp[k] = 0.0f; m->vel[k] = 0.0f;
                              m->gain[k] = MOTION_DEFAULT_GAIN; m->bias[k] = 0.0f; }
    m->dt = (dt > 0.0f) ? dt : (1.0f / 1000.0f);
    // gravity EMA at ~0.5 Hz cutoff: alpha = 2*pi*fc*dt (clamped <1)
    m->gravity_alpha  = clampf(2.0f * 3.14159265f * 0.5f * m->dt, 0.0f, 1.0f);
    // activity RMS-EMA, tau ~0.2 s
    m->activity_alpha = clampf(m->dt / 0.2f, 0.0f, 1.0f);
    m->leak       = clampf(m->dt / 0.5f, 0.0f, 1.0f);  // velocity HP ~0.3 Hz
    m->zupt_thresh = 0.02f;                            // 20 mg of dynamic accel
    m->source = MOTION_SRC_ACCEL;
    m->act_ms = 0.0f; m->activity = 0.0f; m->speed = 0.0f; m->n = 0;
}

void motion_est_configure(motion_est_t *m, float gravity_fc_hz, float activity_tau_s,
                          float leak, float zupt_thresh) {
    if (gravity_fc_hz  >= 0.0f) m->gravity_alpha  = clampf(2.0f*3.14159265f*gravity_fc_hz*m->dt, 0.0f, 1.0f);
    if (activity_tau_s >  0.0f) m->activity_alpha = clampf(m->dt / activity_tau_s, 0.0f, 1.0f);
    if (leak           >= 0.0f) m->leak           = clampf(leak, 0.0f, 1.0f);
    if (zupt_thresh    >= 0.0f) m->zupt_thresh     = zupt_thresh;
}

void motion_est_set_calib(motion_est_t *m, int axis, float gain, float bias) {
    if (axis < 0 || axis > 2) return;
    m->gain[axis] = gain;
    m->bias[axis] = bias;
}

// Shared core: takes dynamic acceleration (g, gravity already removed) and
// updates the activity index + leaky-integrated speed proxy.
static void motion_apply_dynamic(motion_est_t *m, float dx, float dy, float dz) {
    float mag2 = dx*dx + dy*dy + dz*dz;
    // activity index: RMS via mean-square EMA
    m->act_ms += m->activity_alpha * (mag2 - m->act_ms);
    m->activity = sqrtf(m->act_ms);

    // speed proxy: integrate dynamic accel, leak to bound drift, ZUPT when still
    m->vel[0] += dx * m->dt;  m->vel[1] += dy * m->dt;  m->vel[2] += dz * m->dt;
    float keep = 1.0f - m->leak;
    if (m->activity < m->zupt_thresh) keep *= 0.5f;   // fast bleed when at rest
    m->vel[0] *= keep;  m->vel[1] *= keep;  m->vel[2] *= keep;
    m->speed = sqrtf(m->vel[0]*m->vel[0] + m->vel[1]*m->vel[1] + m->vel[2]*m->vel[2]);

    if (m->n < 0xFFFFFFFFu) m->n++;
}

void motion_est_push_accel(motion_est_t *m, int16_t x, int16_t y, int16_t z) {
    float a[3];
    int k;
    int16_t raw[3]; raw[0]=x; raw[1]=y; raw[2]=z;
    for (k = 0; k < 3; k++) a[k] = ((float)raw[k] - m->bias[k]) * m->gain[k];
    // gravity vector estimate (slow EMA) then remove it
    for (k = 0; k < 3; k++) m->g_lp[k] += m->gravity_alpha * (a[k] - m->g_lp[k]);
    // seed the gravity estimate on the very first sample so we don't integrate the
    // full 1 g startup step as "motion"
    if (m->n == 0) { m->g_lp[0]=a[0]; m->g_lp[1]=a[1]; m->g_lp[2]=a[2]; }
    motion_apply_dynamic(m, a[0]-m->g_lp[0], a[1]-m->g_lp[1], a[2]-m->g_lp[2]);
}

void motion_est_push_imu(motion_est_t *m, float lx, float ly, float lz) {
    // BNO055 LINEAR_ACCEL already has gravity removed -> feed directly.
    motion_apply_dynamic(m, lx, ly, lz);
}

float motion_est_speed(const motion_est_t *m)    { return m->speed; }
float motion_est_activity(const motion_est_t *m) { return m->activity; }
int   motion_est_settled(const motion_est_t *m)  { return m->n >= MOTION_SETTLE_TRIPLETS; }
