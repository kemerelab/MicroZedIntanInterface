// Native-gcc unit test for motion_estimator.c (no Xilinx deps).
//   build+run:  cd firmware/test && bash run_motion_test.sh
#include "motion_estimator.h"
#include <stdio.h>
#include <math.h>

static int failures = 0;
static void check(int cond, const char *msg) {
    if (!cond) { printf("  FAIL: %s\n", msg); failures++; }
}

// counts for 1 g at the default gain (1.25e-4 g/count) -> 8000 counts/g
#define G_COUNTS 8000

int main(void) {
    motion_est_t m;
    motion_est_init(&m, 1.0f / 1000.0f);   // 1 kHz triplets

    // --- rest: gravity on z only ---
    for (int i = 0; i < 2000; i++)
        motion_est_push_accel(&m, 0, 0, G_COUNTS);
    printf("rest:    g_lp=(%.3f,%.3f,%.3f) activity=%.4f speed=%.4f\n",
           m.g_lp[0], m.g_lp[1], m.g_lp[2], motion_est_activity(&m), motion_est_speed(&m));
    check(fabsf(m.g_lp[2] - 1.0f) < 0.02f, "gravity converges to ~1g on z");
    check(fabsf(m.g_lp[0]) < 0.02f && fabsf(m.g_lp[1]) < 0.02f, "no gravity on x/y");
    check(motion_est_activity(&m) < 0.01f, "activity ~0 at rest");
    check(motion_est_speed(&m) < 0.01f, "speed proxy ~0 at rest");
    check(motion_est_settled(&m), "settled flag set after many triplets");

    // --- movement: oscillate x at ~5 Hz, +/-0.25 g, keep gravity on z ---
    float t = 0.0f;
    float peak_act = 0.0f, peak_speed = 0.0f;
    for (int i = 0; i < 1000; i++) {
        t += m.dt;
        int16_t x = (int16_t)(0.25f * G_COUNTS * sinf(2.0f * 3.14159265f * 5.0f * t));
        motion_est_push_accel(&m, x, 0, G_COUNTS);
        if (motion_est_activity(&m) > peak_act)   peak_act = motion_est_activity(&m);
        if (motion_est_speed(&m)    > peak_speed) peak_speed = motion_est_speed(&m);
    }
    printf("move:    activity=%.4f (peak %.4f) speed=%.4f (peak %.4f)\n",
           motion_est_activity(&m), peak_act, motion_est_speed(&m), peak_speed);
    check(peak_act > 0.1f, "activity rises during movement");
    check(peak_speed > 0.001f, "speed proxy rises during movement");
    check(motion_est_activity(&m) > 0.05f, "activity elevated while moving");

    // --- back to rest: activity decays, ZUPT bleeds speed back down ---
    for (int i = 0; i < 2000; i++)
        motion_est_push_accel(&m, 0, 0, G_COUNTS);
    printf("requiet: activity=%.4f speed=%.4f\n",
           motion_est_activity(&m), motion_est_speed(&m));
    check(motion_est_activity(&m) < 0.01f, "activity decays back to ~0");
    check(motion_est_speed(&m) < 0.01f, "speed proxy bleeds back to ~0 (ZUPT)");

    // --- drift bound: long rest must NOT accumulate speed ---
    for (int i = 0; i < 100000; i++) motion_est_push_accel(&m, 1, -1, G_COUNTS);
    printf("drift:   speed=%.6f (must stay bounded)\n", motion_est_speed(&m));
    check(motion_est_speed(&m) < 0.02f, "no runaway drift over long rest");

    // --- IMU path: linear accel feeds the same core ---
    motion_est_t mi;
    motion_est_init(&mi, 1.0f / 100.0f);
    float pk = 0.0f;
    for (int i = 0; i < 500; i++) {
        float ti = i * mi.dt;
        motion_est_push_imu(&mi, 0.3f * sinf(2.0f*3.14159265f*2.0f*ti), 0.0f, 0.0f);
        if (motion_est_activity(&mi) > pk) pk = motion_est_activity(&mi);
    }
    check(pk > 0.1f, "IMU linear-accel drives activity");

    if (failures == 0) printf("RESULT: PASS\n");
    else               printf("RESULT: FAIL (%d failures)\n", failures);
    return failures ? 1 : 0;
}
