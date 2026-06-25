// motion.h -- PS firmware glue for the movement estimator.
//
// motion_stream_service() drains complete [x,y,z] accelerometer blocks from the
// PL accel BRAM (0x88000000) by CDMA (whole blocks, never Xil_In32 -- DMA rule),
// feeds them to the source-agnostic motion_estimator core, holds the resulting
// speed proxy + activity index in PS globals for realtime consumers + get_status,
// and (optionally) forwards the raw decimated block on UDP 5005 for host recording.
#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

// Init: estimator + UDP pcb. Call once from core-0 init (after pl_dma_init/lwip).
void  motion_init(void);

// Drain newly-completed accel blocks; call from the core-0 maintenance loop.
void  motion_stream_service(void);

// Configure the PL accel-extract engine (writes CTRL_REG_ACCEL_CFG) + track it for
// get_status. ema_shift is the per-axis EMA leak K; decim_M = packets per triplet.
void  motion_set_config(uint8_t enable, uint8_t headstage, uint8_t ema_shift, uint16_t decim_M);

// Tune the estimator DSP (firmware maps CMD_MOTION_SET_PARAMS here).
void  motion_set_dsp(float gravity_fc_hz, float activity_tau_s, float leak, float zupt_thresh);

// Outputs for PS realtime code + status.
float motion_get_speed(void);
float motion_get_activity(void);

// Tracked config / counters (mirrored into status_response_t).
extern uint8_t  motion_cfg_enable, motion_cfg_headstage, motion_cfg_ema_shift;
extern uint16_t motion_cfg_decim_M;
extern uint32_t motion_blocks_processed;
extern uint32_t motion_udp_packets_sent;

#endif // MOTION_H
