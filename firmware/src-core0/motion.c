// motion.c -- see motion.h. PS-side movement front-end: DMA decimated accel
// blocks out of the PL accel BRAM, run the (prototype) estimator on the PS, hold
// the speed/activity for realtime consumers, optionally forward raw on UDP 5005.
#include "main.h"
#include "motion.h"
#include "motion_estimator.h"
#include "pl_dma.h"
#include "xil_io.h"
#include "shared_print.h"   // send_message
#include "lwip/udp.h"
#include "lwip/pbuf.h"

// Nominal broadband packet rate (= accel sample/packet rate). dt per triplet =
// decim_M / this. The exact rate is the PL acquisition rate; 30 kHz nominal.
#define ACCEL_PACKET_RATE_HZ   30000.0f

// Tracked config (mirrored into get_status).
uint8_t  motion_cfg_enable    = 0;
uint8_t  motion_cfg_headstage = 0;
uint8_t  motion_cfg_ema_shift = 4;
uint16_t motion_cfg_decim_M   = 30;     // 30 packets/triplet -> ~1 kHz triplets
uint32_t motion_blocks_processed = 0;
uint32_t motion_udp_packets_sent = 0;

static motion_est_t  est;
static struct udp_pcb *motion_pcb = NULL;
static uint32_t motion_read_word = 0;   // PS read pointer into the accel BRAM ring
static uint32_t cur_decim_M = 0;        // decim_M the estimator dt is currently set for
static volatile float motion_speed_g = 0.0f;
static volatile float motion_activity_g = 0.0f;

// Saved DSP tuning so a decim_M change (which re-inits dt) can re-apply it.
static float dsp_gravity_fc = 0.5f, dsp_activity_tau = 0.2f, dsp_leak = -1.0f, dsp_zupt = 0.02f;

#define ACCEL_BLOCK_WORDS  (ACCEL_BLOCK_HDR_WORDS + 2 * ACCEL_DEFAULT_N_TRIPLETS)

void motion_init(void) {
    motion_pcb = udp_new();
    if (motion_pcb == NULL) send_message("ERROR: Could not create movement UDP PCB\r\n");
    motion_read_word = 0;
    cur_decim_M = 0;
    motion_est_init(&est, (float)motion_cfg_decim_M / ACCEL_PACKET_RATE_HZ);
}

void motion_set_config(uint8_t enable, uint8_t headstage, uint8_t ema_shift, uint16_t decim_M) {
    motion_cfg_enable    = enable ? 1 : 0;
    motion_cfg_headstage = headstage & 0x3;
    motion_cfg_ema_shift = ema_shift & 0xF;
    motion_cfg_decim_M   = (decim_M == 0) ? 1 : (decim_M & 0x7FFF);
    uint32_t cfg = (motion_cfg_enable ? ACCEL_CFG_EN : 0)
                 | ((uint32_t)motion_cfg_headstage << ACCEL_CFG_HEADSTAGE_SHIFT)
                 | ((uint32_t)motion_cfg_ema_shift  << ACCEL_CFG_EMA_SHIFT_SHIFT)
                 | ((uint32_t)motion_cfg_decim_M    << ACCEL_CFG_DECIM_M_SHIFT);
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_ACCEL_CFG_OFFSET, cfg);
    send_message("Movement: en=%u headstage=%u ema_shift=%u decim_M=%u\r\n",
                 motion_cfg_enable, motion_cfg_headstage, motion_cfg_ema_shift, motion_cfg_decim_M);
}

void motion_set_dsp(float gravity_fc_hz, float activity_tau_s, float leak, float zupt_thresh) {
    // Negative = keep the saved value (so partial updates don't clobber the rest).
    if (gravity_fc_hz  >= 0.0f) dsp_gravity_fc   = gravity_fc_hz;
    if (activity_tau_s >  0.0f) dsp_activity_tau = activity_tau_s;
    if (leak           >= 0.0f) dsp_leak         = leak;
    if (zupt_thresh    >= 0.0f) dsp_zupt         = zupt_thresh;
    motion_est_configure(&est, gravity_fc_hz, activity_tau_s, leak, zupt_thresh);
}

float motion_get_speed(void)    { return motion_speed_g; }
float motion_get_activity(void) { return motion_activity_g; }

void motion_stream_service(void) {
    if (!motion_cfg_enable || motion_pcb == NULL) return;

    const uint32_t mask = ACCEL_BRAM_SIZE_WORDS - 1;
    uint32_t st = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_14_OFFSET);
    uint32_t wr_word = (st & 0xFFFF) >> 2;          // byte addr (past last block) -> word index
    uint32_t *blk = (uint32_t *)ACCEL_DMA_BUF_ADDR; // non-cacheable accel staging

    int budget = 4;                                  // cap blocks/call (they're ~10 Hz anyway)
    while (budget-- > 0) {
        if (((wr_word - motion_read_word) & mask) < ACCEL_BLOCK_WORDS) break;   // no full block yet

        // CDMA the whole block (header + triplets) into staging, split at ring wrap.
        int derr;
        if ((motion_read_word + ACCEL_BLOCK_WORDS) <= ACCEL_BRAM_SIZE_WORDS) {
            derr = pl_dma_read_addr(blk, ACCEL_BRAM_BASE_ADDR + (motion_read_word << 2), ACCEL_BLOCK_WORDS);
        } else {
            uint32_t first = ACCEL_BRAM_SIZE_WORDS - motion_read_word;
            derr  = pl_dma_read_addr(blk, ACCEL_BRAM_BASE_ADDR + (motion_read_word << 2), first);
            derr |= pl_dma_read_addr(blk + first, ACCEL_BRAM_BASE_ADDR, ACCEL_BLOCK_WORDS - first);
        }
        if (derr) { dma_errors++; break; }

        if (blk[0] == ACCEL_MAGIC_LOW && blk[1] == ACCEL_MAGIC_HIGH) {
            uint32_t n_trip  = blk[4] & 0xFF;
            uint32_t decim_M = (blk[4] >> ACCEL_CFG_DECIM_M_SHIFT) & 0x7FFF;
            if (n_trip > ACCEL_DEFAULT_N_TRIPLETS) n_trip = ACCEL_DEFAULT_N_TRIPLETS;
            if (decim_M != cur_decim_M && decim_M != 0) {
                // dt changed -> re-init the estimator at the new rate, re-apply tuning.
                motion_est_init(&est, (float)decim_M / ACCEL_PACKET_RATE_HZ);
                motion_est_configure(&est, dsp_gravity_fc, dsp_activity_tau, dsp_leak, dsp_zupt);
                cur_decim_M = decim_M;
            }
            for (uint32_t t = 0; t < n_trip; t++) {
                uint32_t wa = blk[ACCEL_BLOCK_HDR_WORDS + 2*t];
                uint32_t wb = blk[ACCEL_BLOCK_HDR_WORDS + 2*t + 1];
                int16_t x = (int16_t)(wa & 0xFFFF);
                int16_t y = (int16_t)(wa >> 16);
                int16_t z = (int16_t)(wb & 0xFFFF);
                motion_est_push_accel(&est, x, y, z);
            }
            motion_speed_g    = motion_est_speed(&est);
            motion_activity_g = motion_est_activity(&est);
            motion_blocks_processed++;

            // Optional: forward the raw decimated block to the host for recording.
            struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, ACCEL_BLOCK_WORDS * 4, PBUF_REF);
            if (p != NULL) {
                p->payload = (void*)blk;
                ip_addr_t dst; dst.addr = udp_dest_ip;
                if (udp_sendto(motion_pcb, p, &dst, ACCEL_UDP_PORT) == ERR_OK)
                    motion_udp_packets_sent++;
                pbuf_free(p);
            }
        }
        motion_read_word = (motion_read_word + ACCEL_BLOCK_WORDS) & mask;
    }
}
