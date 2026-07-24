// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// data_generator_aux_wire_tb.sv
//
// Integration test: proves the always-on aux engine's commands (and the override
// rewrite) reach the chip correctly by decoding the serialized COPI wire out of
// the real data_generator_core. Verifies:
//   - channel cycles 0..N_CHAN_CMDS-1 carry the host CONVERT table
//   - aux cycle AUX_CYC0+AUX_SWEEP_SLOT loops its programmed 3-entry program
//   - aux cycle AUX_CYC0+AUX_FS_SLOT: software fast settle -> WRITE(0,0xFE) for one
//     packet, then steady, then WRITE(0,0xDE) on the OFF edge
//   - a one-shot injection appears on the inject slot for exactly one packet
//   - DSP reset (reg22 dsp_sw): forces bit H (the CONVERT LSB) high on EVERY
//     channel CONVERT while held, and clears it when released
//   - Reg-3 digout (reg22 digout_sw): substitutes the live D0 (reg3_shadow) into
//     a WRITE(3) command's low byte on the wire
//
// The last two fold in the override-rewrite coverage that used to sit in
// aux_command_engine_tb, at the level that actually reaches the chip.
//
// Run: bash programmable_logic/sim/run_aux_wire_tb.sh  ("RESULT: PASS")

`timescale 1ns/1ps

import acq_frame_pkg::*;

module data_generator_aux_wire_tb;

logic clk = 0; always #5 clk = ~clk;
logic rstn = 0;
int n_checks = 0, n_errors = 0;
task automatic chk(input string what, input logic [15:0] got, input logic [15:0] exp);
    n_checks++;
    if (got !== exp) begin n_errors++; $display("ERROR: %s got=%04h exp=%04h", what, got, exp); end
endtask

logic [32*25-1:0] ctrl = '0;
logic cipo0 = 1, cipo1 = 0;             // deterministic CIPO: regular words FFFF/0000
logic [7:0] digital_in = 8'h00;

logic n_fifo_we, n_pkt_end;
logic [127:0] n_fifo_wd;
logic [7:0]  n_mask;
logic n_csn, n_sclk, n_copi;
logic [32*10-1:0] n_status;
logic [31:0] aux_status, aux_read_result;

data_generator_core dut (
    .clk(clk), .rstn(rstn),
    .ctrl_regs_pl(ctrl), .status_regs_pl(n_status),
    .aux_status(aux_status), .aux_read_result(aux_read_result),
    .fifo_write_en(n_fifo_we), .fifo_write_data(n_fifo_wd),
    .fifo_channel_mask(n_mask), .fifo_full(1'b0), .fifo_count(9'd0),
    .fifo_packet_end_flag(n_pkt_end),
    .csn(n_csn), .sclk(n_sclk), .copi(n_copi),
    .cipo_a0(cipo0), .cipo_a1(cipo1), .cipo_b0(1'b0), .cipo_b1(1'b0),
    .digital_in(digital_in)
);

// ---- control-register helpers ----
task automatic set_ctrl(input int idx, input logic [31:0] val); ctrl[idx*32 +: 32] = val; endtask
function automatic logic [15:0] convert_word(input int i); return {2'b00, 6'(i), 8'h00}; endfunction
task automatic load_convert_table;
    for (int j = 0; j < 18; j++) begin
        logic [31:0] v;
        v[15:0]  = convert_word(2*j);
        v[31:16] = (2*j+1 < N_FRAME_CMDS) ? convert_word(2*j+1) : 16'h0;
        set_ctrl(4+j, v);
    end
endtask
logic wr_tog = 0, inj_tog = 0; logic [15:0] inj_cmd_sh = 0;
task automatic aux_write(input int slot, input bit bank, input bit is_len, input logic [5:0] addr, input logic [15:0] data);
    set_ctrl(23, {6'b0, is_len, bank, 2'(slot), addr, data});
    repeat (2) @(negedge clk); wr_tog = ~wr_tog;
    set_ctrl(24, {inj_cmd_sh, 14'b0, inj_tog, wr_tog});
    repeat (4) @(negedge clk);
endtask
task automatic aux_inject(input logic [15:0] cmd);
    inj_cmd_sh = cmd; inj_tog = ~inj_tog;
    set_ctrl(24, {inj_cmd_sh, 14'b0, inj_tog, wr_tog});
    repeat (4) @(negedge clk);
endtask

// ---- COPI wire decoder: sample at SCLK rising edges, MSB first ----
logic sclk_d = 0; int bit_cnt = 0; logic [15:0] rx; logic [15:0] frame [0:N_FRAME_CMDS-1];
int fidx = 0; bit frame_done = 0;
always @(posedge clk) begin
    sclk_d <= n_sclk;
    if (n_sclk && !sclk_d) begin
        rx = {rx[14:0], n_copi}; bit_cnt++;
        if (bit_cnt == 16) begin
            frame[fidx] = rx; bit_cnt = 0;
            if (fidx == N_FRAME_CMDS-1) begin fidx = 0; frame_done = 1; end else fidx++;
        end
    end
end
task automatic next_packet(output logic [15:0] w [0:N_FRAME_CMDS-1]);
    frame_done = 0; wait (frame_done == 1);
    for (int i = 0; i < N_FRAME_CMDS; i++) w[i] = frame[i];
endtask

logic [15:0] pkt [0:N_FRAME_CMDS-1];
localparam int SWEEP_CYC  = AUX_CYC0 + AUX_SWEEP_SLOT;
localparam int FS_CYC     = AUX_CYC0 + AUX_FS_SLOT;
localparam int INJECT_CYC = AUX_CYC0 + AUX_INJECT_SLOT;

initial begin
    load_convert_table();
    set_ctrl(1, 32'd0);            // loop_count = 0 (continuous)
    set_ctrl(2, 32'h000F_0000);    // channel_enable = 0x0F (CTRL_REG_2 [23:16])
    repeat (8) @(negedge clk); rstn = 1; repeat (8) @(negedge clk);

    // slot 0 (program): a 3-entry loop. slot 1 (fs register): a single WRITE(0,0)
    // set via target 1 (bank/addr/length ignored -- it does not cycle).
    aux_write(AUX_SWEEP_SLOT, 0, 0, 0, 16'h2000);
    aux_write(AUX_SWEEP_SLOT, 0, 0, 1, 16'h2100);
    aux_write(AUX_SWEEP_SLOT, 0, 0, 2, 16'h2200);
    aux_write(AUX_SWEEP_SLOT, 0, 1, 0, {2'b00, 6'd2, 2'b00, 6'd0});    // loop 0..2
    aux_write(AUX_FS_SLOT,    0, 0, 0, 16'h8000);                      // WRITE(0,0x00) -> fs_cmd
    // reg3_static + no aux enable bit needed (always on)
    set_ctrl(22, {8'h1C, 24'b0});
    repeat (4) @(negedge clk);
    set_ctrl(0, 32'h1);            // start transmission

    // packet 0: channel cycles carry the CONVERT table; sweep slot plays entry 0
    next_packet(pkt);
    for (int i = 0; i < N_CHAN_CMDS; i++) chk($sformatf("p0 conv[%0d]", i), pkt[i], convert_word(i));
    chk("p0 sweep[0]", pkt[SWEEP_CYC], 16'h2000);
    next_packet(pkt); chk("p1 sweep[1]", pkt[SWEEP_CYC], 16'h2100);
    next_packet(pkt); chk("p2 sweep[2]", pkt[SWEEP_CYC], 16'h2200);
    next_packet(pkt); chk("p3 sweep wrap", pkt[SWEEP_CYC], 16'h2000);

    // ---- fast settle ON edge -> WRITE(0,0xFE) on the FS slot for one packet ----
    set_ctrl(22, {8'h1C, 19'b0, 1'b1, 4'b0});   // fs_sw = bit 4
    next_packet(pkt); if (pkt[FS_CYC] != RHD_WR0_FS_ON) next_packet(pkt);   // allow one packet to catch the edge
    chk("fs ON inject", pkt[FS_CYC], RHD_WR0_FS_ON);
    next_packet(pkt);                            // steady: WRITE(0,0) with D5 forced
    chk("fs steady D5", pkt[FS_CYC], 16'h8000 | (16'h1 << RHD_FS_BIT));
    set_ctrl(22, {8'h1C, 24'b0});                // OFF edge
    next_packet(pkt); if (pkt[FS_CYC] != RHD_WR0_FS_OFF) next_packet(pkt);
    chk("fs OFF inject", pkt[FS_CYC], RHD_WR0_FS_OFF);

    // ---- one-shot injection whole-replaces the inject slot for one packet ----
    aux_inject(16'hFE00);                        // READ(62)
    next_packet(pkt); if (pkt[INJECT_CYC] != 16'hFE00) next_packet(pkt);
    chk("inject on wire", pkt[INJECT_CYC], 16'hFE00);
    next_packet(pkt);                            // reverts to slot 2's register default (CONVERT 49, temperature)
    chk("inject revert", pkt[INJECT_CYC], convert_word(49));

    // ---- DSP reset ("digital fast settle"): reg22 dsp_sw (bit 9) forces bit H
    //      (the CONVERT LSB) high on EVERY channel CONVERT while held. This is the
    //      override's coherent-bit substitution -- silent on the wire if it breaks. ----
    set_ctrl(22, {8'h1C, 14'b0, 1'b1, 9'b0});    // dsp_sw = reg22 bit 9
    next_packet(pkt); next_packet(pkt);          // let the once-per-packet latch settle
    for (int i = 0; i < N_CHAN_CMDS; i++)
        chk($sformatf("dsp-on H[%0d]", i), pkt[i], convert_word(i) | 16'h0001);
    set_ctrl(22, {8'h1C, 24'b0});                // release
    next_packet(pkt); next_packet(pkt);
    for (int i = 0; i < N_CHAN_CMDS; i++)
        chk($sformatf("dsp-off H[%0d]", i), pkt[i], convert_word(i));

    // ---- Reg-3 digout: put a WRITE(3) in the fs slot, then reg22 digout_sw (bit 14)
    //      substitutes the live D0 (reg3_shadow = {reg3_static[7:1], digout}) into its
    //      low byte on the wire. reg3_static = 0x1C, so D0=1 -> 0x1D, D0=0 -> 0x1C. ----
    aux_write(AUX_FS_SLOT, 0, 0, 0, {RHD_CMD_WRITE, RHD_REG_DIGOUT, 8'h00});  // slot 1 = WRITE(3,0)
    set_ctrl(22, {8'h1C, 9'b0, 1'b1, 14'b0});    // digout_sw = reg22 bit 14
    next_packet(pkt); next_packet(pkt);
    chk("digout ON D0",  pkt[FS_CYC], {RHD_CMD_WRITE, RHD_REG_DIGOUT, 8'h1D});
    set_ctrl(22, {8'h1C, 24'b0});                // digout off
    next_packet(pkt); next_packet(pkt);
    chk("digout OFF D0", pkt[FS_CYC], {RHD_CMD_WRITE, RHD_REG_DIGOUT, 8'h1C});

    set_ctrl(0, 32'h0);
    repeat (50) @(negedge clk);
    $display("Checks: %0d, Errors: %0d", n_checks, n_errors);
    if (n_errors == 0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
    $finish;
end

initial begin #20_000_000; $display("ERROR: watchdog"); $display("RESULT: FAIL"); $finish; end

endmodule
