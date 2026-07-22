// aux_command_engine_tb.sv
//
// Spec-based self-checking testbench for aux_command_engine. Drives the engine
// through its three raw control registers (reg 22/23/24) the same way the host
// does -- so this also exercises the engine's own register decode + toggle->pulse
// edge detection -- and checks the final aux_cmds + override outputs.
//
// Verifies the invariants of the engine (one program + two registers):
//   * slot 0 is the one cycling program (aux_program -- the accel sweep): steps one
//     entry per packet, loops end->loop, atomic bank swap at the boundary.
//   * slot 1 is a fixed fs register (write target 1) -- does NOT cycle; the override
//     fast-settle whole-replaces it and forces WRITE(0) D5.
//   * slot 2 is a fixed register (write target 2); a one-shot injection whole-
//     replaces it for one packet, then it reverts.
//
// Run: bash programmable_logic/sim/run_aux_engine_tb.sh  ("RESULT: PASS")

`timescale 1ns/1ps

import acq_frame_pkg::*;

module aux_command_engine_tb;

logic clk = 0; always #5 clk = ~clk;
logic rstn = 0;

int n_checks = 0, n_errors = 0;
task automatic chk(input string what, input logic [15:0] got, input logic [15:0] exp);
    n_checks++;
    if (got !== exp) begin n_errors++; $display("ERROR: %s got=%04h exp=%04h", what, got, exp); end
endtask

// DUT I/O -- the engine now owns its register decode, so we drive the raw words.
logic seq_advance = 0, packet_start = 0, transmission_active = 0;
logic [7:0]  digital_in = 0;
logic [31:0] ctrl_reg   = 32'h1C00_0000;   // reg 22: reg3_static = 0x1C in [31:24]
logic [31:0] write_reg  = 0;               // reg 23: write port payload
logic [31:0] strobe_reg = 0;               // reg 24: toggles + inject command
logic [N_AUX*16-1:0] aux_cmds;
logic dsp_force_h, fast_settle_active, digout_state, inject_active;
logic [N_AUX-1:0] bank_active;
logic [N_AUX*6-1:0] slot_indices;

aux_command_engine #(.ADDR_W(6)) dut (
    .clk(clk), .rstn(rstn),
    .seq_advance(seq_advance), .packet_start(packet_start),
    .transmission_active(transmission_active),
    .digital_in(digital_in),
    .aux_ctrl_reg(ctrl_reg), .aux_write_reg(write_reg), .aux_strobe_reg(strobe_reg),
    .aux_cmds(aux_cmds), .dsp_force_h(dsp_force_h),
    .fast_settle_active(fast_settle_active), .digout_state(digout_state),
    .inject_active(inject_active), .bank_active(bank_active), .slot_indices(slot_indices)
);

function automatic logic [15:0] slot(input int s); return aux_cmds[s*16 +: 16]; endfunction
function automatic logic [15:0] convert_default(input int s); return {2'b00, 6'(AUX_CYC0 + s), 8'h00}; endfunction
// Register power-on defaults (mirror aux_command_engine's FS_DEFAULT / INJECT_DEFAULT).
localparam logic [15:0] FS_DEFAULT_EXP     = 16'hE800;  // READ(40) = 'I' of INTAN ROM
localparam logic [15:0] INJECT_DEFAULT_EXP = 16'h3100;  // CONVERT(49) = temperature channel

// Host-style stimulus: payloads are set first, then a toggle flip strobes them.
bit wr_tog = 0, inj_tog = 0;
logic [15:0] inj_cmd_hold = 0;
task automatic push_strobe;  strobe_reg = {inj_cmd_hold, 14'b0, inj_tog, wr_tog}; endtask

// One write via the unified port. target IS the slot index (0=program, 1/2=registers).
task automatic aw(input int target, input bit bank, input bit is_len, input logic [5:0] addr, input logic [15:0] data);
    write_reg = {6'b0, is_len, bank, 2'(target), addr, data};
    @(negedge clk);
    wr_tog = ~wr_tog; push_strobe();        // engine turns the edge into a 1-cycle wr_en
    repeat (4) @(negedge clk);
endtask
// Arm a one-shot injection (slot 2). Consumed at the next seq_advance.
task automatic arm_inject(input logic [15:0] cmd);
    inj_cmd_hold = cmd; inj_tog = ~inj_tog; push_strobe();
    repeat (2) @(negedge clk);              // let inject_pending latch before end_packet
endtask

// Latch override state for the current packet (aux_cmds becomes valid ~now)
task automatic start_packet;  @(negedge clk); packet_start = 1; @(negedge clk); packet_start = 0; repeat (2) @(negedge clk); endtask
// Advance the programs to the next entry
task automatic end_packet;    @(negedge clk); seq_advance = 1; @(negedge clk); seq_advance = 0; repeat (2) @(negedge clk); endtask

initial begin
    repeat (4) @(negedge clk); rstn = 1; repeat (4) @(negedge clk);

    // ---- A. power-on state: slot 0 boots the sweep; slots 1/2 read their register defaults ----
    transmission_active = 0; start_packet();
    chk("poweron slot0", slot(AUX_SWEEP_SLOT),  convert_default(0));   // sweep entry 0 = CONVERT(32)
    chk("poweron slot1", slot(AUX_FS_SLOT),     FS_DEFAULT_EXP);       // 'I' = READ(40)
    chk("poweron slot2", slot(AUX_INJECT_SLOT), INJECT_DEFAULT_EXP);   // temp = CONVERT(49)

    // ---- A2. the slot-0 boot sweep cycles 32 -> 33 -> 34 -> 32, no host upload ----
    transmission_active = 1; @(negedge clk);
    start_packet(); chk("boot sweep p0",   slot(AUX_SWEEP_SLOT), convert_default(0)); end_packet();  // 32
    start_packet(); chk("boot sweep p1",   slot(AUX_SWEEP_SLOT), convert_default(1)); end_packet();  // 33
    start_packet(); chk("boot sweep p2",   slot(AUX_SWEEP_SLOT), convert_default(2)); end_packet();  // 34
    start_packet(); chk("boot sweep wrap", slot(AUX_SWEEP_SLOT), convert_default(0)); end_packet();  // 32
    transmission_active = 0; repeat (2) @(negedge clk);

    // ---- program while idle ----
    // slot 0 (program): a 3-entry looping program.
    aw(AUX_SWEEP_SLOT, 0, 0, 0, 16'h2000); aw(AUX_SWEEP_SLOT, 0, 0, 1, 16'h2100); aw(AUX_SWEEP_SLOT, 0, 0, 2, 16'h2200);
    aw(AUX_SWEEP_SLOT, 0, 1, 0, {2'b00, 6'd2, 2'b00, 6'd0});          // loop 0..2
    // slot 2 (inject): a single command register -- injection whole-replaces it
    // for one packet. No length record; it does NOT cycle.
    aw(AUX_INJECT_SLOT, 0, 0, 0, 16'hAB00);                          // -> inject_base_r
    // slot 1 (fs register): a single WRITE(0,x) so the D5 force + FS whole-replace
    // can act on it. No length record -- it is a register, not a program.
    aw(AUX_FS_SLOT, 0, 0, 0, 16'h8000);                              // WRITE(0, 0x00) -> fs_cmd
    // standby bank for the swap test (slot 0, bank 1)
    aw(AUX_SWEEP_SLOT, 1, 0, 0, 16'h3000); aw(AUX_SWEEP_SLOT, 1, 0, 1, 16'h3100);
    aw(AUX_SWEEP_SLOT, 1, 1, 0, {2'b00, 6'd1, 2'b00, 6'd0});          // loop 0..1

    // ---- B. run: only slot 0 cycles; slots 1 and 2 are fixed registers ----
    transmission_active = 1; @(negedge clk);
    start_packet();  // packet 0 (index 0)
    chk("p0 prog[0]",    slot(AUX_SWEEP_SLOT),  16'h2000);
    chk("p0 inject fix", slot(AUX_INJECT_SLOT), 16'hAB00);  // slot-2 register holds
    chk("p0 fs fixed",   slot(AUX_FS_SLOT),     16'h8000);  // fs register holds
    end_packet();
    start_packet();  // packet 1
    chk("p1 prog[1]",    slot(AUX_SWEEP_SLOT),  16'h2100);
    chk("p1 inject fix", slot(AUX_INJECT_SLOT), 16'hAB00);  // slot 2 does NOT cycle
    end_packet();
    start_packet();  // packet 2
    chk("p2 prog[2]",    slot(AUX_SWEEP_SLOT),  16'h2200);
    chk("p2 fs fixed",   slot(AUX_FS_SLOT),     16'h8000);  // fs still does NOT cycle
    end_packet();
    start_packet();  // packet 3: program wraps to 0
    chk("p3 prog wrap",  slot(AUX_SWEEP_SLOT),  16'h2000);
    end_packet();

    // ---- C. WRITE(0) D5 force + fast-settle whole-replace on the fs slot ----
    ctrl_reg[4] = 1'b1;    // fs_sw ON edge on the next packet_start
    start_packet();
    chk("fs ON inject", slot(AUX_FS_SLOT), RHD_WR0_FS_ON);
    n_checks++; if (fast_settle_active !== 1'b1) begin n_errors++; $display("ERROR: fast_settle_active not set"); end
    end_packet();
    start_packet();        // steady ON: fs slot's WRITE(0) gets D5 forced, no whole-replace
    chk("fs steady D5", slot(AUX_FS_SLOT), 16'h8000 | (16'h1 << RHD_FS_BIT));
    end_packet();
    ctrl_reg[4] = 1'b0;    // fs_sw OFF edge
    start_packet();
    chk("fs OFF inject", slot(AUX_FS_SLOT), RHD_WR0_FS_OFF);
    end_packet();

    // ---- D. DSP reset follows dsp_sw (packet-latched) ----
    ctrl_reg[9] = 1'b1; start_packet();
    n_checks++; if (dsp_force_h !== 1'b1) begin n_errors++; $display("ERROR: dsp_force_h not set"); end
    end_packet();
    ctrl_reg[9] = 1'b0; start_packet();
    n_checks++; if (dsp_force_h !== 1'b0) begin n_errors++; $display("ERROR: dsp_force_h stuck"); end
    end_packet();

    // ---- E. one-shot injection on slot 2: replaces one packet, then reverts ----
    arm_inject(16'hFE00);                         // READ(62); inject_pending latches
    end_packet();                                 // seq_advance consumes -> inject_active for the next packet
    start_packet();                               // the injected packet
    chk("inject on wire", slot(AUX_INJECT_SLOT), 16'hFE00);
    n_checks++; if (inject_active !== 1'b1) begin n_errors++; $display("ERROR: inject_active not set"); end
    end_packet();
    start_packet();                               // revert: slot 2 back to its register value
    chk("inject revert", slot(AUX_INJECT_SLOT), 16'hAB00);
    end_packet();

    // ---- F. bank swap lands at a boundary; bank_active confirms (slot 0) ----
    ctrl_reg[0] = 1'b1;                           // prog_bank_select -> slot 0 bank 1
    end_packet();                                 // request; swap applies at boundary
    start_packet();
    n_checks++; if (bank_active[AUX_SWEEP_SLOT] !== 1'b1) begin n_errors++; $display("ERROR: bank_active did not confirm swap"); end
    chk("swap prog[0]", slot(AUX_SWEEP_SLOT), 16'h3000);
    end_packet();

    $display("Checks: %0d, Errors: %0d", n_checks, n_errors);
    if (n_errors == 0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
    $finish;
end

initial begin #2_000_000; $display("ERROR: watchdog"); $display("RESULT: FAIL"); $finish; end

endmodule
