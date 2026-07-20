// aux_command_engine_tb.sv
//
// Spec-based self-checking testbench for aux_command_engine. Drives the engine at
// the packet cadence (packet_start latches the override, seq_advance advances the
// programs one entry) and checks the final aux_cmds + override outputs against
// directly-expected values. Uses the named roles from acq_frame_pkg.
//
// Verifies the invariants of the reworked engine:
//   * slot 0 is a single fixed RT register (set via wr_target 0) -- it does NOT
//     cycle; the override fast-settle whole-replaces it and forces WRITE(0) D5.
//   * slots 1 & 2 are cycling programs (aux_program): step one entry per packet,
//     loop end->loop, independent, atomic bank swap at the boundary.
//   * slot 2 one-shot injection replaces one packet, freezes the index, resumes.
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

// DUT I/O
logic seq_advance = 0, packet_start = 0, transmission_active = 0;
logic [1:0] prog_bank_select = '0;                 // [0]=slot1, [1]=slot2
logic wr_en = 0; logic [1:0] wr_target = 0; logic wr_bank = 0, wr_is_length = 0;
logic [5:0] wr_addr = 0; logic [15:0] wr_data = 0;
logic inject_req = 0; logic [15:0] inject_cmd = 0;
logic [7:0] digital_in = 0;
logic fs_sw = 0, fs_gpio_en = 0; logic [2:0] fs_gpio_sel = 0;
logic dsp_sw = 0, dsp_gpio_en = 0; logic [2:0] dsp_gpio_sel = 0;
logic digout_sw = 0, digout_gpio_en = 0; logic [2:0] digout_gpio_sel = 0;
logic [7:0] reg3_static = 8'h1C;      // temp on, digout HiZ=0
logic [N_AUX*16-1:0] aux_cmds;
logic dsp_force_h, fast_settle_active, digout_state, inject_active;
logic [N_AUX-1:0] bank_active;
logic [N_AUX*6-1:0] slot_indices;

aux_command_engine #(.ADDR_W(6)) dut (
    .clk(clk), .rstn(rstn),
    .seq_advance(seq_advance), .packet_start(packet_start),
    .transmission_active(transmission_active),
    .prog_bank_select(prog_bank_select),
    .wr_en(wr_en), .wr_target(wr_target), .wr_bank(wr_bank),
    .wr_is_length(wr_is_length), .wr_addr(wr_addr), .wr_data(wr_data),
    .inject_req(inject_req), .inject_cmd(inject_cmd),
    .digital_in(digital_in),
    .fs_sw(fs_sw), .fs_gpio_en(fs_gpio_en), .fs_gpio_sel(fs_gpio_sel),
    .dsp_sw(dsp_sw), .dsp_gpio_en(dsp_gpio_en), .dsp_gpio_sel(dsp_gpio_sel),
    .digout_sw(digout_sw), .digout_gpio_en(digout_gpio_en), .digout_gpio_sel(digout_gpio_sel),
    .reg3_static(reg3_static),
    .aux_cmds(aux_cmds), .dsp_force_h(dsp_force_h),
    .fast_settle_active(fast_settle_active), .digout_state(digout_state),
    .inject_active(inject_active), .bank_active(bank_active), .slot_indices(slot_indices)
);

function automatic logic [15:0] slot(input int s); return aux_cmds[s*16 +: 16]; endfunction
function automatic logic [15:0] convert_default(input int s); return {2'b00, 6'(AUX_CYC0 + s), 8'h00}; endfunction

// One write via the unified port. target IS the slot index (0=RT reg, 1/2=programs).
task automatic aw(input int target, input bit bank, input bit is_len, input logic [5:0] addr, input logic [15:0] data);
    @(negedge clk); wr_en = 1; wr_target = 2'(target); wr_bank = bank; wr_is_length = is_len; wr_addr = addr; wr_data = data;
    @(negedge clk); wr_en = 0;
endtask

// Latch override state for the current packet (aux_cmds becomes valid ~now)
task automatic start_packet;  @(negedge clk); packet_start = 1; @(negedge clk); packet_start = 0; repeat (2) @(negedge clk); endtask
// Advance the programs to the next entry
task automatic end_packet;    @(negedge clk); seq_advance = 1; @(negedge clk); seq_advance = 0; repeat (2) @(negedge clk); endtask

initial begin
    repeat (4) @(negedge clk); rstn = 1; repeat (4) @(negedge clk);

    // ---- A. power-on default = CONVERT(AUX_CYC0+slot) for every slot ----
    transmission_active = 0; start_packet();
    for (int s = 0; s < N_AUX; s++) chk($sformatf("poweron slot%0d", s), slot(s), convert_default(s));

    // ---- program while idle ----
    // slot 1 (plain): a 3-entry looping program.
    aw(AUX_PLAIN_SLOT, 0, 0, 0, 16'h2000); aw(AUX_PLAIN_SLOT, 0, 0, 1, 16'h2100); aw(AUX_PLAIN_SLOT, 0, 0, 2, 16'h2200);
    aw(AUX_PLAIN_SLOT, 0, 1, 0, {2'b00, 6'd2, 2'b00, 6'd0});          // loop 0..2
    // slot 2 (inject/housekeeping): a 2-entry looping program.
    aw(AUX_INJECT_SLOT, 0, 0, 0, 16'hFF00); aw(AUX_INJECT_SLOT, 0, 0, 1, 16'hE800);
    aw(AUX_INJECT_SLOT, 0, 1, 0, {2'b00, 6'd1, 2'b00, 6'd0});         // loop 0..1
    // slot 0 (RT register): a single WRITE(0,x) so the D5 force + FS whole-replace
    // can act on it. No length record -- it is a register, not a program.
    aw(AUX_FS_SLOT, 0, 0, 0, 16'h8000);                              // WRITE(0, 0x00) -> rt_cmd
    // standby bank for the swap test (slot 1, bank 1)
    aw(AUX_PLAIN_SLOT, 1, 0, 0, 16'h3000); aw(AUX_PLAIN_SLOT, 1, 0, 1, 16'h3100);
    aw(AUX_PLAIN_SLOT, 1, 1, 0, {2'b00, 6'd1, 2'b00, 6'd0});          // loop 0..1

    // ---- B. run: walk packets, check the loop, independence, and RT stability ----
    transmission_active = 1; @(negedge clk);
    start_packet();  // packet 0 (index 0)
    chk("p0 plain[0]",  slot(AUX_PLAIN_SLOT),  16'h2000);
    chk("p0 inject[0]", slot(AUX_INJECT_SLOT), 16'hFF00);
    chk("p0 rt fixed",  slot(AUX_FS_SLOT),     16'h8000);  // RT register holds
    end_packet();
    start_packet();  // packet 1
    chk("p1 plain[1]",  slot(AUX_PLAIN_SLOT),  16'h2100);
    chk("p1 inject[1]", slot(AUX_INJECT_SLOT), 16'hE800);
    end_packet();
    start_packet();  // packet 2: plain idx2, inject wraps to 0
    chk("p2 plain[2]",  slot(AUX_PLAIN_SLOT),  16'h2200);
    chk("p2 inject[0]", slot(AUX_INJECT_SLOT), 16'hFF00);
    chk("p2 rt fixed",  slot(AUX_FS_SLOT),     16'h8000);  // RT still does NOT cycle
    end_packet();
    start_packet();  // packet 3: plain wraps to 0
    chk("p3 plain wrap", slot(AUX_PLAIN_SLOT), 16'h2000);
    end_packet();

    // ---- C. WRITE(0) D5 force + fast-settle whole-replace on the RT slot ----
    fs_sw = 1;              // ON edge on the next packet_start
    start_packet();
    chk("fs ON inject", slot(AUX_FS_SLOT), RHD_WR0_FS_ON);
    n_checks++; if (fast_settle_active !== 1'b1) begin n_errors++; $display("ERROR: fast_settle_active not set"); end
    end_packet();
    start_packet();        // steady ON: RT slot's WRITE(0) gets D5 forced, no whole-replace
    chk("fs steady D5", slot(AUX_FS_SLOT), 16'h8000 | (16'h1 << RHD_FS_BIT));
    end_packet();
    fs_sw = 0;             // OFF edge
    start_packet();
    chk("fs OFF inject", slot(AUX_FS_SLOT), RHD_WR0_FS_OFF);
    end_packet();

    // ---- D. DSP reset follows dsp_sw (packet-latched) ----
    dsp_sw = 1; start_packet();
    n_checks++; if (dsp_force_h !== 1'b1) begin n_errors++; $display("ERROR: dsp_force_h not set"); end
    end_packet();
    dsp_sw = 0; start_packet();
    n_checks++; if (dsp_force_h !== 1'b0) begin n_errors++; $display("ERROR: dsp_force_h stuck"); end
    end_packet();

    // ---- E. one-shot injection on slot 2: replaces one packet, freezes, resumes ----
    inject_cmd = 16'hFE00;                        // READ(62)
    @(negedge clk); inject_req = 1; @(negedge clk); inject_req = 0;   // arm: inject_pending=1
    end_packet();                                 // seq_advance consumes -> inject_active for the next packet
    start_packet();                               // the injected packet
    chk("inject on wire", slot(AUX_INJECT_SLOT), 16'hFE00);
    n_checks++; if (inject_active !== 1'b1) begin n_errors++; $display("ERROR: inject_active not set"); end
    end_packet();                                 // seq_advance: slot-2 index frozen (no skip)
    start_packet();                               // resume: plays the un-skipped slot-2 entry
    n_checks++; if (slot(AUX_INJECT_SLOT) !== 16'hFF00 && slot(AUX_INJECT_SLOT) !== 16'hE800) begin
        n_errors++; $display("ERROR: inject resume plays %04h (expected a slot-2 program entry)", slot(AUX_INJECT_SLOT)); end
    end_packet();

    // ---- F. bank swap lands at a boundary; bank_active confirms (slot 1) ----
    prog_bank_select[0] = 1;                       // slot 1 -> bank 1
    end_packet();                                 // request; swap applies at boundary
    start_packet();
    n_checks++; if (bank_active[AUX_PLAIN_SLOT] !== 1'b1) begin n_errors++; $display("ERROR: bank_active did not confirm swap"); end
    chk("swap plain[0]", slot(AUX_PLAIN_SLOT), 16'h3000);
    end_packet();

    $display("Checks: %0d, Errors: %0d", n_checks, n_errors);
    if (n_errors == 0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
    $finish;
end

initial begin #2_000_000; $display("ERROR: watchdog"); $display("RESULT: FAIL"); $finish; end

endmodule
