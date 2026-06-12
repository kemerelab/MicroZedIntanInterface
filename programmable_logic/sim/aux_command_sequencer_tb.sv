// aux_command_sequencer_tb.sv
//
// Self-checking xsim testbench for aux_command_sequencer. Maintains an
// independent shadow model (banks, lengths, indices, swap/injection rules)
// and compares the DUT's outputs against it after every packet boundary.
//
// Run: bash programmable_logic/sim/run_aux_seq_tb.sh   (greps "RESULT: PASS")
//
// Checks:
//   A. power-on default: every slot emits CONVERT(32+s) before programming
//   B. index sequencing: 0 .. end_idx, wrap to loop_idx (incl. loop_idx != 0
//      run-once preamble and a length-1 bank)
//   C. the three slots advance fully independently
//   D. bank swaps land ONLY at a packet boundary; bank_active confirms
//   E. length is bound to the bank (banks of different lengths swap atomically)
//   F. writing the standby bank never disturbs the active program
//   G. one-shot injection replaces slot 3 for exactly one packet, freezes the
//      slot-3 index, and the program resumes unperturbed
//   H. seq_hold parks indices at 0 and applies bank swaps immediately
//   I. data-path: aux_cmds always equals shadow mem[{active_bank, index}]

`timescale 1ns/1ps

module aux_command_sequencer_tb;

localparam int ADDR_W  = 6;
localparam int NSLOTS  = 3;
localparam int ENTRIES = 1 << ADDR_W;

logic clk = 0, rstn = 0;
logic seq_advance = 0, seq_hold = 1;
logic [NSLOTS-1:0] bank_select = '0;
logic wr_en = 0, wr_bank = 0, wr_is_length = 0;
logic [1:0] wr_slot = 0;
logic [ADDR_W-1:0] wr_addr = 0;
logic [15:0] wr_data = 0;
logic inject_req = 0;
logic [15:0] inject_cmd = 0;
logic inject_active;
logic [NSLOTS*16-1:0] aux_cmds;
logic [NSLOTS-1:0] bank_active;
logic [NSLOTS*ADDR_W-1:0] slot_indices;

aux_command_sequencer #(.ADDR_W(ADDR_W), .NSLOTS(NSLOTS)) dut (.*);

always #5 clk = ~clk;

// ---------------------------------------------------------------------------
// Shadow model
// ---------------------------------------------------------------------------
logic [15:0] sh_mem [0:NSLOTS-1][0:1][0:ENTRIES-1];
int sh_loop [0:NSLOTS-1][0:1];
int sh_end  [0:NSLOTS-1][0:1];
int sh_idx  [0:NSLOTS-1];
bit sh_bank [0:NSLOTS-1];
bit sh_inj_pending, sh_inj_active;

int n_checks = 0, n_errors = 0;

task automatic check(input string what, input logic [15:0] got, input logic [15:0] exp);
    n_checks++;
    if (got !== exp) begin
        n_errors++;
        $display("ERROR @%0t: %s  got=0x%04h exp=0x%04h", $time, what, got, exp);
    end
endtask

task automatic check_bit(input string what, input logic got, input logic exp);
    n_checks++;
    if (got !== exp) begin
        n_errors++;
        $display("ERROR @%0t: %s  got=%b exp=%b", $time, what, got, exp);
    end
endtask

// Compare all DUT outputs against the shadow model (call >=2 clocks after any
// boundary so the registered read has settled).
task automatic check_outputs(input string tag);
    for (int s = 0; s < NSLOTS; s++) begin
        logic [15:0] exp;
        exp = ((s == NSLOTS-1) && sh_inj_active) ? inject_cmd
                                                 : sh_mem[s][sh_bank[s]][sh_idx[s]];
        check($sformatf("%s slot%0d cmd", tag, s), aux_cmds[s*16 +: 16], exp);
        check($sformatf("%s slot%0d idx", tag, s),
              16'(slot_indices[s*ADDR_W +: ADDR_W]), 16'(sh_idx[s]));
        check_bit($sformatf("%s slot%0d bank_active", tag, s), bank_active[s], sh_bank[s]);
    end
    check_bit($sformatf("%s inject_active", tag), inject_active, sh_inj_active);
endtask

// Advance the shadow model exactly as the DUT spec says a packet boundary does.
task automatic shadow_advance;
    for (int s = 0; s < NSLOTS; s++) begin
        if (bank_select[s] != sh_bank[s]) begin
            sh_bank[s] = bank_select[s];
            sh_idx[s]  = 0;
        end else if ((s == NSLOTS-1) && sh_inj_active) begin
            // frozen
        end else begin
            sh_idx[s] = (sh_idx[s] == sh_end[s][sh_bank[s]]) ? sh_loop[s][sh_bank[s]]
                                                             : sh_idx[s] + 1;
        end
    end
    sh_inj_active  = sh_inj_pending;
    sh_inj_pending = 0;
endtask

// Pulse one packet boundary and settle
task automatic packet_boundary;
    @(negedge clk); seq_advance = 1;
    @(negedge clk); seq_advance = 0;
    shadow_advance();
    repeat (3) @(negedge clk);
endtask

// Program write helpers (mirror into the shadow)
task automatic write_word(input int s, input bit b, input int a, input logic [15:0] d);
    @(negedge clk);
    wr_en = 1; wr_slot = 2'(s); wr_bank = b; wr_is_length = 0;
    wr_addr = ADDR_W'(a); wr_data = d;
    @(negedge clk); wr_en = 0;
    sh_mem[s][b][a] = d;
endtask

task automatic write_length(input int s, input bit b, input int loop_i, input int end_i);
    @(negedge clk);
    wr_en = 1; wr_slot = 2'(s); wr_bank = b; wr_is_length = 1;
    wr_data = {2'b00, 6'(end_i), 2'b00, 6'(loop_i)};
    @(negedge clk); wr_en = 0;
    sh_loop[s][b] = loop_i;
    sh_end[s][b]  = end_i;
endtask

task automatic arm_inject(input logic [15:0] cmd);
    @(negedge clk);
    inject_cmd = cmd; inject_req = 1;
    @(negedge clk); inject_req = 0;
    sh_inj_pending = 1;
endtask

// ---------------------------------------------------------------------------
// Stimulus
// ---------------------------------------------------------------------------
initial begin
    // Shadow power-on state mirrors the DUT's initial blocks
    for (int s = 0; s < NSLOTS; s++) begin
        for (int b = 0; b < 2; b++) begin
            for (int e = 0; e < ENTRIES; e++)
                sh_mem[s][b][e] = {2'b00, 6'(32 + s), 8'h00};
            sh_loop[s][b] = 0; sh_end[s][b] = 0;
        end
        sh_idx[s] = 0; sh_bank[s] = 0;
    end
    sh_inj_pending = 0; sh_inj_active = 0;

    repeat (4) @(negedge clk);
    rstn = 1;
    repeat (4) @(negedge clk);

    // ---- A: power-on defaults while held --------------------------------
    check_outputs("A.hold-default");
    seq_hold = 0;
    repeat (3) @(negedge clk);
    check_outputs("A.run-default");
    // unprogrammed banks: advancing keeps emitting the default command
    repeat (3) packet_boundary();
    check_outputs("A.default-loop");

    // ---- Program slot 0 bank 0: 5 entries, pure loop (loop=0,end=4) ------
    // ---- Program slot 1 bank 0: 3 entries with run-once preamble ---------
    //      (loop=2, end=2 after preamble 0,1 -> then sit at 2? No:
    //       entries 0,1 preamble, then loop 2..2) loop_idx=2, end_idx=2
    // ---- Slot 2 bank 0: length-1 program (loop=0,end=0) ------------------
    seq_hold = 1; @(negedge clk);   // park so programming starts deterministic
    for (int s = 0; s < NSLOTS; s++) sh_idx[s] = 0;   // hold parks indices
    for (int b = 0; b < NSLOTS; b++) sh_bank[b] = bank_select[b];

    for (int a = 0; a < 5; a++) write_word(0, 0, a, 16'hA000 + 16'(a));
    write_length(0, 0, 0, 4);
    for (int a = 0; a < 3; a++) write_word(1, 0, a, 16'hB000 + 16'(a));
    write_length(1, 0, 2, 2);
    write_word(2, 0, 0, 16'hC000);
    write_length(2, 0, 0, 0);

    seq_hold = 0;
    repeat (3) @(negedge clk);
    check_outputs("B.entry0");

    // ---- B/C: sequencing + independence over 12 packets ------------------
    for (int p = 0; p < 12; p++) begin
        packet_boundary();
        check_outputs($sformatf("B/C.pkt%0d", p));
    end
    // After 12 advances: slot0 = 12 % 5; slot1 preamble 0,1 then loops at 2
    n_checks++;
    if (sh_idx[0] != (12 % 5)) begin n_errors++; $display("ERROR: slot0 idx model self-check"); end
    n_checks++;
    if (sh_idx[1] != 2) begin n_errors++; $display("ERROR: slot1 preamble loop self-check"); end

    // ---- F: write standby bank (bank 1) of slot 0 mid-run ----------------
    for (int a = 0; a < 3; a++) write_word(0, 1, a, 16'hD000 + 16'(a));
    write_length(0, 1, 0, 2);
    repeat (2) @(negedge clk);
    check_outputs("F.standby-write-no-disturb");
    packet_boundary();
    check_outputs("F.still-bank0");

    // ---- D: swap request mid-packet must NOT take effect until boundary --
    bank_select[0] = 1;
    repeat (5) @(negedge clk);
    check_outputs("D.swap-pending-held");      // shadow still bank 0
    packet_boundary();                          // shadow_advance applies swap
    check_outputs("D.swap-applied-at-boundary");
    n_checks++;
    if (sh_bank[0] != 1 || sh_idx[0] != 0) begin
        n_errors++; $display("ERROR: model swap state");
    end

    // ---- E: length bound to bank: new bank loops 0..2 ---------------------
    for (int p = 0; p < 7; p++) begin
        packet_boundary();
        check_outputs($sformatf("E.bank1-loop.pkt%0d", p));
    end

    // swap slot 0 back to bank 0 (length 5) and confirm its length rules
    bank_select[0] = 0;
    packet_boundary();
    check_outputs("E.swap-back");
    for (int p = 0; p < 6; p++) begin
        packet_boundary();
        check_outputs($sformatf("E.bank0-loop.pkt%0d", p));
    end

    // ---- G: one-shot injection on slot 2 (slot index NSLOTS-1) -----------
    arm_inject(16'hE805);                       // e.g. READ(40)-style word
    check_outputs("G.armed-no-effect-yet");     // current packet unaffected
    packet_boundary();                          // injected packet begins
    check_outputs("G.injected-packet");         // slot2 = inject_cmd, idx frozen
    packet_boundary();
    check_outputs("G.resumed");                 // program resumes, no skip
    packet_boundary();
    check_outputs("G.resumed2");

    // ---- H: hold parks indices and applies swaps immediately --------------
    bank_select[0] = 1;
    seq_hold = 1;
    repeat (3) @(negedge clk);
    for (int s = 0; s < NSLOTS; s++) sh_idx[s] = 0;
    sh_bank[0] = 1;
    sh_inj_pending = 0; sh_inj_active = 0;
    check_outputs("H.hold");
    seq_hold = 0;
    repeat (3) @(negedge clk);
    check_outputs("H.release");

    // ---- Summary ----------------------------------------------------------
    $display("Checks: %0d, Errors: %0d", n_checks, n_errors);
    if (n_errors == 0) $display("RESULT: PASS");
    else               $display("RESULT: FAIL");
    $finish;
end

// Global watchdog
initial begin
    #500_000;
    $display("ERROR: watchdog timeout");
    $display("RESULT: FAIL");
    $finish;
end

endmodule
