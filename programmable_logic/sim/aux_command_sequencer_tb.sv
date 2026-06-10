// ============================================================================
// aux_command_sequencer_tb.sv
// ----------------------------------------------------------------------------
// Self-checking testbench for aux_command_sequencer. Drives the host write
// port and the packet_start advance pulse, and asserts:
//   A. index loops loop_idx -> end_idx -> loop_idx (steady-state wrap target).
//   B. multiple slots advance independently with per-slot loop/length.
//   C. a bank swap takes effect ONLY at packet_start, never when bank_select
//      changes asynchronously between packets.
//   D. length is bound to the bank: two banks with different lengths, and after
//      a swap the wrap uses the ACTIVE bank's length (the swapped-in one).
//   E. writing a standby bank does not disturb the active output or index.
//   F. aux_seq_cmd always reflects mem[{active_bank, index}] (data-path check,
//      driven off the DUT's own index_out/bank_active against a shadow model).
//
// Reference model: a shadow memory + per-bank length regs updated in lockstep
// with every host write, so checks are independent of the DUT internals.
// Run:  xvlog -sv aux_command_sequencer.sv aux_command_sequencer_tb.sv
//       xelab work.aux_command_sequencer_tb -s tb -R
// ============================================================================
`timescale 1ns/1ps

module aux_command_sequencer_tb;

    // DUT parameters (match defaults)
    localparam integer N_SLOTS = 3;
    localparam integer ADDR_W  = 6;
    localparam integer NBANKS  = 2;
    localparam integer BANK_W  = 1;
    localparam integer ENTRIES = (1 << ADDR_W);

    // Clock / reset
    logic clk = 0;
    logic rstn;
    always #5 clk = ~clk;   // 100 MHz

    // DUT boundary
    logic                      packet_start;
    logic [N_SLOTS*16-1:0]     aux_seq_cmd;
    logic [N_SLOTS*ADDR_W-1:0] index_out;
    logic                      wr_en;
    logic [1:0]                wr_slot;
    logic [BANK_W-1:0]         wr_bank;
    logic [ADDR_W-1:0]         wr_addr;
    logic [15:0]               wr_data;
    logic [N_SLOTS*BANK_W-1:0] bank_select;
    logic [N_SLOTS*BANK_W-1:0] bank_active;

    aux_command_sequencer #(
        .N_SLOTS(N_SLOTS), .ADDR_W(ADDR_W), .NBANKS(NBANKS)
    ) dut (
        .clk(clk), .rstn(rstn),
        .packet_start(packet_start),
        .aux_seq_cmd(aux_seq_cmd),
        .index_out(index_out),
        .wr_en(wr_en), .wr_slot(wr_slot), .wr_bank(wr_bank),
        .wr_addr(wr_addr), .wr_data(wr_data),
        .bank_select(bank_select), .bank_active(bank_active)
    );

    // ---- reference / shadow model ----
    logic [15:0]      model_mem  [N_SLOTS][NBANKS][ENTRIES];
    logic [ADDR_W-1:0] model_loop [N_SLOTS][NBANKS];
    logic [ADDR_W-1:0] model_end  [N_SLOTS][NBANKS];

    integer errors = 0;
    integer checks = 0;

    // ---- per-slot accessors (the boundary is flat-packed) ----
    function automatic [15:0] cmd_of(input int slot);
        return aux_seq_cmd[slot*16 +: 16];
    endfunction
    function automatic [ADDR_W-1:0] idx_of(input int slot);
        return index_out[slot*ADDR_W +: ADDR_W];
    endfunction
    function automatic [BANK_W-1:0] active_of(input int slot);
        return bank_active[slot*BANK_W +: BANK_W];
    endfunction

    // ---- checkers ----
    task automatic check(input logic [31:0] got, input logic [31:0] exp,
                         input string msg);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %s : got=0x%0h exp=0x%0h  (t=%0t)", msg, got, exp, $time);
        end else begin
            $display("  [ ok ] %s : 0x%0h", msg, got);
        end
    endtask

    // F: aux_seq_cmd must equal model_mem[slot][active_bank][index], using the
    //    DUT's own reported active_bank/index. Pure data-path assertion.
    task automatic check_datapath(input int slot);
        logic [BANK_W-1:0] b; logic [ADDR_W-1:0] a; logic [15:0] exp;
        b = active_of(slot); a = idx_of(slot);
        exp = model_mem[slot][b][a];
        check(cmd_of(slot), exp,
              $sformatf("slot%0d cmd==mem[bank%0d][idx%0d]", slot, b, a));
    endtask

    // ---- stimulus helpers ----
    task automatic do_reset();
        rstn = 0; packet_start = 0; wr_en = 0; wr_slot = 0; wr_bank = 0;
        wr_addr = 0; wr_data = 0; bank_select = 0;
        repeat (3) @(negedge clk);
        rstn = 1;
        @(negedge clk);
    endtask

    task automatic pulse_packet_start();
        @(negedge clk); packet_start = 1;
        @(negedge clk); packet_start = 0;
    endtask

    task automatic write_word(input int slot, input int bank,
                              input [ADDR_W-1:0] addr, input [15:0] data);
        @(negedge clk);
        wr_en = 1; wr_slot = slot[1:0]; wr_bank = bank[BANK_W-1:0];
        wr_addr = addr; wr_data = data;
        @(negedge clk);
        wr_en = 0;
        // mirror into the shadow model
        if (addr == 0) begin
            model_loop[slot][bank] = data[ADDR_W-1:0];
            model_end [slot][bank] = data[2*ADDR_W-1:ADDR_W];
        end else begin
            model_mem[slot][bank][addr] = data;
        end
    endtask

    // length record packs {end_idx, loop_idx} into the low 2*ADDR_W bits
    task automatic write_length(input int slot, input int bank,
                                input [ADDR_W-1:0] loop_idx,
                                input [ADDR_W-1:0] end_idx);
        write_word(slot, bank, 0, {{(16-2*ADDR_W){1'b0}}, end_idx, loop_idx});
    endtask

    integer k;
    logic [15:0] cmd_hold;
    logic [ADDR_W-1:0] idx_hold;

    initial begin
        // init model to a known value so unwritten cells never read X in checks
        for (int s=0; s<N_SLOTS; s++)
          for (int b=0; b<NBANKS; b++) begin
            model_loop[s][b]=0; model_end[s][b]=0;
            for (int e=0; e<ENTRIES; e++) model_mem[s][b][e]=16'h0;
          end

        do_reset();

        // --------------------------------------------------------------------
        // Load programs.
        //  slot0 bank0: loop=1 end=3  (len 3)  A001 A002 A003
        //  slot0 bank1: loop=1 end=5  (len 5)  B001..B005   (different length)
        //  slot1 bank0: loop=2 end=4  (len 3, loop point != 1) C101..C104
        //  slot2 bank0: loop=1 end=1  (len 1, degenerate)     D201
        // --------------------------------------------------------------------
        write_length(0,0, 1,3);
        write_word(0,0,1,16'hA001); write_word(0,0,2,16'hA002); write_word(0,0,3,16'hA003);
        write_length(0,1, 1,5);
        write_word(0,1,1,16'hB001); write_word(0,1,2,16'hB002); write_word(0,1,3,16'hB003);
        write_word(0,1,4,16'hB004); write_word(0,1,5,16'hB005);

        write_length(1,0, 2,4);
        write_word(1,0,1,16'hC101); write_word(1,0,2,16'hC102);
        write_word(1,0,3,16'hC103); write_word(1,0,4,16'hC104);

        write_length(2,0, 1,1);
        write_word(2,0,1,16'hD201);

        // re-reset so all indices start clean at 1 with banks loaded
        do_reset();

        // ====================================================================
        $display("\n=== Scenario A: slot0 bank0 loop 1->3 (len 3) ===");
        // after reset: index=1
        check(idx_of(0), 1, "A: idx after reset");
        check(cmd_of(0), 16'hA001, "A: cmd after reset");
        check_datapath(0);
        pulse_packet_start(); check(idx_of(0),2,"A: idx"); check(cmd_of(0),16'hA002,"A: cmd"); check_datapath(0);
        pulse_packet_start(); check(idx_of(0),3,"A: idx"); check(cmd_of(0),16'hA003,"A: cmd"); check_datapath(0);
        pulse_packet_start(); check(idx_of(0),1,"A: idx WRAP->loop"); check(cmd_of(0),16'hA001,"A: cmd wrap"); check_datapath(0);
        pulse_packet_start(); check(idx_of(0),2,"A: idx"); check(cmd_of(0),16'hA002,"A: cmd"); check_datapath(0);

        // ====================================================================
        $display("\n=== Scenario B: 3 slots advance independently ===");
        do_reset();
        // slot1 loop=2,end=4: reset forces idx=1 (pre-roll), then 2,3,4, wraps to loop=2.
        // slot2 loop=1,end=1: always idx=1 -> D201 every packet.
        // slot0 bank0 loop=1,end=3 as before.
        check(idx_of(1),1,"B: slot1 idx reset(preroll)");
        check(idx_of(2),1,"B: slot2 idx reset");
        check(cmd_of(2),16'hD201,"B: slot2 cmd");
        pulse_packet_start();
        check(idx_of(0),2,"B: slot0 idx"); check(idx_of(1),2,"B: slot1 idx"); check(idx_of(2),1,"B: slot2 idx(len1)");
        check(cmd_of(2),16'hD201,"B: slot2 cmd const");
        pulse_packet_start();
        check(idx_of(0),3,"B: slot0 idx"); check(idx_of(1),3,"B: slot1 idx"); check(idx_of(2),1,"B: slot2 idx(len1)");
        pulse_packet_start();
        check(idx_of(0),1,"B: slot0 WRAP(len3)"); check(idx_of(1),4,"B: slot1 idx"); check(idx_of(2),1,"B: slot2 idx");
        pulse_packet_start();
        // slot1 now wraps: 4==end -> loop=2 (NOT 1) ; slot0 -> 2
        check(idx_of(0),2,"B: slot0 idx"); check(idx_of(1),2,"B: slot1 WRAP->loop=2"); check(idx_of(2),1,"B: slot2 idx");
        check(cmd_of(1),16'hC102,"B: slot1 cmd@loop");
        check_datapath(0); check_datapath(1); check_datapath(2);

        // ====================================================================
        $display("\n=== Scenario C: bank swap is atomic at packet_start only ===");
        do_reset();
        // advance slot0 to a mid-sequence index on bank0
        pulse_packet_start(); // idx=2
        check(idx_of(0),2,"C: pre-swap idx"); check(active_of(0),0,"C: pre-swap active=bank0");
        cmd_hold = cmd_of(0); idx_hold = idx_of(0);
        // request bank 1 asynchronously, WITHOUT a packet_start
        @(negedge clk); bank_select[0] = 1'b1;
        repeat (6) @(negedge clk);   // let several clocks pass
        check(active_of(0),0,"C: active still bank0 (no packet_start)");
        check(idx_of(0),idx_hold,"C: idx unchanged async");
        check(cmd_of(0),cmd_hold,"C: cmd unchanged async (still bank0 data)");
        // now the boundary: swap should land, index jumps to bank1 loop point
        pulse_packet_start();
        check(active_of(0),1,"C: active==bank1 after packet_start");
        check(idx_of(0),1,"C: idx==bank1 loop(1) after swap");
        check(cmd_of(0),16'hB001,"C: cmd==bank1 data after swap");
        check_datapath(0);

        // ====================================================================
        $display("\n=== Scenario D: length bound to bank (bank1 len 5) ===");
        // continue on bank1 (loop=1,end=5); must wrap at 5, not bank0's 3
        pulse_packet_start(); check(idx_of(0),2,"D: idx"); check(cmd_of(0),16'hB002,"D: cmd");
        pulse_packet_start(); check(idx_of(0),3,"D: idx"); check(cmd_of(0),16'hB003,"D: cmd (past bank0 end, no wrap)");
        pulse_packet_start(); check(idx_of(0),4,"D: idx"); check(cmd_of(0),16'hB004,"D: cmd");
        pulse_packet_start(); check(idx_of(0),5,"D: idx"); check(cmd_of(0),16'hB005,"D: cmd");
        pulse_packet_start(); check(idx_of(0),1,"D: WRAP at bank1 len 5"); check(cmd_of(0),16'hB001,"D: cmd wrap");
        check_datapath(0);

        // ====================================================================
        $display("\n=== Scenario E: writing standby bank does not disturb active ===");
        do_reset();
        // active = bank0; advance a little
        pulse_packet_start(); // idx=2 on bank0
        cmd_hold = cmd_of(0); idx_hold = idx_of(0);
        check(active_of(0),0,"E: active bank0");
        // hammer the STANDBY bank1 (incl. its length record) -- no packet_start
        write_length(0,1, 1,2);                 // change bank1 length
        write_word(0,1,1,16'h7771);
        write_word(0,1,2,16'h7772);
        write_word(0,1,3,16'h7773);
        // active output + index must be untouched
        check(active_of(0),0,"E: active still bank0");
        check(idx_of(0),idx_hold,"E: idx unchanged by standby write");
        check(cmd_of(0),cmd_hold,"E: cmd unchanged by standby write");
        check_datapath(0);
        // and when we DO swap, we see the freshly written standby contents+length
        @(negedge clk); bank_select[0] = 1'b1;
        pulse_packet_start();
        check(active_of(0),1,"E: swapped to bank1");
        check(idx_of(0),1,"E: idx==bank1 loop");
        check(cmd_of(0),16'h7771,"E: cmd==newly written bank1");
        pulse_packet_start(); check(idx_of(0),2,"E: idx"); check(cmd_of(0),16'h7772,"E: cmd");
        pulse_packet_start(); check(idx_of(0),1,"E: WRAP at new bank1 len 2"); check(cmd_of(0),16'h7771,"E: cmd wrap");
        check_datapath(0);

        // ====================================================================
        $display("\n========================================================");
        $display(" CHECKS RUN: %0d   ERRORS: %0d", checks, errors);
        if (errors == 0) $display(" RESULT: PASS");
        else             $display(" RESULT: FAIL");
        $display("========================================================");
        $finish;
    end

    // safety timeout
    initial begin
        #200000;
        $display(" RESULT: FAIL (timeout)");
        $finish;
    end

endmodule
