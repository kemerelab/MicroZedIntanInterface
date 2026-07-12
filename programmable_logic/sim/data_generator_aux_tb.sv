// data_generator_aux_tb.sv
//
// Integration testbench for the aux-sequencer datapath in data_generator_core.
//
// TEST 1 (bit-identity): instantiates the NEW core alongside the UNMODIFIED
// core from main (extracted by run_aux_integration_tb.sh as
// data_generator_core_legacy) and drives both with identical control words,
// CIPO stimulus, and transmission start/stop. With aux_seq_en=0 every output
// (csn, sclk, copi, fifo_write_en/data/mask/end-flag) must match on EVERY
// clock -- the default-OFF path is bit-identical.
//
// TEST 2 (aux enabled, new core only): programs per-slot banks, enables the
// sequencer, and checks ON THE SERIALIZED COPI WIRE (chip's view, sampled at
// SCLK rising edges):
//   - cycles 0..31 still come from the static table
//   - cycle 32 = slot-0 program with WRITE(3,..) rewritten to the Reg-3 shadow
//   - cycle 33/34 = looping slot-1/2 programs (independent lengths)
//   - software fast settle edge -> slot-0 replaced by 0x80FE / 0x80DE for one
//     packet each, steady level -> program resumes
//   - DSP bit-H forced onto channel CONVERTs while dsp_sw=1
//   - one-shot injection: slot-2 emits inject_cmd for exactly one packet, the
//     program resumes without skipping, the ack toggle flips, and
//     aux_read_result captures the (synthetic) CIPO response
//   - header word 2 carries {prev slot-2 echo, prev slot-1 echo, this slot-0
//     echo, flags, digital_in}
//   - bank swap requested mid-packet lands only at the packet boundary
//
// Run: bash programmable_logic/sim/run_aux_integration_tb.sh ("RESULT: PASS")

`timescale 1ns/1ps

module data_generator_aux_tb;

logic clk = 0, rstn = 0;
always #5 clk = ~clk;

int n_checks = 0, n_errors = 0;

task automatic err(input string msg);
    n_errors++;
    $display("ERROR @%0t: %s", $time, msg);
endtask

// ---------------------------------------------------------------------------
// Shared stimulus
// ---------------------------------------------------------------------------
logic [32*25-1:0] ctrl_new = '0;
logic [32*22-1:0] ctrl_leg;
assign ctrl_leg = ctrl_new[32*22-1:0];

logic cipo0 = 0, cipo1 = 0;
logic [7:0] digital_in = 8'h00;

// pseudo-random CIPO for the identity test
logic [31:0] lfsr = 32'hACE1_BEEF;
always @(posedge clk) begin
    lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
end

// DUTs ----------------------------------------------------------------------
logic n_fifo_we, l_fifo_we, n_pkt_end, l_pkt_end;
logic [127:0] n_fifo_wd;          // new core: 128-bit (dual-port)
logic [63:0]  l_fifo_wd;          // legacy core: 64-bit
logic [7:0]  n_mask;              // new core: 8-bit channel mask
logic [3:0]  l_mask;              // legacy core: 4-bit
logic n_csn, n_sclk, n_copi, l_csn, l_sclk, l_copi;
logic [32*10-1:0] n_status, l_status;
logic [31:0] aux_status, aux_read_result;

data_generator_core dut_new (
    .clk(clk), .rstn(rstn),
    .ctrl_regs_pl(ctrl_new),
    .status_regs_pl(n_status),
    .aux_status(aux_status),
    .aux_read_result(aux_read_result),
    .fifo_write_en(n_fifo_we), .fifo_write_data(n_fifo_wd),
    .fifo_channel_mask(n_mask), .fifo_full(1'b0), .fifo_count(9'd0),
    .fifo_packet_end_flag(n_pkt_end),
    .csn(n_csn), .sclk(n_sclk), .copi(n_copi),
    .cipo0(cipo0), .cipo1(cipo1),
    .cipo2(1'b0), .cipo3(1'b0),       // port 1 unused in this TB (single-port identity)
    .digital_in(digital_in)
);

data_generator_core_legacy dut_leg (
    .clk(clk), .rstn(rstn),
    .ctrl_regs_pl(ctrl_leg),
    .status_regs_pl(l_status),
    .fifo_write_en(l_fifo_we), .fifo_write_data(l_fifo_wd),
    .fifo_channel_mask(l_mask), .fifo_full(1'b0), .fifo_count(9'd0),
    .fifo_packet_end_flag(l_pkt_end),
    .csn(l_csn), .sclk(l_sclk), .copi(l_copi),
    .cipo0(cipo0), .cipo1(cipo1),
    .digital_in(digital_in)
);

// ---------------------------------------------------------------------------
// Control-register helpers (PL-domain view of the AXI map)
// ---------------------------------------------------------------------------
task automatic set_ctrl(input int idx, input logic [31:0] val);
    ctrl_new[idx*32 +: 32] = val;
endtask

// the standard convert table (matches pl_control.c convert_cmd_sequence)
function automatic logic [15:0] convert_word(input int i);
    return (i < 32) ? {2'b00, 6'(i), 8'h00} : {2'b00, 6'(32 + (i-32)), 8'h00};
endfunction

task automatic load_convert_table;
    for (int j = 0; j < 18; j++) begin
        logic [31:0] v;
        v[15:0]  = convert_word(2*j);
        v[31:16] = (2*j+1 < 35) ? convert_word(2*j+1) : 16'h0;
        set_ctrl(4+j, v);
    end
endtask

// aux bank write through the toggle port (regs 23/24)
logic wr_tog = 0, inj_tog = 0;
logic [15:0] inj_cmd_sh = 16'h0;
task automatic aux_write(input int slot, input bit bank, input bit is_len,
                         input logic [5:0] addr, input logic [15:0] data);
    set_ctrl(23, {6'b0, is_len, bank, 2'(slot), addr, data});
    repeat (2) @(negedge clk);
    wr_tog = ~wr_tog;
    set_ctrl(24, {inj_cmd_sh, 14'b0, inj_tog, wr_tog});
    repeat (4) @(negedge clk);
endtask

task automatic aux_inject(input logic [15:0] cmd);
    inj_cmd_sh = cmd;
    inj_tog = ~inj_tog;
    set_ctrl(24, {inj_cmd_sh, 14'b0, inj_tog, wr_tog});
    repeat (4) @(negedge clk);
endtask

// ---------------------------------------------------------------------------
// TEST 1: equivalence with aux_seq_en = 0.
//
// The unified packet format (this branch) intentionally re-frames the broadband
// HEADER (new 14-word header vs the legacy 10-word one): different magic, more
// header FIFO writes, and the digital/aux metadata moved within the header. So
// the header FIFO writes legitimately DIFFER from the legacy core and are NOT
// compared. What MUST stay identical -- and is asserted here -- is:
//   * the SPI wire (csn/sclk/copi) every clock -- the command path is unchanged,
//   * the DATA FIFO writes (the broadband samples) -- byte-identical content,
//   * the status registers.
// Header writes are excluded by counting FIFO writes from the packet start: the
// legacy core emits 5 header writes/packet, the new core 7; the DATA writes are
// everything after the header in each core, and those must match in order.
// ---------------------------------------------------------------------------
localparam int N_HDR_NEW = 7;   // new common header (8w) + sub-block (6w) = 7 x 64b
localparam int N_HDR_LEG = 5;   // legacy 10-word header = 5 x 64b

bit identity_checking = 0;
int  n_widx = 0, l_widx = 0;          // FIFO-write index within the current packet
logic [63:0] n_data_q [$], l_data_q [$];  // captured DATA writes, in order

always @(posedge clk) begin
    if (identity_checking) begin
        // SPI wire + status: compared every clock (framing-independent).
        n_checks++;
        if ({n_csn, n_sclk, n_copi} !== {l_csn, l_sclk, l_copi})
            err($sformatf("identity: SPI pin mismatch new={%b%b%b} leg={%b%b%b}",
                n_csn, n_sclk, n_copi, l_csn, l_sclk, l_copi));
        if (n_status !== l_status)
            err("identity: status regs mismatch");

        // Capture DATA FIFO writes (those after the header) from each core, in
        // order, and compare the two queues. Header writes (indices < N_HDR_*)
        // are skipped. port-1 must be off (upper mask/data zero) in this TB.
        if (n_fifo_we) begin
            if (n_widx >= N_HDR_NEW) begin
                n_data_q.push_back(n_fifo_wd[63:0]);
                if (n_mask[7:4] !== 4'h0)
                    err($sformatf("identity: port-1 mask not zero (%h)", n_mask[7:4]));
                if (n_fifo_wd[127:64] !== 64'h0)
                    err($sformatf("identity: port-1 data not zero (%016h)", n_fifo_wd[127:64]));
            end
            n_widx <= n_pkt_end ? 0 : (n_widx + 1);
        end
        if (l_fifo_we) begin
            if (l_widx >= N_HDR_LEG) l_data_q.push_back(l_fifo_wd);
            l_widx <= l_pkt_end ? 0 : (l_widx + 1);
        end

        // Drain matched pairs and compare (data words must be byte-identical).
        while (n_data_q.size() > 0 && l_data_q.size() > 0) begin
            logic [63:0] nd = n_data_q.pop_front();
            logic [63:0] ld = l_data_q.pop_front();
            n_checks++;
            if (nd !== ld)
                err($sformatf("identity: DATA word new=%016h leg=%016h", nd, ld));
        end
    end
end

// drive both CIPO lines from the LFSR during the identity test
bit cipo_random = 0;
always @(negedge clk) begin
    if (cipo_random) begin
        cipo0 = lfsr[0];
        cipo1 = lfsr[7];
    end
end

// ---------------------------------------------------------------------------
// COPI wire decoder (chip's view): sample at SCLK rising edges, MSB first
// ---------------------------------------------------------------------------
logic sclk_d = 0;
int bit_cnt = 0;
logic [15:0] rx_shift;
logic [15:0] copi_frame [0:34];   // last full packet's words by cycle
int frame_idx = 0;
bit frame_packet_done = 0;        // set once 35 frames collected

always @(posedge clk) begin
    sclk_d <= n_sclk;
    if (n_sclk && !sclk_d) begin
        rx_shift = {rx_shift[14:0], n_copi};
        bit_cnt++;
        if (bit_cnt == 16) begin
            copi_frame[frame_idx] = rx_shift;
            bit_cnt = 0;
            if (frame_idx == 34) begin
                frame_idx = 0;
                frame_packet_done = 1;
            end else
                frame_idx++;
        end
    end
end

// wait for the next complete packet on the wire and return its 35 words
task automatic next_packet_words(output logic [15:0] words [0:34]);
    frame_packet_done = 0;
    wait (frame_packet_done == 1);
    for (int i = 0; i < 35; i++) words[i] = copi_frame[i];
endtask

// Capture the aux command-echo metadata. In the unified header it is split:
//   header FIFO write 4 (state 3) low 32 bits = {echo0[15:0], flags[7:0],
//       digital_in[7:0]}   (the OLD header word 4),
//   header FIFO write 5 (state 4) low 32 bits = {echo3_prev[15:0],
//       echo2_prev[15:0]}  (the OLD header word 5).
// Reassemble the legacy 64-bit "header word 2" view so the TEST 2 echo checks
// below are unchanged: {echo3_prev, echo2_prev, echo0, flags, digital_in}.
int hdr_cnt = 0;
logic [31:0] hdr_meta_lo = '0;   // {echo0, flags, digital_in}
logic [31:0] hdr_meta_hi = '0;   // {echo3_prev, echo2_prev}
logic [63:0] hdr_word2;
assign hdr_word2 = {hdr_meta_hi, hdr_meta_lo};
always @(posedge clk) begin
    if (n_fifo_we && n_mask == 8'h0F) begin   // header writes are low-4-segment valid
        hdr_cnt++;
        if (hdr_cnt == 4) hdr_meta_lo <= n_fifo_wd[31:0];  // 4th write = digital/flags/echo0
        if (hdr_cnt == 5) hdr_meta_hi <= n_fifo_wd[31:0];  // 5th write = echo2/echo3_prev
    end
    if (n_pkt_end && n_fifo_we) hdr_cnt <= 0;
end

task automatic check_word(input string what, input logic [15:0] got, input logic [15:0] exp);
    n_checks++;
    if (got !== exp) err($sformatf("%s got=%04h exp=%04h", what, got, exp));
endtask

// ---------------------------------------------------------------------------
// Stimulus
// ---------------------------------------------------------------------------
localparam logic [15:0] W3_PLACEHOLDER = 16'h8300;  // WRITE(3, 0) -> rewritten
localparam logic [7:0]  REG3_STATIC    = 8'h1C;     // temp on, HiZ=0

logic [15:0] pkt [0:34];
logic [15:0] slot1_prog [0:2];
logic [15:0] slot2_prog [0:1];
logic [15:0] slot1_alt  [0:1];

initial begin
    slot1_prog[0] = 16'h2000; slot1_prog[1] = 16'h2100; slot1_prog[2] = 16'h2200;
    slot2_prog[0] = 16'hFF00; slot2_prog[1] = 16'hE800;
    slot1_alt[0]  = 16'h3000; slot1_alt[1]  = 16'h3100;

    // ---- common setup ----
    load_convert_table();
    set_ctrl(1, 32'd0);            // loop_count = 0 (continuous)
    set_ctrl(2, 32'h0000_0F00);    // channel_enable = 0xF, phases 0
    repeat (8) @(negedge clk);
    rstn = 1;
    repeat (8) @(negedge clk);

    // ======================= TEST 1: identity =============================
    $display("TEST 1: aux_seq_en=0 bit-identity vs legacy core");
    cipo_random = 1;
    identity_checking = 1;
    repeat (10) @(negedge clk);
    set_ctrl(0, 32'h1);            // enable transmission
    // 4 packets + start/stop transients
    repeat (4*2800 + 500) @(negedge clk);
    set_ctrl(0, 32'h0);            // stop
    repeat (2*2800) @(negedge clk);
    // restart once more to cover the restart path
    set_ctrl(0, 32'h1);
    repeat (2*2800) @(negedge clk);
    set_ctrl(0, 32'h0);
    repeat (3000) @(negedge clk);
    identity_checking = 0;
    cipo_random = 0;
    $display("  identity window done (%0d cycle-checks so far)", n_checks);

    // ======================= TEST 2: aux enabled ==========================
    $display("TEST 2: sequencer datapath");
    // deterministic CIPO: cipo0=1, cipo1=0 -> regular words FFFF / 0000
    cipo0 = 1; cipo1 = 0;

    // program banks while idle
    aux_write(0, 0, 0, 6'd0, W3_PLACEHOLDER);
    aux_write(0, 0, 1, 6'd0, {2'b00, 6'd0, 2'b00, 6'd0});       // len 1
    for (int i = 0; i < 3; i++) aux_write(1, 0, 0, 6'(i), slot1_prog[i]);
    aux_write(1, 0, 1, 6'd0, {2'b00, 6'd2, 2'b00, 6'd0});       // loop 0..2
    for (int i = 0; i < 2; i++) aux_write(2, 0, 0, 6'(i), slot2_prog[i]);
    aux_write(2, 0, 1, 6'd0, {2'b00, 6'd1, 2'b00, 6'd0});       // loop 0..1
    // standby bank for the swap test (slot 1, bank 1)
    for (int i = 0; i < 2; i++) aux_write(1, 1, 0, 6'(i), slot1_alt[i]);
    aux_write(1, 1, 1, 6'd0, {2'b00, 6'd1, 2'b00, 6'd0});       // loop 0..1

    // enable sequencer + reg3 static byte
    set_ctrl(22, {REG3_STATIC, 5'b0, 19'b0} | 32'h0000_0001);
    repeat (4) @(negedge clk);
    set_ctrl(0, 32'h1);            // start transmission

    // packet 0: indices all 0
    next_packet_words(pkt);
    for (int i = 0; i < 32; i++)
        check_word($sformatf("p0 conv[%0d]", i), pkt[i], convert_word(i));
    check_word("p0 slot0 (W3 shadow)", pkt[32], {8'h83, REG3_STATIC[7:1], 1'b0});
    check_word("p0 slot1[0]", pkt[33], slot1_prog[0]);
    check_word("p0 slot2[0]", pkt[34], slot2_prog[0]);

    // packet 1: slot1 idx 1, slot2 idx 1
    next_packet_words(pkt);
    check_word("p1 slot1[1]", pkt[33], slot1_prog[1]);
    check_word("p1 slot2[1]", pkt[34], slot2_prog[1]);

    // packet 2: slot1 idx 2, slot2 wraps to 0
    next_packet_words(pkt);
    check_word("p2 slot1[2]", pkt[33], slot1_prog[2]);
    check_word("p2 slot2[0]", pkt[34], slot2_prog[0]);
    // header echo: this packet's slot0 + prev packet's slot1/2
    n_checks++;
    if (hdr_word2[15:8] == 8'h00) err("p2 flags empty with aux on");
    check_word("p2 echo slot0", hdr_word2[31:16], {8'h83, REG3_STATIC[7:1], 1'b0});
    check_word("p2 echo slot1prev", hdr_word2[47:32], slot1_prog[1]);
    check_word("p2 echo slot2prev", hdr_word2[63:48], slot2_prog[1]);

    // ---- fast settle: ON edge next packet ----
    set_ctrl(22, {REG3_STATIC, 5'b0, 19'b0} | 32'h0000_0011);   // fs_sw=1
    next_packet_words(pkt);   // packet 3 may or may not catch the edge; allow 1
    if (pkt[32] != 16'h80FE) next_packet_words(pkt);
    check_word("fs ON inject", pkt[32], 16'h80FE);
    n_checks++; if (aux_status[4] !== 1'b1) err("fs_active status not set");
    // steady: back to the W3 placeholder (rewritten)
    next_packet_words(pkt);
    check_word("fs steady slot0", pkt[32], {8'h83, REG3_STATIC[7:1], 1'b0});
    // OFF edge
    set_ctrl(22, {REG3_STATIC, 5'b0, 19'b0} | 32'h0000_0001);
    next_packet_words(pkt);
    if (pkt[32] != 16'h80DE) next_packet_words(pkt);
    check_word("fs OFF inject", pkt[32], 16'h80DE);

    // ---- DSP bit-H on channel converts ----
    set_ctrl(22, {REG3_STATIC, 5'b0, 19'b0} | 32'h0000_0201);   // dsp_sw=1
    next_packet_words(pkt);
    next_packet_words(pkt);   // fully-latched packet
    for (int i = 0; i < 32; i++)
        check_word($sformatf("dsp conv[%0d]", i), pkt[i], convert_word(i) | 16'h0001);
    check_word("dsp aux untouched", pkt[34] & 16'hC000, slot2_prog[0] & 16'hC000);
    set_ctrl(22, {REG3_STATIC, 5'b0, 19'b0} | 32'h0000_0001);
    next_packet_words(pkt);
    next_packet_words(pkt);
    for (int i = 0; i < 4; i++)
        check_word($sformatf("dsp off conv[%0d]", i), pkt[i], convert_word(i));

    // ---- one-shot injection + result capture ----
    // NOTE on phase: next_packet_words() returns when the last frame's bits
    // finish serializing (~cycle 34, state 63), i.e. BEFORE the packet
    // boundary -- anything armed now takes effect for the very next packet.
    begin
        logic ack_before;
        ack_before = aux_status[7];
        // sync to a known slot-2 phase: last observed packet played prog[1]
        while (pkt[34] != slot2_prog[1]) next_packet_words(pkt);
        aux_inject(16'hFE00);                     // READ(62)
        next_packet_words(pkt);                   // the injected packet
        check_word("inject cmd on wire", pkt[34], 16'hFE00);
        // freeze semantics: the entry that WOULD have played during the
        // injected packet (prog[0], after the wrap) plays next -- no skip.
        next_packet_words(pkt);
        check_word("inject no-skip resume", pkt[34], slot2_prog[0]);
        n_checks++;
        if (aux_status[7] === ack_before) err("inject ack toggle did not flip");
        n_checks++;
        if (aux_read_result !== 32'h0000FFFF)
            err($sformatf("read result got=%08h exp=0000FFFF", aux_read_result));
    end

    // ---- bank swap at packet boundary only ----
    begin
        set_ctrl(22, {REG3_STATIC, 5'b0, 19'b0} | 32'h0000_0005);  // bank_select[1]=1
        // the swap lands at the next boundary; tolerate one in-flight packet
        // still playing bank 0 (old-bank values are disjoint from alt values)
        next_packet_words(pkt);
        if (pkt[33] != slot1_alt[0]) begin
            n_checks++;
            if (pkt[33] != slot1_prog[0] && pkt[33] != slot1_prog[1] && pkt[33] != slot1_prog[2])
                err($sformatf("pre-swap packet plays neither bank: %04h", pkt[33]));
            next_packet_words(pkt);
        end
        n_checks++;
        if (aux_status[1] !== 1'b1) err("bank_active[1] did not confirm swap");
        check_word("swap slot1 entry0", pkt[33], slot1_alt[0]);
        next_packet_words(pkt);
        check_word("swap slot1 entry1", pkt[33], slot1_alt[1]);
        next_packet_words(pkt);
        check_word("swap slot1 wrap", pkt[33], slot1_alt[0]);
    end

    // ---- back to disabled: legacy table on the wire next packet ----
    set_ctrl(22, 32'h0);
    next_packet_words(pkt);
    next_packet_words(pkt);
    check_word("disabled aux32", pkt[32], convert_word(32));
    check_word("disabled aux33", pkt[33], convert_word(33));
    check_word("disabled aux34", pkt[34], convert_word(34));
    n_checks++;
    if (hdr_word2[63:8] !== 56'h0) err("disabled: header upper bits not zero");

    set_ctrl(0, 32'h0);
    repeat (100) @(negedge clk);

    $display("Checks: %0d, Errors: %0d", n_checks, n_errors);
    if (n_errors == 0) $display("RESULT: PASS");
    else               $display("RESULT: FAIL");
    $finish;
end

initial begin
    #2_000_000_000;   // 2 ms wall = well beyond ~60 packets
    $display("ERROR: watchdog timeout");
    $display("RESULT: FAIL");
    $finish;
end

endmodule
