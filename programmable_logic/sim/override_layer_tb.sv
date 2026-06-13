// override_layer_tb.sv
//
// Self-checking xsim testbench for override_layer. Models packets as
// packet_start pulses; after each pulse the latched per-packet state is
// compared against a shadow model and the command rewrite is checked
// exhaustively for the rules:
//
//   A. enable=0: pure pass-through, all status flags 0 (bit-identical path)
//   B. software fast settle: edge -> slot-1 replaced with 0x80FE / 0x80DE for
//      exactly one packet each, level steady -> no replacement
//   C. GPIO fast settle: selected digital_in pin edges inject; pin selection
//      respected; sampling only at packet_start (mid-packet wiggle ignored)
//   D. WRITE(0,..) in any slot gets D5 forced to the live fast-settle state;
//      WRITE(3,..) in any slot gets its data byte replaced by the Reg-3 shadow
//      {reg3_static[7:1], digout}; other commands untouched; slots 2/3 are
//      never whole-command replaced (the invariant)
//   E. digout mirror: digout_state follows sw/GPIO level packet-by-packet and
//      lands in D0 of rewritten WRITE(3) commands
//   F. DSP reset: dsp_force_h follows the (sw|pin) level, packet-latched
//
// Run: bash programmable_logic/sim/run_override_tb.sh (greps "RESULT: PASS")

`timescale 1ns/1ps

module override_layer_tb;

logic clk = 0, rstn = 0;
logic packet_start = 0, enable = 0;
logic [7:0] digital_in = '0;
logic fs_sw = 0, fs_gpio_en = 0;
logic [2:0] fs_gpio_sel = 0;
logic dsp_sw = 0, dsp_gpio_en = 0;
logic [2:0] dsp_gpio_sel = 0;
logic digout_sw = 0, digout_gpio_en = 0;
logic [2:0] digout_gpio_sel = 0;
logic [7:0] reg3_static = 8'h02;     // init default: HiZ=1, temp off
logic [47:0] cmds_in = 48'h0;
logic [47:0] cmds_out;
logic dsp_force_h, fast_settle_active, digout_state;

override_layer dut (.*);

always #5 clk = ~clk;

int n_checks = 0, n_errors = 0;

// shadow per-packet state
bit sh_fs_state = 0, sh_fs_inject = 0, sh_dsp = 0, sh_digout = 0;

function automatic bit fs_level();
    return enable && (fs_sw || (fs_gpio_en && digital_in[fs_gpio_sel]));
endfunction
function automatic bit dsp_level();
    return enable && (dsp_sw || (dsp_gpio_en && digital_in[dsp_gpio_sel]));
endfunction
function automatic bit digout_level();
    return enable && (digout_sw || (digout_gpio_en && digital_in[digout_gpio_sel]));
endfunction

// expected rewrite of one slot command given current shadow state
function automatic logic [15:0] exp_slot(input int s, input logic [15:0] c);
    logic [15:0] r;
    r = c;
    if (enable) begin
        if (c[15:14] == 2'b10 && c[13:8] == 6'd0) r[5] = sh_fs_state;
        else if (c[15:14] == 2'b10 && c[13:8] == 6'd3)
            r[7:0] = {reg3_static[7:1], sh_digout};
        if (s == 0 && sh_fs_inject) r = sh_fs_state ? 16'h80FE : 16'h80DE;
    end
    return r;
endfunction

task automatic check(input string what, input logic [47:0] got, input logic [47:0] exp);
    n_checks++;
    if (got !== exp) begin
        n_errors++;
        $display("ERROR @%0t: %s got=%012h exp=%012h", $time, what, got, exp);
    end
endtask

task automatic check_bit(input string what, input logic got, input logic exp);
    n_checks++;
    if (got !== exp) begin
        n_errors++;
        $display("ERROR @%0t: %s got=%b exp=%b", $time, what, got, exp);
    end
endtask

// pulse a packet boundary; update the shadow exactly as the DUT spec
task automatic new_packet;
    @(negedge clk); packet_start = 1;
    // shadow latches from the levels present AT the packet_start edge
    sh_fs_inject = (fs_level() != sh_fs_state);
    sh_fs_state  = fs_level();
    sh_dsp       = dsp_level();
    sh_digout    = digout_level();
    @(negedge clk); packet_start = 0;
    repeat (2) @(negedge clk);
endtask

task automatic check_all(input string tag);
    logic [47:0] exp;
    for (int s = 0; s < 3; s++)
        exp[s*16 +: 16] = exp_slot(s, cmds_in[s*16 +: 16]);
    check({tag, " cmds"}, cmds_out, exp);
    check_bit({tag, " fs_active"}, fast_settle_active, enable ? sh_fs_state : 1'b0);
    check_bit({tag, " digout"},    digout_state,       enable ? sh_digout   : 1'b0);
    check_bit({tag, " dsp"},       dsp_force_h,        enable ? sh_dsp      : 1'b0);
endtask

initial begin
    repeat (3) @(negedge clk);
    rstn = 1;
    repeat (2) @(negedge clk);

    // representative bundle: slot0 = WRITE(3, 0xAA) placeholder (digout host),
    // slot1 = CONVERT(33) accel, slot2 = WRITE(0, 0xDE) (someone touching reg0)
    cmds_in = {16'h80DE, 16'h2100, 16'h83AA};

    // ---- A: disabled -> pass-through ------------------------------------
    fs_sw = 1; digout_sw = 1; dsp_sw = 1;   // config set but enable=0
    new_packet(); check_all("A.disabled");
    fs_sw = 0; digout_sw = 0; dsp_sw = 0;

    // ---- B: software fast settle ----------------------------------------
    enable = 1;
    new_packet(); check_all("B.idle");
    fs_sw = 1;
    new_packet(); check_all("B.on-edge");        // slot0 -> 0x80FE
    n_checks++; if (cmds_out[15:0] !== 16'h80FE) begin n_errors++; $display("ERROR: B on-edge literal"); end
    new_packet(); check_all("B.on-hold");        // no injection, WRITE(3) rewrite back
    fs_sw = 0;
    new_packet(); check_all("B.off-edge");       // slot0 -> 0x80DE
    n_checks++; if (cmds_out[15:0] !== 16'h80DE) begin n_errors++; $display("ERROR: B off-edge literal"); end
    new_packet(); check_all("B.off-hold");

    // ---- C: GPIO fast settle, pin select + packet sampling ---------------
    fs_gpio_en = 1; fs_gpio_sel = 3'd5;
    digital_in[5] = 1;
    new_packet(); check_all("C.pin-on-edge");
    // mid-packet wiggle must be ignored until next packet_start
    digital_in[5] = 0;
    repeat (2) @(negedge clk);
    check_all("C.mid-packet-ignored");
    new_packet(); check_all("C.pin-off-edge");
    // a different pin must not trigger
    digital_in[2] = 1;
    new_packet(); check_all("C.other-pin-ignored");
    digital_in[2] = 0; fs_gpio_en = 0;

    // ---- D: WRITE(0)/WRITE(3) coherence in non-RT slots -------------------
    fs_sw = 1; new_packet();                     // settle ON (injection packet)
    new_packet();                                 // settle steady
    // slot2 carries WRITE(0,0xDE): D5 must be forced to 1 -> 0x80FE
    n_checks++;
    if (cmds_out[47:32] !== 16'h80FE) begin
        n_errors++; $display("ERROR: D slot2 WRITE0 D5 force got=%04h", cmds_out[47:32]);
    end
    check_all("D.write0-coherence");
    // CONVERT in slot1 untouched
    n_checks++;
    if (cmds_out[31:16] !== 16'h2100) begin n_errors++; $display("ERROR: D convert touched"); end
    fs_sw = 0; new_packet(); new_packet();

    // ---- E: digout mirror --------------------------------------------------
    reg3_static = 8'h1C;                          // temp bits set, HiZ=0
    digout_sw = 1;
    new_packet(); check_all("E.digout-on");
    n_checks++;
    if (cmds_out[7:0] !== 8'h1D) begin n_errors++; $display("ERROR: E shadow byte got=%02h", cmds_out[7:0]); end
    digout_sw = 0; digout_gpio_en = 1; digout_gpio_sel = 3'd7;
    digital_in[7] = 1;
    new_packet(); check_all("E.digout-gpio");
    digital_in[7] = 0;
    new_packet(); check_all("E.digout-gpio-off");
    digout_gpio_en = 0;

    // ---- F: DSP reset level -------------------------------------------------
    dsp_sw = 1;
    new_packet(); check_all("F.dsp-on");
    dsp_sw = 0; dsp_gpio_en = 1; dsp_gpio_sel = 3'd1;
    digital_in[1] = 1;
    new_packet(); check_all("F.dsp-pin");
    digital_in[1] = 0;
    new_packet(); check_all("F.dsp-off");
    dsp_gpio_en = 0;

    // ---- A2: disable again -> flags drop, pass-through ----------------------
    enable = 0;
    sh_fs_state = 0; sh_fs_inject = 0; sh_dsp = 0; sh_digout = 0;
    new_packet(); check_all("A2.re-disabled");

    $display("Checks: %0d, Errors: %0d", n_checks, n_errors);
    if (n_errors == 0) $display("RESULT: PASS");
    else               $display("RESULT: FAIL");
    $finish;
end

initial begin
    #200_000;
    $display("ERROR: watchdog timeout");
    $display("RESULT: FAIL");
    $finish;
end

endmodule
