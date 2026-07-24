// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// aux_command_engine.sv
//
// The single command source for the 3 auxiliary COPI positions of the RHD frame
// (cycles AUX_CYC0..LAST_CYC). The three positions have DIFFERENT roles, wired as
// three NAMED things -- one cycling program and two command registers:
//
//   slot 0 (cycle 32, AUX_SWEEP_SLOT)  : the ONE cycling program (aux_program) --
//                                        the accelerometer / aux-ADC sweep. This is
//                                        the only banked sequencer in the design.
//                                        On slot 0 so its reply lands in data word 34
//                                        of the SAME packet (intra-packet axis pairing).
//   slot 1 (cycle 33, AUX_FS_SLOT)     : a fixed command register, rewritten LIVE by
//                                        the override (fast-settle whole-replace +
//                                        Reg-3 digout). Default READ(40) = INTAN ROM
//                                        'I'. Fast-settle whole-replaces THIS slot, so
//                                        the accel sweep on slot 0 is never clobbered.
//   slot 2 (cycle 34, AUX_INJECT_SLOT) : a fixed command register, whole-replaced for
//                                        ONE packet by a one-shot register READ/WRITE
//                                        injection (control plane; the reply is
//                                        captured downstream). Default CONVERT(49) =
//                                        temperature aux-ADC channel.
//
// So: one program (slot 0) + two registers (slots 1, 2). slot 1 is rewritten by the
// real-time override; slot 2 by one-shot injection. Neither register cycles.
//
// Aux is ALWAYS ON (there is no enable). At power-on the engine runs the standard
// acquisition config: slot 0 boots into the accelerometer / aux-ADC sweep (reads
// channels 32/33/34, one per packet), slot 1 reads the INTAN ROM 'I' register, and
// slot 2 reads the temperature channel. The override is pass-through and injection is
// idle until fast-settle / digout / DSP-reset / a register access is requested.
//
// Pipeline: program / register read -> injection mux -> override rewrite -> aux_cmds.
// The command settles ~2500 clocks before it serializes.

import acq_frame_pkg::*;

module aux_command_engine #(
    parameter integer ADDR_W = 6              // log2(entries per program bank) = 64
)(
    input  logic clk,
    input  logic rstn,

    // Packet strobes from the acquisition FSM
    input  logic seq_advance,          // 1-cycle pulse at the END of each active packet
    input  logic packet_start,         // 1-cycle pulse at the FIRST state of each packet
    input  logic transmission_active,  // low => idle: the program index parks at 0, swap now

    // External digital inputs (fast-settle / DSP-reset / digout GPIO triggers)
    input  logic [7:0]  digital_in,

    // The engine's three raw AXI control registers (PL-domain after the CDC). The
    // engine OWNS and decodes these itself -- the field maps below mirror
    // firmware/include/main.h (CTRL_REG_AUX_*), so the framing loop never has to.
    input  logic [31:0] aux_ctrl_reg,    // reg 22: prog bank select + fs/dsp/digout config
    input  logic [31:0] aux_write_reg,   // reg 23: write port payload (registers / program)
    input  logic [31:0] aux_strobe_reg,  // reg 24: write/inject toggles + inject command

    // Final commands for cycles AUX_CYC0..LAST_CYC (slot i at [i*16 +: 16])
    output logic [N_AUX*16-1:0]     aux_cmds,
    // DSP reset: force bit H on the channel CONVERTs (cycles 0..N_CHAN_CMDS-1)
    output logic                    dsp_force_h,
    output logic                    fast_settle_active,
    output logic                    digout_state,
    output logic                    inject_active,
    // Status: only slot 0 (the program) has a bank/index; slots 1 and 2 report 0.
    output logic [N_AUX-1:0]        bank_active,
    output logic [N_AUX*ADDR_W-1:0] slot_indices
);

// slot-1 (fs register) power-on command: read the INTAN ROM 'I' byte (register 40).
// The +2 readback puts its reply in data word 0 of the NEXT packet.
localparam logic [15:0] FS_DEFAULT     = {2'b11, 6'd40, 8'h00}; // READ(40) = 0xE800  ('I' of INTAN)
// slot-2 (inject register) power-on command: read the temperature aux-ADC channel.
// Valid temperature needs the on-chip temp sensor enabled (RHD Reg-3); until then this
// simply samples aux channel 49. Reply lands in data word 1 of the NEXT packet.
localparam logic [15:0] INJECT_DEFAULT = {2'b00, 6'd49, 8'h00}; // CONVERT(49) = 0x3100 (temperature)
// slot-0 boot program: the accelerometer / aux-ADC sweep -- reads aux channels
// 32, 33, 34 one per packet (each axis at 30kHz/3 ~= 10 kHz). Reply lands in data
// word 34 of the SAME packet, so the host de-interleaves each axis intra-packet.
localparam logic [15:0] SWEEP_0 = {2'b00, 6'(AUX_CYC0 + 0), 8'h00};   // CONVERT(32)
localparam logic [15:0] SWEEP_1 = {2'b00, 6'(AUX_CYC0 + 1), 8'h00};   // CONVERT(33)
localparam logic [15:0] SWEEP_2 = {2'b00, 6'(AUX_CYC0 + 2), 8'h00};   // CONVERT(34)

// seq_hold high while idle: the program index parks at 0 and a bank swap applies now.
wire seq_hold = !transmission_active;

// ===========================================================================
// Control-register decode. The engine owns its three registers; these field maps
// mirror firmware/include/main.h (CTRL_REG_AUX_*).
// ===========================================================================
// reg 22 (aux_ctrl_reg): [0] slot-0 program bank select, [4] fs_sw, [5] fs_gpio_en,
//   [8:6] fs_gpio_sel, [9] dsp_sw, [10] dsp_gpio_en, [13:11] dsp_gpio_sel,
//   [14] digout_sw, [15] digout_gpio_en, [18:16] digout_gpio_sel, [31:24] reg3_static.
wire        prog_bank_select = aux_ctrl_reg[0];   // only slot 0 cycles -> one bank bit
wire        fs_sw            = aux_ctrl_reg[4];
wire        fs_gpio_en       = aux_ctrl_reg[5];
wire [2:0]  fs_gpio_sel      = aux_ctrl_reg[6 +: 3];
wire        dsp_sw           = aux_ctrl_reg[9];
wire        dsp_gpio_en      = aux_ctrl_reg[10];
wire [2:0]  dsp_gpio_sel     = aux_ctrl_reg[11 +: 3];
wire        digout_sw        = aux_ctrl_reg[14];
wire        digout_gpio_en   = aux_ctrl_reg[15];
wire [2:0]  digout_gpio_sel  = aux_ctrl_reg[16 +: 3];
wire [7:0]  reg3_static      = aux_ctrl_reg[24 +: 8];
// reg 23 (aux_write_reg): [15:0] data, [21:16] addr, [23:22] target (slot index),
//   [24] bank, [25] is_length.
wire [15:0]       wr_data      = aux_write_reg[0 +: 16];
wire [ADDR_W-1:0] wr_addr      = aux_write_reg[16 +: ADDR_W];
wire [1:0]        wr_target    = aux_write_reg[22 +: 2];
wire              wr_bank      = aux_write_reg[24];
wire              wr_is_length = aux_write_reg[25];
// reg 24 (aux_strobe_reg): [0] write toggle, [1] inject toggle, [31:16] inject cmd.
wire        wr_toggle  = aux_strobe_reg[0];
wire        inj_toggle = aux_strobe_reg[1];
wire [15:0] inject_cmd = aux_strobe_reg[16 +: 16];

// Toggle -> 1-cycle pulse (the payload regs are written in prior AXI transactions,
// so they are long stable when the host flips the toggle).
logic wr_toggle_d, inj_toggle_d, wr_en, inject_req;
always_ff @(posedge clk) begin
    if (!rstn) begin
        wr_toggle_d <= 1'b0;  inj_toggle_d <= 1'b0;  wr_en <= 1'b0;  inject_req <= 1'b0;
    end else begin
        wr_toggle_d  <= wr_toggle;
        inj_toggle_d <= inj_toggle;
        wr_en        <= wr_toggle  ^ wr_toggle_d;
        inject_req   <= inj_toggle ^ inj_toggle_d;
    end
end

// ===========================================================================
// slots 1 and 2 -- fixed command registers (no bank, no index). Each is set via
// the write port; slot 1 is rewritten by the override, slot 2 by injection.
// A length write to either target is a no-op (registers have no length).
// ===========================================================================
logic [15:0] fs_cmd_r;        // slot 1 base (override-rewritten)
logic [15:0] inject_base_r;   // slot 2 base (injection-rewritten)
always_ff @(posedge clk) begin
    if (!rstn) begin
        fs_cmd_r      <= FS_DEFAULT;
        inject_base_r <= INJECT_DEFAULT;
    end else if (wr_en && !wr_is_length) begin
        if (wr_target == 2'(AUX_FS_SLOT))     fs_cmd_r      <= wr_data;
        if (wr_target == 2'(AUX_INJECT_SLOT)) inject_base_r <= wr_data;
    end
end

// ===========================================================================
// slot 0 -- the one cycling program. This is the ONLY banked-sequencer machinery.
// ===========================================================================
logic [15:0]       prog_cmd;
logic [ADDR_W-1:0] prog_index;
logic              prog_bank;

aux_program #(.ADDR_W(ADDR_W),
              .INIT_LEN(3),
              .INIT_CMDS({16'h0, SWEEP_2, SWEEP_1, SWEEP_0})) prog (   // boot: sweep 32/33/34
    .clk(clk), .rstn(rstn),
    .seq_advance(seq_advance), .seq_hold(seq_hold),
    .bank_select(prog_bank_select),
    .wr_en(wr_en && (wr_target == 2'(AUX_SWEEP_SLOT))),
    .wr_is_length(wr_is_length), .wr_bank(wr_bank), .wr_addr(wr_addr), .wr_data(wr_data),
    .cmd(prog_cmd), .index(prog_index), .bank_active(prog_bank)
);

// One-shot injection control: arm any time; takes effect for the packet that
// begins at the next boundary, then self-clears.
logic inject_pending;
always_ff @(posedge clk) begin
    if (!rstn) begin
        inject_pending <= 1'b0;
        inject_active  <= 1'b0;
    end else if (seq_hold) begin
        inject_pending <= 1'b0;
        inject_active  <= 1'b0;
    end else if (seq_advance) begin
        inject_active  <= inject_pending;
        inject_pending <= inject_req;
    end else if (inject_req) begin
        inject_pending <= 1'b1;
    end
end

// Assemble the three raw (pre-override) slot commands by NAME.
logic [N_AUX*16-1:0] seq_cmds;
assign seq_cmds[AUX_SWEEP_SLOT*16  +: 16] = prog_cmd;                                    // slot 0
assign seq_cmds[AUX_FS_SLOT*16     +: 16] = fs_cmd_r;                                    // slot 1
assign seq_cmds[AUX_INJECT_SLOT*16 +: 16] = inject_active ? inject_cmd : inject_base_r;  // slot 2

// ===========================================================================
// Real-time override rewrite. Live trigger levels; latched once per packet at
// packet_start so a mid-packet pin change can't tear a serialized command.
// ===========================================================================
wire fs_level_now     = fs_sw     || (fs_gpio_en     && digital_in[fs_gpio_sel]);
wire dsp_level_now    = dsp_sw    || (dsp_gpio_en    && digital_in[dsp_gpio_sel]);
wire digout_level_now = digout_sw || (digout_gpio_en && digital_in[digout_gpio_sel]);

logic fs_state;      // live Reg-0 D5 value
logic fs_inject;     // this packet: whole-replace the fs register (slot 1) with fast-settle
logic dsp_state;
logic digout_level;

always_ff @(posedge clk) begin
    if (!rstn) begin
        fs_state <= 1'b0;  fs_inject <= 1'b0;  dsp_state <= 1'b0;  digout_level <= 1'b0;
    end else if (packet_start) begin
        fs_inject    <= (fs_level_now != fs_state);   // edge -> one injection packet
        fs_state     <= fs_level_now;
        dsp_state    <= dsp_level_now;
        digout_level <= digout_level_now;
    end
end

wire [7:0] reg3_shadow = {reg3_static[7:1], digout_level};

always_comb begin
    aux_cmds = seq_cmds;
    // Coherent live-bit substitution: ANY WRITE(0)/WRITE(3) in the aux group gets
    // the live fast-settle / Reg-3-shadow bits. This loop is over the three
    // ASSEMBLED commands for bit-coherence -- it does not assume the slots are
    // interchangeable (their sources above are distinct and named).
    for (int s2 = 0; s2 < N_AUX; s2++) begin
        logic [15:0] c;
        c = seq_cmds[s2*16 +: 16];
        if (c[15:14] == RHD_CMD_WRITE && c[13:8] == RHD_REG_FS)
            c[RHD_FS_BIT] = fs_state;                 // WRITE(0,...): force D5 to live fast-settle
        else if (c[15:14] == RHD_CMD_WRITE && c[13:8] == RHD_REG_DIGOUT)
            c[7:0] = reg3_shadow;                     // WRITE(3,...): substitute Reg-3 shadow
        aux_cmds[s2*16 +: 16] = c;
    end
    // Whole-command replacement on a fast-settle edge: the fs register (slot 1) only,
    // so the accel sweep on slot 0 is never displaced.
    if (fs_inject)
        aux_cmds[AUX_FS_SLOT*16 +: 16] = fs_state ? RHD_WR0_FS_ON : RHD_WR0_FS_OFF;
end

assign dsp_force_h        = dsp_state;
assign fast_settle_active = fs_state;
assign digout_state       = digout_level;

// Status: only slot 0 (the program) has a bank/index; the two register slots
// report 0 (they neither bank-swap nor cycle).
assign bank_active = {1'b0, 1'b0, prog_bank};   // [2]=slot2, [1]=slot1, [0]=slot0
assign slot_indices[AUX_SWEEP_SLOT*ADDR_W  +: ADDR_W] = prog_index;
assign slot_indices[AUX_FS_SLOT*ADDR_W     +: ADDR_W] = '0;
assign slot_indices[AUX_INJECT_SLOT*ADDR_W +: ADDR_W] = '0;

endmodule
