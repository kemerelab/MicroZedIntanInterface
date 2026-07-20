// aux_command_engine.sv
//
// The single command source for the 3 auxiliary COPI positions of the RHD frame
// (cycles AUX_CYC0..LAST_CYC). The three positions have DIFFERENT roles, so they
// are wired as three NAMED things -- not a homogeneous array you loop over:
//
//   slot 0 (cycle 32, AUX_FS_SLOT)     : a single fixed RT command register.
//                                        Does not cycle. The override rewrites it
//                                        live (fast-settle whole-replace, Reg-3
//                                        digout). Default CONVERT(32).
//   slot 1 (cycle 33, AUX_PLAIN_SLOT)  : one cycling program (aux_program) --
//                                        the ADC/accelerometer sweep.
//   slot 2 (cycle 34, AUX_INJECT_SLOT) : one cycling program (aux_program) --
//                                        the housekeeping rotation -- AND the
//                                        target of one-shot register injection,
//                                        which freezes the rotation for a packet.
//
// Only the two slots that genuinely cycle instantiate the banked sequencer
// (aux_program); the RT slot is a plain register. This is the whole point of the
// module: the structure states which slots cycle and which don't.
//
// Aux is ALWAYS ON (there is no enable): the RT register + both programs power up
// with their legacy CONVERT(AUX_CYC0+slot), so an un-programmed board emits
// exactly the legacy static aux stream, and the override is pass-through until
// fast-settle / digout / DSP-reset are actually requested.
//
// Pipeline: RT reg / program read -> injection mux -> override rewrite -> aux_cmds.
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
    input  logic transmission_active,  // low => idle: indices park at 0, swaps apply now

    // Live bank per cycling program: [0] = slot 1, [1] = slot 2 (CDC-synced)
    input  logic [1:0]  prog_bank_select,

    // Unified write port (1-cycle pulse). wr_target is the SLOT INDEX:
    //   0 (AUX_FS_SLOT)     = slot-0 RT command register (single word; bank/addr/
    //                         is_length ignored),
    //   1 (AUX_PLAIN_SLOT)  = slot-1 program,
    //   2 (AUX_INJECT_SLOT) = slot-2 program.
    input  logic        wr_en,
    input  logic [1:0]  wr_target,
    input  logic        wr_is_length,  // program length record (ignored for RT target)
    input  logic        wr_bank,
    input  logic [ADDR_W-1:0] wr_addr,
    input  logic [15:0] wr_data,       // length record: {.., end[13:8], .., loop[5:0]}

    // One-shot injection (slot 2 only)
    input  logic        inject_req,
    input  logic [15:0] inject_cmd,

    // Real-time override config (fast settle / DSP reset / aux digital out)
    input  logic [7:0]  digital_in,
    input  logic        fs_sw,
    input  logic        fs_gpio_en,
    input  logic [2:0]  fs_gpio_sel,
    input  logic        dsp_sw,
    input  logic        dsp_gpio_en,
    input  logic [2:0]  dsp_gpio_sel,
    input  logic        digout_sw,
    input  logic        digout_gpio_en,
    input  logic [2:0]  digout_gpio_sel,
    input  logic [7:0]  reg3_static,   // host-owned Reg-3 bits D7..D1 (D0 = live digout)

    // Final commands for cycles AUX_CYC0..LAST_CYC (slot i at [i*16 +: 16])
    output logic [N_AUX*16-1:0]     aux_cmds,
    // DSP reset: force bit H on the channel CONVERTs (cycles 0..N_CHAN_CMDS-1)
    output logic                    dsp_force_h,
    output logic                    fast_settle_active,
    output logic                    digout_state,
    output logic                    inject_active,
    // Status: slot 0 has no bank/index (RT register) -> reported as 0.
    output logic [N_AUX-1:0]        bank_active,
    output logic [N_AUX*ADDR_W-1:0] slot_indices
);

// Legacy static command per slot: CONVERT(AUX_CYC0 + slot) = {2'b00, ch, 8'h00}.
localparam logic [15:0] RT_DEFAULT    = {2'b00, 6'(AUX_CYC0 + AUX_FS_SLOT),     8'h00}; // CONVERT(32)
localparam logic [15:0] PROG1_DEFAULT = {2'b00, 6'(AUX_CYC0 + AUX_PLAIN_SLOT),  8'h00}; // CONVERT(33)
localparam logic [15:0] PROG2_DEFAULT = {2'b00, 6'(AUX_CYC0 + AUX_INJECT_SLOT), 8'h00}; // CONVERT(34)

// seq_hold high while idle: program indices park at 0 and bank swaps apply now.
wire seq_hold = !transmission_active;

// ===========================================================================
// slot 0 -- the fixed RT command register (no bank, no index). Written via the
// unified write port with wr_target == 2; the override rewrites it downstream.
// ===========================================================================
logic [15:0] rt_cmd_r;
always_ff @(posedge clk) begin
    if (!rstn)
        rt_cmd_r <= RT_DEFAULT;
    else if (wr_en && (wr_target == 2'(AUX_FS_SLOT)) && !wr_is_length)
        rt_cmd_r <= wr_data;              // single word; a length write here is a no-op
end

// ===========================================================================
// slots 1 & 2 -- the two cycling programs. This is the ONLY banked-sequencer
// machinery in the design (instantiated exactly twice).
// ===========================================================================
logic [15:0]       prog1_cmd, prog2_cmd;
logic [ADDR_W-1:0] prog1_index, prog2_index;
logic              prog1_bank, prog2_bank;

aux_program #(.ADDR_W(ADDR_W), .DEFAULT_CMD(PROG1_DEFAULT)) prog1 (
    .clk(clk), .rstn(rstn),
    .seq_advance(seq_advance), .seq_hold(seq_hold),
    .bank_select(prog_bank_select[0]),
    .freeze(1'b0),                                   // slot 1 never freezes
    .wr_en(wr_en && (wr_target == 2'(AUX_PLAIN_SLOT))),
    .wr_is_length(wr_is_length), .wr_bank(wr_bank), .wr_addr(wr_addr), .wr_data(wr_data),
    .cmd(prog1_cmd), .index(prog1_index), .bank_active(prog1_bank)
);

aux_program #(.ADDR_W(ADDR_W), .DEFAULT_CMD(PROG2_DEFAULT)) prog2 (
    .clk(clk), .rstn(rstn),
    .seq_advance(seq_advance), .seq_hold(seq_hold),
    .bank_select(prog_bank_select[1]),
    .freeze(inject_active),                          // hold housekeeping during injection
    .wr_en(wr_en && (wr_target == 2'(AUX_INJECT_SLOT))),
    .wr_is_length(wr_is_length), .wr_bank(wr_bank), .wr_addr(wr_addr), .wr_data(wr_data),
    .cmd(prog2_cmd), .index(prog2_index), .bank_active(prog2_bank)
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
assign seq_cmds[AUX_FS_SLOT*16     +: 16] = rt_cmd_r;                                // slot 0
assign seq_cmds[AUX_PLAIN_SLOT*16  +: 16] = prog1_cmd;                              // slot 1
assign seq_cmds[AUX_INJECT_SLOT*16 +: 16] = inject_active ? inject_cmd : prog2_cmd; // slot 2

// ===========================================================================
// Real-time override rewrite (was override_layer). Live trigger levels; latched
// once per packet at packet_start so a mid-packet pin change can't tear a
// serialized command.
// ===========================================================================
wire fs_level_now     = fs_sw     || (fs_gpio_en     && digital_in[fs_gpio_sel]);
wire dsp_level_now    = dsp_sw    || (dsp_gpio_en    && digital_in[dsp_gpio_sel]);
wire digout_level_now = digout_sw || (digout_gpio_en && digital_in[digout_gpio_sel]);

logic fs_state;      // live Reg-0 D5 value
logic fs_inject;     // this packet: whole-replace the RT slot with fast-settle
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
    // Whole-command replacement on a fast-settle edge: the RT slot only.
    if (fs_inject)
        aux_cmds[AUX_FS_SLOT*16 +: 16] = fs_state ? RHD_WR0_FS_ON : RHD_WR0_FS_OFF;
end

assign dsp_force_h        = dsp_state;
assign fast_settle_active = fs_state;
assign digout_state       = digout_level;

// Status: the RT slot (0) has no bank or index; report 0 there so the existing
// 3-slot status layout is preserved (firmware/host read slot-0 fields as 0).
assign bank_active = {prog2_bank, prog1_bank, 1'b0};
assign slot_indices[AUX_FS_SLOT*ADDR_W     +: ADDR_W] = '0;
assign slot_indices[AUX_PLAIN_SLOT*ADDR_W  +: ADDR_W] = prog1_index;
assign slot_indices[AUX_INJECT_SLOT*ADDR_W +: ADDR_W] = prog2_index;

endmodule
