// aux_command_engine.sv
//
// The single command source for the 3 auxiliary COPI positions of the RHD frame
// (cycles AUX_CYC0..LAST_CYC). It merges what used to be three bolted-together
// pieces -- aux_command_sequencer (banked looping store), override_layer (the
// real-time fast-settle / digout / DSP-reset rewrite), and the core's aux mux --
// into one module whose fixed roles are NAMED, not disguised as parameters:
//
//   slot AUX_FS_SLOT     (cycle 32): the ONLY slot fast-settle whole-replaces
//   slot AUX_PLAIN_SLOT  (cycle 33): plain looping program
//   slot AUX_INJECT_SLOT (cycle 34): runtime READ/WRITE-register injection
//
// Aux is ALWAYS ON (there is no enable): the store powers up with each slot's
// legacy CONVERT(AUX_CYC0+slot), so an un-programmed board emits exactly the
// legacy static aux stream, and the override is pass-through until fast-settle /
// digout / DSP-reset are actually requested. Removing the old `aux_seq_en` gate
// also removes its footgun -- the fast-settle OFF injection can no longer be
// stranded by "disable before clearing the config" (there is no disable).
//
// Pipeline: banked store -> per-packet index -> registered read -> override
// rewrite -> aux_cmds. The command settles ~2500 clocks before it serializes.

import acq_frame_pkg::*;

module aux_command_engine #(
    parameter integer ADDR_W = 6              // log2(entries per bank) = 64
)(
    input  logic clk,
    input  logic rstn,

    // Packet strobes from the acquisition FSM
    input  logic seq_advance,          // 1-cycle pulse at the END of each active packet
    input  logic packet_start,         // 1-cycle pulse at the FIRST state of each packet
    input  logic transmission_active,  // low => idle: indices park at 0, swaps apply now

    // Bank selection (quasi-static, CDC-synced): one bit per aux slot
    input  logic [N_AUX-1:0] bank_select,

    // Program write port (1-cycle pulse, PL clock domain)
    input  logic        wr_en,
    input  logic [1:0]  wr_slot,
    input  logic        wr_bank,
    input  logic        wr_is_length,
    input  logic [ADDR_W-1:0] wr_addr,
    input  logic [15:0] wr_data,       // length record: {2'b0, end[5:0], 2'b0, loop[5:0]}

    // One-shot injection (AUX_INJECT_SLOT only)
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
    output logic [N_AUX-1:0]        bank_active,
    output logic [N_AUX*ADDR_W-1:0] slot_indices
);

localparam integer ENTRIES = (1 << ADDR_W);

// seq_hold high while idle: indices park at 0 and bank swaps apply immediately.
wire seq_hold = !transmission_active;

// ===========================================================================
// Banked, looping command store -- genuinely uniform across the N_AUX slots, so
// this part IS a generate loop. The per-slot ROLES (inject) are named, not
// derived from "the last slot".
// ===========================================================================
logic [N_AUX*16-1:0] seq_cmds;   // raw (pre-override) sequencer outputs
logic inject_pending;

genvar s;
generate
for (s = 0; s < N_AUX; s++) begin : g_slot

    logic [15:0] mem [0:2*ENTRIES-1];        // 2 banks x ENTRIES, distributed RAM
    logic [ADDR_W-1:0] loop_idx_r [0:1];
    logic [ADDR_W-1:0] end_idx_r  [0:1];
    logic              active_bank;
    logic [ADDR_W-1:0] index;
    logic [15:0]       cmd_reg;

    // Power-on: each slot replays its legacy static command CONVERT(AUX_CYC0+s),
    // so an un-programmed board is bit-identical to the pre-sequencer aux stream.
    initial begin
        for (int e = 0; e < 2*ENTRIES; e++)
            mem[e] = {2'b00, 6'(AUX_CYC0 + s), 8'h00};
    end

    // Program write port (mem intentionally not reset -> LUTRAM)
    always_ff @(posedge clk) begin
        if (wr_en && !wr_is_length && (wr_slot == 2'(s)))
            mem[{wr_bank, wr_addr}] <= wr_data;
    end

    always_ff @(posedge clk) begin
        if (!rstn) begin
            loop_idx_r[0] <= '0;  end_idx_r[0] <= '0;
            loop_idx_r[1] <= '0;  end_idx_r[1] <= '0;
        end else if (wr_en && wr_is_length && (wr_slot == 2'(s))) begin
            loop_idx_r[wr_bank] <= wr_data[0 +: ADDR_W];
            end_idx_r[wr_bank]  <= wr_data[8 +: ADDR_W];
        end
    end

    // Bank swap + index sequencing (advance one entry per packet, wrap end->loop)
    always_ff @(posedge clk) begin
        if (!rstn) begin
            active_bank <= 1'b0;
            index       <= '0;
        end else if (seq_hold) begin
            active_bank <= bank_select[s];
            index       <= '0;
        end else if (seq_advance) begin
            if (bank_select[s] != active_bank) begin
                active_bank <= bank_select[s];      // atomic swap at packet boundary
                index       <= '0;
            end else if ((s == AUX_INJECT_SLOT) && inject_active) begin
                index <= index;                     // injection consumed this slot: freeze
            end else begin
                index <= (index == end_idx_r[active_bank])
                         ? loop_idx_r[active_bank]
                         : index + 1'b1;
            end
        end
    end

    always_ff @(posedge clk)
        cmd_reg <= mem[{active_bank, index}];       // registered read

    assign bank_active[s] = active_bank;
    assign slot_indices[s*ADDR_W +: ADDR_W] = index;

    if (s == AUX_INJECT_SLOT) begin : g_inject_mux
        assign seq_cmds[s*16 +: 16] = inject_active ? inject_cmd : cmd_reg;
    end else begin : g_plain
        assign seq_cmds[s*16 +: 16] = cmd_reg;
    end
end
endgenerate

// One-shot injection control: arm any time; takes effect for the packet that
// begins at the next boundary, then self-clears.
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

// ===========================================================================
// Real-time override rewrite (was override_layer). Live trigger levels; latched
// once per packet at packet_start so a mid-packet pin change can't tear a
// serialized command.
// ===========================================================================
wire fs_level_now     = fs_sw     || (fs_gpio_en     && digital_in[fs_gpio_sel]);
wire dsp_level_now    = dsp_sw    || (dsp_gpio_en    && digital_in[dsp_gpio_sel]);
wire digout_level_now = digout_sw || (digout_gpio_en && digital_in[digout_gpio_sel]);

logic fs_state;      // live Reg-0 D5 value
logic fs_inject;     // this packet: whole-replace the fast-settle slot
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
    // Coherent bit substitution on any WRITE(0)/WRITE(3) in any slot.
    for (int s2 = 0; s2 < N_AUX; s2++) begin
        logic [15:0] c;
        c = seq_cmds[s2*16 +: 16];
        if (c[15:14] == RHD_CMD_WRITE && c[13:8] == RHD_REG_FS)
            c[RHD_FS_BIT] = fs_state;                 // WRITE(0,...): force D5 to live fast-settle
        else if (c[15:14] == RHD_CMD_WRITE && c[13:8] == RHD_REG_DIGOUT)
            c[7:0] = reg3_shadow;                     // WRITE(3,...): substitute Reg-3 shadow
        aux_cmds[s2*16 +: 16] = c;
    end
    // Whole-command replacement: the fast-settle slot only (the invariant).
    if (fs_inject)
        aux_cmds[AUX_FS_SLOT*16 +: 16] = fs_state ? RHD_WR0_FS_ON : RHD_WR0_FS_OFF;
end

assign dsp_force_h        = dsp_state;
assign fast_settle_active = fs_state;
assign digout_state       = digout_level;

endmodule
