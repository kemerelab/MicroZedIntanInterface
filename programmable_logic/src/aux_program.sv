// aux_program.sv
//
// ONE cycling auxiliary-command program: a double-buffered command store that
// steps one entry per acquisition packet and loops end->loop. This is the only
// place the "banked sequencer" machinery lives -- it is instantiated exactly
// twice, for the two aux slots that actually cycle (slot 1 = the ADC/accel
// sweep, slot 2 = the housekeeping rotation). The fixed RT slot (slot 0) is a
// plain register in aux_command_engine and does NOT use this module.
//
// Double buffer: two banks per program; bank_select picks the live bank and a
// change takes effect atomically at the next packet boundary (host uploads into
// the standby bank, then flips). Power-on fills every entry with DEFAULT_CMD so
// an un-programmed board emits the legacy static CONVERT for this slot.
//
// `freeze` holds the index for one advance -- slot 2 asserts it during a one-shot
// register injection so the housekeeping rotation resumes exactly where it left
// off (housekeeping may be missed for that packet, which is fine).

import acq_frame_pkg::*;

module aux_program #(
    parameter integer      ADDR_W      = 6,        // log2(entries per bank) = 64
    parameter logic [15:0] DEFAULT_CMD = 16'h0     // power-on command (legacy static)
)(
    input  logic              clk,
    input  logic              rstn,

    input  logic              seq_advance,   // 1-cycle pulse at the END of each active packet
    input  logic              seq_hold,      // idle: park index at 0, apply bank swap now
    input  logic              bank_select,   // requested live bank (quasi-static, CDC-synced)
    input  logic              freeze,        // hold the index for this advance (slot-2 inject)

    // Program write port (already qualified for THIS program by the engine)
    input  logic              wr_en,
    input  logic              wr_is_length,
    input  logic              wr_bank,
    input  logic [ADDR_W-1:0] wr_addr,
    input  logic [15:0]       wr_data,       // length record: {.., end[13:8], .., loop[5:0]}

    output logic [15:0]       cmd,           // current command (registered read)
    output logic [ADDR_W-1:0] index,
    output logic              bank_active
);

localparam integer ENTRIES = (1 << ADDR_W);

logic [15:0]       mem [0:2*ENTRIES-1];   // 2 banks x ENTRIES, distributed RAM
logic [ADDR_W-1:0] loop_idx_r [0:1];
logic [ADDR_W-1:0] end_idx_r  [0:1];
logic              active_bank;
logic [ADDR_W-1:0] idx;
logic [15:0]       cmd_reg;

// Power-on: legacy static command for this slot (mem intentionally not reset -> LUTRAM).
initial
    for (int e = 0; e < 2*ENTRIES; e++)
        mem[e] = DEFAULT_CMD;

always_ff @(posedge clk)
    if (wr_en && !wr_is_length)
        mem[{wr_bank, wr_addr}] <= wr_data;

always_ff @(posedge clk) begin
    if (!rstn) begin
        loop_idx_r[0] <= '0;  end_idx_r[0] <= '0;
        loop_idx_r[1] <= '0;  end_idx_r[1] <= '0;
    end else if (wr_en && wr_is_length) begin
        loop_idx_r[wr_bank] <= wr_data[0 +: ADDR_W];
        end_idx_r[wr_bank]  <= wr_data[8 +: ADDR_W];
    end
end

// Bank swap + index sequencing (advance one entry per packet, wrap end->loop).
always_ff @(posedge clk) begin
    if (!rstn) begin
        active_bank <= 1'b0;
        idx         <= '0;
    end else if (seq_hold) begin
        active_bank <= bank_select;         // idle: adopt requested bank, park at 0
        idx         <= '0;
    end else if (seq_advance) begin
        if (bank_select != active_bank) begin
            active_bank <= bank_select;      // atomic swap at the packet boundary
            idx         <= '0;
        end else if (freeze) begin
            idx <= idx;                      // injection consumed this slot: hold
        end else begin
            idx <= (idx == end_idx_r[active_bank])
                   ? loop_idx_r[active_bank]
                   : idx + 1'b1;
        end
    end
end

always_ff @(posedge clk)
    cmd_reg <= mem[{active_bank, idx}];      // registered read

assign cmd         = cmd_reg;
assign index       = idx;
assign bank_active = active_bank;

endmodule
