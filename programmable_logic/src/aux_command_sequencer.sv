// aux_command_sequencer.sv
//
// Programmable, looping, banked command source for the 3 aux COPI positions
// (cycle_counter 32/33/34) of the acquisition loop. Implements the banked half
// of docs/command-bank-design.md:
//
//   * 3 independent slots in ONE module (generate loop, per-slot state).
//   * 2 banks per slot (active + standby) so the PS can write a standby bank
//     while the FSM plays the active one, then swap atomically.
//   * Length is bound to the BANK: each bank carries its own (loop_idx, end_idx)
//     register pair, so a bank swap atomically swaps the length too.
//   * The per-slot index starts at 0 when a bank becomes active, advances once
//     per packet, and wraps end_idx -> loop_idx. loop_idx > 0 gives a run-once
//     preamble (e.g. config commands) followed by a looping tail.
//   * Bank swaps requested via bank_select are latched ONLY at a packet
//     boundary (seq_advance) while running, so a program is never torn.
//     bank_active reports the live bank for the PS confirm-before-reuse poll.
//   * One-shot injection (slot 3 / index 2): a pending request replaces slot
//     3's command for exactly one packet; that slot's index is frozen for the
//     packet so the loaded program is not perturbed. Used for runtime
//     READ_REGISTER / WRITE_REGISTER with command-echo identity.
//
// Storage: 3 slots x 2 banks x 64 x 16 bit with a REGISTERED (synchronous)
// read into cmd_reg, which lets Vivado infer block RAM (1.5 of the device's
// ~140 idle BRAM tiles). This is deliberate: the routing congestion that
// killed the previous integration attempt was SLICEM/LUTRAM contention around
// fifo_bram_interface (docs/routing_report.md), so the command store stays off
// the contended LUTRAM resource entirely. The 1-cycle read latency is free:
// the index updates at the packet boundary and the command is not serialized
// until cycle 32 of the 35x80-state packet, ~2500 clocks later.
//
// Memory is initialized to the slot's legacy command (CONVERT(32+slot)) with
// length 0, so enabling the sequencer before programming it reproduces
// today's aux behavior exactly.
//
// Clocking: everything is on the 84 MHz PL clock. The write port and config
// inputs arrive via the AXI-lite control registers' 2-flop synchronizers; the
// wrapper converts a write-strobe toggle into the 1-cycle wr_en pulse.

module aux_command_sequencer #(
    parameter integer ADDR_W = 6,            // log2(entries per bank) = 64
    parameter integer NSLOTS = 3
)(
    input  logic clk,
    input  logic rstn,

    // Packet-boundary strobes from the acquisition FSM
    input  logic seq_advance,   // 1-cycle pulse at the END of each active packet
    input  logic seq_hold,      // high while idle/disabled: indices park at 0,
                                // bank swaps apply immediately

    // Bank selection (quasi-static, CDC-synced): one bit per slot
    input  logic [NSLOTS-1:0] bank_select,

    // Program write port (1-cycle pulse, PL clock domain)
    input  logic        wr_en,
    input  logic [1:0]  wr_slot,        // 0..2
    input  logic        wr_bank,
    input  logic        wr_is_length,   // 1: write (loop,end) record, 0: command word
    input  logic [ADDR_W-1:0] wr_addr,
    input  logic [15:0] wr_data,        // length record: {2'b0, end[5:0], 2'b0, loop[5:0]}

    // One-shot slot-3 command injection
    input  logic        inject_req,     // 1-cycle pulse: arm injection
    input  logic [15:0] inject_cmd,
    output logic        inject_active,  // high for the one packet using inject_cmd

    // Command outputs for cycles 32..34 (stable through each packet)
    output logic [NSLOTS*16-1:0]     aux_cmds,
    output logic [NSLOTS-1:0]        bank_active,
    output logic [NSLOTS*ADDR_W-1:0] slot_indices
);

localparam integer ENTRIES = (1 << ADDR_W);

logic inject_pending;

genvar s;
generate
for (s = 0; s < NSLOTS; s++) begin : g_slot

    // Command store: 2 banks x 64 entries, distributed RAM
    logic [15:0] mem [0:2*ENTRIES-1];

    // Per-bank length records
    logic [ADDR_W-1:0] loop_idx_r [0:1];
    logic [ADDR_W-1:0] end_idx_r  [0:1];

    // Per-slot play state
    logic              active_bank;
    logic [ADDR_W-1:0] index;
    logic [15:0]       cmd_reg;

    // Power-on contents replicate today's static aux command for this slot:
    // CONVERT(32+s) = {2'b00, 6'(32+s), 8'h00}, single-entry loop.
    initial begin
        for (int e = 0; e < 2*ENTRIES; e++)
            mem[e] = {2'b00, 6'(32 + s), 8'h00};
    end

    // Program write port (mem intentionally not reset -- LUTRAM)
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

    // Bank swap + index sequencing
    always_ff @(posedge clk) begin
        if (!rstn) begin
            active_bank <= 1'b0;
            index       <= '0;
        end else if (seq_hold) begin
            // Idle/disabled: swaps apply immediately, program parks at entry 0
            active_bank <= bank_select[s];
            index       <= '0;
        end else if (seq_advance) begin
            if (bank_select[s] != active_bank) begin
                // Atomic swap at the packet boundary; new program starts at 0
                active_bank <= bank_select[s];
                index       <= '0;
            end else if ((s == NSLOTS-1) && inject_active) begin
                // Injection consumed this slot's position: hold the program
                index <= index;
            end else begin
                index <= (index == end_idx_r[active_bank])
                         ? loop_idx_r[active_bank]
                         : index + 1'b1;
            end
        end
    end

    // Registered combinational read: settles 1 clock after any index/bank
    // change, ~2500 clocks before the command is serialized.
    always_ff @(posedge clk) begin
        cmd_reg <= mem[{active_bank, index}];
    end

    assign bank_active[s] = active_bank;
    assign slot_indices[s*ADDR_W +: ADDR_W] = index;

    if (s == NSLOTS-1) begin : g_inject_mux
        assign aux_cmds[s*16 +: 16] = inject_active ? inject_cmd : cmd_reg;
    end else begin : g_plain
        assign aux_cmds[s*16 +: 16] = cmd_reg;
    end

end
endgenerate

// One-shot injection control: arm any time; the request takes effect for the
// packet that begins at the next boundary, then self-clears.
always_ff @(posedge clk) begin
    if (!rstn) begin
        inject_pending <= 1'b0;
        inject_active  <= 1'b0;
    end else if (seq_hold) begin
        inject_pending <= 1'b0;
        inject_active  <= 1'b0;
    end else if (seq_advance) begin
        inject_active  <= inject_pending;       // consume: next packet is the injected one
        inject_pending <= inject_req;           // a request landing on the boundary re-arms
    end else if (inject_req) begin
        inject_pending <= 1'b1;
    end
end

endmodule
