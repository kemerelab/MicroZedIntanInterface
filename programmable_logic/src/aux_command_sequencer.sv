// ============================================================================
// aux_command_sequencer.sv
// ----------------------------------------------------------------------------
// Programmable source for the three "auxiliary" COPI command positions
// (command cycles 32/33/34 of each packet). Replaces the fixed aux entries of
// copi_words_reg when aux_seq_en is asserted in data_generator_core.
//
// Three independent slots (Intan AuxCmd1/2/3 model), uniform hardware, roles
// assigned purely by what the host loads:
//   slot 0 -> real-time control (digout / fast settle home)
//   slot 1 -> ADC-only accelerometer sweep (10 kHz)
//   slot 2 -> config + measurements (banked)
//
// Per slot:
//   * a small inferred RAM (distributed/LUTRAM at these sizes) holding the
//     16-bit COPI commands, NBANKS banks deep;
//   * a looping index that advances once per packet (packet_start), wrapping
//     end_idx -> loop_idx;
//   * NBANKS preloaded banks with an atomic, packet-boundary bank swap so a
//     standby bank can be written while the active one is read;
//   * LENGTH BOUND TO THE BANK: the host writes a length record to address 0 of
//     each bank ({end_idx, loop_idx}); it is captured in per-bank registers so a
//     bank-select swaps the length together with the program. Commands live at
//     addresses 1..2^ADDR_W-1.
//
// Boundary uses flat packed busses (slot i occupies bit-field i) so it can be
// instantiated cleanly from the Verilog wrapper. Internals use a generate loop.
// ============================================================================

module aux_command_sequencer #(
    parameter integer N_SLOTS = 3,
    parameter integer ADDR_W  = 6,                 // 64 entries per bank (addr 0 = length record)
    parameter integer NBANKS  = 2,                 // banks per slot (>=2 for double-buffer)
    parameter integer BANK_W  = (NBANKS <= 1) ? 1 : $clog2(NBANKS)
)(
    input  wire                          clk,
    input  wire                          rstn,

    // Advance pulse: one cycle at the start of each packet/sample.
    input  wire                          packet_start,

    // Command outputs (combinational, stable across a packet). Slot i -> [i*16 +: 16].
    output wire [N_SLOTS*16-1:0]         aux_seq_cmd,
    // Current per-slot command index (for the command-echo / drop detection).
    output wire [N_SLOTS*ADDR_W-1:0]     index_out,

    // Host write port (driven later by the AXI control registers). One word per
    // strobe. wr_addr==0 writes the length record {end_idx,loop_idx}; wr_addr>0
    // writes a command word.
    input  wire                          wr_en,
    input  wire [1:0]                    wr_slot,   // which slot (0..N_SLOTS-1)
    input  wire [BANK_W-1:0]             wr_bank,
    input  wire [ADDR_W-1:0]             wr_addr,
    input  wire [15:0]                   wr_data,

    // Bank control / status (per slot, flat).
    input  wire [N_SLOTS*BANK_W-1:0]     bank_select,
    output wire [N_SLOTS*BANK_W-1:0]     bank_active
);

    localparam integer ENTRIES = (1 << ADDR_W);

    genvar gi;
    generate
        for (gi = 0; gi < N_SLOTS; gi = gi + 1) begin : slot

            // ---- storage ----
            // Command RAM: NBANKS banks overlaid by the high address bits.
            (* ram_style = "distributed" *)
            logic [15:0]        mem [0:NBANKS*ENTRIES-1];

            // Length record per bank, captured from address-0 writes.
            logic [ADDR_W-1:0]  loop_reg [0:NBANKS-1];
            logic [ADDR_W-1:0]  end_reg  [0:NBANKS-1];

            // ---- running state ----
            logic [BANK_W-1:0]  active_bank;
            logic [ADDR_W-1:0]  index;

            wire  [BANK_W-1:0]  sel_bank = bank_select[gi*BANK_W +: BANK_W];

            // ---- host write port ----
            always_ff @(posedge clk) begin
                if (wr_en && (wr_slot == gi[1:0])) begin
                    if (wr_addr == {ADDR_W{1'b0}}) begin
                        // length record at addr 0: {end_idx, loop_idx}
                        loop_reg[wr_bank] <= wr_data[ADDR_W-1:0];
                        end_reg [wr_bank] <= wr_data[2*ADDR_W-1:ADDR_W];
                    end else begin
                        mem[{wr_bank, wr_addr}] <= wr_data;
                    end
                end
            end

            // ---- index advance + atomic bank swap (once per packet) ----
            always_ff @(posedge clk) begin
                if (!rstn) begin
                    active_bank <= {BANK_W{1'b0}};
                    index       <= {{(ADDR_W-1){1'b0}}, 1'b1}; // 1: first command (addr 0 is length)
                end else if (packet_start) begin
                    active_bank <= sel_bank;
                    if (sel_bank != active_bank) begin
                        // swap: start the newly selected bank at its loop point
                        index <= loop_reg[sel_bank];
                    end else if (index == end_reg[active_bank]) begin
                        index <= loop_reg[active_bank];
                    end else begin
                        index <= index + 1'b1;
                    end
                end
            end

            // ---- outputs (combinational read of the active bank) ----
            assign aux_seq_cmd[gi*16 +: 16]        = mem[{active_bank, index}];
            assign index_out[gi*ADDR_W +: ADDR_W]  = index;
            assign bank_active[gi*BANK_W +: BANK_W] = active_bank;
        end
    endgenerate

endmodule
