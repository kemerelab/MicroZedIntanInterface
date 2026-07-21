// Implemented as 3 major always blocks.
// 1. Run the master cycled state machine. Maintain the timestamp consistently
//    regardless of whether we acquiring/transmitting data. Process control data 
//    (which comes from the AXI interface), like enable/disable transmission and
//    reset timestamps.
// 2. Run the data acquisiton state machine. This uses the cycles/states controlled
//    by state machine #1.
// 3. Run the data exfiltration state machine. This loads data, prefaced by a
//    a header and a timestamp, into a FIFO for transmission via the dual port BRAM
//    to the PS. (FIFO and BRAM are external to this file.)

module data_generator_core (
    input  logic        clk,
    input  logic        rstn,

    // Control and status interfaces. 25 control registers (see main.h):
    //   0..3   run control, channel/phase config, chirp config (CTRL_REG_3)
    //   4..21  the 36-word COPI command table (host-set per-channel CONVERTs)
    //   22..24 the aux command engine (decoded inside aux_command_engine)
    input  logic [32*25-1:0] ctrl_regs_pl,
    output logic [32*10-1:0]  status_regs_pl,  // 10 regs here; the wrapper appends reg 11 (FIFO/BRAM)
    // Aux engine status (the wrapper maps these to status regs 11/12)
    output logic [31:0] aux_status,
    output logic [31:0] aux_read_result,
    
    // FIFO interface (128-bit = up to 8 x 16-bit segments: 2 SPI ports x
    // {regular,DDR} x {CIPO0,CIPO1}; gets packed to 32-bit for BRAM)
    output logic        fifo_write_en,
    output logic [127:0] fifo_write_data,
    output logic [7:0]  fifo_channel_mask,     // Which 16-bit segments are valid

    input  logic        fifo_full,
    input  logic [8:0]  fifo_count,
    
    output logic        fifo_packet_end_flag, // gets written with each word. 1 if it's the last word in a packet
        
    // Serial interface signals
    output logic        csn,        // Chip select (active low)
    output logic        sclk,       // Serial clock
    output logic        copi,       // Controller Out, Peripheral In
    input  logic        cipo_a0,      // Port 0 (cable A) Controller In, Peripheral Out 0
    input  logic        cipo_a1,      // Port 0 (cable A) Controller In, Peripheral Out 1
    input  logic        cipo_b0,      // Port 1 (cable B) CIPO0  (dual-port; tie 0 if unused)
    input  logic        cipo_b1,      // Port 1 (cable B) CIPO1

    // External digital input
    input  logic [7:0]  digital_in,

    // DSP tap: the de-interleaved per-slot sample stream, for the on-PL LFP/DSP
    // engine. Pulses on each data-word write (NOT the header), carrying the same
    // 8x16-bit lanes as fifo_write_data with the slot index (cycle_counter) and a
    // once-per-packet tick. Offset-binary->signed conversion + amplifier-slot
    // gating happen downstream in lfp_dsp_block. See docs/lfp-dsp-engine-design.md.
    output logic        dsp_sample_valid,
    output logic [127:0] dsp_sample_data,
    output logic [5:0]  dsp_sample_slot,
    output logic        dsp_packet_tick,
    // Live 64-bit master sample count, exposed so the LFP engine can stamp each
    // decimated frame with the master timestamp of the LAST broadband sample that
    // contributed to it (latched in lfp_dsp_block on the decimation tick). This is
    // the SAME counter the broadband packet header stamps (header word 1).
    output logic [63:0] dsp_master_timestamp,
    // Broadband channel-enable mask (single source of truth for the LFP lane
    // mask -- the LFP filters exactly the broadband-enabled lanes).
    output logic [7:0]  dsp_channel_enable
);

import acq_frame_pkg::*;   // frame geometry + RHD command encoding, single source of truth

// Extract control bits
wire enable_transmission = ctrl_regs_pl[0*32 + 0];

// Declared ahead of first use (the safe-control block below) so the file also
// elaborates under xsim, which rejects use-before-declaration.
logic transmission_active;

// Safe control registers - only updated when transmission is not active
logic reset_timestamp_reg;
logic debug_mode_reg;
logic [31:0] loop_count_reg;
logic [3:0] phase_a0_reg;
logic [3:0] phase_a1_reg;
logic [3:0] phase_b0_reg;       // port 1 (cable B) CIPO0 cable-delay phase
logic [3:0] phase_b1_reg;       // port 1 (cable B) CIPO1 cable-delay phase
logic [7:0] channel_enable_reg;  // [3:0] = port 0 streams, [7:4] = port 1 streams
// Protected COPI message words (36 x 16-bit words) - only updated when transmission inactive
logic [15:0] copi_words_reg [0:35];

// The chirp/synthetic-signal config (CTRL_REG_3) is decoded and latched inside
// test_signal_gen (instantiated below); the core just forwards the raw register.
// debug_mode (CTRL_REG_0[3]) stays here -- it is the synthetic-vs-real data-path
// select, applied at the FIFO data write, not a test_signal_gen field.

// Safe control register updates - only when transmission is not active
always_ff @(posedge clk) begin
    if (!rstn) begin
        reset_timestamp_reg <= 1'b0;
        debug_mode_reg <= 1'b0;
        loop_count_reg <= 32'd0;
        phase_a0_reg <= 4'd0;
        phase_a1_reg <= 4'd0;
        phase_b0_reg <= 4'd0;
        phase_b1_reg <= 4'd0;
        channel_enable_reg <= 8'b0000_1111;  // Default: cable A all 4 streams on, cable B off
        
        // Initialize COPI words to safe defaults
        for (int j = 0; j < 36; j++) begin
            copi_words_reg[j] <= 16'h0;
        end
    end else begin
        // Only update control registers when transmission is not active
        if (!transmission_active) begin
            reset_timestamp_reg <= ctrl_regs_pl[0*32 + 1];
            debug_mode_reg <= ctrl_regs_pl[0*32 + 3];
            loop_count_reg <= ctrl_regs_pl[1*32 +: 32];
            // CTRL_REG_2 layout. channel_enable is 8 bits at [15:8]; a host that
            // writes only the low nibble [11:8] leaves cable-B streams ([15:12])
            // at 0, i.e. cable A only.
            //   [3:0] phase_a0, [7:4] phase_a1, [15:8] channel_enable,
            //   [19:16] phase_b0, [23:20] phase_b1
            phase_a0_reg <= ctrl_regs_pl[2*32 + 3  : 2*32 + 0];
            phase_a1_reg <= ctrl_regs_pl[2*32 + 7  : 2*32 + 4];
            channel_enable_reg <= ctrl_regs_pl[2*32 + 8 +: 8];
            phase_b0_reg <= ctrl_regs_pl[2*32 + 16 +: 4];
            phase_b1_reg <= ctrl_regs_pl[2*32 + 20 +: 4];
            
            // Update COPI words from control registers 4-21 (18 registers total)
            for (int j = 0; j < 18; j++) begin
                copi_words_reg[2*j]     <= ctrl_regs_pl[(j+4)*32 +: 16];      // Low 16 bits
                copi_words_reg[2*j + 1] <= ctrl_regs_pl[(j+4)*32 + 16 +: 16]; // High 16 bits
            end
        end
    end
end

// CIPO received data storage (4 separate 16-bit registers per cycle)
logic [31:0] cipo_a0_data [0:34];  // Port 0 CIPO0 line, register A (low 16 bits) and B (upper 16 bits)
logic [31:0] cipo_a1_data [0:34];  // Port 0 CIPO1 line
logic [31:0] cipo_b0_data [0:34];  // Port 1 CIPO0 line (dual-port)
logic [31:0] cipo_b1_data [0:34];  // Port 1 CIPO1 line

// Registers for COPI data from the 4 CIPO lines (2 per port)
reg [73:0] cipo_a0_4x_oversampled;
reg [73:0] cipo_a1_4x_oversampled;
reg [73:0] cipo_b0_4x_oversampled;
reg [73:0] cipo_b1_4x_oversampled;
reg [31:0] cipo_a0_phase_selected;
reg [31:0] cipo_a1_phase_selected;
reg [31:0] cipo_b0_phase_selected;
reg [31:0] cipo_b1_phase_selected;

// One phase selector per CIPO line: each picks the sample point that compensates
// the round-trip SCLK->CIPO cable delay. Every line has its own phase (cable A =
// a0/a1, cable B = b0/b1) since the two cables can be different lengths.
CIPO_combined_phase_selector cipo_a0_selector(
    .phase_select(phase_a0_reg),
    .CIPO4x(cipo_a0_4x_oversampled),
    .CIPO(cipo_a0_phase_selected)
);
CIPO_combined_phase_selector cipo_a1_selector(
    .phase_select(phase_a1_reg),
    .CIPO4x(cipo_a1_4x_oversampled),
    .CIPO(cipo_a1_phase_selected)
);
CIPO_combined_phase_selector cipo_b0_selector(
    .phase_select(phase_b0_reg),
    .CIPO4x(cipo_b0_4x_oversampled),
    .CIPO(cipo_b0_phase_selected)
);
CIPO_combined_phase_selector cipo_b1_selector(
    .phase_select(phase_b1_reg),
    .CIPO4x(cipo_b1_4x_oversampled),
    .CIPO(cipo_b1_phase_selected)
);

// Control counters
logic [6:0] state_counter;
logic [5:0] cycle_counter;

// Unified common-header constants (identical across all PL streams).
localparam logic [31:0] UNIFIED_MAGIC       = 32'hCAFEBABE;  // header word 0
localparam logic [7:0]  STREAM_TYPE_BROADBAND = 8'd1;
localparam logic [7:0]  UNIFIED_VERSION     = 8'd1;
// TYPE_VER (header word 1) = stream_type[7:0] | version[15:8] | flags[31:16].
// Broadband sets no flags (0).
localparam logic [31:0] BB_TYPE_VER =
    {16'd0, UNIFIED_VERSION, STREAM_TYPE_BROADBAND};
logic [63:0] timestamp;

// Status tracking (transmission_active is declared near the top of the module)
logic [31:0] packets_sent;
logic        loop_limit_reached;
logic [31:0] loop_counter;

// Per-stream broadband sequence number (header word 4). Monotonic +1 per emitted
// broadband packet so the host can prove zero loss. Independent of packets_sent
// (a status counter); this one is stamped into the wire header. It is sampled at
// the packet start (state 0/cycle 0 of the FIFO header write) and advanced at the
// packet end, so each packet carries a unique, gap-free value.
logic [31:0] bb_seq;          // value stamped into the current packet's header
logic [31:0] bb_seq_next;     // running counter (advances at packet end)

// Number of enabled stream lanes (popcount of channel_enable). Used only to size
// the packet: the header AUX0[23:8] carries bb_num_data_words = the count of
// 32-bit DATA words = ceil(35 slots * num_enabled_lanes / 2). Combinational off
// the transmission-stable channel_enable_reg.
logic [3:0]  num_enabled_lanes;
always_comb begin
    num_enabled_lanes = 4'd0;
    for (int b = 0; b < 8; b++)
        num_enabled_lanes = num_enabled_lanes + {3'd0, channel_enable_reg[b]};
end
// 35 * lanes fits in 9 bits (max 35*8 = 280); +1 then >>1 = round-up /2.
wire [15:0] bb_num_data_words = (16'(35 * num_enabled_lanes) + 16'd1) >> 1;

// Synthetic data for the current slot -- fixed sine or swept chirp, produced by
// test_signal_gen. Used at the FIFO data write only when debug_mode is set.
logic [127:0] synth_lanes;

// Helper signals for state machine logic
wire is_last_state = (state_counter == 7'd79);
wire is_first_cycle = (cycle_counter == 6'd0);
wire is_last_cycle = (cycle_counter == LAST_CYC);

// Test-signal source. It decodes/latches its own config (CTRL_REG_3) and marches
// its sine index + chirp NCO once per emitted packet (pulsed here at the last
// state of the last cycle while transmitting with the FIFO not full).
wire synth_packet_advance = transmission_active && !fifo_full &&
                            is_last_cycle && is_last_state;
test_signal_gen test_signal_gen_inst (
    .clk                (clk),
    .rstn               (rstn),
    .packet_advance     (synth_packet_advance),
    .cycle_counter      (cycle_counter),
    .chirp_cfg_reg      (ctrl_regs_pl[3*32 +: 32]),
    .transmission_active(transmission_active),
    .lanes              (synth_lanes)
);

// ============================================================================
// AUX COMMAND ENGINE + OVERRIDE (always on; un-programmed => plain per-slot CONVERTs)
// ============================================================================
// Aux control registers 22..24 are decoded INSIDE aux_command_engine (bank
// select, fast-settle/DSP/digout config, the write port, inject/write toggles).
// The framing loop just forwards the three raw register words to the engine.

// Packet strobes. packet_start = first state of each transmitted packet (same
// instant digital_in is latched); seq_advance = last state (use-then-advance,
// so the first packet after start plays bank entry 0).
wire packet_start = transmission_active && is_first_cycle && (state_counter == 7'd0);
wire seq_advance  = transmission_active && is_last_cycle  && is_last_state;

logic [N_AUX*16-1:0] aux_cmds_final;   // final post-override aux commands (cycles 32..34)
logic [N_AUX-1:0]    aux_bank_active;
logic [N_AUX*6-1:0]  aux_slot_indices;
logic                aux_inject_active;
logic                aux_dsp_force_h, aux_fs_active, aux_digout_state;

// Aux is ALWAYS ON. The engine wires a fixed RT register (slot 0) plus two cycling
// programs (slots 1,2, aux_program) through the override. Every source powers up
// as CONVERT(AUX_CYC0+slot), so an un-programmed engine simply emits the plain
// per-slot CONVERTs until a program or override is configured.
aux_command_engine #(.ADDR_W(6)) aux_engine_inst (
    .clk                (clk),
    .rstn               (rstn),
    .seq_advance        (seq_advance),
    .packet_start       (packet_start),
    .transmission_active(transmission_active),
    .digital_in         (digital_in),
    // The engine decodes these three raw registers itself.
    .aux_ctrl_reg       (ctrl_regs_pl[22*32 +: 32]),
    .aux_write_reg      (ctrl_regs_pl[23*32 +: 32]),
    .aux_strobe_reg     (ctrl_regs_pl[24*32 +: 32]),
    .aux_cmds           (aux_cmds_final),
    .dsp_force_h        (aux_dsp_force_h),
    .fast_settle_active (aux_fs_active),
    .digout_state       (aux_digout_state),
    .inject_active      (aux_inject_active),
    .bank_active        (aux_bank_active),
    .slot_indices       (aux_slot_indices)
);

// Command-echo identity: the header echoes each aux command so the host can pair
// it with its reply. SPI readback is +2 cycles, so the reply to the command at
// cycle C lands at cycle (C+2) mod 35:
//   fs slot     (cycle 32) -> reply in data word 34 of THIS packet
//   plain slot  (cycle 33) -> reply at cycle 0 of the NEXT packet
//   inject slot (cycle 34) -> reply at cycle 1 of the NEXT packet
// So the header carries this packet's fs-slot command (header word 6) plus the
// PREVIOUS packet's plain- and inject-slot commands (word 8) -- each reply is
// fully labeled by the command that produced it.
logic [15:0] echo_plain_prev, echo_inject_prev;   // prev packet's plain/inject commands
logic        echo_valid;           // 0 for the first packet after start (no "prev" yet)
logic        inject_result_pkt;    // the cycle-1 reply of THIS packet answers an injection
always_ff @(posedge clk) begin
    if (!rstn) begin
        echo_plain_prev   <= 16'h0;
        echo_inject_prev  <= 16'h0;
        echo_valid        <= 1'b0;
        inject_result_pkt <= 1'b0;
    end else if (!transmission_active) begin
        echo_valid        <= 1'b0;
        inject_result_pkt <= 1'b0;
    end else if (seq_advance) begin
        echo_plain_prev   <= aux_cmds_final[AUX_PLAIN_SLOT*16  +: 16];
        echo_inject_prev  <= aux_cmds_final[AUX_INJECT_SLOT*16 +: 16];
        echo_valid        <= 1'b1;
        inject_result_pkt <= aux_inject_active;
    end
end

// Runtime register READ/WRITE readback. NOTE: this is NOT fast settle -- fast
// settle is the override whole-replacing the fs slot (handled in the engine).
// This is the separate "inject" path: when the firmware injects a one-shot RHD
// READ/WRITE command into the inject slot (cycle 34, e.g. to read the chip ID or
// a register), the chip's reply arrives +2 cycles later at cycle 1 of the NEXT
// packet (AUX_INJECT_REPLY_CYC), settled at AUX_INJECT_REPLY_STATE. Latch it into
// aux_read_result (the firmware's READ_REGISTER path) and flip the ack toggle so
// the firmware knows the result landed. [15:0] of each CIPO word = regular (non-DDR).
logic        aux_inj_ack;
logic [31:0] aux_read_result_reg;
always_ff @(posedge clk) begin
    if (!rstn) begin
        aux_inj_ack         <= 1'b0;
        aux_read_result_reg <= 32'h0;
    end else if (transmission_active && inject_result_pkt &&
                 (cycle_counter == AUX_INJECT_REPLY_CYC) &&
                 (state_counter == AUX_INJECT_REPLY_STATE)) begin
        aux_read_result_reg <= {cipo_a1_data[1][15:0], cipo_a0_data[1][15:0]};
        aux_inj_ack         <= ~aux_inj_ack;
    end
end

// Packet metadata flags (header word 2 bits [15:8]). The aux engine is always on.
logic [7:0] aux_flags;
assign aux_flags = {2'b00,
                    inject_result_pkt,   // [5]
                    echo_valid,          // [4]
                    aux_dsp_force_h,     // [3]
                    aux_digout_state,    // [2]
                    aux_fs_active,       // [1]
                    1'b1};               // [0] aux engine active (always)

// State machine and control logic 
always_ff @(posedge clk) begin
    if (!rstn) begin
        state_counter <= 7'd0;
        cycle_counter <= 6'd0;
        timestamp <= 64'd0;
        transmission_active <= 1'b0;
        loop_limit_reached <= 1'b0;
        loop_counter <= 32'd1; // 1 indexed
    end else begin        
        // State machine goes from 0 to 79, then repeats
        if (is_last_state) begin
            state_counter <= 7'd0;
            if (is_last_cycle) begin
                cycle_counter <= 6'd0;

                if (!enable_transmission && reset_timestamp_reg) begin
                    timestamp <= 64'd0;
                end else begin
                    timestamp <= timestamp + 1; // timestamp increments whether transmitting or not
                end

                if (!enable_transmission) begin // either this just happened or is still true
                    transmission_active <= 1'b0;
                    loop_limit_reached <= 1'b0; 
                end

                if (transmission_active) begin
                    if (loop_limit_reached) begin
                        transmission_active <= 1'b0;
                    end
                    loop_counter <= loop_counter + 1;
                    loop_limit_reached <= (loop_count_reg != 32'd0) && (loop_counter >= loop_count_reg);

                end else begin // transmission is not currently active
                    if (enable_transmission && !loop_limit_reached) begin
                        loop_counter <= 32'd1;  // Reset when starting new transmission
                        loop_limit_reached <= (loop_count_reg != 32'd0) && (loop_count_reg <= 32'd1); // Catch the tricky single transmission case
                        transmission_active <= 1'b1;
                    end
                end

            end else begin
                cycle_counter <= cycle_counter + 1;
            end
        end else begin
            state_counter <= state_counter + 1;
        end
    end
end

/*
Complete Serial Protocol Timing (80-state machine):

State 0:  CSn=0, SCLK=0, COPI=0 (default) [first of 35 cycles - fifo enqueue magic header words]
State 1:  CSn=0, SCLK=0, COPI=copi_words[cycle_counter][15] (setup bit 15) 
State 2:  CSn=0, SCLK=1, COPI=copi_words[cycle_counter][15] (clock bit 15) [first of 35 cycles - fifo enqueue timestamp words]
State 3:  CSn=0, SCLK=1, COPI=copi_words[cycle_counter][15] (hold)
State 4:  CSn=0, SCLK=0, COPI=copi_words[cycle_counter][15] (transition)
State 5:  CSn=0, SCLK=0, COPI=copi_words[cycle_counter][14] (setup bit 14)
State 6:  CSn=0, SCLK=1, COPI=copi_words[cycle_counter][14] (clock bit 14)
State 7:  CSn=0, SCLK=1, COPI=copi_words[cycle_counter][14] (hold)
...
State 57: CSn=0, SCLK=0, COPI=copi_words[cycle_counter][1] (setup bit 1)
State 58: CSn=0, SCLK=1, COPI=copi_words[cycle_counter][1] (clock bit 1)
State 59: CSn=0, SCLK=1, COPI=copi_words[cycle_counter][1] (hold)
State 60: CSn=0, SCLK=0, COPI=copi_words[cycle_counter][1] (transition)
State 61: CSn=0, SCLK=0, COPI=copi_words[cycle_counter][0] (setup bit 0)
State 62: CSn=0, SCLK=1, COPI=copi_words[cycle_counter][0] (clock bit 0 - LAST RISING EDGE)
State 63: CSn=0, SCLK=1, COPI=copi_words[cycle_counter][0] (hold - LAST CLOCK HIGH)
State 64: CSn=0, SCLK=0, COPI=copi_words[cycle_counter][0] (LAST FALLING EDGE)
State 65: CSn=0, SCLK=0, COPI=copi_words[cycle_counter][0] (hold low)

*** CSn GOES HIGH HERE ***
State 66: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 67: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 68: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 69: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 70: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 71: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 72: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 73: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 74: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 75: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO)  
State 76: CSn=1, SCLK=0, COPI=0 (register buffer data from phase selector)
State 77: CSn=1, SCLK=0, COPI=0 (inactive) [fifo enqueue 64b of combined CIPO data]
State 78: CSn=1, SCLK=0, COPI=0 (inactive)
State 79: CSn=1, SCLK=0, COPI=0 (inactive)

Key Timing:
- CSn active: States 0-65 (66 states total)  
- 16 clocks: Rising edges at states 2,6,10,14,18,22,26,30,34,38,42,46,50,54,58,62
- Last clock high: State 63
- Last falling edge: State 64
- CSn goes HIGH: State 66
- Inactive period: States 66-79 (14 states)
*/

// Serial interface control - CSn, SCLK, and COPI generation 
always_ff @(posedge clk) begin
    if (!rstn) begin
        csn <= 1'b1;           // Default high (inactive)
        sclk <= 1'b0;          // Default low
        copi <= 1'b0;          // Default low
    end else begin
        // Default values (used when not transmitting or not in protocol)
        csn <= 1'b1;           // CSn high when not in protocol
        sclk <= 1'b0;          // SCLK low when not active
        copi <= 1'b0;          // COPI low when not active
        
        if (transmission_active) begin
            if (state_counter <= 7'd65) begin
                // CSn goes low during protocol (states 0-65)
                csn <= 1'b0;
                
                // SCLK is 1/4th the rate of the master clock, and there are 16 clock cycles
                // Clock high when bit 1 is set and state <= 63
                if ((state_counter[1] == 1'b1) && (state_counter <= 7'd63)) begin
                    sclk <= 1'b1;
                end
            end
                
            // COPI data transmission - MSB first, set on states 0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60
            // Uses copi_words_reg[cycle_counter] as the source for each cycle's transmission  
            // Bit index is just the bitwise NOT of state_counter[5:2] (since 15-x = ~x for 4-bit x)

            if (state_counter <= 7'd63) begin   // COPI bits are shifted out during states 0..63
                logic [3:0]  bit_index;
                logic [1:0]  aux_slot;
                logic [15:0] tx_word;
                bit_index = ~state_counter[5:2];  // MSB first: ~0=15, ~1=14, ..., ~15=0
                // Aux slot index for the aux cycles (AUX_CYC0..LAST_CYC -> 0..N_AUX-1).
                aux_slot = (cycle_counter >= N_CHAN_CMDS)
                         ? 2'(cycle_counter - AUX_CYC0) : 2'd0;
                // Command source: aux cycles come from the always-on aux engine, the
                // channel cycles (0..N_CHAN_CMDS-1) from the host COPI table.
                if (cycle_counter >= N_CHAN_CMDS)
                    tx_word = aux_cmds_final[aux_slot*16 +: 16];
                else
                    tx_word = copi_words_reg[cycle_counter];
                // DSP reset ("digital fast settle"): force bit H on the channel
                // CONVERTs (cycles 0..N_CHAN_CMDS-1) while requested.
                if (aux_dsp_force_h &&
                    (cycle_counter < N_CHAN_CMDS) && (tx_word[15:14] == 2'b00))
                    tx_word[0] = 1'b1;
                copi <= tx_word[bit_index];
            end
            
        end
    end
end

// CIPO data sampling - 8 registers total (2 per input line, 2 lines per port)
always_ff @(posedge clk) begin
    if (!rstn) begin
        // Reset all received data
        for (int j = 0; j < 35; j++) begin
            cipo_a0_data[j] <= 32'h0;
            cipo_a1_data[j] <= 32'h0;
            cipo_b0_data[j] <= 32'h0;
            cipo_b1_data[j] <= 32'h0;
        end
        cipo_a0_4x_oversampled <= 74'h0;
        cipo_a1_4x_oversampled <= 74'h0;
        cipo_b0_4x_oversampled <= 74'h0;
        cipo_b1_4x_oversampled <= 74'h0;
    end else begin
        if (transmission_active && (state_counter >= 7'd2) && (state_counter <= 75)) begin
            cipo_a0_4x_oversampled[state_counter - 2] <= cipo_a0; // Latch data into the phase selector input
            cipo_a1_4x_oversampled[state_counter - 2] <= cipo_a1;
            cipo_b0_4x_oversampled[state_counter - 2] <= cipo_b0;
            cipo_b1_4x_oversampled[state_counter - 2] <= cipo_b1;
        end else if(transmission_active && state_counter == 7'd76) begin
            cipo_a0_data[cycle_counter] <= cipo_a0_phase_selected; // Get the phase selector output
            cipo_a1_data[cycle_counter] <= cipo_a1_phase_selected; // It's ready one clock cycle after being latched in
            cipo_b0_data[cycle_counter] <= cipo_b0_phase_selected;
            cipo_b1_data[cycle_counter] <= cipo_b1_phase_selected;
        end
    end
end


// Digital input
// Register for latching digital inputs
logic [7:0] digital_in_latched;

// Latch digital inputs at the start of each packet. (The aux engine's override
// samples digital_in at this same instant for its fast-settle / digout GPIO
// triggers.)

always_ff @(posedge clk) begin
    if (!rstn) begin
        digital_in_latched <= 8'h0;
    end else begin
        if (transmission_active && is_first_cycle && state_counter == 7'd0) begin
            // Latch digital inputs at the beginning of each packet
            digital_in_latched <= digital_in;
        end
    end
end


// Data-to-BRAM processing
always_ff @(posedge clk) begin
    if (!rstn) begin
        fifo_write_en <= 1'b0;
        fifo_write_data <= 128'h0;
        fifo_channel_mask <= 8'h0;
        packets_sent <= 32'd0;
        bb_seq      <= 32'd0;
        bb_seq_next <= 32'd0;
        fifo_packet_end_flag <= 1'b0;

        dsp_sample_valid <= 1'b0;
        dsp_sample_slot  <= 6'd0;
        dsp_packet_tick  <= 1'b0;
    end else begin
        // Default: no FIFO write
        fifo_write_en <= 1'b0;
        // DSP tap defaults (single-cycle pulses)
        dsp_sample_valid <= 1'b0;
        dsp_packet_tick  <= 1'b0;

        if (transmission_active && !fifo_full) begin
            // ---- Unified packet header (broadband, stream_type=1) -------------
            // An 8-word common header + a 6-word broadband sub-block, written as
            // 7 x 64-bit FIFO writes (states 0..6 of cycle 0), one 64-bit value per
            // state -> 14 BRAM words ahead of the data. See
            // docs/unified-packet-format.md.
            //
            // BRAM word layout (LE), per 64-bit write {high32, low32}:
            //   write0 (w0/w1):  MAGIC=0xCAFEBABE           | TYPE_VER=1|ver<<8|flags<<16
            //   write1 (w2/w3):  TS_LO                      | TS_HI
            //   write2 (w4/w5):  SEQ (broadband)            | AUX0=ce|num_data_words<<8
            //   write3 (w6/w7):  AUX1=digital/flags/fs-echo | RSVD=0
            //   write4 (w8/w9):  plain+inject echoes        | analog ch0-1   (sub-block)
            //   write5 (w10/w11):analog ch2-3              | analog ch4-5
            //   write6 (w12/w13):analog ch6-7              | reserved=0
            // Latch the per-packet broadband seq at the packet start, before it is
            // stamped into header word 4 (FIFO state 2). bb_seq_next advances at
            // the packet end below, so consecutive packets get +1 with no gap.
            if (is_first_cycle && state_counter == 7'd0)
                bb_seq <= bb_seq_next;

            if (state_counter inside {7'd0, 7'd1, 7'd2, 7'd3, 7'd4, 7'd5, 7'd6}) begin
                if (is_first_cycle) begin
                    fifo_write_en <= 1'b1;
                    // Header is one 64-bit value -> exactly the low 4 segments;
                    // the upper 4 (cable B streams) are masked off, as the header
                    // never carries cable-B data.
                    fifo_channel_mask <= 8'b0000_1111;
                    fifo_packet_end_flag <= 1'b0;  // Header words are never at the end
                    case (state_counter)
                        // ---- common 8-word header (words 0..7) ----
                        7'd0: fifo_write_data <= {BB_TYPE_VER, UNIFIED_MAGIC};
                        7'd1: fifo_write_data <= timestamp;             // TS_LO | TS_HI
                        // SEQ (w4) | AUX0 (w5) = channel_enable | num_data_words<<8
                        7'd2: fifo_write_data <= {
                            {8'd0, bb_num_data_words[15:0], channel_enable_reg}, // w5 AUX0
                            bb_seq};                                            // w4 SEQ
                        // AUX1 (w6) = digital inputs + aux flags + this packet's
                        // fs-slot command echo:
                        //   [7:0] digital_in, [15:8] aux_flags,
                        //   [31:16] fs-slot command (its reply is data word 34).
                        // RSVD (w7) = 0.
                        7'd3: fifo_write_data <= {
                            32'h0,                                              // w7 RSVD
                            aux_cmds_final[AUX_FS_SLOT*16 +: 16],               // w6[31:16] fs-slot echo
                            aux_flags,                                          // w6[15:8]
                            digital_in_latched};                                // w6[7:0]
                        // ---- broadband sub-block (words 8..13) ----
                        // w8 = the previous packet's plain- and inject-slot command
                        //      echoes (their replies land at cycles 0/1 of THIS
                        //      packet); w9 = analog ch0-1 breadcrumb (0). w10..w13 =
                        //      the 8 external-ADC breadcrumbs (currently 0) + reserved.
                        7'd4: fifo_write_data <= {
                            32'h0,                                              // w9 analog ch0-1 (breadcrumb)
                            echo_inject_prev,                                   // w8[31:16] prev inject-slot echo
                            echo_plain_prev};                                   // w8[15:0]  prev plain-slot echo
                        7'd5: fifo_write_data <= 64'h0;  // w10 analog ch2-3 | w11 analog ch4-5
                        7'd6: fifo_write_data <= 64'h0;  // w12 analog ch6-7 | w13 reserved
                    endcase
                end
            end
            
            // Data writes - Pack all four CIPO lines into one 128-bit write with
            // the 8-bit channel mask. Each line contributes {regular, DDR} = 2x16.
            // Segment order (low->high):
            //   cable A: cipo_a0{reg,ddr}, cipo_a1{reg,ddr}   (low 64  = bits[63:0])
            //   cable B: cipo_b0{reg,ddr}, cipo_b1{reg,ddr}   (high 64 = bits[127:64])
            // channel_enable_reg[7:4]==0 masks off the high 64 bits (cable A only).
            if (state_counter == 7'd77) begin
                fifo_write_en <= 1'b1;
                fifo_channel_mask <= channel_enable_reg;  // 8-bit channel enable
                fifo_packet_end_flag <= is_last_cycle;    // Only last cycle's data word ends the packet
                // DSP tap: this is a data-word write -> pulse the engine with the
                // slot index. fifo_write_data (set below) carries the 8 lanes and
                // is exposed combinationally as dsp_sample_data.
                dsp_sample_valid <= 1'b1;
                dsp_sample_slot  <= cycle_counter;

                if (!debug_mode_reg) begin
                    // Real CIPO data, both ports.
                    fifo_write_data <= {cipo_b1_data[cycle_counter], cipo_b0_data[cycle_counter],
                                        cipo_a1_data[cycle_counter], cipo_a0_data[cycle_counter]};
                end else begin
                    // Synthetic data: test_signal_gen produces the 8 lanes for this
                    // slot (fixed sine or swept chirp, its own config), settled well
                    // before this state.
                    fifo_write_data <= synth_lanes;
                end
            end
                    
            if (is_last_cycle) begin
                if (is_last_state) begin
                    packets_sent <= packets_sent + 1;
                    bb_seq_next  <= bb_seq_next + 1;  // per-stream broadband seq
                    // (test_signal_gen advances its own sine index + chirp NCO off
                    // synth_packet_advance, which pulses on this same edge.)
                    // DSP tap: the packet is complete -> advance the engine's ring
                    // (one tick per packet, after all 35 slot samples are ingested).
                    dsp_packet_tick <= 1'b1;
                end
            end

        end else if (!transmission_active) begin
            // Reset the broadband sequence at the start of every streaming session
            // (mirrors the host clearing its expected-seq on START), so the first
            // packet of a fresh stream is always SEQ=0. fifo_full does NOT clear it.
            bb_seq      <= 32'd0;
            bb_seq_next <= 32'd0;
        end
    end
end

// DSP tap data: the registered 128-bit data word (valid when dsp_sample_valid).
assign dsp_sample_data = fifo_write_data;

// Master timestamp + broadband channel-enable taps for the LFP engine. The LFP
// engine latches dsp_master_timestamp on its decimation tick (so each LFP frame
// carries the master count of the last contributing broadband sample) and drives
// its lane_mask from dsp_channel_enable (mirror of the broadband mask).
assign dsp_master_timestamp = timestamp;
assign dsp_channel_enable   = channel_enable_reg;

// Pack status signals
// Status Register 0: Dynamic status and counters (locally generated)
assign status_regs_pl[0*32 +: 32] = {
    15'd0,                // [31:17] - reserved for future flags
    cycle_counter,        // [16:11] - 6 bits
    1'b0,                 // [10] - reserved  
    state_counter,        // [9:3] - 7 bits
    1'b0,                 // [2] - reserved for future flags
    loop_limit_reached,   // [1] - 1 bit
    transmission_active   // [0] - 1 bit
};

// Status Register 1: Reflected control parameters (registered versions).
// channel_enable[3:0] stays at [23:20] (unchanged position for existing
// firmware/host); the new port-1 nibble channel_enable[7:4] goes in the
// former reserved [27:24]. phase_b0/phase_b1 are read back via the CTRL_REG_2
// mirror (status reg 8).
assign status_regs_pl[1*32 +: 32] = {
    4'd0,                     // [31:28] - reserved
    channel_enable_reg[7:4],  // [27:24] - port-1 channel enable
    channel_enable_reg[3:0],  // [23:20] - port-0 channel enable
    phase_a1_reg,               // [19:16] - 4 bits
    phase_a0_reg,               // [15:12] - 4 bits
    8'd0,                     // [11:4] - reserved
    debug_mode_reg,           // [3] - 1 bit
    1'b0,                     // [2] - reserved
    reset_timestamp_reg,      // [1] - 1 bit
    enable_transmission   // [0] - 1 bit (current value, not registered)
};

assign status_regs_pl[2*32 +: 32] = packets_sent;
assign status_regs_pl[3*32 +: 32] = timestamp[31:0];
assign status_regs_pl[4*32 +: 32] = timestamp[63:32];
assign status_regs_pl[5*32 +: 32] = loop_count_reg;
assign status_regs_pl[6*32 +: 32] = ctrl_regs_pl[0*32 +: 32]; // reflected
assign status_regs_pl[7*32 +: 32] = ctrl_regs_pl[1*32 +: 32]; // reflected
assign status_regs_pl[8*32 +: 32] = ctrl_regs_pl[2*32 +: 32]; // reflected
assign status_regs_pl[9*32 +: 32] = ctrl_regs_pl[3*32 +: 32]; // reflected

// Status register 11 will be added by wrapper (FIFO/BRAM). Aux sequencer
// status goes out via dedicated ports -> wrapper status regs 11/12:
//   aux_status: [2:0] bank_active, [3] aux engine active (always 1),
//               [4] fast_settle_active, [5] digout_state, [6] dsp_force_h,
//               [7] inject ack toggle, [13:8] slot-0 index, [21:16] slot-1
//               index, [29:24] slot-2 index.
//   aux_read_result: {cipo_a1_regular[15:0], cipo_a0_regular[15:0]} of the last
//               injected command's response (firmware READ_REGISTER path).
assign aux_status = {
    2'b00,
    aux_slot_indices[17:12],   // [29:24] slot-2 (cycle 34) index
    2'b00,
    aux_slot_indices[11:6],    // [21:16] slot-1 (cycle 33) index
    2'b00,
    aux_slot_indices[5:0],     // [13:8]  slot-0 (cycle 32) index
    aux_inj_ack,               // [7]
    aux_dsp_force_h,           // [6]
    aux_digout_state,          // [5]
    aux_fs_active,             // [4]
    1'b1,                      // [3] aux engine active (always on)
    aux_bank_active            // [2:0]
};
assign aux_read_result = aux_read_result_reg;

endmodule
