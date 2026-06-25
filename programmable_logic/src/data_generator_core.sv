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

    // Control and status interfaces
    // Regs 0..21 are the legacy map; 22..24 configure the aux command
    // sequencer / override layer (see firmware/include/main.h).
    input  logic [32*25-1:0] ctrl_regs_pl,
    output logic [32*10-1:0]  status_regs_pl,  // Only 10 registers, including mirroring 4 control - wrapper adds 11th
    // Aux sequencer / override status (wrapper maps these to status regs 11/12)
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
    input  logic        cipo0,      // Port 0 (cable A) Controller In, Peripheral Out 0
    input  logic        cipo1,      // Port 0 (cable A) Controller In, Peripheral Out 1
    input  logic        cipo2,      // Port 1 (cable B) CIPO0  (dual-port; tie 0 if unused)
    input  logic        cipo3,      // Port 1 (cable B) CIPO1

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
    output logic [7:0]  dsp_channel_enable,

    // Accel tap for the movement extractor: the rotating slot-1 accel CONVERT
    // echo (echo_slot2_prev) that labels the accel result at data word 0, plus a
    // valid bit (aux sequencer on AND echo established). accel_extract_block uses
    // this to de-interleave the 3 axes -- the SAME echo the host/plugin decode.
    output logic [15:0] dsp_accel_cmd,
    output logic        dsp_accel_cmd_valid
);

// Extract control bits
wire enable_transmission = ctrl_regs_pl[0*32 + 0];

// Declared ahead of first use (the safe-control block below) so the file also
// elaborates under xsim, which rejects use-before-declaration.
logic transmission_active;

// Safe control registers - only updated when transmission is not active
logic reset_timestamp_reg;
logic debug_mode_reg;
logic [31:0] loop_count_reg;
logic [3:0] phase0_reg;
logic [3:0] phase1_reg;
logic [3:0] phase2_reg;       // port 1 (cable B) CIPO0 cable-delay phase
logic [3:0] phase3_reg;       // port 1 (cable B) CIPO1 cable-delay phase
logic [7:0] channel_enable_reg;  // [3:0] = port 0 streams, [7:4] = port 1 streams
// Protected COPI message words (36 x 16-bit words) - only updated when transmission inactive
logic [15:0] copi_words_reg [0:35];

// Reserved control register, repurposed as the analytic-chirp config (CTRL_REG_3).
// Compact single-register encoding (kept clear of STFT regs 28-30 / playback 31):
//   [0]     chirp_mode  (1 = emit a memory-free swept-sine instead of the
//                        fixed-frequency debug sine; independent of debug_mode_reg
//                        which must also be set for any synthetic data)
//   [1]     reserved    (future: log/exp sweep select)
//   [7:2]   phase_stride (per-channel phase offset stride, 6-bit; applied as
//                        channel_offset * (stride << CHIRP_STRIDE_SHIFT) so
//                        channels are visibly distinguishable)
//   [19:8]  f_span      (12-bit; f_max = f_span << CHIRP_FSPAN_SHIFT in 32-bit
//                        phase-accumulator units -> ~0.46 Hz/step, full = ~1.9 kHz)
//   [31:20] sweep_rate  (12-bit; freq_acc increment per packet =
//                        sweep_rate << CHIRP_RATE_SHIFT; sets sweep speed/period)
// See data_generator_core.sv chirp NCO block + docs/lfp-dsp-engine-design.md.
wire [31:0] ctrl_reg_3 = ctrl_regs_pl[3*32 +: 32];  // chirp config
localparam int CHIRP_PHW          = 32;  // phase/freq accumulator width
localparam int CHIRP_LUT_IDX_HI   = CHIRP_PHW - 1;          // top 9 bits = LUT idx
localparam int CHIRP_LUT_IDX_LO   = CHIRP_PHW - 9;          // -> [31:23]
localparam int CHIRP_FSPAN_SHIFT  = 16;  // f_span (12b) -> f_max in phase units
localparam int CHIRP_RATE_SHIFT   = 9;   // sweep_rate (12b) -> freq_acc incr/packet
localparam int CHIRP_STRIDE_SHIFT = 24;  // per-channel phase stride placement

logic        chirp_mode_reg;
logic [5:0]  chirp_stride_reg;
logic [11:0] chirp_fspan_reg;
logic [11:0] chirp_rate_reg;

// Safe control register updates - only when transmission is not active
always_ff @(posedge clk) begin
    if (!rstn) begin
        reset_timestamp_reg <= 1'b0;
        debug_mode_reg <= 1'b0;
        chirp_mode_reg <= 1'b0;
        chirp_stride_reg <= 6'd0;
        chirp_fspan_reg <= 12'd0;
        chirp_rate_reg <= 12'd0;
        loop_count_reg <= 32'd0;
        phase0_reg <= 4'd0;
        phase1_reg <= 4'd0;
        phase2_reg <= 4'd0;
        phase3_reg <= 4'd0;
        channel_enable_reg <= 8'b0000_1111;  // Default: port-0 all channels, port-1 off (bit-identical)
        
        // Initialize COPI words to safe defaults
        for (int j = 0; j < 36; j++) begin
            copi_words_reg[j] <= 16'h0;
        end
    end else begin
        // Only update control registers when transmission is not active
        if (!transmission_active) begin
            reset_timestamp_reg <= ctrl_regs_pl[0*32 + 1];
            debug_mode_reg <= ctrl_regs_pl[0*32 + 3];
            // Chirp config (CTRL_REG_3); latched while inactive like debug_mode.
            chirp_mode_reg   <= ctrl_regs_pl[3*32 + 0];
            chirp_stride_reg <= ctrl_regs_pl[3*32 + 2  +: 6];
            chirp_fspan_reg  <= ctrl_regs_pl[3*32 + 8  +: 12];
            chirp_rate_reg   <= ctrl_regs_pl[3*32 + 20 +: 12];
            loop_count_reg <= ctrl_regs_pl[1*32 +: 32];
            // CTRL_REG_2 layout (widened for the second port; low bits unchanged
            // so a host that only writes the original 4-bit channel_enable at
            // [11:8] gets port-1 streams = 0 -> single-port path is unchanged):
            //   [3:0] phase0, [7:4] phase1, [15:8] channel_enable (8-bit),
            //   [19:16] phase2, [23:20] phase3
            phase0_reg <= ctrl_regs_pl[2*32 + 3  : 2*32 + 0];
            phase1_reg <= ctrl_regs_pl[2*32 + 7  : 2*32 + 4];
            channel_enable_reg <= ctrl_regs_pl[2*32 + 8 +: 8];
            phase2_reg <= ctrl_regs_pl[2*32 + 16 +: 4];
            phase3_reg <= ctrl_regs_pl[2*32 + 20 +: 4];
            
            // Update COPI words from control registers 4-21 (18 registers total)
            for (int j = 0; j < 18; j++) begin
                copi_words_reg[2*j]     <= ctrl_regs_pl[(j+4)*32 +: 16];      // Low 16 bits
                copi_words_reg[2*j + 1] <= ctrl_regs_pl[(j+4)*32 + 16 +: 16]; // High 16 bits
            end
        end
    end
end

// CIPO received data storage (4 separate 16-bit registers per cycle)
logic [31:0] cipo0_data [0:34];  // Port 0 CIPO0 line, register A (low 16 bits) and B (upper 16 bits)
logic [31:0] cipo1_data [0:34];  // Port 0 CIPO1 line
logic [31:0] cipo2_data [0:34];  // Port 1 CIPO0 line (dual-port)
logic [31:0] cipo3_data [0:34];  // Port 1 CIPO1 line

// Registers for COPI data from the 4 CIPO lines (2 per port)
reg [73:0] cipo0_4x_oversampled;
reg [73:0] cipo1_4x_oversampled;
reg [73:0] cipo2_4x_oversampled;
reg [73:0] cipo3_4x_oversampled;
reg [31:0] cipo0_phase_selected;
reg [31:0] cipo1_phase_selected;
reg [31:0] cipo2_phase_selected;
reg [31:0] cipo3_phase_selected;

// Instantiate phase selector modules that correct for CIPO delay because of long cable length.
// Port 1's two lines have their OWN phase (phase2/phase3) since cable B may differ in length.
CIPO_combined_phase_selector cipo0_selector(
    .phase_select(phase0_reg),
    .CIPO4x(cipo0_4x_oversampled),
    .CIPO(cipo0_phase_selected)
);
CIPO_combined_phase_selector cipo1_selector(
    .phase_select(phase1_reg),
    .CIPO4x(cipo1_4x_oversampled),
    .CIPO(cipo1_phase_selected)
);
CIPO_combined_phase_selector cipo2_selector(
    .phase_select(phase2_reg),
    .CIPO4x(cipo2_4x_oversampled),
    .CIPO(cipo2_phase_selected)
);
CIPO_combined_phase_selector cipo3_selector(
    .phase_select(phase3_reg),
    .CIPO4x(cipo3_4x_oversampled),
    .CIPO(cipo3_phase_selected)
);

// Control counters
logic [6:0] state_counter;
logic [5:0] cycle_counter;

// Constants
localparam logic [31:0] MAGIC_NUMBER_LOW  = 32'hDEADBEEF;
localparam logic [31:0] MAGIC_NUMBER_HIGH = 32'hCAFEBABE;
logic [63:0] timestamp;

// Status tracking (transmission_active is declared near the top of the module)
logic [31:0] packets_sent;
logic        loop_limit_reached;
logic [31:0] loop_counter;

// Debug mode sine wave table index
logic [8:0] dummy_data_index;
// Debug mode 512-entry sine lookup table (unsigned 16-bit values)
logic [15:0] sine_lut [0:511];

// ---- Analytic chirp NCO (memory-free; reuses the sine LUT) ----------------
// Dual accumulator updated once per packet:
//   freq_acc  triangles between 0 and f_max (= fspan << CHIRP_FSPAN_SHIFT),
//             stepping by (rate << CHIRP_RATE_SHIFT) each packet,
//   phase_acc += freq_acc each packet.
// The per-channel LUT index = (phase_acc + channel_offset*stride)[31:23].
logic [CHIRP_PHW-1:0] chirp_freq_acc;   // current sweep frequency (phase units)
logic [CHIRP_PHW-1:0] chirp_phase_acc;  // running phase
logic                 chirp_sweep_up;   // triangle direction
wire  [CHIRP_PHW-1:0] chirp_fmax  = {chirp_fspan_reg, {CHIRP_FSPAN_SHIFT{1'b0}}};
wire  [CHIRP_PHW-1:0] chirp_rstep = {{(CHIRP_PHW-12-CHIRP_RATE_SHIFT){1'b0}},
                                     chirp_rate_reg, {CHIRP_RATE_SHIFT{1'b0}}};

// Per-slot chirp phase, REGISTERED off the current cycle_counter. The
// channel_offset*stride multiply lives here in its own pipeline stage so it
// stays OFF the cycle_counter->fifo_write_data critical path (which fails 84 MHz
// otherwise). cycle_counter is stable across a cycle's 80 states and the per-
// packet chirp_phase_acc only advances at the packet boundary, so the registered
// ch_phase is the correct value for the current slot well before it is consumed
// at state 77. The 8 lane LUT reads + output mux (short) stay at state 77.
logic [CHIRP_PHW-1:0] chirp_ch_phase;
always_ff @(posedge clk) begin : chirp_phase_precompute
    logic [5:0]  c_off;
    logic [10:0] s_prod;
    c_off  = (cycle_counter >= 6'd2) ? (cycle_counter - 6'd2) : 6'd0;
    s_prod = c_off[4:0] * chirp_stride_reg;     // 5b*6b, isolated stage
    chirp_ch_phase <= chirp_phase_acc +
                      ({{(CHIRP_PHW-11){1'b0}}, s_prod} <<< CHIRP_STRIDE_SHIFT);
end

// Initialize sine lookup table
initial begin
    // Generate 512-point sine wave (signed 16-bit, ±32767 range)
    for (int i = 0; i < 512; i++) begin
        real angle = 2.0 * 3.14159265359 * i / 512.0;
        real sine_real = 32767.0 / 16 * $sin(angle) + 32767.0;
        sine_lut[i] = $rtoi(sine_real);
    end
end

// Helper signals for state machine logic
wire is_last_state = (state_counter == 7'd79);
wire is_first_cycle = (cycle_counter == 6'd0);
wire is_last_cycle = (cycle_counter == 6'd34);

// ============================================================================
// AUX COMMAND SEQUENCER + OVERRIDE LAYER (default OFF -> bit-identical)
// ============================================================================
// Control register decode (regs 22..24, already PL-domain after the CDC):
//   reg 22: [0] aux_seq_en, [3:1] bank_select, [4] fs_sw, [5] fs_gpio_en,
//           [8:6] fs_gpio_sel, [9] dsp_sw, [10] dsp_gpio_en, [13:11] dsp_gpio_sel,
//           [14] digout_sw, [15] digout_gpio_en, [18:16] digout_gpio_sel,
//           [31:24] reg3_static (host-owned Reg-3 bits D7..D1; D0 substituted)
//   reg 23: bank write port payload: [15:0] data, [21:16] addr, [23:22] slot,
//           [24] bank, [25] is_length
//   reg 24: [0] write toggle (edge = strobe one word), [1] inject toggle,
//           [31:16] inject command (one-shot slot-3 injection)
wire        aux_seq_en      = ctrl_regs_pl[22*32 + 0];
wire [2:0]  aux_bank_sel    = ctrl_regs_pl[22*32 + 1 +: 3];
wire        aux_fs_sw       = ctrl_regs_pl[22*32 + 4];
wire        aux_fs_gpio_en  = ctrl_regs_pl[22*32 + 5];
wire [2:0]  aux_fs_gpio_sel = ctrl_regs_pl[22*32 + 6 +: 3];
wire        aux_dsp_sw      = ctrl_regs_pl[22*32 + 9];
wire        aux_dsp_gpio_en = ctrl_regs_pl[22*32 + 10];
wire [2:0]  aux_dsp_gpio_sel = ctrl_regs_pl[22*32 + 11 +: 3];
wire        aux_dig_sw      = ctrl_regs_pl[22*32 + 14];
wire        aux_dig_gpio_en = ctrl_regs_pl[22*32 + 15];
wire [2:0]  aux_dig_gpio_sel = ctrl_regs_pl[22*32 + 16 +: 3];
wire [7:0]  aux_reg3_static = ctrl_regs_pl[22*32 + 24 +: 8];
wire [15:0] aux_wr_data     = ctrl_regs_pl[23*32 + 0 +: 16];
wire [5:0]  aux_wr_addr     = ctrl_regs_pl[23*32 + 16 +: 6];
wire [1:0]  aux_wr_slot     = ctrl_regs_pl[23*32 + 22 +: 2];
wire        aux_wr_bank     = ctrl_regs_pl[23*32 + 24];
wire        aux_wr_is_len   = ctrl_regs_pl[23*32 + 25];
wire        aux_wr_toggle   = ctrl_regs_pl[24*32 + 0];
wire        aux_inj_toggle  = ctrl_regs_pl[24*32 + 1];
wire [15:0] aux_inj_cmd     = ctrl_regs_pl[24*32 + 16 +: 16];

// Toggle -> 1-cycle pulse converters (payload regs are written by the host in
// prior AXI transactions, so they are long stable when the toggle flips).
logic aux_wr_toggle_d, aux_inj_toggle_d;
logic aux_wr_en, aux_inj_req;
always_ff @(posedge clk) begin
    if (!rstn) begin
        aux_wr_toggle_d  <= 1'b0;
        aux_inj_toggle_d <= 1'b0;
        aux_wr_en        <= 1'b0;
        aux_inj_req      <= 1'b0;
    end else begin
        aux_wr_toggle_d  <= aux_wr_toggle;
        aux_inj_toggle_d <= aux_inj_toggle;
        aux_wr_en        <= aux_wr_toggle  ^ aux_wr_toggle_d;
        aux_inj_req      <= aux_inj_toggle ^ aux_inj_toggle_d;
    end
end

// Packet strobes. packet_start = first state of each transmitted packet (same
// instant digital_in is latched); seq_advance = last state (use-then-advance,
// so the first packet after start plays bank entry 0).
wire packet_start = transmission_active && is_first_cycle && (state_counter == 7'd0);
wire seq_advance  = transmission_active && is_last_cycle  && is_last_state;

// aux_seq_en latched once per packet boundary: the command-source mux and the
// override can never change source mid-packet.
logic aux_seq_en_pkt;
always_ff @(posedge clk) begin
    if (!rstn)
        aux_seq_en_pkt <= 1'b0;
    else if (!transmission_active || (is_last_cycle && is_last_state))
        aux_seq_en_pkt <= aux_seq_en;
end

logic [47:0] aux_seq_cmds;     // raw sequencer outputs (slot i at [i*16 +: 16])
logic [47:0] aux_cmds_final;   // post-override commands actually serialized
logic [2:0]  aux_bank_active;
logic [17:0] aux_slot_indices;
logic        aux_inject_active;
logic        aux_dsp_force_h, aux_fs_active, aux_digout_state;

aux_command_sequencer #(.ADDR_W(6), .NSLOTS(3)) aux_seq_inst (
    .clk          (clk),
    .rstn         (rstn),
    .seq_advance  (seq_advance),
    .seq_hold     (!transmission_active || !aux_seq_en),
    .bank_select  (aux_bank_sel),
    .wr_en        (aux_wr_en),
    .wr_slot      (aux_wr_slot),
    .wr_bank      (aux_wr_bank),
    .wr_is_length (aux_wr_is_len),
    .wr_addr      (aux_wr_addr),
    .wr_data      (aux_wr_data),
    .inject_req   (aux_inj_req),
    .inject_cmd   (aux_inj_cmd),
    .inject_active(aux_inject_active),
    .aux_cmds     (aux_seq_cmds),
    .bank_active  (aux_bank_active),
    .slot_indices (aux_slot_indices)
);

override_layer override_inst (
    .clk               (clk),
    .rstn              (rstn),
    .packet_start      (packet_start),
    .enable            (aux_seq_en_pkt),
    .digital_in        (digital_in),
    .fs_sw             (aux_fs_sw),
    .fs_gpio_en        (aux_fs_gpio_en),
    .fs_gpio_sel       (aux_fs_gpio_sel),
    .dsp_sw            (aux_dsp_sw),
    .dsp_gpio_en       (aux_dsp_gpio_en),
    .dsp_gpio_sel      (aux_dsp_gpio_sel),
    .digout_sw         (aux_dig_sw),
    .digout_gpio_en    (aux_dig_gpio_en),
    .digout_gpio_sel   (aux_dig_gpio_sel),
    .reg3_static       (aux_reg3_static),
    .cmds_in           (aux_seq_cmds),
    .cmds_out          (aux_cmds_final),
    .dsp_force_h       (aux_dsp_force_h),
    .fast_settle_active(aux_fs_active),
    .digout_state      (aux_digout_state)
);

// Command-echo identity (command-bank-design.md). SPI readback alignment:
// the response to the command issued at cycle C is captured at cycle C+2, so
// packet word 34 = response to THIS packet's slot-1 command (cycle 32) and
// packet words 0/1 = responses to the PREVIOUS packet's slot-2/3 commands
// (cycles 33/34). The header therefore echoes {this slot-1, prev slot-2,
// prev slot-3} -- each packet fully labels its own aux payload.
logic [15:0] echo_slot2_prev, echo_slot3_prev;
logic        echo_valid;           // 0 for the first packet after start
logic        inject_result_pkt;    // word 1 of THIS packet answers an injection
always_ff @(posedge clk) begin
    if (!rstn) begin
        echo_slot2_prev   <= 16'h0;
        echo_slot3_prev   <= 16'h0;
        echo_valid        <= 1'b0;
        inject_result_pkt <= 1'b0;
    end else if (!transmission_active) begin
        echo_valid        <= 1'b0;
        inject_result_pkt <= 1'b0;
    end else if (seq_advance) begin
        echo_slot2_prev   <= aux_cmds_final[31:16];
        echo_slot3_prev   <= aux_cmds_final[47:32];
        echo_valid        <= 1'b1;
        inject_result_pkt <= aux_inject_active;
    end
end

// One-shot injection result capture: the response to an injected slot-3
// command (cycle 34 of packet N) lands in cipoX_data[1] of packet N+1
// (written at state 76 of cycle 1). Latch it one state later and flip the
// ack toggle for the firmware handshake. [15:0] of each CIPO word is the
// regular (non-DDR) stream.
logic        aux_inj_ack;
logic [31:0] aux_read_result_reg;
always_ff @(posedge clk) begin
    if (!rstn) begin
        aux_inj_ack         <= 1'b0;
        aux_read_result_reg <= 32'h0;
    end else if (transmission_active && inject_result_pkt &&
                 (cycle_counter == 6'd1) && (state_counter == 7'd77)) begin
        aux_read_result_reg <= {cipo1_data[1][15:0], cipo0_data[1][15:0]};
        aux_inj_ack         <= ~aux_inj_ack;
    end
end

// Packet metadata flags (header word 2 bits [15:8]; 0 when sequencer is off)
logic [7:0] aux_flags;
assign aux_flags = aux_seq_en_pkt ? {2'b00,
                                     inject_result_pkt,   // [5]
                                     echo_valid,          // [4]
                                     aux_dsp_force_h,     // [3]
                                     aux_digout_state,    // [2]
                                     aux_fs_active,       // [1]
                                     1'b1}                // [0] aux_seq active
                                  : 8'h00;

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

            if  (state_counter <= 7'd63) begin //removed part of conditional
                logic [3:0]  bit_index;
                logic [1:0]  aux_slot;
                logic [15:0] tx_word;
                bit_index = ~state_counter[5:2];  // MSB first: ~0=15, ~1=14, ..., ~15=0
                // Aux slot for cycles 32..34 (32->0, 33->1, 34->2); 0 elsewhere
                // so the part-select below is always in range.
                aux_slot = (cycle_counter >= 6'd32) ? cycle_counter[1:0] : 2'd0;
                // Command-source mux: when the aux sequencer is enabled (latched
                // per packet) the 3 aux cycles come from the post-override
                // sequencer bundle; otherwise the legacy static table -- with
                // aux_seq_en==0 this is bit-identical to the original datapath.
                if (aux_seq_en_pkt && (cycle_counter >= 6'd32))
                    tx_word = aux_cmds_final[aux_slot*16 +: 16];
                else
                    tx_word = copi_words_reg[cycle_counter];
                // DSP reset ("digital fast settle"): force bit H on channel
                // CONVERTs (cycles 0..31) while requested.
                if (aux_seq_en_pkt && aux_dsp_force_h &&
                    (cycle_counter < 6'd32) && (tx_word[15:14] == 2'b00))
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
            cipo0_data[j] <= 32'h0;
            cipo1_data[j] <= 32'h0;
            cipo2_data[j] <= 32'h0;
            cipo3_data[j] <= 32'h0;
        end
        cipo0_4x_oversampled <= 74'h0;
        cipo1_4x_oversampled <= 74'h0;
        cipo2_4x_oversampled <= 74'h0;
        cipo3_4x_oversampled <= 74'h0;
    end else begin
        if (transmission_active && (state_counter >= 7'd2) && (state_counter <= 75)) begin
            cipo0_4x_oversampled[state_counter - 2] <= cipo0; // Latch data into the phase selector input
            cipo1_4x_oversampled[state_counter - 2] <= cipo1;
            cipo2_4x_oversampled[state_counter - 2] <= cipo2;
            cipo3_4x_oversampled[state_counter - 2] <= cipo3;
        end else if(transmission_active && state_counter == 7'd76) begin
            cipo0_data[cycle_counter] <= cipo0_phase_selected; // Get the phase selector output
            cipo1_data[cycle_counter] <= cipo1_phase_selected; // It's ready one clock cycle after being latched in
            cipo2_data[cycle_counter] <= cipo2_phase_selected;
            cipo3_data[cycle_counter] <= cipo3_phase_selected;
        end
    end
end


// Digital input
// Register for latching digital inputs
logic [7:0] digital_in_latched;

// Latch digital inputs at the start of each packet
// (Fast settle / digout GPIO triggers sample digital_in at the same instant,
// inside override_layer.)

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
        fifo_packet_end_flag <= 1'b0;

        dummy_data_index <= 9'd0;
        chirp_freq_acc   <= '0;
        chirp_phase_acc  <= '0;
        chirp_sweep_up   <= 1'b1;
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
            // Header writes (first cycle only) - always fully valid
            if (state_counter inside {7'd0, 7'd1, 7'd2, 7'd3, 7'd4}) begin
                if (is_first_cycle) begin
                    fifo_write_en <= 1'b1;
                    // Header is one 64-bit value -> exactly the low 4 segments,
                    // upper 4 masked off (port-2 streams never appear in the
                    // header). Bit-identical to the single-port header.
                    fifo_channel_mask <= 8'b0000_1111;
                    fifo_packet_end_flag <= 1'b0;  // Header words are never at the end
                    case (state_counter)
                        7'd0: fifo_write_data <= {MAGIC_NUMBER_HIGH, MAGIC_NUMBER_LOW}; // magic number
                        7'd1: fifo_write_data <= timestamp;
                        // Digital inputs + aux flags + command-echo identity:
                        //   [7:0]   digital_in, [15:8] aux_flags (0 when seq off),
                        //   [31:16] this packet's slot-1 command (result @ word 34),
                        //   [47:32] prev packet's slot-2 command (result @ word 0),
                        //   [63:48] prev packet's slot-3 command (result @ word 1).
                        // With aux_seq_en==0 everything above [7:0] is 0 -> the
                        // word is bit-identical to the legacy header.
                        7'd2: fifo_write_data <= {
                            aux_seq_en_pkt ? echo_slot3_prev      : 16'h0,
                            aux_seq_en_pkt ? echo_slot2_prev      : 16'h0,
                            aux_seq_en_pkt ? aux_cmds_final[15:0] : 16'h0,
                            aux_flags,
                            digital_in_latched};
                        7'd3: fifo_write_data <= 64'h0;  // BREADCRUMB: Future analog channels 0-3 (4x16-bit values)
                        7'd4: fifo_write_data <= 64'h0;  // BREADCRUMB: Future analog channels 4-7 (4x16-bit values)                        // TODO: Add another state here to transmit 64 bits of non-neural input and other sample-specific metadata
                        // Metadata - value of the current "extra" register read from this cycle
                        // Metadata - value of fast settle
                    endcase
                end
            end 
            
            // Data writes - Pack both ports' CIPO lines into one 128-bit write
            // with the 8-bit channel mask. Segment order (low->high):
            //   port0 cipo0{reg,ddr}, port0 cipo1{reg,ddr},  (low 64 = bits[63:0])
            //   port1 cipo0{reg,ddr}, port1 cipo1{reg,ddr}.  (high 64 = bits[127:64])
            // When channel_enable_reg[7:4]==0 the high 64 bits are masked off and
            // the packet is byte-identical to the single-port datapath.
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
                    fifo_write_data <= {cipo3_data[cycle_counter], cipo2_data[cycle_counter],
                                        cipo1_data[cycle_counter], cipo0_data[cycle_counter]};
                end else if (chirp_mode_reg) begin
                    // ---- Analytic chirp: one swept sinusoid (same frequency on
                    // every channel) with a host-configurable per-channel phase
                    // stride so all 8 lanes x 32 slots are visibly distinguishable.
                    // The phase comes from the dual-accumulator NCO (advanced once
                    // per packet); the top 9 bits of (phase_acc + per-channel
                    // offset) index the existing 512-entry sine LUT. No BRAM.
                    logic [15:0]           cv [0:7];         // 8 lane sine values
                    // ch_phase (= phase_acc + slot*stride) is the REGISTERED
                    // chirp_ch_phase (the multiply is pipelined off this path).
                    for (int l = 0; l < 8; l++) begin
                        logic [CHIRP_PHW-1:0] lane_phase;
                        // fan the 8 lanes out by 1/8-period steps so a single
                        // packet shows 8 distinct phases too.
                        lane_phase = chirp_ch_phase +
                                     ({{(CHIRP_PHW-3){1'b0}}, l[2:0]} <<< (CHIRP_PHW-3));
                        cv[l] = sine_lut[lane_phase[CHIRP_LUT_IDX_HI:CHIRP_LUT_IDX_LO]];
                    end
                    fifo_write_data <= {cv[7], cv[6], cv[5], cv[4],
                                        cv[3], cv[2], cv[1], cv[0]};
                end else begin
                    // Debug synthetic sine. Phase 1 bandwidth test: port 1 is
                    // filled too (phase-offset from port 0 so it is visibly
                    // distinct) so a doubled-size packet exercises the PS->UDP path.
                    logic [5:0] channel_offset;  // Only needs 6 bits for values 0-32
                    logic [15:0] cipo0_regular_val, cipo0_ddr_val, cipo1_regular_val, cipo1_ddr_val;
                    logic [15:0] cipo2_regular_val, cipo2_ddr_val, cipo3_regular_val, cipo3_ddr_val;
                    logic [8:0] base_phase;         // index into 512-entry LUT
                    logic [8:0] base_phase_p1;      // port-1 phase (offset half a period)

                    // Calculate base sample index (0-32 for cycles 2-34)
                    channel_offset = (cycle_counter >= 6'd2) ? (cycle_counter - 6'd2) : 6'd0;

                    // Base phase for this sample (9 bits total)
                    base_phase = dummy_data_index + channel_offset;
                    base_phase_p1 = base_phase + 9'd128;   // port-1 phase offset (90 deg)

                    // Generate sine values with frequency multiplication using left shifts
                    cipo0_regular_val = sine_lut[base_phase];                       // 1× = 58.6 Hz
                    cipo0_ddr_val     = sine_lut[(base_phase << 1) & 9'h1FF];       // 2× = 117.2 Hz
                    cipo1_regular_val = sine_lut[(base_phase << 2) & 9'h1FF];       // 4× = 234.4 Hz
                    cipo1_ddr_val     = sine_lut[(base_phase << 3) & 9'h1FF];       // 8× = 468.8 Hz
                    cipo2_regular_val = sine_lut[base_phase_p1];                    // port1, offset
                    cipo2_ddr_val     = sine_lut[(base_phase_p1 << 1) & 9'h1FF];
                    cipo3_regular_val = sine_lut[(base_phase_p1 << 2) & 9'h1FF];
                    cipo3_ddr_val     = sine_lut[(base_phase_p1 << 3) & 9'h1FF];

                    fifo_write_data <= {
                        {cipo3_ddr_val, cipo3_regular_val}, {cipo2_ddr_val, cipo2_regular_val},
                        {cipo1_ddr_val, cipo1_regular_val}, {cipo0_ddr_val, cipo0_regular_val}};
                end
            end
                    
            if (is_last_cycle) begin
                if (is_last_state) begin
                    packets_sent <= packets_sent + 1;
                    // Increment dummy data index for continuous sine wave across packets
                    dummy_data_index <= dummy_data_index + 9'd1;

                    // Analytic chirp NCO advance (once per packet). freq_acc
                    // triangles 0<->f_max; phase_acc integrates it. Clamp at the
                    // turning points so the sweep reverses cleanly.
                    if (chirp_sweep_up) begin
                        if (chirp_freq_acc + chirp_rstep >= chirp_fmax) begin
                            chirp_freq_acc <= chirp_fmax;
                            chirp_sweep_up <= 1'b0;
                        end else
                            chirp_freq_acc <= chirp_freq_acc + chirp_rstep;
                    end else begin
                        if (chirp_freq_acc <= chirp_rstep) begin
                            chirp_freq_acc <= '0;
                            chirp_sweep_up <= 1'b1;
                        end else
                            chirp_freq_acc <= chirp_freq_acc - chirp_rstep;
                    end
                    chirp_phase_acc <= chirp_phase_acc + chirp_freq_acc;
                    // DSP tap: the packet is complete -> advance the engine's ring
                    // (one tick per packet, after all 35 slot samples are ingested).
                    dsp_packet_tick <= 1'b1;
                end
            end

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

// Accel tap: echo_slot2_prev is the rotating slot-1 (cycle 33) accel CONVERT whose
// result lands at data word 0; valid only when the aux sequencer is on (per packet)
// and the echo has been established (echo_valid). The movement extractor decodes the
// axis from this and reads the accel sample at dsp_sample_slot==0.
assign dsp_accel_cmd       = echo_slot2_prev;
assign dsp_accel_cmd_valid = aux_seq_en_pkt & echo_valid;

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
// former reserved [27:24]. phase2/phase3 are read back via the CTRL_REG_2
// mirror (status reg 8).
assign status_regs_pl[1*32 +: 32] = {
    4'd0,                     // [31:28] - reserved
    channel_enable_reg[7:4],  // [27:24] - port-1 channel enable
    channel_enable_reg[3:0],  // [23:20] - port-0 channel enable
    phase1_reg,               // [19:16] - 4 bits
    phase0_reg,               // [15:12] - 4 bits
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
//   aux_status: [2:0] bank_active, [3] aux_seq_en (per-packet latched),
//               [4] fast_settle_active, [5] digout_state, [6] dsp_force_h,
//               [7] inject ack toggle, [13:8] slot-0 index, [21:16] slot-1
//               index, [29:24] slot-2 index.
//   aux_read_result: {cipo1_regular[15:0], cipo0_regular[15:0]} of the last
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
    aux_seq_en_pkt,            // [3]
    aux_bank_active            // [2:0]
};
assign aux_read_result = aux_read_result_reg;

endmodule
