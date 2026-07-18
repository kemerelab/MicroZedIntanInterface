// =====================================================================
// cic_to_halfband.sv
//
// Glue between cic_decimator (emits per-channel /5 outputs, one channel at a
// time, lane-major/slot-major within a frame) and lfp_halfband (consumes the
// per-slot 8-lane-packed stream + a packet_tick, like the acquisition tap).
//
// It re-packs a full CIC output frame into [slot][lane] words and then replays
// the 32 slots into the halfband, each as one sample_valid pulse, followed by a
// packet_tick. The halfband then runs its own /2 decimation. Disabled lanes are
// written 0 (the halfband's lane_mask gates them anyway).
//
// Timing: a CIC frame arrives every 5 input packets; the halfband replay (32
// slots + tick) is a few dozen cycles, far inside the 5*2800-clk window. The
// CIC asserts out_frame_start on the first output of each frame and emits
// exactly popcount(lane_mask)*N_SLOTS outputs per frame; we detect frame end by
// counting outputs against the expected total (driven by lane_mask), which is
// robust to the lane/slot emission order.
// =====================================================================

module cic_to_halfband #(
    parameter int N_LANES = 8,
    parameter int N_SLOTS = 32,
    parameter int DATA_W  = 16,
    localparam int SLOT_W = (N_SLOTS <= 1) ? 1 : $clog2(N_SLOTS),
    localparam int CH_W   = $clog2(N_LANES * N_SLOTS),
    localparam int LANE_W = (N_LANES <= 1) ? 1 : $clog2(N_LANES)
) (
    input  logic                      clk,
    input  logic                      rstn,
    input  logic [N_LANES-1:0]        lane_mask,

    // ---- from cic_decimator ----
    input  logic                      cic_valid,
    input  logic [CH_W-1:0]           cic_channel,    // lane*N_SLOTS + slot
    input  logic [DATA_W-1:0]         cic_data,       // signed
    input  logic                      cic_frame_start,

    // ---- to lfp_halfband (per-slot 8-lane-packed stream) ----
    output logic                      hb_valid,
    output logic [N_LANES*DATA_W-1:0] hb_data,
    output logic [SLOT_W-1:0]         hb_slot,
    output logic                      hb_tick
);

    // per-(slot,lane) frame buffer; double-buffer not needed (replay finishes
    // long before the next CIC frame).
    logic signed [DATA_W-1:0] fbuf [0:N_SLOTS-1][0:N_LANES-1];

    // count outputs in the current CIC frame to detect frame completion
    function automatic int unsigned popcnt(input logic [N_LANES-1:0] m);
        int unsigned c; c = 0;
        for (int i = 0; i < N_LANES; i++) c += m[i];
        return c;
    endfunction

    logic [CH_W:0] frame_cnt;          // outputs seen this frame
    logic [CH_W:0] frame_total;        // popcount(lane_mask)*N_SLOTS
    logic          frame_ready;        // pulse: a full CIC frame is buffered

    wire [SLOT_W-1:0]  cic_slot = cic_channel[SLOT_W-1:0];
    wire [LANE_W-1:0]  cic_lane = cic_channel[CH_W-1:SLOT_W];

    always_ff @(posedge clk) begin
        if (!rstn) begin
            frame_cnt   <= '0;
            frame_total <= '0;
            frame_ready <= 1'b0;
        end else begin
            frame_ready <= 1'b0;
            if (cic_valid) begin
                fbuf[cic_slot][cic_lane] <= $signed(cic_data);
                if (cic_frame_start) begin
                    frame_cnt   <= 1;
                    frame_total <= CH_W'(popcnt(lane_mask)) * N_SLOTS;
                end else begin
                    frame_cnt <= frame_cnt + 1'b1;
                    if (frame_cnt + 1'b1 == frame_total)
                        frame_ready <= 1'b1;   // last output of the frame written
                end
            end
        end
    end

    // replay FSM: stream slots 0..N_SLOTS-1 into the halfband, then a tick.
    typedef enum logic [1:0] {R_IDLE, R_STREAM, R_TICK} rstate_t;
    rstate_t          rstate;
    logic [SLOT_W:0]  rslot;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            rstate <= R_IDLE; rslot <= '0;
            hb_valid <= 1'b0; hb_tick <= 1'b0; hb_slot <= '0; hb_data <= '0;
        end else begin
            hb_valid <= 1'b0; hb_tick <= 1'b0;
            case (rstate)
                R_IDLE: if (frame_ready) begin rstate <= R_STREAM; rslot <= '0; end
                R_STREAM: begin
                    hb_valid <= 1'b1;
                    hb_slot  <= rslot[SLOT_W-1:0];
                    for (int l = 0; l < N_LANES; l++)
                        hb_data[l*DATA_W +: DATA_W] <= fbuf[rslot[SLOT_W-1:0]][l];
                    if (rslot == SLOT_W'(N_SLOTS-1)) rstate <= R_TICK;
                    else                             rslot  <= rslot + 1'b1;
                end
                R_TICK: begin hb_tick <= 1'b1; rstate <= R_IDLE; end
                default: rstate <= R_IDLE;
            endcase
        end
    end
endmodule
