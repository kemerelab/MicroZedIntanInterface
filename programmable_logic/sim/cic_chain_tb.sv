`timescale 1ns/1ps
// End-to-end chain TB: cic_decimator(/5) -> cic_to_halfband -> lfp_halfband(/2)
// = /10. Streams input packets at a realistic-ish cadence; checks the /10 outputs
// against gen_cic_chain_vectors.py.
module cic_chain_tb;
    localparam int N_LANES=8, N_SLOTS=32, DATA_W=16;
    localparam int R_CIC=5, N_ORDER=4, ACC_W=32, GAIN_SHIFT=10;
    localparam int COEF_W=18, COEF_FRAC=17, OUT_W=16, HB_TAPS=43, HB_RING=64;
    localparam int K_PACKETS=200;
    localparam logic [7:0] LANE_MASK=8'b1010_0101;
    localparam int SLOT_W=$clog2(N_SLOTS), CH_W=$clog2(N_LANES*N_SLOTS);
    localparam int HB_TAPN_W=$clog2(HB_RING+1), HB_RING_AW=$clog2(HB_RING), MAXOUT=8192;

    logic clk=0, rstn=0; always #5 clk=~clk;

    // stimulus
    logic sample_valid; logic [N_LANES*DATA_W-1:0] sample_data; logic [SLOT_W-1:0] sample_slot; logic packet_tick;
    logic [N_LANES-1:0] lane_mask;

    // CIC
    logic cic_valid, cic_fs, cic_busy, cic_ov; logic [CH_W-1:0] cic_ch; logic [OUT_W-1:0] cic_d;
    cic_decimator #(.N_LANES(N_LANES),.N_SLOTS(N_SLOTS),.DATA_W(DATA_W),.R(R_CIC),
        .N_ORDER(N_ORDER),.ACC_W(ACC_W),.OUT_W(OUT_W),.GAIN_SHIFT(GAIN_SHIFT)) u_cic (
        .clk(clk),.rstn(rstn),.sample_valid(sample_valid),.sample_data(sample_data),
        .sample_slot(sample_slot),.packet_tick(packet_tick),.en(1'b1),.lane_mask(lane_mask),
        .out_valid(cic_valid),.out_channel(cic_ch),.out_data(cic_d),
        .out_frame_start(cic_fs),.busy(cic_busy),.compute_overrun(cic_ov));

    // glue
    logic hb_v; logic [N_LANES*DATA_W-1:0] hb_d; logic [SLOT_W-1:0] hb_s; logic hb_t;
    cic_to_halfband #(.N_LANES(N_LANES),.N_SLOTS(N_SLOTS),.DATA_W(DATA_W)) u_glue (
        .clk(clk),.rstn(rstn),.lane_mask(lane_mask),
        .cic_valid(cic_valid),.cic_channel(cic_ch),.cic_data(cic_d),.cic_frame_start(cic_fs),
        .hb_valid(hb_v),.hb_data(hb_d),.hb_slot(hb_s),.hb_tick(hb_t));

    // halfband
    logic [COEF_W-1:0] coef_wr_data; logic [HB_RING_AW-1:0] coef_wr_addr; logic coef_wr_en;
    logic hbo_v, hbo_fs, hbo_busy, hbo_ov; logic [CH_W-1:0] hbo_ch; logic [OUT_W-1:0] hbo_d;
    lfp_halfband #(.N_LANES(N_LANES),.N_SLOTS(N_SLOTS),.DATA_W(DATA_W),.COEF_W(COEF_W),
        .COEF_FRAC(COEF_FRAC),.RING_DEPTH(HB_RING),.OUT_W(OUT_W)) u_hb (
        .clk(clk),.rstn(rstn),.sample_valid(hb_v),.sample_data(hb_d),.sample_slot(hb_s),
        .packet_tick(hb_t),.en(1'b1),.lane_mask(lane_mask),.num_taps(HB_TAPS[HB_TAPN_W-1:0]),
        .coef_wr_en(coef_wr_en),.coef_wr_addr(coef_wr_addr),.coef_wr_data(coef_wr_data),
        .out_valid(hbo_v),.out_channel(hbo_ch),.out_data(hbo_d),
        .out_frame_start(hbo_fs),.busy(hbo_busy),.compute_overrun(hbo_ov));

    logic [COEF_W-1:0] coefs[0:HB_TAPS-1];
    logic [127:0] samples[0:K_PACKETS*N_SLOTS-1];
    logic [OUT_W-1:0] exp_val[0:MAXOUT-1]; logic [15:0] exp_chan[0:MAXOUT-1];
    logic [OUT_W-1:0] got_val[0:MAXOUT-1]; logic [CH_W-1:0] got_chan[0:MAXOUT-1]; int n_got=0;
    always @(posedge clk) if(rstn&&hbo_v&&n_got<MAXOUT)begin got_val[n_got]=hbo_d;got_chan[n_got]=hbo_ch;n_got++; end
    int n_expected, errors=0;

    initial begin
        $readmemh("cicch_coefs.hex",coefs);$readmemh("cicch_samples.hex",samples);
        $readmemh("cicch_exp_val.hex",exp_val);$readmemh("cicch_exp_chan.hex",exp_chan);
        n_expected=((K_PACKETS/R_CIC)/2)*$countones(LANE_MASK)*N_SLOTS;
        sample_valid=0;sample_data=0;sample_slot=0;packet_tick=0;lane_mask=LANE_MASK;
        coef_wr_en=0;coef_wr_addr=0;coef_wr_data=0;
        repeat(5)@(posedge clk);rstn=1;@(posedge clk);
        // load halfband coefs
        for(int j=0;j<HB_TAPS;j++)begin @(negedge clk);coef_wr_en=1;coef_wr_addr=j[HB_RING_AW-1:0];coef_wr_data=coefs[j]; end
        @(negedge clk);coef_wr_en=0;@(posedge clk);
        // stream input packets at ~2800-clk/packet cadence so both stages keep up
        for(int p=0;p<K_PACKETS;p++)begin
            for(int s=0;s<N_SLOTS;s++)begin
                @(negedge clk);sample_valid=1;sample_slot=s[SLOT_W-1:0];sample_data=samples[p*N_SLOTS+s];
                @(negedge clk);sample_valid=0; repeat(76)@(negedge clk);
            end
            repeat(40)@(negedge clk);
            @(negedge clk);packet_tick=1;@(negedge clk);packet_tick=0;
        end
        repeat(30000)@(posedge clk);
        if(n_got!=n_expected)begin $display("COUNT got %0d exp %0d",n_got,n_expected);errors++; end
        for(int i=0;i<n_expected;i++) if(got_val[i]!==exp_val[i]||got_chan[i]!==exp_chan[i][CH_W-1:0])begin
            if(errors<20)$display("MISMATCH i=%0d got c=%0d v=%04h exp c=%0d v=%04h",i,got_chan[i],got_val[i],exp_chan[i],exp_val[i]);errors++; end
        if(cic_ov)begin $display("CIC OVERRUN");errors++; end
        if(hbo_ov)begin $display("HB OVERRUN");errors++; end
        if(errors==0)$display("RESULT: PASS (chain /10, %0d outs, no overrun)",n_expected);
        else $display("RESULT: FAIL (%0d errors)",errors);
        $finish;
    end
    initial begin #400ms;$display("RESULT: FAIL (timeout)");$finish; end
endmodule
