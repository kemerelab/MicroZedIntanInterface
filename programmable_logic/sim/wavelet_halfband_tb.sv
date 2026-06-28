`timescale 1ns/1ps
// Unit TB for wavelet_halfband.sv (the standalone reusable ÷2 primitive).
// Loads the same halfband coeffs as the engine TB, drives one output's worth
// of taps (newest..oldest), and checks the result against the Python
// halfband_decimate of a known window. Self-contained (one output).
module wavelet_halfband_tb;
    localparam int DATA_W=16, COEF_W=18, COEF_FRAC=17, HB_TAPS=7, OUT_W=16;
    localparam int TAP_W = $clog2(HB_TAPS);

    logic clk=0, rstn=0; always #5 clk=~clk;
    logic coef_wr_en=0; logic [TAP_W-1:0] coef_wr_addr=0; logic signed [COEF_W-1:0] coef_wr_data=0;
    logic tap_valid=0, tap_first=0, tap_last=0; logic signed [DATA_W-1:0] tap_sample=0; logic [TAP_W-1:0] tap_index=0;
    logic out_valid; logic signed [OUT_W-1:0] out_data;

    wavelet_halfband #(.DATA_W(DATA_W), .COEF_W(COEF_W), .COEF_FRAC(COEF_FRAC),
        .HB_TAPS(HB_TAPS), .OUT_W(OUT_W)) dut (.*);

    // halfband coeffs from gen_wavelet_vectors.halfband_coeffs():
    // [-1143, 0, 33010, 67339, 33010, 0, -1143]
    logic signed [COEF_W-1:0] hb [0:HB_TAPS-1];
    // a known newest..oldest window x[254..248] = [97, 78, 55, 25, -1, -28, -53]
    // (newest first). Python halfband_decimate output for this window = 26.
    logic signed [DATA_W-1:0] win [0:HB_TAPS-1];

    task automatic neg; @(negedge clk); endtask
    int j, errs=0;
    // capture out_data exactly when out_valid is asserted (it is a 1-cycle pulse)
    logic signed [OUT_W-1:0] captured; logic got=0;
    always @(posedge clk) if (out_valid) begin captured <= out_data; got <= 1; end
    initial begin
        hb[0]=-1143; hb[1]=0; hb[2]=33010; hb[3]=67339; hb[4]=33010; hb[5]=0; hb[6]=-1143;
        win[0]=97; win[1]=78; win[2]=55; win[3]=25; win[4]=-1; win[5]=-28; win[6]=-53;

        rstn=0; repeat(4) neg(); rstn=1; neg();
        for (j=0;j<HB_TAPS;j++) begin neg(); coef_wr_en=1; coef_wr_addr=j[TAP_W-1:0]; coef_wr_data=hb[j]; end
        neg(); coef_wr_en=0; neg();

        // drive HB_TAPS taps: sample=win[j], coef index=j (newest=hb[0])
        for (j=0;j<HB_TAPS;j++) begin
            neg();
            tap_valid=1; tap_first=(j==0); tap_last=(j==HB_TAPS-1);
            tap_sample=win[j]; tap_index=j[TAP_W-1:0];
        end
        neg(); tap_valid=0; tap_first=0; tap_last=0;

        // wait for the captured output
        fork begin while(!got) neg(); end begin repeat(50) neg(); end join_any
        repeat(2) neg();

        if (captured !== 16'sd26) begin
            $display("  MISMATCH out=%0d expected=26", captured); errs++;
        end
        if (errs==0) $display("RESULT: PASS (halfband out=%0d)", captured);
        else         $display("RESULT: FAIL");
        $finish;
    end
    initial begin #1ms; $display("RESULT: FAIL (timeout)"); $finish; end
endmodule
