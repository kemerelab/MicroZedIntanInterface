module output_lvds(
    input wire csn,
    input wire sclk,
    input wire copi,
    output wire csn_p,
    output wire csn_n,
    output wire sclk_p,
    output wire sclk_n,
    output wire copi_p,
    output wire copi_n
);

    OBUFDS #(
        .IOSTANDARD("LVDS_25")
    ) csn_buf (
        .I(csn),
        .O(csn_p),
        .OB(csn_n)
    );

    OBUFDS #(
        .IOSTANDARD("LVDS_25")
    ) sclk_buf (
        .I(sclk),
        .O(sclk_p),
        .OB(sclk_n)
    );

    OBUFDS #(
        .IOSTANDARD("LVDS_25")
    ) copi_buf (
        .I(copi),
        .O(copi_p),
        .OB(copi_n)
    );

endmodule