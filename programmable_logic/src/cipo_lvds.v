module cipo_lvds(
    input wire cipo0_p,
    input wire cipo0_n,
    input wire cipo1_p,
    input wire cipo1_n,
    output wire cipo0,
    output wire cipo1
);
IBUFDS #(
    .IOSTANDARD("LVDS_25"),
    .DIFF_TERM("TRUE")
) cipo0_ibuf (
    .I(cipo0_p),
    .IB(cipo0_n),
    .O(cipo0)
);

IBUFDS #(
    .IOSTANDARD("LVDS_25"),
    .DIFF_TERM("TRUE")
) cipo1_ibuf (
    .I(cipo1_p),
    .IB(cipo1_n),
    .O(cipo1)
);


endmodule