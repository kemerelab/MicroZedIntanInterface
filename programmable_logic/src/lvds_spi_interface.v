// Simple LVDS buffer module that converts single-ended Intan SPI to differential LVDS outputs
// This module takes an intan_spi interface as input and provides intan_spi_diff LVDS outputs

module intan_spi_lvds_buffer (
    // Input: Single-ended Intan SPI interface
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi sclk" *)
    input wire sclk,
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi csn" *)
    input wire csn,
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi copi" *)
    input wire copi,
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi cipo0" *)
    output wire cipo0,
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi cipo1" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF intan_spi" *)
    output wire cipo1,
    
    // Output: Differential LVDS Intan SPI interface
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 spi_lvds sclk_p" *)
    output wire spi_sclk_p,
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 spi_lvds sclk_n" *)
    output wire spi_sclk_n,
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 spi_lvds csn_p" *)
    output wire spi_csn_p,
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 spi_lvds csn_n" *)
    output wire spi_csn_n,
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 spi_lvds copi_p" *)
    output wire spi_copi_p,
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 spi_lvds copi_n" *)
    output wire spi_copi_n,
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 spi_lvds cipo0_p" *)
    input wire spi_cipo0_p,
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 spi_lvds cipo0_n" *)
    input wire spi_cipo0_n,
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 spi_lvds cipo1_p" *)
    input wire spi_cipo1_p,
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 spi_lvds cipo1_n" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF spi_lvds" *)
    input wire spi_cipo1_n
);

    // LVDS output buffers for outgoing signals (Master outputs)
    // These convert single-ended signals to differential LVDS
    
    (* IOB = "TRUE" *)
    (* IOSTANDARD = "LVDS_25" *)
    OBUFDS #(
        .IOSTANDARD("LVDS_25"),
        .SLEW("FAST")
    ) obufds_sclk (
        .O(spi_sclk_p),     // Positive output
        .OB(spi_sclk_n),    // Negative output  
        .I(sclk)        // Single-ended input
    );
    
    (* IOB = "TRUE" *)
    (* IOSTANDARD = "LVDS_25" *)
    OBUFDS #(
        .IOSTANDARD("LVDS_25"),
        .SLEW("FAST")
    ) obufds_csn (
        .O(spi_csn_p),
        .OB(spi_csn_n),
        .I(csn)
    );
    
    (* IOB = "TRUE" *)
    (* IOSTANDARD = "LVDS_25" *)
    OBUFDS #(
        .IOSTANDARD("LVDS_25"),
        .SLEW("FAST")
    ) obufds_copi (
        .O(spi_copi_p),
        .OB(spi_copi_n),
        .I(copi)
    );
    
    // LVDS input buffers for incoming signals (Master inputs)
    // These convert differential LVDS to single-ended signals
    
    (* IOB = "TRUE" *)
    (* IOSTANDARD = "LVDS_25" *)
    IBUFDS #(
        .IOSTANDARD("LVDS_25"),
        .DIFF_TERM("TRUE")   // Enable differential termination
    ) ibufds_cipo0 (
        .O(cipo0),       // Single-ended output
        .I(spi_cipo0_p),     // Positive input
        .IB(spi_cipo0_n)     // Negative input
    );
    
    (* IOB = "TRUE" *)
    (* IOSTANDARD = "LVDS_25" *)
    IBUFDS #(
        .IOSTANDARD("LVDS_25"),
        .DIFF_TERM("TRUE")   // Enable differential termination
    ) ibufds_cipo1 (
        .O(cipo1),
        .I(spi_cipo1_p),
        .IB(spi_cipo1_n)
    );

endmodule