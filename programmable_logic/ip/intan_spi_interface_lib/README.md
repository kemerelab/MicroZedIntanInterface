# Intan SPI Interface Library

This library provides custom SPI interface definitions optimized for Intan RHD chips with dual CIPO channels.
Two interface variants are available: single-ended and differential signaling.

## Interface Variants:

### 1. Single-Ended Interface (intan_spi)
- **VLNV**: kemerelab.org:intan:intan_spi:1.0
- **Signals**: sclk, csn, copi, cipo0, cipo1

### 2. Differential Interface (intan_spi_diff)  
- **VLNV**: kemerelab.org:intan:intan_spi_diff:1.0
- **Signals**: sclk_p/n, csn_p/n, copi_p/n, cipo0_p/n, cipo1_p/n

## Setup Instructions:

### 1. Add to Vivado Project:
- Open your Vivado project
- Go to Settings → IP → Repository
- Click the '+' button to add IP Repository
- Browse to and select this directory: intan_spi_interface_lib
- Click OK and Apply

### 2. Verify Installation:
After adding the repository, you should see "intan_spi_interface_lib" in the IP repositories list.

### 3. Use in Verilog Code:

#### Single-Ended Interface:
```verilog
// Intan SPI Single-Ended Interface
(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi sclk" *)
output wire spi_sclk,

(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi csn" *)
output wire spi_csn,

(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi copi" *)
output wire spi_copi,

(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi cipo0" *)
input wire spi_cipo0,

(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi cipo1" *)
input wire spi_cipo1,
```

#### Differential Interface:
```verilog
// Intan SPI Differential Interface
(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 intan_spi_diff sclk_p" *)
output wire spi_sclk_p,
(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 intan_spi_diff sclk_n" *)
output wire spi_sclk_n,

(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 intan_spi_diff csn_p" *)
output wire spi_csn_p,
(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 intan_spi_diff csn_n" *)
output wire spi_csn_n,

(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 intan_spi_diff copi_p" *)
output wire spi_copi_p,
(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 intan_spi_diff copi_n" *)
output wire spi_copi_n,

(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 intan_spi_diff cipo0_p" *)
input wire spi_cipo0_p,
(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 intan_spi_diff cipo0_n" *)
input wire spi_cipo0_n,

(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 intan_spi_diff cipo1_p" *)
input wire spi_cipo1_p,
(* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi_diff:1.0 intan_spi_diff cipo1_n" *)
input wire spi_cipo1_n,
```

### 4. Package as IP:
When you package your RTL as IP (Tools → Create and Package New IP), 
the interface annotations will cause the signals to be grouped 
into a single bus interface in block design.

## Troubleshooting:
If you get "Interface VLNV not found" errors:
1. Verify the interface library is added to IP repositories
2. Check that the XML files exist in the correct subdirectories
3. Try refreshing the IP catalog (IP Catalog → Refresh)
4. Restart Vivado if necessary

## Interface Details:
- **Purpose**: Optimized for Intan RHD neurophysiology chips
- **Features**: Dual CIPO channels for enhanced data throughput
- **Type**: Point-to-point interface (1 master, 1 slave)
- **Signaling**: Both single-ended and differential variants supported

