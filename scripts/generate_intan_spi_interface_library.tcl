# Fixed approach: Create interface definitions and save them properly
# This creates the correct directory structure that Vivado expects

# Set output directory
set lib_name "intan_spi_interface_lib"
set output_dir [pwd]/$lib_name

# Clean up and create directory structure
if {[file exists $output_dir]} {
    file delete -force $output_dir
}

# Create the full directory structure that Vivado expects
file mkdir $output_dir
file mkdir $output_dir/kemerelab.org
file mkdir $output_dir/kemerelab.org/intan
file mkdir $output_dir/kemerelab.org/intan/intan_spi
file mkdir $output_dir/kemerelab.org/intan/intan_spi/1.0
file mkdir $output_dir/kemerelab.org/intan/intan_spi_rtl
file mkdir $output_dir/kemerelab.org/intan/intan_spi_rtl/1.0
file mkdir $output_dir/kemerelab.org/intan/intan_spi_diff
file mkdir $output_dir/kemerelab.org/intan/intan_spi_diff/1.0
file mkdir $output_dir/kemerelab.org/intan/intan_spi_diff_rtl
file mkdir $output_dir/kemerelab.org/intan/intan_spi_diff_rtl/1.0

puts "Creating Intan SPI interfaces (single-ended and differential) with proper file structure..."

# Create single-ended bus definition
set bus_def [ipx::create_bus_definition kemerelab.org intan intan_spi 1.0]
set_property display_name "Intan SPI Dual CIPO Interface" $bus_def
set_property description "Custom SPI interface for Intan RHD chips with dual Controller-In-Peripheral-Out channels" $bus_def

# Create single-ended abstraction definition  
set abs_def [ipx::create_abstraction_definition kemerelab.org intan intan_spi_rtl 1.0]
set_property display_name "Intan SPI RTL Interface" $abs_def
set_property description "RTL-level abstraction for Intan SPI interface with dual CIPO channels" $abs_def
set_property bus_type_vlnv kemerelab.org:intan:intan_spi:1.0 $abs_def

# Create differential bus definition
set bus_def_diff [ipx::create_bus_definition kemerelab.org intan intan_spi_diff 1.0]
set_property display_name "Intan SPI Differential Dual CIPO Interface" $bus_def_diff
set_property description "Custom differential SPI interface for Intan RHD chips with dual Controller-In-Peripheral-Out channels" $bus_def_diff

# Create differential abstraction definition  
set abs_def_diff [ipx::create_abstraction_definition kemerelab.org intan intan_spi_diff_rtl 1.0]
set_property display_name "Intan SPI Differential RTL Interface" $abs_def_diff
set_property description "RTL-level abstraction for differential Intan SPI interface with dual CIPO channels" $abs_def_diff
set_property bus_type_vlnv kemerelab.org:intan:intan_spi_diff:1.0 $abs_def_diff

# Add ports to single-ended abstraction definition
# SCLK
ipx::add_bus_abstraction_port sclk $abs_def
set sclk_port [ipx::get_bus_abstraction_ports sclk -of_objects $abs_def]
set_property master_presence required $sclk_port
set_property slave_presence required $sclk_port  
set_property master_direction out $sclk_port
set_property slave_direction in $sclk_port

# CSN
ipx::add_bus_abstraction_port csn $abs_def
set csn_port [ipx::get_bus_abstraction_ports csn -of_objects $abs_def]
set_property master_presence required $csn_port
set_property slave_presence required $csn_port
set_property master_direction out $csn_port
set_property slave_direction in $csn_port

# COPI
ipx::add_bus_abstraction_port copi $abs_def
set copi_port [ipx::get_bus_abstraction_ports copi -of_objects $abs_def]
set_property master_presence required $copi_port
set_property slave_presence required $copi_port
set_property master_direction out $copi_port
set_property slave_direction in $copi_port

# CIPO0
ipx::add_bus_abstraction_port cipo0 $abs_def
set cipo0_port [ipx::get_bus_abstraction_ports cipo0 -of_objects $abs_def]
set_property master_presence required $cipo0_port
set_property slave_presence required $cipo0_port
set_property master_direction in $cipo0_port
set_property slave_direction out $cipo0_port

# CIPO1
ipx::add_bus_abstraction_port cipo1 $abs_def
set cipo1_port [ipx::get_bus_abstraction_ports cipo1 -of_objects $abs_def]
set_property master_presence required $cipo1_port
set_property slave_presence required $cipo1_port
set_property master_direction in $cipo1_port
set_property slave_direction out $cipo1_port

# Add ports to differential abstraction definition
# SCLK differential pair
ipx::add_bus_abstraction_port sclk_p $abs_def_diff
set sclk_p_port [ipx::get_bus_abstraction_ports sclk_p -of_objects $abs_def_diff]
set_property master_presence required $sclk_p_port
set_property slave_presence required $sclk_p_port  
set_property master_direction out $sclk_p_port
set_property slave_direction in $sclk_p_port

ipx::add_bus_abstraction_port sclk_n $abs_def_diff
set sclk_n_port [ipx::get_bus_abstraction_ports sclk_n -of_objects $abs_def_diff]
set_property master_presence required $sclk_n_port
set_property slave_presence required $sclk_n_port  
set_property master_direction out $sclk_n_port
set_property slave_direction in $sclk_n_port

# CSN differential pair
ipx::add_bus_abstraction_port csn_p $abs_def_diff
set csn_p_port [ipx::get_bus_abstraction_ports csn_p -of_objects $abs_def_diff]
set_property master_presence required $csn_p_port
set_property slave_presence required $csn_p_port
set_property master_direction out $csn_p_port
set_property slave_direction in $csn_p_port

ipx::add_bus_abstraction_port csn_n $abs_def_diff
set csn_n_port [ipx::get_bus_abstraction_ports csn_n -of_objects $abs_def_diff]
set_property master_presence required $csn_n_port
set_property slave_presence required $csn_n_port
set_property master_direction out $csn_n_port
set_property slave_direction in $csn_n_port

# COPI differential pair
ipx::add_bus_abstraction_port copi_p $abs_def_diff
set copi_p_port [ipx::get_bus_abstraction_ports copi_p -of_objects $abs_def_diff]
set_property master_presence required $copi_p_port
set_property slave_presence required $copi_p_port
set_property master_direction out $copi_p_port
set_property slave_direction in $copi_p_port

ipx::add_bus_abstraction_port copi_n $abs_def_diff
set copi_n_port [ipx::get_bus_abstraction_ports copi_n -of_objects $abs_def_diff]
set_property master_presence required $copi_n_port
set_property slave_presence required $copi_n_port
set_property master_direction out $copi_n_port
set_property slave_direction in $copi_n_port

# CIPO0 differential pair
ipx::add_bus_abstraction_port cipo0_p $abs_def_diff
set cipo0_p_port [ipx::get_bus_abstraction_ports cipo0_p -of_objects $abs_def_diff]
set_property master_presence required $cipo0_p_port
set_property slave_presence required $cipo0_p_port
set_property master_direction in $cipo0_p_port
set_property slave_direction out $cipo0_p_port

ipx::add_bus_abstraction_port cipo0_n $abs_def_diff
set cipo0_n_port [ipx::get_bus_abstraction_ports cipo0_n -of_objects $abs_def_diff]
set_property master_presence required $cipo0_n_port
set_property slave_presence required $cipo0_n_port
set_property master_direction in $cipo0_n_port
set_property slave_direction out $cipo0_n_port

# CIPO1 differential pair
ipx::add_bus_abstraction_port cipo1_p $abs_def_diff
set cipo1_p_port [ipx::get_bus_abstraction_ports cipo1_p -of_objects $abs_def_diff]
set_property master_presence required $cipo1_p_port
set_property slave_presence required $cipo1_p_port
set_property master_direction in $cipo1_p_port
set_property slave_direction out $cipo1_p_port

ipx::add_bus_abstraction_port cipo1_n $abs_def_diff
set cipo1_n_port [ipx::get_bus_abstraction_ports cipo1_n -of_objects $abs_def_diff]
set_property master_presence required $cipo1_n_port
set_property slave_presence required $cipo1_n_port
set_property master_direction in $cipo1_n_port
set_property slave_direction out $cipo1_n_port

# CRITICAL: Save to the correct locations with explicit paths
set bus_def_path "$output_dir/kemerelab.org/intan/intan_spi/1.0"
set abs_def_path "$output_dir/kemerelab.org/intan/intan_spi_rtl/1.0"
set bus_def_diff_path "$output_dir/kemerelab.org/intan/intan_spi_diff/1.0"
set abs_def_diff_path "$output_dir/kemerelab.org/intan/intan_spi_diff_rtl/1.0"

# Set the file paths before saving
set_property xml_file_name "$bus_def_path/intan_spi.xml" $bus_def
set_property xml_file_name "$abs_def_path/intan_spi_rtl.xml" $abs_def
set_property xml_file_name "$bus_def_diff_path/intan_spi_diff.xml" $bus_def_diff
set_property xml_file_name "$abs_def_diff_path/intan_spi_diff_rtl.xml" $abs_def_diff

# Save the definitions
ipx::save_bus_definition $bus_def
ipx::save_abstraction_definition $abs_def
ipx::save_bus_definition $bus_def_diff
ipx::save_abstraction_definition $abs_def_diff

# Verify files were created
if {[file exists "$bus_def_path/intan_spi.xml"]} {
    puts "✓ Single-ended bus definition saved to: $bus_def_path/intan_spi.xml"
} else {
    puts "✗ ERROR: Single-ended bus definition not saved!"
}

if {[file exists "$abs_def_path/intan_spi_rtl.xml"]} {
    puts "✓ Single-ended abstraction definition saved to: $abs_def_path/intan_spi_rtl.xml"
} else {
    puts "✗ ERROR: Single-ended abstraction definition not saved!"
}

if {[file exists "$bus_def_diff_path/intan_spi_diff.xml"]} {
    puts "✓ Differential bus definition saved to: $bus_def_diff_path/intan_spi_diff.xml"
} else {
    puts "✗ ERROR: Differential bus definition not saved!"
}

if {[file exists "$abs_def_diff_path/intan_spi_diff_rtl.xml"]} {
    puts "✓ Differential abstraction definition saved to: $abs_def_diff_path/intan_spi_diff_rtl.xml"
} else {
    puts "✗ ERROR: Differential abstraction definition not saved!"
}

# Create a catalog file that helps Vivado discover the interfaces
set catalog_xml {<?xml version="1.0" encoding="UTF-8"?>
<catalog xmlns="urn:oasis:names:tc:entity:xmlns:xml:catalog">
  <rewriteURI uriStartString="kemerelab.org/intan/intan_spi/1.0/" rewritePrefix="kemerelab.org/intan/intan_spi/1.0/"/>
  <rewriteURI uriStartString="kemerelab.org/intan/intan_spi_rtl/1.0/" rewritePrefix="kemerelab.org/intan/intan_spi_rtl/1.0/"/>
  <rewriteURI uriStartString="kemerelab.org/intan/intan_spi_diff/1.0/" rewritePrefix="kemerelab.org/intan/intan_spi_diff/1.0/"/>
  <rewriteURI uriStartString="kemerelab.org/intan/intan_spi_diff_rtl/1.0/" rewritePrefix="kemerelab.org/intan/intan_spi_diff_rtl/1.0/"/>
</catalog>}

set catalog_file [open "$output_dir/catalog.xml" w]
puts $catalog_file $catalog_xml
close $catalog_file

# Create component.xml for the library
set component_xml {<?xml version="1.0" encoding="UTF-8"?>
<spirit:component xmlns:xilinx="http://www.xilinx.com" xmlns:spirit="http://www.spiritconsortium.org/XMLSchema/SPIRIT/1685-2009" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <spirit:vendor>kemerelab.org</spirit:vendor>
  <spirit:library>intan</spirit:library>  
  <spirit:name>intan_spi_interface_lib</spirit:name>
  <spirit:version>1.0</spirit:version>
  <spirit:description>Library containing Intan SPI dual-channel interface definitions</spirit:description>
</spirit:component>}

set component_file [open "$output_dir/component.xml" w]
puts $component_file $component_xml
close $component_file

# Create README
set readme_content {# Intan SPI Interface Library

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
}

set readme_file [open "$output_dir/README.md" w]
puts $readme_file $readme_content
close $readme_file

puts "=================================="
puts "Intan SPI Interface Library Created Successfully!"
puts "=================================="
puts "Library location: $output_dir"
puts ""
puts "Interfaces created:"
puts "1. Single-ended: kemerelab.org:intan:intan_spi:1.0"
puts "2. Differential: kemerelab.org:intan:intan_spi_diff:1.0"
puts ""
puts "Next steps:"
puts "1. In Vivado: Settings → IP → Repository → Add Repository"
puts "2. Point to: $output_dir"
puts "3. Use the interface annotations in your Verilog code"
puts ""
puts "Files created:"
puts "  - $bus_def_path/intan_spi.xml"
puts "  - $abs_def_path/intan_spi_rtl.xml"
puts "  - $bus_def_diff_path/intan_spi_diff.xml"
puts "  - $abs_def_diff_path/intan_spi_diff_rtl.xml"
puts "  - $output_dir/component.xml"
puts "  - $output_dir/catalog.xml"
puts "=================================="