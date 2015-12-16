# ------------------------------------------------------------------------------
# SDC for B&R Antares Interface
# ------------------------------------------------------------------------------

source ../../common/timing/sram.sdc
source ../../common/timing/jtag.sdc

# ------------------------------------------------------------------------------
# Clock definitions
# -> Define clocks in design (depends on PLL settings!)
#    (under "Compilation Report" - "TimeQuest Timing Analyzer" - "Clocks")
# -> Derive PLL clocks

set ext_clk     EXT_CLK
set clk50       pllInst|altpll_component|auto_generated|pll1|clk[0]
set clk100      pllInst|altpll_component|auto_generated|pll1|clk[1]
set clk25       pllInst|altpll_component|auto_generated|pll1|clk[2]
set clk125      pllInst|altpll_component|auto_generated|pll1|clk[3]
set clkpciecore inst|pcie_subsystem|pcie_ip|pcie_internal_hip|cyclone_iii.cycloneiv_hssi_pcie_hip|coreclkout

derive_pll_clocks -create_base_clocks
derive_clock_uncertainty

# -> Ignore phy ref clock for now
set_false_path -from * -to [get_ports oRmiiRefClk]

# ------------------------------------------------------------------------------
# SRAM definitions

timing_sram $clk100

# ------------------------------------------------------------------------------
# JTAG definitions

timing_jtag

# ------------------------------------------------------------------------------
# EPCS
# -> Cut path
set_false_path -from [get_registers *]      -to [get_ports oFlash_Clk]
set_false_path -from [get_registers *]      -to [get_ports oFlash_nCS]
set_false_path -from [get_registers *]      -to [get_ports oFlash_DI]
set_false_path -from [get_ports iFlash_DO]  -to [get_registers *]

# ------------------------------------------------------------------------------
# PCIe
# -> Cut from/to pcie core clock
set_clock_groups -asynchronous -group [get_clocks $clkpciecore]

# ------------------------------------------------------------------------------
# PHY LINK and LEDs
# -> Cut path
set_false_path -from [get_ports iLinkPlkPhy]    -to *
set_false_path -from *                          -to [get_ports onPlkLinkLed]
set_false_path -from *                          -to [get_ports onPlkActLed]
set_false_path -from *                          -to [get_ports onPlkStatLedRot]
set_false_path -from *                          -to [get_ports onPlkStatLedGruen]
set_false_path -from *                          -to [get_ports onPlkActLedGelb]
set_false_path -from *                          -to [get_ports onReserveLed]
