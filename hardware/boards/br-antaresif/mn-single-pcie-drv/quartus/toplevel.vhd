-------------------------------------------------------------------------------
--! @file toplevel.vhd
--
--! @brief Toplevel of Nios MN design Pcp part
--
--! @details This is the toplevel of the Nios MN FPGA Pcp design for the
--! B&R Antares Interface.
--
-------------------------------------------------------------------------------
--
--    (c) B&R, 2014
--    (c) Kalycito Infotech Private Limited, 2015
--
--    Redistribution and use in source and binary forms, with or without
--    modification, are permitted provided that the following conditions
--    are met:
--
--    1. Redistributions of source code must retain the above copyright
--       notice, this list of conditions and the following disclaimer.
--
--    2. Redistributions in binary form must reproduce the above copyright
--       notice, this list of conditions and the following disclaimer in the
--       documentation and/or other materials provided with the distribution.
--
--    3. Neither the name of B&R nor the names of its
--       contributors may be used to endorse or promote products derived
--       from this software without prior written permission. For written
--       permission, please contact office@br-automation.com
--
--    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
--    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
--    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
--    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
--    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
--    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
--    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
--    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
--    POSSIBILITY OF SUCH DAMAGE.
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library libcommon;
use libcommon.global.all;

entity toplevel is
    port (
        iClk                : in    std_logic;
        inReset             : in    std_logic;
        -- PHY Interfaces
        oRmiiRefClk         : out   std_logic;
        iRmiiRxErr          : in    std_logic_vector(0 downto 0);
        iRmiiCrsDv          : in    std_logic_vector(0 downto 0);
        iRmiiRxData         : in    std_logic_vector(1 downto 0);
        oRmiiTxEn           : out   std_logic_vector(0 downto 0);
        oRmiiTxData         : out   std_logic_vector(1 downto 0);
        bMDIOPlkPhy         : inout std_logic_vector(0 downto 0);
        oMDCPlkPhy          : out   std_logic_vector(0 downto 0);
        onPlkPhyRst         : out   std_logic_vector(0 downto 0);
        iLinkPlkPhy         : in    std_logic;
        -- Configuration FLASH
        oFlash_Clk          : out   std_logic;
        oFlash_nCS          : out   std_logic;
        oFlash_DI           : out   std_logic;
        iFlash_DO           : in    std_logic;
        -- 256x16 SRAM
        onPlkRamOE          : out   std_logic;
        onPlkRamWE          : out   std_logic;
        oPlkRamAddr         : out   std_logic_vector(18 downto 0);
        onPlkRamBE          : out   std_logic_vector(1 downto 0);
        bPlkRamData         : inout std_logic_vector(15 downto 0);
        -- PCIe interface
        iPCIe_Rx1p     : in  std_ulogic;
        oPCIe_Tx1p     : out std_ulogic;
        iPCIe_RefClk_p : in  std_ulogic;
        -- LED
        onPlkLinkLed        : out   std_logic;
        onPlkActLed         : out   std_logic;
        onPlkStatLedRot     : out   std_logic;
        onPlkStatLedGruen   : out   std_logic;
        onPlkActLedGelb     : out   std_logic;
        onReserveLed        : out   std_logic;

        oDbgTxD             : out   std_logic;
        iDbgRxD             : in    std_logic;
        oDbgRTS             : out   std_logic;
        iDbgCTS             : in    std_logic;

        iIF1RxD             : in    std_logic;
        oIF1TxD             : out   std_logic;
        iIF2RxD             : in    std_logic;
        oIF2TxD             : out   std_logic;
        oIF2OE              : out   std_logic;
        oIF1RxDLed          : out   std_logic;
        oIF1TxDLed          : out   std_logic;
        oIF2RxDLed          : out   std_logic;
        oIF2TxDLed          : out   std_logic;

        inConfigIF1CAN      : in    std_logic;
        inConfigIF2CAN      : in    std_logic;
        inConfigIF2RS485    : in    std_logic;
        inConfigIF2RS232    : in    std_logic;
        inConfigX2X         : in    std_logic;
        inConfigPLK         : in    std_logic;
        inLegacyRS232       : in    std_logic;
        inConfigFPGAType    : in    std_logic;
        inConfigUndef1      : in    std_logic;
        inConfigUndef2      : in    std_logic;
        inHWK0              : in    std_logic;
        inHWK1              : in    std_logic;
        oDCDCEn             : out   std_logic;
        onClkReq            : out   std_logic;
        nConfig_PG_CRC      : out   std_logic
    );
end toplevel;

architecture rtl of toplevel is

    component mnSinglePcieDrv is
        port (
            clk25_clk                                   : in    std_logic;
            clk50_clk                                   : in    std_logic;
            clk100_clk                                  : in    std_logic;
            clk125_clk                                  : in    std_logic;
            reset_reset_n                               : in    std_logic;
            tri_state_0_tcm_address_out                 : out   std_logic_vector(18 downto 0);
            tri_state_0_tcm_byteenable_n_out            : out   std_logic_vector(1 downto 0);
            tri_state_0_tcm_read_n_out                  : out   std_logic;
            tri_state_0_tcm_write_n_out                 : out   std_logic;
            tri_state_0_tcm_data_out                    : inout std_logic_vector(15 downto 0);
            tri_state_0_tcm_chipselect_n_out            : out   std_logic;
            pcp_0_benchmark_pio_export                  : out   std_logic_vector(7 downto 0);
            openmac_0_smi_nPhyRst                       : out   std_logic_vector(0 downto 0);
            openmac_0_smi_clk                           : out   std_logic_vector(0 downto 0);
            openmac_0_smi_dio                           : inout std_logic_vector(0 downto 0);
            openmac_0_rmii_txEnable                     : out   std_logic_vector(0 downto 0);
            openmac_0_rmii_txData                       : out   std_logic_vector(1 downto 0);
            openmac_0_rmii_rxError                      : in    std_logic_vector(0 downto 0);
            openmac_0_rmii_rxCrsDataValid               : in    std_logic_vector(0 downto 0);
            openmac_0_rmii_rxData                       : in    std_logic_vector(1 downto 0);
            openmac_0_pktactivity_export                : out   std_logic;
            epcs_flash_dclk                             : out   std_logic;
            epcs_flash_sce                              : out   std_logic;
            epcs_flash_sdo                              : out   std_logic;
            epcs_flash_data0                            : in    std_logic;
            pcie_ip_rx_in_rx_datain_0                   : in    std_logic                     := 'X';             -- rx_datain_0
            pcie_ip_tx_out_tx_dataout_0                 : out   std_logic;                                        -- tx_dataout_0
            pcie_ip_reconfig_togxb_data                 : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- data
            pcie_ip_reconfig_fromgxb_0_data             : out   std_logic_vector(4 downto 0);                     -- data
            pcie_ip_refclk_export                       : in    std_logic                     := 'X';
            pcie_ip_test_in_test_in                     : in    std_logic_vector(39 downto 0);
            pcie_ip_pcie_rstn_export                    : in    std_logic                     := 'X';             -- export
            pcie_ip_clocks_sim_clk250_export            : out   std_logic;                                        -- clk250_export
            pcie_ip_clocks_sim_clk500_export            : out   std_logic;                                        -- clk500_export
            pcie_ip_clocks_sim_clk125_export            : out   std_logic;                                        -- clk125_export
            pcie_ip_reconfig_busy_busy_altgxb_reconfig  : in    std_logic                     := 'X';             -- busy_altgxb_reconfig
            pcie_ip_pipe_ext_pipe_mode                  : in    std_logic                     := 'X';             -- pipe_mode
            pcie_ip_pipe_ext_phystatus_ext              : in    std_logic                     := 'X';             -- phystatus_ext
            pcie_ip_pipe_ext_rate_ext                   : out   std_logic;                                        -- rate_ext
            pcie_ip_pipe_ext_powerdown_ext              : out   std_logic_vector(1 downto 0);                     -- powerdown_ext
            pcie_ip_pipe_ext_txdetectrx_ext             : out   std_logic;                                        -- txdetectrx_ext
            pcie_ip_pipe_ext_rxelecidle0_ext            : in    std_logic                     := 'X';             -- rxelecidle0_ext
            pcie_ip_pipe_ext_rxdata0_ext                : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- rxdata0_ext
            pcie_ip_pipe_ext_rxstatus0_ext              : in    std_logic_vector(2 downto 0)  := (others => 'X'); -- rxstatus0_ext
            pcie_ip_pipe_ext_rxvalid0_ext               : in    std_logic                     := 'X';             -- rxvalid0_ext
            pcie_ip_pipe_ext_rxdatak0_ext               : in    std_logic                     := 'X';             -- rxdatak0_ext
            pcie_ip_pipe_ext_txdata0_ext                : out   std_logic_vector(7 downto 0);                     -- txdata0_ext
            pcie_ip_pipe_ext_txdatak0_ext               : out   std_logic;                                        -- txdatak0_ext
            pcie_ip_pipe_ext_rxpolarity0_ext            : out   std_logic;                                        -- rxpolarity0_ext
            pcie_ip_pipe_ext_txcompl0_ext               : out   std_logic;                                        -- txcompl0_ext
            pcie_ip_pipe_ext_txelecidle0_ext            : out   std_logic;                                        -- txelecidle0_ext
            pcie_ip_powerdown_pll_powerdown             : in    std_logic                     := 'X';             -- pll_powerdown
            pcie_ip_powerdown_gxb_powerdown             : in    std_logic                     := 'X';             -- gxb_powerdown
            pcie_0_benchmark_pio_export                 : out   std_logic_vector(7 downto 0);
            pcp_0_cpu_resetrequest_resetrequest         : in    std_logic;
            pcp_0_cpu_resetrequest_resettaken           : out   std_logic;
            pcp_0_powerlink_led_export                  : out   std_logic_vector(1 downto 0);
            testport_pio_export                         : out   std_logic_vector(7 downto 0)
        );
    end component mnSinglePcieDrv;

    -- PLL component
    component pll
        port (
            inclk0  : in std_logic;
            c0      : out std_logic;
            c1      : out std_logic;
            c2      : out std_logic;
            c3      : out std_logic;
            locked  : out std_logic
        );
    end component;

    signal clk25        : std_logic;
    signal clk50        : std_logic;
    signal clk100       : std_logic;
    signal clk125       : std_logic;
    signal pllLocked    : std_logic;

    -- GX reconfig component
    component gxReconfig
        port (
            reconfig_clk        : in std_logic;
            reconfig_fromgxb    : in std_logic_vector(4 downto 0);
            busy                : out std_logic;
            reconfig_togxb      : out std_logic_vector(3 downto 0)
        );
    end component;

    signal reconfigToGxb    : std_logic_vector(3 downto 0);
    signal reconfigFromGxb  : std_logic_vector(4 downto 0);
    signal reconfigBusy     : std_logic;

    signal plkSeLed         : std_logic_vector(1 downto 0);
    alias  plkStatusLed     : std_logic is plkSeLed(0);
    alias  plkErrorLed      : std_logic is plkSeLed(1);

    signal macActivity      : std_logic;
    signal plkActivity      : std_logic;

    signal testport                 : std_logic_vector(7 downto 0);
    alias  testportEnable           : std_logic is testport(7);
    alias  testportPlkActLed        : std_logic is testport(5);
    alias  testportPlkLinkLed       : std_logic is testport(4);
    alias  testportPlkActLedGelb    : std_logic is testport(3);
    alias  testportReservedLed      : std_logic is testport(2);
    alias  testportStatLedRot       : std_logic is testport(1);
    alias  testportStatLedGruen     : std_logic is testport(0);
begin
    oRmiiRefClk <= clk50; --FIXME: Use phase shift clock?

    nConfig_PG_CRC <= 'Z'; --FIXME: Connect to remote update control for factory reconfig!

    ----------------------------------------------------------------------------
    -- LEDs
    --FIXME: Mismatch data sheet LEDs / schematic

    plkActivity <= iLinkPlkPhy and not macActivity; -- On = Link / Blink = Activity

    -- LED RJ45
    onPlkActLed         <=  not testportPlkActLed when testportEnable = cActivated else
                            not plkActivity;
    onPlkLinkLed        <=  not testportPlkLinkLed when testportEnable = cActivated else
                            not plkStatusLed;

    -- LED pair red/green L2
    onPlkActLedGelb     <=  not testportPlkActLedGelb when testportEnable = cActivated else
                            not plkActivity;
    onReserveLed        <=  not testportReservedLed when testportEnable = cActivated else
                            cnInactivated; -- Unused

    -- LED pair red/green L3
    onPlkStatLedRot     <=  not testportStatLedRot when testportEnable = cActivated else
                            not plkErrorLed;
    onPlkStatLedGruen   <=  not testportStatLedGruen when testportEnable = cActivated else
                            not plkStatusLed;
    ----------------------------------------------------------------------------

    inst : component mnSinglePcieDrv
        port map (
            clk25_clk                                   => clk25,
            clk50_clk                                   => clk50,
            clk100_clk                                  => clk100,
            clk125_clk                                  => clk125,
            reset_reset_n                               => pllLocked,

            pcp_0_cpu_resetrequest_resetrequest         => '0',
            pcp_0_cpu_resetrequest_resettaken           => open,

            tri_state_0_tcm_address_out                 => oPlkRamAddr,
            tri_state_0_tcm_read_n_out                  => onPlkRamOE,
            tri_state_0_tcm_byteenable_n_out            => onPlkRamBE,
            tri_state_0_tcm_write_n_out                 => onPlkRamWE,
            tri_state_0_tcm_data_out                    => bPlkRamData,
            tri_state_0_tcm_chipselect_n_out            => open,

            pcp_0_benchmark_pio_export                  => open,

            openmac_0_smi_nPhyRst                       => onPlkPhyRst,
            openmac_0_smi_clk                           => oMDCPlkPhy,
            openmac_0_smi_dio                           => bMDIOPlkPhy,
            openmac_0_rmii_txEnable                     => oRmiiTxEn,
            openmac_0_rmii_txData                       => oRmiiTxData,
            openmac_0_rmii_rxError                      => iRmiiRxErr,
            openmac_0_rmii_rxCrsDataValid               => iRmiiCrsDv,
            openmac_0_rmii_rxData                       => iRmiiRxData,
            openmac_0_pktactivity_export                => macActivity,

            epcs_flash_dclk                             => oFlash_Clk,
            epcs_flash_sce                              => oFlash_nCS,
            epcs_flash_sdo                              => oFlash_DI,
            epcs_flash_data0                            => iFlash_DO,

            pcie_ip_rx_in_rx_datain_0                   => iPCIe_Rx1p,
            pcie_ip_tx_out_tx_dataout_0                 => oPCIe_Tx1p,
            pcie_ip_reconfig_togxb_data                 => reconfigToGxb,
            pcie_ip_reconfig_fromgxb_0_data             => reconfigFromGxb,
            pcie_ip_reconfig_busy_busy_altgxb_reconfig  => reconfigBusy,
            pcie_ip_refclk_export                       => iPCIe_RefClk_p,
            pcie_ip_test_in_test_in                     => (others => cInactivated),
            pcie_ip_pcie_rstn_export                    => pllLocked,
            pcie_ip_clocks_sim_clk250_export            => open,
            pcie_ip_clocks_sim_clk500_export            => open,
            pcie_ip_clocks_sim_clk125_export            => open,
            pcie_ip_pipe_ext_pipe_mode                  => cInactivated,
            pcie_ip_pipe_ext_phystatus_ext              => cInactivated,
            pcie_ip_pipe_ext_rate_ext                   => open,
            pcie_ip_pipe_ext_powerdown_ext              => open,
            pcie_ip_pipe_ext_txdetectrx_ext             => open,
            pcie_ip_pipe_ext_rxelecidle0_ext            => cInactivated,
            pcie_ip_pipe_ext_rxdata0_ext                => (others => cInactivated),
            pcie_ip_pipe_ext_rxstatus0_ext              => (others => cInactivated),
            pcie_ip_pipe_ext_rxvalid0_ext               => cInactivated,
            pcie_ip_pipe_ext_rxdatak0_ext               => cInactivated,
            pcie_ip_pipe_ext_txdata0_ext                => open,
            pcie_ip_pipe_ext_txdatak0_ext               => open,
            pcie_ip_pipe_ext_rxpolarity0_ext            => open,
            pcie_ip_pipe_ext_txcompl0_ext               => open,
            pcie_ip_pipe_ext_txelecidle0_ext            => open,
            pcie_ip_powerdown_pll_powerdown             => cInactivated,
            pcie_ip_powerdown_gxb_powerdown             => cInactivated,
            pcie_0_benchmark_pio_export                 => open,
            pcp_0_powerlink_led_export                  => plkSeLed,
            testport_pio_export                         => testport
        );

    -- Pll Instance
    pllInst : pll
        port map (
            inclk0  => iClk,
            c0      => clk50,
            c1      => clk100,
            c2      => clk25,
            c3      => clk125,
            locked  => pllLocked
        );

    -- GX reconfig instance
    gxReconfigInst : gxReconfig
        port map (
            reconfig_clk        => clk50,
            reconfig_fromgxb    => reconfigFromGxb,
            busy                => reconfigBusy,
            reconfig_togxb      => reconfigToGxb
        );
end rtl;
