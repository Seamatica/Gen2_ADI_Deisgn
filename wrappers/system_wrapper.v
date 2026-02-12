//Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2019.1 (win64) Build 2552052 Fri May 24 14:49:42 MDT 2019
//Date        : Mon Feb  9 15:48:14 2026
//Host        : DESKTOP-V4M9N49 running 64-bit major release  (build 9200)
//Command     : generate_target system_wrapper.bd
//Design      : system_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module system_wrapper
   (UART_1_rxd,
    UART_1_txd,
    dac_cs,
    dac_sclk,
    dac_sdi,
    ddr_addr,
    ddr_ba,
    ddr_cas_n,
    ddr_ck_n,
    ddr_ck_p,
    ddr_cke,
    ddr_cs_n,
    ddr_dm,
    ddr_dq,
    ddr_dqs_n,
    ddr_dqs_p,
    ddr_odt,
    ddr_ras_n,
    ddr_reset_n,
    ddr_we_n,
    enable_0,
    enable_1,
    fixed_io_ddr_vrn,
    fixed_io_ddr_vrp,
    fixed_io_mio,
    fixed_io_ps_clk,
    fixed_io_ps_porb,
    fixed_io_ps_srstb,
    gpio_i,
    gpio_o,
    gpio_t,
    gps_pps,
    hdmi_data,
    hdmi_data_e,
    hdmi_hsync,
    hdmi_out_clk,
    hdmi_vsync,
    iic_mux_scl_i,
    iic_mux_scl_o,
    iic_mux_scl_t,
    iic_mux_sda_i,
    iic_mux_sda_o,
    iic_mux_sda_t,
    rx_clk_in_0_n,
    rx_clk_in_0_p,
    rx_clk_in_1_n,
    rx_clk_in_1_p,
    rx_data_in_0_n,
    rx_data_in_0_p,
    rx_data_in_1_n,
    rx_data_in_1_p,
    rx_frame_in_0_n,
    rx_frame_in_0_p,
    rx_frame_in_1_n,
    rx_frame_in_1_p,
    spdif,
    spi0_clk_i,
    spi0_clk_o,
    spi0_csn_0_o,
    spi0_csn_1_o,
    spi0_csn_2_o,
    spi0_csn_i,
    spi0_sdi_i,
    spi0_sdo_i,
    spi0_sdo_o,
    spi1_clk_i,
    spi1_clk_o,
    spi1_csn_0_o,
    spi1_csn_1_o,
    spi1_csn_2_o,
    spi1_csn_i,
    spi1_sdi_i,
    spi1_sdo_i,
    spi1_sdo_o,
    sys_100m_resetn,
    tx_clk_out_0_n,
    tx_clk_out_0_p,
    tx_clk_out_1_n,
    tx_clk_out_1_p,
    tx_data_out_0_n,
    tx_data_out_0_p,
    tx_data_out_1_n,
    tx_data_out_1_p,
    tx_frame_out_0_n,
    tx_frame_out_0_p,
    tx_frame_out_1_n,
    tx_frame_out_1_p,
    txnrx_0,
    txnrx_1,
    up_enable_0,
    up_enable_1,
    up_txnrx_0,
    up_txnrx_1);
  input UART_1_rxd;
  output UART_1_txd;
  output dac_cs;
  output dac_sclk;
  output dac_sdi;
  inout [14:0]ddr_addr;
  inout [2:0]ddr_ba;
  inout ddr_cas_n;
  inout ddr_ck_n;
  inout ddr_ck_p;
  inout ddr_cke;
  inout ddr_cs_n;
  inout [3:0]ddr_dm;
  inout [31:0]ddr_dq;
  inout [3:0]ddr_dqs_n;
  inout [3:0]ddr_dqs_p;
  inout ddr_odt;
  inout ddr_ras_n;
  inout ddr_reset_n;
  inout ddr_we_n;
  output enable_0;
  output enable_1;
  inout fixed_io_ddr_vrn;
  inout fixed_io_ddr_vrp;
  inout [53:0]fixed_io_mio;
  inout fixed_io_ps_clk;
  inout fixed_io_ps_porb;
  inout fixed_io_ps_srstb;
  input [63:0]gpio_i;
  output [63:0]gpio_o;
  output [63:0]gpio_t;
  input gps_pps;
  output [23:0]hdmi_data;
  output hdmi_data_e;
  output hdmi_hsync;
  output hdmi_out_clk;
  output hdmi_vsync;
  input [1:0]iic_mux_scl_i;
  output [1:0]iic_mux_scl_o;
  output iic_mux_scl_t;
  input [1:0]iic_mux_sda_i;
  output [1:0]iic_mux_sda_o;
  output iic_mux_sda_t;
  input rx_clk_in_0_n;
  input rx_clk_in_0_p;
  input rx_clk_in_1_n;
  input rx_clk_in_1_p;
  input [5:0]rx_data_in_0_n;
  input [5:0]rx_data_in_0_p;
  input [5:0]rx_data_in_1_n;
  input [5:0]rx_data_in_1_p;
  input rx_frame_in_0_n;
  input rx_frame_in_0_p;
  input rx_frame_in_1_n;
  input rx_frame_in_1_p;
  output spdif;
  input spi0_clk_i;
  output spi0_clk_o;
  output spi0_csn_0_o;
  output spi0_csn_1_o;
  output spi0_csn_2_o;
  input spi0_csn_i;
  input spi0_sdi_i;
  input spi0_sdo_i;
  output spi0_sdo_o;
  input spi1_clk_i;
  output spi1_clk_o;
  output spi1_csn_0_o;
  output spi1_csn_1_o;
  output spi1_csn_2_o;
  input spi1_csn_i;
  input spi1_sdi_i;
  input spi1_sdo_i;
  output spi1_sdo_o;
  output [0:0]sys_100m_resetn;
  output tx_clk_out_0_n;
  output tx_clk_out_0_p;
  output tx_clk_out_1_n;
  output tx_clk_out_1_p;
  output [5:0]tx_data_out_0_n;
  output [5:0]tx_data_out_0_p;
  output [5:0]tx_data_out_1_n;
  output [5:0]tx_data_out_1_p;
  output tx_frame_out_0_n;
  output tx_frame_out_0_p;
  output tx_frame_out_1_n;
  output tx_frame_out_1_p;
  output txnrx_0;
  output txnrx_1;
  input up_enable_0;
  input up_enable_1;
  input up_txnrx_0;
  input up_txnrx_1;

  wire UART_1_rxd;
  wire UART_1_txd;
  wire dac_cs;
  wire dac_sclk;
  wire dac_sdi;
  wire [14:0]ddr_addr;
  wire [2:0]ddr_ba;
  wire ddr_cas_n;
  wire ddr_ck_n;
  wire ddr_ck_p;
  wire ddr_cke;
  wire ddr_cs_n;
  wire [3:0]ddr_dm;
  wire [31:0]ddr_dq;
  wire [3:0]ddr_dqs_n;
  wire [3:0]ddr_dqs_p;
  wire ddr_odt;
  wire ddr_ras_n;
  wire ddr_reset_n;
  wire ddr_we_n;
  wire enable_0;
  wire enable_1;
  wire fixed_io_ddr_vrn;
  wire fixed_io_ddr_vrp;
  wire [53:0]fixed_io_mio;
  wire fixed_io_ps_clk;
  wire fixed_io_ps_porb;
  wire fixed_io_ps_srstb;
  wire [63:0]gpio_i;
  wire [63:0]gpio_o;
  wire [63:0]gpio_t;
  wire gps_pps;
  wire [23:0]hdmi_data;
  wire hdmi_data_e;
  wire hdmi_hsync;
  wire hdmi_out_clk;
  wire hdmi_vsync;
  wire [1:0]iic_mux_scl_i;
  wire [1:0]iic_mux_scl_o;
  wire iic_mux_scl_t;
  wire [1:0]iic_mux_sda_i;
  wire [1:0]iic_mux_sda_o;
  wire iic_mux_sda_t;
  wire rx_clk_in_0_n;
  wire rx_clk_in_0_p;
  wire rx_clk_in_1_n;
  wire rx_clk_in_1_p;
  wire [5:0]rx_data_in_0_n;
  wire [5:0]rx_data_in_0_p;
  wire [5:0]rx_data_in_1_n;
  wire [5:0]rx_data_in_1_p;
  wire rx_frame_in_0_n;
  wire rx_frame_in_0_p;
  wire rx_frame_in_1_n;
  wire rx_frame_in_1_p;
  wire spdif;
  wire spi0_clk_i;
  wire spi0_clk_o;
  wire spi0_csn_0_o;
  wire spi0_csn_1_o;
  wire spi0_csn_2_o;
  wire spi0_csn_i;
  wire spi0_sdi_i;
  wire spi0_sdo_i;
  wire spi0_sdo_o;
  wire spi1_clk_i;
  wire spi1_clk_o;
  wire spi1_csn_0_o;
  wire spi1_csn_1_o;
  wire spi1_csn_2_o;
  wire spi1_csn_i;
  wire spi1_sdi_i;
  wire spi1_sdo_i;
  wire spi1_sdo_o;
  wire [0:0]sys_100m_resetn;
  wire tx_clk_out_0_n;
  wire tx_clk_out_0_p;
  wire tx_clk_out_1_n;
  wire tx_clk_out_1_p;
  wire [5:0]tx_data_out_0_n;
  wire [5:0]tx_data_out_0_p;
  wire [5:0]tx_data_out_1_n;
  wire [5:0]tx_data_out_1_p;
  wire tx_frame_out_0_n;
  wire tx_frame_out_0_p;
  wire tx_frame_out_1_n;
  wire tx_frame_out_1_p;
  wire txnrx_0;
  wire txnrx_1;
  wire up_enable_0;
  wire up_enable_1;
  wire up_txnrx_0;
  wire up_txnrx_1;

  system system_i
       (.UART_1_rxd(UART_1_rxd),
        .UART_1_txd(UART_1_txd),
        .dac_cs(dac_cs),
        .dac_sclk(dac_sclk),
        .dac_sdi(dac_sdi),
        .ddr_addr(ddr_addr),
        .ddr_ba(ddr_ba),
        .ddr_cas_n(ddr_cas_n),
        .ddr_ck_n(ddr_ck_n),
        .ddr_ck_p(ddr_ck_p),
        .ddr_cke(ddr_cke),
        .ddr_cs_n(ddr_cs_n),
        .ddr_dm(ddr_dm),
        .ddr_dq(ddr_dq),
        .ddr_dqs_n(ddr_dqs_n),
        .ddr_dqs_p(ddr_dqs_p),
        .ddr_odt(ddr_odt),
        .ddr_ras_n(ddr_ras_n),
        .ddr_reset_n(ddr_reset_n),
        .ddr_we_n(ddr_we_n),
        .enable_0(enable_0),
        .enable_1(enable_1),
        .fixed_io_ddr_vrn(fixed_io_ddr_vrn),
        .fixed_io_ddr_vrp(fixed_io_ddr_vrp),
        .fixed_io_mio(fixed_io_mio),
        .fixed_io_ps_clk(fixed_io_ps_clk),
        .fixed_io_ps_porb(fixed_io_ps_porb),
        .fixed_io_ps_srstb(fixed_io_ps_srstb),
        .gpio_i(gpio_i),
        .gpio_o(gpio_o),
        .gpio_t(gpio_t),
        .gps_pps(gps_pps),
        .hdmi_data(hdmi_data),
        .hdmi_data_e(hdmi_data_e),
        .hdmi_hsync(hdmi_hsync),
        .hdmi_out_clk(hdmi_out_clk),
        .hdmi_vsync(hdmi_vsync),
        .iic_mux_scl_i(iic_mux_scl_i),
        .iic_mux_scl_o(iic_mux_scl_o),
        .iic_mux_scl_t(iic_mux_scl_t),
        .iic_mux_sda_i(iic_mux_sda_i),
        .iic_mux_sda_o(iic_mux_sda_o),
        .iic_mux_sda_t(iic_mux_sda_t),
        .rx_clk_in_0_n(rx_clk_in_0_n),
        .rx_clk_in_0_p(rx_clk_in_0_p),
        .rx_clk_in_1_n(rx_clk_in_1_n),
        .rx_clk_in_1_p(rx_clk_in_1_p),
        .rx_data_in_0_n(rx_data_in_0_n),
        .rx_data_in_0_p(rx_data_in_0_p),
        .rx_data_in_1_n(rx_data_in_1_n),
        .rx_data_in_1_p(rx_data_in_1_p),
        .rx_frame_in_0_n(rx_frame_in_0_n),
        .rx_frame_in_0_p(rx_frame_in_0_p),
        .rx_frame_in_1_n(rx_frame_in_1_n),
        .rx_frame_in_1_p(rx_frame_in_1_p),
        .spdif(spdif),
        .spi0_clk_i(spi0_clk_i),
        .spi0_clk_o(spi0_clk_o),
        .spi0_csn_0_o(spi0_csn_0_o),
        .spi0_csn_1_o(spi0_csn_1_o),
        .spi0_csn_2_o(spi0_csn_2_o),
        .spi0_csn_i(spi0_csn_i),
        .spi0_sdi_i(spi0_sdi_i),
        .spi0_sdo_i(spi0_sdo_i),
        .spi0_sdo_o(spi0_sdo_o),
        .spi1_clk_i(spi1_clk_i),
        .spi1_clk_o(spi1_clk_o),
        .spi1_csn_0_o(spi1_csn_0_o),
        .spi1_csn_1_o(spi1_csn_1_o),
        .spi1_csn_2_o(spi1_csn_2_o),
        .spi1_csn_i(spi1_csn_i),
        .spi1_sdi_i(spi1_sdi_i),
        .spi1_sdo_i(spi1_sdo_i),
        .spi1_sdo_o(spi1_sdo_o),
        .sys_100m_resetn(sys_100m_resetn),
        .tx_clk_out_0_n(tx_clk_out_0_n),
        .tx_clk_out_0_p(tx_clk_out_0_p),
        .tx_clk_out_1_n(tx_clk_out_1_n),
        .tx_clk_out_1_p(tx_clk_out_1_p),
        .tx_data_out_0_n(tx_data_out_0_n),
        .tx_data_out_0_p(tx_data_out_0_p),
        .tx_data_out_1_n(tx_data_out_1_n),
        .tx_data_out_1_p(tx_data_out_1_p),
        .tx_frame_out_0_n(tx_frame_out_0_n),
        .tx_frame_out_0_p(tx_frame_out_0_p),
        .tx_frame_out_1_n(tx_frame_out_1_n),
        .tx_frame_out_1_p(tx_frame_out_1_p),
        .txnrx_0(txnrx_0),
        .txnrx_1(txnrx_1),
        .up_enable_0(up_enable_0),
        .up_enable_1(up_enable_1),
        .up_txnrx_0(up_txnrx_0),
        .up_txnrx_1(up_txnrx_1));
endmodule
