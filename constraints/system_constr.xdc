# constraints

# ad9361 0
set_property -dict {PACKAGE_PIN AE22 IOSTANDARD LVDS_25} [get_ports ref_clk_p]
set_property -dict {PACKAGE_PIN AF22 IOSTANDARD LVDS_25} [get_ports ref_clk_n]
#set_property -dict {PACKAGE_PIN AF22 IOSTANDARD LVCMOS33} [get_ports ref_clk_n]
set_property -dict {PACKAGE_PIN AF14 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports rx_clk_in_0_p]
set_property -dict {PACKAGE_PIN AG14 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports rx_clk_in_0_n]
set_property -dict {PACKAGE_PIN AA15 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports rx_frame_in_0_p]
set_property -dict {PACKAGE_PIN AA14 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports rx_frame_in_0_n]

set_property -dict {PACKAGE_PIN AB12 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_p[0]}]
set_property -dict {PACKAGE_PIN AC12 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_n[0]}]
set_property -dict {PACKAGE_PIN AC14 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_p[1]}]
set_property -dict {PACKAGE_PIN AC13 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_n[1]}]
set_property -dict {PACKAGE_PIN AD16 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_p[2]}]
set_property -dict {PACKAGE_PIN AD15 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_n[2]}]
set_property -dict {PACKAGE_PIN AE16 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_p[3]}]
set_property -dict {PACKAGE_PIN AE15 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_n[3]}]
set_property -dict {PACKAGE_PIN AE18 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_p[4]}]
set_property -dict {PACKAGE_PIN AE17 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_n[4]}]
set_property -dict {PACKAGE_PIN AF18 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_p[5]}]
set_property -dict {PACKAGE_PIN AF17 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_0_n[5]}]

set_property -dict {PACKAGE_PIN AG17 IOSTANDARD LVDS_25} [get_ports tx_clk_out_0_p]
set_property -dict {PACKAGE_PIN AG16 IOSTANDARD LVDS_25} [get_ports tx_clk_out_0_n]
set_property -dict {PACKAGE_PIN AH17 IOSTANDARD LVDS_25} [get_ports tx_frame_out_0_p]
set_property -dict {PACKAGE_PIN AH16 IOSTANDARD LVDS_25} [get_ports tx_frame_out_0_n]
set_property -dict {PACKAGE_PIN AJ14 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_p[0]}]
set_property -dict {PACKAGE_PIN AJ13 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_n[0]}]
set_property -dict {PACKAGE_PIN AK13 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_p[1]}]
set_property -dict {PACKAGE_PIN AK12 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_n[1]}]
set_property -dict {PACKAGE_PIN AJ15 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_p[2]}]
set_property -dict {PACKAGE_PIN AK15 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_n[2]}]
set_property -dict {PACKAGE_PIN AE12 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_p[3]}]
set_property -dict {PACKAGE_PIN AF12 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_n[3]}]
set_property -dict {PACKAGE_PIN AH18 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_p[4]}]
set_property -dict {PACKAGE_PIN AJ18 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_n[4]}]
set_property -dict {PACKAGE_PIN AJ16 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_p[5]}]
set_property -dict {PACKAGE_PIN AK16 IOSTANDARD LVDS_25} [get_ports {tx_data_out_0_n[5]}]


set_property -dict {PACKAGE_PIN AB22 IOSTANDARD LVCMOS33} [get_ports mcs_sync]
set_property -dict {PACKAGE_PIN AG19 IOSTANDARD LVCMOS33} [get_ports gpio_resetb_0]
set_property -dict {PACKAGE_PIN AB21 IOSTANDARD LVCMOS33} [get_ports enable_0]
set_property -dict {PACKAGE_PIN Y21 IOSTANDARD LVCMOS33} [get_ports gpio_en_agc_0]

set_property -dict {PACKAGE_PIN AJ20 IOSTANDARD LVCMOS33} [get_ports txnrx_0]

set_property -dict {PACKAGE_PIN AJ24 IOSTANDARD LVCMOS33} [get_ports {gpio_status_0[0]}]
set_property -dict {PACKAGE_PIN AK23 IOSTANDARD LVCMOS33} [get_ports {gpio_status_0[1]}]
set_property -dict {PACKAGE_PIN AJ23 IOSTANDARD LVCMOS33} [get_ports {gpio_status_0[2]}]
set_property -dict {PACKAGE_PIN AH24 IOSTANDARD LVCMOS33} [get_ports {gpio_status_0[3]}]
set_property -dict {PACKAGE_PIN AH23 IOSTANDARD LVCMOS33} [get_ports {gpio_status_0[4]}]
set_property -dict {PACKAGE_PIN AJ21 IOSTANDARD LVCMOS33} [get_ports {gpio_status_0[5]}]
set_property -dict {PACKAGE_PIN AK22 IOSTANDARD LVCMOS33} [get_ports {gpio_status_0[6]}]
set_property -dict {PACKAGE_PIN AK21 IOSTANDARD LVCMOS33} [get_ports {gpio_status_0[7]}]

set_property -dict {PACKAGE_PIN AD24 IOSTANDARD LVCMOS33} [get_ports {gpio_ctl_0[0]}]
set_property -dict {PACKAGE_PIN AF23 IOSTANDARD LVCMOS33} [get_ports {gpio_ctl_0[1]}]
set_property -dict {PACKAGE_PIN AK18 IOSTANDARD LVCMOS33} [get_ports {gpio_ctl_0[2]}]
set_property -dict {PACKAGE_PIN AG24 IOSTANDARD LVCMOS33} [get_ports {gpio_ctl_0[3]}]



# spi 0
set_property PACKAGE_PIN AK17 [get_ports spi_csn_0]
set_property IOSTANDARD LVCMOS33 [get_ports spi_csn_0]
set_property PULLUP true [get_ports spi_csn_0]
set_property -dict {PACKAGE_PIN AK20 IOSTANDARD LVCMOS33} [get_ports spi_clk_0]
set_property -dict {PACKAGE_PIN AH19 IOSTANDARD LVCMOS33} [get_ports spi_mosi_0]
set_property -dict {PACKAGE_PIN W21 IOSTANDARD LVCMOS33} [get_ports spi_miso_0]



# ad9361 1
set_property -dict {PACKAGE_PIN AC28 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports rx_clk_in_1_p]
set_property -dict {PACKAGE_PIN AD28 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports rx_clk_in_1_n]
set_property -dict {PACKAGE_PIN AE25 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports rx_frame_in_1_p]
set_property -dict {PACKAGE_PIN AF25 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports rx_frame_in_1_n]

set_property -dict {PACKAGE_PIN AJ30 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_p[0]}]
set_property -dict {PACKAGE_PIN AK30 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_n[0]}]
set_property -dict {PACKAGE_PIN AJ28 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_p[1]}]
set_property -dict {PACKAGE_PIN AJ29 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_n[1]}]
set_property -dict {PACKAGE_PIN AH28 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_p[2]}]
set_property -dict {PACKAGE_PIN AH29 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_n[2]}]
set_property -dict {PACKAGE_PIN AD30 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_p[3]}]
set_property -dict {PACKAGE_PIN AE30 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_n[3]}]
set_property -dict {PACKAGE_PIN AF30 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_p[4]}]
set_property -dict {PACKAGE_PIN AG30 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_n[4]}]
set_property -dict {PACKAGE_PIN AF29 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_p[5]}]
set_property -dict {PACKAGE_PIN AG29 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports {rx_data_in_1_n[5]}]

set_property -dict {PACKAGE_PIN AD14 IOSTANDARD LVDS_25} [get_ports tx_clk_out_1_p]
set_property -dict {PACKAGE_PIN AD13 IOSTANDARD LVDS_25} [get_ports tx_clk_out_1_n]
set_property -dict {PACKAGE_PIN AH14 IOSTANDARD LVDS_25} [get_ports tx_frame_out_1_p]
set_property -dict {PACKAGE_PIN AH13 IOSTANDARD LVDS_25} [get_ports tx_frame_out_1_n]

set_property -dict {PACKAGE_PIN AB29 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_p[0]}]
set_property -dict {PACKAGE_PIN AB30 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_n[0]}]
set_property -dict {PACKAGE_PIN Y26 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_p[1]}]
set_property -dict {PACKAGE_PIN Y27 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_n[1]}]
set_property -dict {PACKAGE_PIN AA27 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_p[2]}]
set_property -dict {PACKAGE_PIN AA28 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_n[2]}]
set_property -dict {PACKAGE_PIN AB25 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_p[3]}]
set_property -dict {PACKAGE_PIN AB26 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_n[3]}]
set_property -dict {PACKAGE_PIN Y28 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_p[4]}]
set_property -dict {PACKAGE_PIN AA29 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_n[4]}]
set_property -dict {PACKAGE_PIN Y30 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_p[5]}]
set_property -dict {PACKAGE_PIN AA30 IOSTANDARD LVDS_25} [get_ports {tx_data_out_1_n[5]}]

set_property -dict {PACKAGE_PIN P25 IOSTANDARD LVCMOS33} [get_ports txnrx_1]
set_property -dict {PACKAGE_PIN T28 IOSTANDARD LVCMOS33} [get_ports gpio_resetb_1]
set_property -dict {PACKAGE_PIN N26 IOSTANDARD LVCMOS33} [get_ports enable_1]
set_property -dict {PACKAGE_PIN T24 IOSTANDARD LVCMOS33} [get_ports gpio_en_agc_1]

set_property -dict {PACKAGE_PIN P30 IOSTANDARD LVCMOS33} [get_ports {gpio_status_1[0]}]
set_property -dict {PACKAGE_PIN R30 IOSTANDARD LVCMOS33} [get_ports {gpio_status_1[1]}]
set_property -dict {PACKAGE_PIN T30 IOSTANDARD LVCMOS33} [get_ports {gpio_status_1[2]}]
set_property -dict {PACKAGE_PIN W29 IOSTANDARD LVCMOS33} [get_ports {gpio_status_1[3]}]
set_property -dict {PACKAGE_PIN W30 IOSTANDARD LVCMOS33} [get_ports {gpio_status_1[4]}]
set_property -dict {PACKAGE_PIN U30 IOSTANDARD LVCMOS33} [get_ports {gpio_status_1[5]}]
set_property -dict {PACKAGE_PIN V27 IOSTANDARD LVCMOS33} [get_ports {gpio_status_1[6]}]
set_property -dict {PACKAGE_PIN V29 IOSTANDARD LVCMOS33} [get_ports {gpio_status_1[7]}]

set_property -dict {PACKAGE_PIN N29 IOSTANDARD LVCMOS33} [get_ports {gpio_ctl_1[0]}]
set_property -dict {PACKAGE_PIN N28 IOSTANDARD LVCMOS33} [get_ports {gpio_ctl_1[1]}]
set_property -dict {PACKAGE_PIN P28 IOSTANDARD LVCMOS33} [get_ports {gpio_ctl_1[2]}]
set_property -dict {PACKAGE_PIN P29 IOSTANDARD LVCMOS33} [get_ports {gpio_ctl_1[3]}]


# spi 1
set_property PACKAGE_PIN T29 [get_ports spi_csn_1]
set_property IOSTANDARD LVCMOS33 [get_ports spi_csn_1]
set_property PULLUP true [get_ports spi_csn_1]
set_property -dict {PACKAGE_PIN R28 IOSTANDARD LVCMOS33} [get_ports spi_clk_1]
set_property -dict {PACKAGE_PIN N27 IOSTANDARD LVCMOS33} [get_ports spi_mosi_1]
set_property -dict {PACKAGE_PIN W28 IOSTANDARD LVCMOS33} [get_ports spi_miso_1]



# gpio bd
# gps m8n
set_property -dict {PACKAGE_PIN V23 IOSTANDARD LVCMOS33} [get_ports rxd_uart_out]
set_property -dict {PACKAGE_PIN U24 IOSTANDARD LVCMOS33} [get_ports txd_uart_in]
set_property -dict {PACKAGE_PIN U22 IOSTANDARD LVCMOS33} [get_ports gps_pps]
set_property -dict {PACKAGE_PIN V24 IOSTANDARD LVCMOS33} [get_ports pps_led]
# dac 5311
set_property -dict {PACKAGE_PIN W23 IOSTANDARD LVCMOS33} [get_ports CLK_DAC_SDI]
set_property -dict {PACKAGE_PIN AJ25 IOSTANDARD LVCMOS33} [get_ports CLK_DAC_CS]
set_property -dict {PACKAGE_PIN AK25 IOSTANDARD LVCMOS33} [get_ports CLK_DAC_SCLK]
## LED
#set_property -dict {PACKAGE_PIN AA22 IOSTANDARD LVCMOS33} [get_ports pl_led[0]]
#set_property -dict {PACKAGE_PIN AA23 IOSTANDARD LVCMOS33} [get_ports pl_led[1]]



# clocks
create_clock -period 4.000 -name rx_0_clk [get_ports rx_clk_in_0_p]
create_clock -period 4.000 -name rx_1_clk [get_ports rx_clk_in_1_p]
#create_clock -period 4.000 -name ref_clk [get_ports ref_clk_p]
create_clock -period 25.000 -name ref_clk [get_ports ref_clk_n]

set_clock_groups -asynchronous -group [get_clocks rx_0_clk] -group [get_clocks rx_1_clk]






create_debug_core u_ila_0 ila
set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU_CNT 2 [get_debug_cores u_ila_0]
set_property C_ADV_TRIGGER false [get_debug_cores u_ila_0]
set_property C_DATA_DEPTH 16384 [get_debug_cores u_ila_0]
set_property C_EN_STRG_QUAL true [get_debug_cores u_ila_0]
set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
set_property port_width 1 [get_debug_ports u_ila_0/clk]
connect_debug_port u_ila_0/clk [get_nets [list i_system_wrapper/system_i/util_ad9361_divclk/inst/clk_out]]
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe0]
set_property port_width 16 [get_debug_ports u_ila_0/probe0]
connect_debug_port u_ila_0/probe0 [get_nets [list {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[0]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[1]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[2]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[3]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[4]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[5]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[6]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[7]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[8]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[9]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[10]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[11]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[12]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[13]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[14]} {i_system_wrapper/system_i/ad9361_2_1030/magnitude_approx_0_magnitude[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe1]
set_property port_width 16 [get_debug_ports u_ila_0/probe1]
connect_debug_port u_ila_0/probe1 [get_nets [list {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[0]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[1]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[2]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[3]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[4]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[5]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[6]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[7]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[8]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[9]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[10]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[11]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[12]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[13]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[14]} {i_system_wrapper/system_i/ad9361_1_1030/magnitude_approx_0_magnitude[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe2]
set_property port_width 16 [get_debug_ports u_ila_0/probe2]
connect_debug_port u_ila_0/probe2 [get_nets [list {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[0]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[1]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[2]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[3]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[4]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[5]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[6]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[7]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[8]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[9]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[10]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[11]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[12]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[13]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[14]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_0[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe3]
set_property port_width 16 [get_debug_ports u_ila_0/probe3]
connect_debug_port u_ila_0/probe3 [get_nets [list {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[0]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[1]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[2]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[3]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[4]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[5]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[6]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[7]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[8]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[9]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[10]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[11]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[12]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[13]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[14]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_3[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe4]
set_property port_width 16 [get_debug_ports u_ila_0/probe4]
connect_debug_port u_ila_0/probe4 [get_nets [list {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[0]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[1]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[2]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[3]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[4]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[5]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[6]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[7]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[8]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[9]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[10]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[11]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[12]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[13]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[14]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_7[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe5]
set_property port_width 16 [get_debug_ports u_ila_0/probe5]
connect_debug_port u_ila_0/probe5 [get_nets [list {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[0]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[1]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[2]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[3]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[4]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[5]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[6]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[7]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[8]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[9]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[10]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[11]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[12]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[13]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[14]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_1[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe6]
set_property port_width 16 [get_debug_ports u_ila_0/probe6]
connect_debug_port u_ila_0/probe6 [get_nets [list {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[0]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[1]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[2]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[3]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[4]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[5]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[6]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[7]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[8]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[9]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[10]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[11]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[12]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[13]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[14]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_4[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe7]
set_property port_width 16 [get_debug_ports u_ila_0/probe7]
connect_debug_port u_ila_0/probe7 [get_nets [list {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[0]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[1]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[2]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[3]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[4]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[5]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[6]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[7]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[8]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[9]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[10]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[11]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[12]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[13]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[14]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_6[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe8]
set_property port_width 16 [get_debug_ports u_ila_0/probe8]
connect_debug_port u_ila_0/probe8 [get_nets [list {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[0]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[1]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[2]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[3]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[4]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[5]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[6]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[7]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[8]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[9]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[10]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[11]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[12]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[13]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[14]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_2[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe9]
set_property port_width 16 [get_debug_ports u_ila_0/probe9]
connect_debug_port u_ila_0/probe9 [get_nets [list {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[0]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[1]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[2]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[3]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[4]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[5]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[6]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[7]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[8]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[9]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[10]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[11]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[12]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[13]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[14]} {i_system_wrapper/system_i/util_ad9361_adc_fifo_dout_data_5[15]}]]
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets u_ila_0_clk_out]
