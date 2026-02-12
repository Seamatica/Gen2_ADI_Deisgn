/***************************************************************************//**
 *   @file   ad9361/src/main.c
 *   @brief  Implementation of Main Function.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <inttypes.h>
#include "app_config.h"
#include "ad9361_api.h"
#include "parameters.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_delay.h"
#ifdef XILINX_PLATFORM
#include <xparameters.h>
#include <xil_cache.h>
#include "xilinx_spi.h"
#include "xilinx_gpio.h"
#include "no_os_irq.h"
#endif

#include "axi_adc_core.h"
#include "axi_dac_core.h"

#include "xilinx_irq.h"


#include "axi_adc_core.h"
#include "axi_dac_core.h"
#include "axi_dmac.h"
#include "no_os_error.h"


#include "xscugic.h"
#include "xscugic_hw.h"
#include "xparameters_ps.h"	/* defines XPAR values */
#include "xgpio.h"
#include "xuartps.h"

#include "netif/xadapter.h"
#include "lwipopts.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/init.h"
#include "lwip/inet.h"
#if LWIP_DHCP==1
#include "lwip/dhcp.h"
extern volatile int dhcp_timoutcntr;
#endif

#include "uart_com.h"
#include "platform.h"
#include "gps_get_second.h"
/******************************************************************************/
/************************ PL intr definitions   *******************************/
/******************************************************************************/
static XScuGic InterruptController;
static XUartPs GPS_Uart_Ps;	//UART

#define GPS_RX_BUFF_SIZE 256
static u8 uartRecvBufferMain[GPS_RX_BUFF_SIZE];



#define DATA_IP_BASEADDR  XPAR_AD9361_1_1030_AXI4LITE_INTR_0_S00_AXI_BASEADDR
#define INTR_IP_BASEADDR  0x43C10000
#define PL_INTERRUPT_ID   87U

#define DATA_IP_BASEADDR_2  XPAR_AD9361_2_1030_AXI4LITE_INTR_0_S00_AXI_BASEADDR
#define INTR_IP_BASEADDR_2  0x43C30000
#define PL_INTERRUPT_ID_2   86U

/* Register Offsets for S_AXI_INTR (Interrupt Controller) */
#define GIE_OFFSET  0x00  // Global Interrupt Enable Register (R/W)
#define IER_OFFSET  0x04  // Interrupt Enable Register (R/W)
#define ISR_OFFSET  0x08  // Interrupt Status Register (R)
#define IAR_OFFSET  0x0C  // Interrupt Acknowledge Register (W)

/* Register Offsets for S00_AXI (Data Registers) */
#define DATA_REG_0  0x00  // Data [31:0]
#define DATA_REG_1  0x04  // Data [63:32]
#define DATA_REG_2  0x08  // Data [95:64]
#define DATA_REG_3  0x0C  // Data [127:96]
#define DATA_REG_4  0x10  // Data [159:128]
#define DATA_REG_5  0x14  // Data [191:160]

/* Interrupt mask for single interrupt (Interrupt 0) */
#define PL_INTR_MASK 0x00000001

volatile int data_ready_flag = 0;
volatile int data_ready_flag_2= 0;


void AXI4_ISR(void *CallbackRef)
{
    volatile int *flag = (volatile int *)CallbackRef;
    Xil_Out32(INTR_IP_BASEADDR + IAR_OFFSET, PL_INTR_MASK);
    *flag = 1;
}

void AXI4_ISR_2(void *CallbackRef)
{
    volatile int *flag = (volatile int *)CallbackRef;
    Xil_Out32(INTR_IP_BASEADDR_2 + IAR_OFFSET, PL_INTR_MASK);
    *flag = 1;
}


void Read192BitData(u32 *data_buffer)
{
    data_buffer[0] = Xil_In32(DATA_IP_BASEADDR + DATA_REG_0);
    data_buffer[1] = Xil_In32(DATA_IP_BASEADDR + DATA_REG_1);
    data_buffer[2] = Xil_In32(DATA_IP_BASEADDR + DATA_REG_2);
    data_buffer[3] = Xil_In32(DATA_IP_BASEADDR + DATA_REG_3);
    data_buffer[4] = Xil_In32(DATA_IP_BASEADDR + DATA_REG_4);
    data_buffer[5] = Xil_In32(DATA_IP_BASEADDR + DATA_REG_5);
}

void Read192BitData_2(u32 *data_buffer)
{
    data_buffer[0] = Xil_In32(DATA_IP_BASEADDR_2 + DATA_REG_0);
    data_buffer[1] = Xil_In32(DATA_IP_BASEADDR_2 + DATA_REG_1);
    data_buffer[2] = Xil_In32(DATA_IP_BASEADDR_2 + DATA_REG_2);
    data_buffer[3] = Xil_In32(DATA_IP_BASEADDR_2 + DATA_REG_3);
    data_buffer[4] = Xil_In32(DATA_IP_BASEADDR_2 + DATA_REG_4);
    data_buffer[5] = Xil_In32(DATA_IP_BASEADDR_2 + DATA_REG_5);
}


uint32_t set_seconds(uint32_t data, uint8_t sec)
{
    sec &= 0x3F;              // keep only lower 6 bits
    data &= ~(0x3F << 10);    // clear bits [15:10]
    data |= (sec << 10);      // insert new seconds
    return data;
}

uint32_t data_192bit[6];

void pack_data_to_sendbuf(char *send_buf, uint32_t *data_192bit)
{
    send_buf[0]  = (data_192bit[5] >> 24) & 0xFF;
    send_buf[1]  = (data_192bit[5] >> 16) & 0xFF;
    send_buf[2]  = (data_192bit[5] >> 8)  & 0xFF;
    send_buf[3]  = (data_192bit[5])       & 0xFF;

    send_buf[4]  = (data_192bit[4] >> 24) & 0xFF;
    send_buf[5]  = (data_192bit[4] >> 16) & 0xFF;
    send_buf[6]  = (data_192bit[4] >> 8)  & 0xFF;
    send_buf[7]  = (data_192bit[4])       & 0xFF;

    send_buf[8]  = (data_192bit[3] >> 24) & 0xFF;
    send_buf[9]  = (data_192bit[3] >> 16) & 0xFF;
    send_buf[10] = (data_192bit[3] >> 8)  & 0xFF;

}



/******************************************************************************/
/************************      GPIO Definitions *******************************/
/******************************************************************************/

//first ad9361
#define AXI_GPIO_DEVICE_ID_0 XPAR_AD9361_1_1030_THRESHOLD_0_DEVICE_ID // threshold, 18 bits
#define AXI_GPIO_DEVICE_ID_1 XPAR_AD9361_1_1030_DEVICE_ID_0_DEVICE_ID // device ID, 16 bits

//second ad9361
#define AXI_GPIO_DEVICE_ID_2 XPAR_AD9361_2_1030_THRESHOLD_1_DEVICE_ID // threshold, 18 bits
#define AXI_GPIO_DEVICE_ID_3 XPAR_AD9361_2_1030_DEVICE_ID_1_DEVICE_ID // device ID, 16 bits


#define GPIO_CHANNEL_1 1 // First GPIO Channel
#define GPIO_CHANNEL_2 2 // Second GPIO Channel
XGpio axi_gpio_inst_0; // AXI GPIO 0 instance, for threshold of first ad9361
XGpio axi_gpio_inst_1; // AXI GPIO 1 instance, for device ID of first ad9361

XGpio axi_gpio_inst_2; // AXI GPIO 2 instance, for threshold of second ad9361
XGpio axi_gpio_inst_3; // AXI GPIO 3 instance, for device ID of second ad9361


/******************************************************************************/
/************************    TCP/IP Definitions *******************************/
/******************************************************************************/

extern volatile int TcpFastTmrFlag;
extern volatile int TcpSlowTmrFlag;

#define DEFAULT_IP_ADDRESS	"192.168.1.50"
#define DEFAULT_IP_MASK		"255.255.255.0"
#define DEFAULT_GW_ADDRESS	"192.168.1.1"


void platform_enable_interrupts(void);
void print_app_header(void);

struct netif server_netif;

/* Client port to connect */
#define UDP_CONN_PORT 1234

/* Server to connect with */
#define UDP_SERVER_IP_ADDRESS "192.168.1.11"

/* 1030 message is 88 bits which is 11 bytes */
#define MESSAGE_SIZE 11

/* UDP buffer length in bytes */
//#define UDP_SEND_BUFSIZE (MESSAGE_SIZE+1)
#define UDP_SEND_BUFSIZE MESSAGE_SIZE

/* Number of parallel UDP clients */
#define NUM_OF_PARALLEL_CLIENTS 1
#define FINISH	1

static struct udp_pcb *pcb;
static char send_buf[UDP_SEND_BUFSIZE];
static char send_buf_2[UDP_SEND_BUFSIZE];

struct pbuf *packet;

void start_udp_application(void)
{
	err_t err;
	ip_addr_t remote_addr;
	u32_t i;

	err = inet_aton(UDP_SERVER_IP_ADDRESS, &remote_addr);
	if (!err) {xil_printf("Invalid Server IP address: %d\r\n", err); return;}

	pcb = udp_new();

	if (!pcb) {xil_printf("Error in PCB creation. out of memory\r\n"); return;}

	err = udp_connect(pcb, &remote_addr, UDP_CONN_PORT);
	if (err != ERR_OK) {
		xil_printf("udp_client: Error on udp_connect: %d\r\n", err);
		udp_remove(pcb);
		return;
	}


	/* Wait for successful connection */
	usleep(10);

	/* initialize data buffer being sent with same as used in iperf */
	for (i = 0; i < UDP_SEND_BUFSIZE; i++)
		send_buf[i] = 0;
}

/** Transmit data on a udp session */
void transfer_udp_data(size_t sndBuffSize)
{
	struct pbuf *local_packet;
	err_t err;

	if (pcb == NULL) return;

	local_packet = pbuf_alloc(PBUF_TRANSPORT, sndBuffSize, PBUF_POOL);
	if (!local_packet) {xil_printf("Radio 1: error allocating pbuf to send\r\n"); return;}

	memcpy(local_packet->payload, send_buf, sndBuffSize);

	err = udp_send(pcb, local_packet);
	if (err != ERR_OK) {xil_printf("Radio 1: Error on udp_send: %d\r\n", err);}

	pbuf_free(local_packet);
}

/** Transmit data on a udp session */
void transfer_udp_data_2(size_t sndBuffSize)
{
	struct pbuf *local_packet_2;
	err_t err;

	if (pcb == NULL) return;

	local_packet_2 = pbuf_alloc(PBUF_TRANSPORT, sndBuffSize, PBUF_POOL);
	if (!local_packet_2) {xil_printf("Radio 2: error allocating pbuf to send\r\n"); return;}

	memcpy(local_packet_2->payload, send_buf_2, sndBuffSize);

	err = udp_send(pcb, local_packet_2);
	if (err != ERR_OK) {xil_printf("Radio 2: Error on udp_send: %d\r\n", err);}

	pbuf_free(local_packet_2);
}


static void print_ip(char *msg, ip_addr_t *ip)
{
	print(msg);
	xil_printf("%d.%d.%d.%d\r\n", ip4_addr1(ip), ip4_addr2(ip),
			ip4_addr3(ip), ip4_addr4(ip));
}

static void print_ip_settings(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{
	print_ip("Board IP:       ", ip);
	print_ip("Netmask :       ", mask);
	print_ip("Gateway :       ", gw);
}

static void assign_default_ip(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{
	int err;

	xil_printf("Configuring default IP %s \r\n", DEFAULT_IP_ADDRESS);

	err = inet_aton(DEFAULT_IP_ADDRESS, ip);
	if (!err)
		xil_printf("Invalid default IP address: %d\r\n", err);

	err = inet_aton(DEFAULT_IP_MASK, mask);
	if (!err)
		xil_printf("Invalid default IP MASK: %d\r\n", err);

	err = inet_aton(DEFAULT_GW_ADDRESS, gw);
	if (!err)
		xil_printf("Invalid default gateway address: %d\r\n", err);
}


/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/

uint16_t adc_buffer[ADC_BUFFER_SAMPLES * ADC_CHANNELS] __attribute__ ((
			aligned));

#define AD9361_ADC_DAC_BYTES_PER_SAMPLE 2

#ifdef XILINX_PLATFORM
struct xil_spi_init_param xil_spi_param = {
#ifdef PLATFORM_MB
	.type = SPI_PL,
#else
	.type = SPI_PS,
#endif
	.flags = 0
};

struct xil_gpio_init_param xil_gpio_param = {
#ifdef PLATFORM_MB
	.type = GPIO_PL,
#else
	.type = GPIO_PS,
#endif
	.device_id = GPIO_DEVICE_ID
};

#define GPIO_OPS	&xil_gpio_ops
#define SPI_OPS		&xil_spi_ops
#define GPIO_PARAM	&xil_gpio_param
#define SPI_PARAM	&xil_spi_param
#endif




struct axi_adc_init rx_adc_init = {
	"cf-ad9361-lpc",
	RX_CORE_BASEADDR,
	4
};
struct axi_dac_init tx_dac_init = {
	"cf-ad9361-dds-core-lpc",
	TX_CORE_BASEADDR,
	4,
	NULL
};
struct axi_dmac_init rx_dmac_init = {
	"rx_dmac",
	CF_AD9361_RX_DMA_BASEADDR,
#ifdef DMA_IRQ_ENABLE
	IRQ_ENABLED
#else
	IRQ_DISABLED
#endif
};
struct axi_dmac *rx_dmac;
struct axi_dmac_init tx_dmac_init = {
	"tx_dmac",
	CF_AD9361_TX_DMA_BASEADDR,
#ifdef DMA_IRQ_ENABLE
	IRQ_ENABLED
#else
	IRQ_DISABLED
#endif
};
struct axi_dmac *tx_dmac;

AD9361_InitParam default_init_param = {
	/* Device selection */
	ID_AD9361,	// dev_sel
	/* Reference Clock */
	40000000UL,	//reference_clk_rate
	/* Base Configuration */
	1,		//two_rx_two_tx_mode_enable *** adi,2rx-2tx-mode-enable
	1,		//one_rx_one_tx_mode_use_rx_num *** adi,1rx-1tx-mode-use-rx-num
	1,		//one_rx_one_tx_mode_use_tx_num *** adi,1rx-1tx-mode-use-tx-num
	1,		//frequency_division_duplex_mode_enable *** adi,frequency-division-duplex-mode-enable
	0,		//frequency_division_duplex_independent_mode_enable *** adi,frequency-division-duplex-independent-mode-enable
	0,		//tdd_use_dual_synth_mode_enable *** adi,tdd-use-dual-synth-mode-enable
	0,		//tdd_skip_vco_cal_enable *** adi,tdd-skip-vco-cal-enable
	0,		//tx_fastlock_delay_ns *** adi,tx-fastlock-delay-ns
	0,		//rx_fastlock_delay_ns *** adi,rx-fastlock-delay-ns
	0,		//rx_fastlock_pincontrol_enable *** adi,rx-fastlock-pincontrol-enable
	0,		//tx_fastlock_pincontrol_enable *** adi,tx-fastlock-pincontrol-enable
	0,		//external_rx_lo_enable *** adi,external-rx-lo-enable
	0,		//external_tx_lo_enable *** adi,external-tx-lo-enable
	5,		//dc_offset_tracking_update_event_mask *** adi,dc-offset-tracking-update-event-mask
	6,		//dc_offset_attenuation_high_range *** adi,dc-offset-attenuation-high-range
	5,		//dc_offset_attenuation_low_range *** adi,dc-offset-attenuation-low-range
	0x28,	//dc_offset_count_high_range *** adi,dc-offset-count-high-range
	0x32,	//dc_offset_count_low_range *** adi,dc-offset-count-low-range
	0,		//split_gain_table_mode_enable *** adi,split-gain-table-mode-enable
	MAX_SYNTH_FREF,	//trx_synthesizer_target_fref_overwrite_hz *** adi,trx-synthesizer-target-fref-overwrite-hz
	0,		// qec_tracking_slow_mode_enable *** adi,qec-tracking-slow-mode-enable
	/* ENSM Control */
	0,		//ensm_enable_pin_pulse_mode_enable *** adi,ensm-enable-pin-pulse-mode-enable
	0,		//ensm_enable_txnrx_control_enable *** adi,ensm-enable-txnrx-control-enable
	/* LO Control */
	2400000000UL,	//rx_synthesizer_frequency_hz *** adi,rx-synthesizer-frequency-hz
	2400000000UL,	//tx_synthesizer_frequency_hz *** adi,tx-synthesizer-frequency-hz
	1,				//tx_lo_powerdown_managed_enable *** adi,tx-lo-powerdown-managed-enable
	/* Rate & BW Control */
	{983040000, 245760000, 122880000, 61440000, 30720000, 30720000},// rx_path_clock_frequencies[6] *** adi,rx-path-clock-frequencies
	{983040000, 122880000, 122880000, 61440000, 30720000, 30720000},// tx_path_clock_frequencies[6] *** adi,tx-path-clock-frequencies
	18000000,//rf_rx_bandwidth_hz *** adi,rf-rx-bandwidth-hz
	18000000,//rf_tx_bandwidth_hz *** adi,rf-tx-bandwidth-hz
	/* RF Port Control */
	0,		//rx_rf_port_input_select *** adi,rx-rf-port-input-select
	0,		//tx_rf_port_input_select *** adi,tx-rf-port-input-select
	/* TX Attenuation Control */
	10000,	//tx_attenuation_mdB *** adi,tx-attenuation-mdB
	0,		//update_tx_gain_in_alert_enable *** adi,update-tx-gain-in-alert-enable
	/* Reference Clock Control */
	0,		//xo_disable_use_ext_refclk_enable *** adi,xo-disable-use-ext-refclk-enable
	{8, 5920},	//dcxo_coarse_and_fine_tune[2] *** adi,dcxo-coarse-and-fine-tune
	CLKOUT_DISABLE,	//clk_output_mode_select *** adi,clk-output-mode-select
	/* Gain Control */
	2,		//gc_rx1_mode *** adi,gc-rx1-mode
	2,		//gc_rx2_mode *** adi,gc-rx2-mode
	58,		//gc_adc_large_overload_thresh *** adi,gc-adc-large-overload-thresh
	4,		//gc_adc_ovr_sample_size *** adi,gc-adc-ovr-sample-size
	47,		//gc_adc_small_overload_thresh *** adi,gc-adc-small-overload-thresh
	8192,	//gc_dec_pow_measurement_duration *** adi,gc-dec-pow-measurement-duration
	0,		//gc_dig_gain_enable *** adi,gc-dig-gain-enable
	800,	//gc_lmt_overload_high_thresh *** adi,gc-lmt-overload-high-thresh
	704,	//gc_lmt_overload_low_thresh *** adi,gc-lmt-overload-low-thresh
	24,		//gc_low_power_thresh *** adi,gc-low-power-thresh
	15,		//gc_max_dig_gain *** adi,gc-max-dig-gain
	0,		//gc_use_rx_fir_out_for_dec_pwr_meas_enable *** adi,gc-use-rx-fir-out-for-dec-pwr-meas-enable
	/* Gain MGC Control */
	2,		//mgc_dec_gain_step *** adi,mgc-dec-gain-step
	2,		//mgc_inc_gain_step *** adi,mgc-inc-gain-step
	0,		//mgc_rx1_ctrl_inp_enable *** adi,mgc-rx1-ctrl-inp-enable
	0,		//mgc_rx2_ctrl_inp_enable *** adi,mgc-rx2-ctrl-inp-enable
	0,		//mgc_split_table_ctrl_inp_gain_mode *** adi,mgc-split-table-ctrl-inp-gain-mode
	/* Gain AGC Control */
	10,		//agc_adc_large_overload_exceed_counter *** adi,agc-adc-large-overload-exceed-counter
	2,		//agc_adc_large_overload_inc_steps *** adi,agc-adc-large-overload-inc-steps
	0,		//agc_adc_lmt_small_overload_prevent_gain_inc_enable *** adi,agc-adc-lmt-small-overload-prevent-gain-inc-enable
	10,		//agc_adc_small_overload_exceed_counter *** adi,agc-adc-small-overload-exceed-counter
	4,		//agc_dig_gain_step_size *** adi,agc-dig-gain-step-size
	3,		//agc_dig_saturation_exceed_counter *** adi,agc-dig-saturation-exceed-counter
	1000,	// agc_gain_update_interval_us *** adi,agc-gain-update-interval-us
	0,		//agc_immed_gain_change_if_large_adc_overload_enable *** adi,agc-immed-gain-change-if-large-adc-overload-enable
	0,		//agc_immed_gain_change_if_large_lmt_overload_enable *** adi,agc-immed-gain-change-if-large-lmt-overload-enable
	10,		//agc_inner_thresh_high *** adi,agc-inner-thresh-high
	1,		//agc_inner_thresh_high_dec_steps *** adi,agc-inner-thresh-high-dec-steps
	12,		//agc_inner_thresh_low *** adi,agc-inner-thresh-low
	1,		//agc_inner_thresh_low_inc_steps *** adi,agc-inner-thresh-low-inc-steps
	10,		//agc_lmt_overload_large_exceed_counter *** adi,agc-lmt-overload-large-exceed-counter
	2,		//agc_lmt_overload_large_inc_steps *** adi,agc-lmt-overload-large-inc-steps
	10,		//agc_lmt_overload_small_exceed_counter *** adi,agc-lmt-overload-small-exceed-counter
	5,		//agc_outer_thresh_high *** adi,agc-outer-thresh-high
	2,		//agc_outer_thresh_high_dec_steps *** adi,agc-outer-thresh-high-dec-steps
	18,		//agc_outer_thresh_low *** adi,agc-outer-thresh-low
	2,		//agc_outer_thresh_low_inc_steps *** adi,agc-outer-thresh-low-inc-steps
	1,		//agc_attack_delay_extra_margin_us; *** adi,agc-attack-delay-extra-margin-us
	0,		//agc_sync_for_gain_counter_enable *** adi,agc-sync-for-gain-counter-enable
	/* Fast AGC */
	64,		//fagc_dec_pow_measuremnt_duration ***  adi,fagc-dec-pow-measurement-duration
	260,	//fagc_state_wait_time_ns ***  adi,fagc-state-wait-time-ns
	/* Fast AGC - Low Power */
	0,		//fagc_allow_agc_gain_increase ***  adi,fagc-allow-agc-gain-increase-enable
	5,		//fagc_lp_thresh_increment_time ***  adi,fagc-lp-thresh-increment-time
	1,		//fagc_lp_thresh_increment_steps ***  adi,fagc-lp-thresh-increment-steps
	/* Fast AGC - Lock Level (Lock Level is set via slow AGC inner high threshold) */
	1,		//fagc_lock_level_lmt_gain_increase_en ***  adi,fagc-lock-level-lmt-gain-increase-enable
	5,		//fagc_lock_level_gain_increase_upper_limit ***  adi,fagc-lock-level-gain-increase-upper-limit
	/* Fast AGC - Peak Detectors and Final Settling */
	1,		//fagc_lpf_final_settling_steps ***  adi,fagc-lpf-final-settling-steps
	1,		//fagc_lmt_final_settling_steps ***  adi,fagc-lmt-final-settling-steps
	3,		//fagc_final_overrange_count ***  adi,fagc-final-overrange-count
	/* Fast AGC - Final Power Test */
	0,		//fagc_gain_increase_after_gain_lock_en ***  adi,fagc-gain-increase-after-gain-lock-enable
	/* Fast AGC - Unlocking the Gain */
	0,		//fagc_gain_index_type_after_exit_rx_mode ***  adi,fagc-gain-index-type-after-exit-rx-mode
	1,		//fagc_use_last_lock_level_for_set_gain_en ***  adi,fagc-use-last-lock-level-for-set-gain-enable
	1,		//fagc_rst_gla_stronger_sig_thresh_exceeded_en ***  adi,fagc-rst-gla-stronger-sig-thresh-exceeded-enable
	5,		//fagc_optimized_gain_offset ***  adi,fagc-optimized-gain-offset
	10,		//fagc_rst_gla_stronger_sig_thresh_above_ll ***  adi,fagc-rst-gla-stronger-sig-thresh-above-ll
	1,		//fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en ***  adi,fagc-rst-gla-engergy-lost-sig-thresh-exceeded-enable
	1,		//fagc_rst_gla_engergy_lost_goto_optim_gain_en ***  adi,fagc-rst-gla-engergy-lost-goto-optim-gain-enable
	10,		//fagc_rst_gla_engergy_lost_sig_thresh_below_ll ***  adi,fagc-rst-gla-engergy-lost-sig-thresh-below-ll
	8,		//fagc_energy_lost_stronger_sig_gain_lock_exit_cnt ***  adi,fagc-energy-lost-stronger-sig-gain-lock-exit-cnt
	1,		//fagc_rst_gla_large_adc_overload_en ***  adi,fagc-rst-gla-large-adc-overload-enable
	1,		//fagc_rst_gla_large_lmt_overload_en ***  adi,fagc-rst-gla-large-lmt-overload-enable
	0,		//fagc_rst_gla_en_agc_pulled_high_en ***  adi,fagc-rst-gla-en-agc-pulled-high-enable
	0,		//fagc_rst_gla_if_en_agc_pulled_high_mode ***  adi,fagc-rst-gla-if-en-agc-pulled-high-mode
	64,		//fagc_power_measurement_duration_in_state5 ***  adi,fagc-power-measurement-duration-in-state5
	2,		//fagc_large_overload_inc_steps *** adi,fagc-adc-large-overload-inc-steps
	/* RSSI Control */
	1,		//rssi_delay *** adi,rssi-delay
	1000,	//rssi_duration *** adi,rssi-duration
	3,		//rssi_restart_mode *** adi,rssi-restart-mode
	0,		//rssi_unit_is_rx_samples_enable *** adi,rssi-unit-is-rx-samples-enable
	1,		//rssi_wait *** adi,rssi-wait
	/* Aux ADC Control */
	256,	//aux_adc_decimation *** adi,aux-adc-decimation
	40000000UL,	//aux_adc_rate *** adi,aux-adc-rate
	/* AuxDAC Control */
	1,		//aux_dac_manual_mode_enable ***  adi,aux-dac-manual-mode-enable
	0,		//aux_dac1_default_value_mV ***  adi,aux-dac1-default-value-mV
	0,		//aux_dac1_active_in_rx_enable ***  adi,aux-dac1-active-in-rx-enable
	0,		//aux_dac1_active_in_tx_enable ***  adi,aux-dac1-active-in-tx-enable
	0,		//aux_dac1_active_in_alert_enable ***  adi,aux-dac1-active-in-alert-enable
	0,		//aux_dac1_rx_delay_us ***  adi,aux-dac1-rx-delay-us
	0,		//aux_dac1_tx_delay_us ***  adi,aux-dac1-tx-delay-us
	0,		//aux_dac2_default_value_mV ***  adi,aux-dac2-default-value-mV
	0,		//aux_dac2_active_in_rx_enable ***  adi,aux-dac2-active-in-rx-enable
	0,		//aux_dac2_active_in_tx_enable ***  adi,aux-dac2-active-in-tx-enable
	0,		//aux_dac2_active_in_alert_enable ***  adi,aux-dac2-active-in-alert-enable
	0,		//aux_dac2_rx_delay_us ***  adi,aux-dac2-rx-delay-us
	0,		//aux_dac2_tx_delay_us ***  adi,aux-dac2-tx-delay-us
	/* Temperature Sensor Control */
	256,	//temp_sense_decimation *** adi,temp-sense-decimation
	1000,	//temp_sense_measurement_interval_ms *** adi,temp-sense-measurement-interval-ms
	0xCE,	//temp_sense_offset_signed *** adi,temp-sense-offset-signed
	1,		//temp_sense_periodic_measurement_enable *** adi,temp-sense-periodic-measurement-enable
	/* Control Out Setup */
	0xFF,	//ctrl_outs_enable_mask *** adi,ctrl-outs-enable-mask
	0,		//ctrl_outs_index *** adi,ctrl-outs-index
	/* External LNA Control */
	0,		//elna_settling_delay_ns *** adi,elna-settling-delay-ns
	0,		//elna_gain_mdB *** adi,elna-gain-mdB
	0,		//elna_bypass_loss_mdB *** adi,elna-bypass-loss-mdB
	0,		//elna_rx1_gpo0_control_enable *** adi,elna-rx1-gpo0-control-enable
	0,		//elna_rx2_gpo1_control_enable *** adi,elna-rx2-gpo1-control-enable
	0,		//elna_gaintable_all_index_enable *** adi,elna-gaintable-all-index-enable
	/* Digital Interface Control */
	0,		//digital_interface_tune_skip_mode *** adi,digital-interface-tune-skip-mode
	0,		//digital_interface_tune_fir_disable *** adi,digital-interface-tune-fir-disable
	1,		//pp_tx_swap_enable *** adi,pp-tx-swap-enable
	1,		//pp_rx_swap_enable *** adi,pp-rx-swap-enable
	0,		//tx_channel_swap_enable *** adi,tx-channel-swap-enable
	0,		//rx_channel_swap_enable *** adi,rx-channel-swap-enable
	1,		//rx_frame_pulse_mode_enable *** adi,rx-frame-pulse-mode-enable
	0,		//two_t_two_r_timing_enable *** adi,2t2r-timing-enable
	0,		//invert_data_bus_enable *** adi,invert-data-bus-enable
	0,		//invert_data_clk_enable *** adi,invert-data-clk-enable
	0,		//fdd_alt_word_order_enable *** adi,fdd-alt-word-order-enable
	0,		//invert_rx_frame_enable *** adi,invert-rx-frame-enable
	0,		//fdd_rx_rate_2tx_enable *** adi,fdd-rx-rate-2tx-enable
	0,		//swap_ports_enable *** adi,swap-ports-enable
	0,		//single_data_rate_enable *** adi,single-data-rate-enable
	1,		//lvds_mode_enable *** adi,lvds-mode-enable
	0,		//half_duplex_mode_enable *** adi,half-duplex-mode-enable
	0,		//single_port_mode_enable *** adi,single-port-mode-enable
	0,		//full_port_enable *** adi,full-port-enable
	0,		//full_duplex_swap_bits_enable *** adi,full-duplex-swap-bits-enable
	0,		//delay_rx_data *** adi,delay-rx-data
	0,		//rx_data_clock_delay *** adi,rx-data-clock-delay
	4,		//rx_data_delay *** adi,rx-data-delay
	7,		//tx_fb_clock_delay *** adi,tx-fb-clock-delay
	0,		//tx_data_delay *** adi,tx-data-delay
#ifdef ALTERA_PLATFORM
	300,	//lvds_bias_mV *** adi,lvds-bias-mV
#else
	150,	//lvds_bias_mV *** adi,lvds-bias-mV
#endif
	1,		//lvds_rx_onchip_termination_enable *** adi,lvds-rx-onchip-termination-enable
	0,		//rx1rx2_phase_inversion_en *** adi,rx1-rx2-phase-inversion-enable
	0xFF,	//lvds_invert1_control *** adi,lvds-invert1-control
	0x0F,	//lvds_invert2_control *** adi,lvds-invert2-control
	/* GPO Control */
	0,		//gpo_manual_mode_enable *** adi,gpo-manual-mode-enable
	0,		//gpo_manual_mode_enable_mask *** adi,gpo-manual-mode-enable-mask
	0,		//gpo0_inactive_state_high_enable *** adi,gpo0-inactive-state-high-enable
	0,		//gpo1_inactive_state_high_enable *** adi,gpo1-inactive-state-high-enable
	0,		//gpo2_inactive_state_high_enable *** adi,gpo2-inactive-state-high-enable
	0,		//gpo3_inactive_state_high_enable *** adi,gpo3-inactive-state-high-enable
	0,		//gpo0_slave_rx_enable *** adi,gpo0-slave-rx-enable
	0,		//gpo0_slave_tx_enable *** adi,gpo0-slave-tx-enable
	0,		//gpo1_slave_rx_enable *** adi,gpo1-slave-rx-enable
	0,		//gpo1_slave_tx_enable *** adi,gpo1-slave-tx-enable
	0,		//gpo2_slave_rx_enable *** adi,gpo2-slave-rx-enable
	0,		//gpo2_slave_tx_enable *** adi,gpo2-slave-tx-enable
	0,		//gpo3_slave_rx_enable *** adi,gpo3-slave-rx-enable
	0,		//gpo3_slave_tx_enable *** adi,gpo3-slave-tx-enable
	0,		//gpo0_rx_delay_us *** adi,gpo0-rx-delay-us
	0,		//gpo0_tx_delay_us *** adi,gpo0-tx-delay-us
	0,		//gpo1_rx_delay_us *** adi,gpo1-rx-delay-us
	0,		//gpo1_tx_delay_us *** adi,gpo1-tx-delay-us
	0,		//gpo2_rx_delay_us *** adi,gpo2-rx-delay-us
	0,		//gpo2_tx_delay_us *** adi,gpo2-tx-delay-us
	0,		//gpo3_rx_delay_us *** adi,gpo3-rx-delay-us
	0,		//gpo3_tx_delay_us *** adi,gpo3-tx-delay-us
	/* Tx Monitor Control */
	37000,	//low_high_gain_threshold_mdB *** adi,txmon-low-high-thresh
	0,		//low_gain_dB *** adi,txmon-low-gain
	24,		//high_gain_dB *** adi,txmon-high-gain
	0,		//tx_mon_track_en *** adi,txmon-dc-tracking-enable
	0,		//one_shot_mode_en *** adi,txmon-one-shot-mode-enable
	511,	//tx_mon_delay *** adi,txmon-delay
	8192,	//tx_mon_duration *** adi,txmon-duration
	2,		//tx1_mon_front_end_gain *** adi,txmon-1-front-end-gain
	2,		//tx2_mon_front_end_gain *** adi,txmon-2-front-end-gain
	48,		//tx1_mon_lo_cm *** adi,txmon-1-lo-cm
	48,		//tx2_mon_lo_cm *** adi,txmon-2-lo-cm
	/* GPIO definitions */
	{
		.number = -1,
		.platform_ops = GPIO_OPS,
		.extra = GPIO_PARAM
	},		//gpio_resetb *** reset-gpios
	/* MCS Sync */
	{
		.number = -1,
		.platform_ops = GPIO_OPS,
		.extra = GPIO_PARAM
	},		//gpio_sync *** sync-gpios

	{
		.number = -1,
		.platform_ops = GPIO_OPS,
		.extra = GPIO_PARAM
	},		//gpio_cal_sw1 *** cal-sw1-gpios

	{
		.number = -1,
		.platform_ops = GPIO_OPS,
		.extra = GPIO_PARAM
	},		//gpio_cal_sw2 *** cal-sw2-gpios

	{
		.device_id = SPI_DEVICE_ID,
		.mode = NO_OS_SPI_MODE_1,
		.chip_select = SPI_CS,
		.platform_ops = SPI_OPS,
		.extra = SPI_PARAM
	},

	/* External LO clocks */
	NULL,	//(*ad9361_rfpll_ext_recalc_rate)()
	NULL,	//(*ad9361_rfpll_ext_round_rate)()
	NULL,	//(*ad9361_rfpll_ext_set_rate)()
#ifndef AXI_ADC_NOT_PRESENT
	&rx_adc_init,	// *rx_adc_init
	&tx_dac_init,   // *tx_dac_init
#endif
};

AD9361_RXFIRConfig rx_fir_config = {	// BPF PASSBAND 3/20 fs to 1/4 fs
	3, // rx
	0, // rx_gain
	1, // rx_dec
	{
		-4, -6, -37, 35, 186, 86, -284, -315,
			107, 219, -4, 271, 558, -307, -1182, -356,
			658, 157, 207, 1648, 790, -2525, -2553, 748,
			865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
			14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
			748, -2553, -2525, 790, 1648, 207, 157, 658,
			-356, -1182, -307, 558, 271, -4, 219, 107,
			-315, -284, 86, 186, 35, -37, -6, -4,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0
		}, // rx_coef[128]
	64, // rx_coef_size
	{0, 0, 0, 0, 0, 0}, //rx_path_clks[6]
	0 // rx_bandwidth
};

AD9361_TXFIRConfig tx_fir_config = {	// BPF PASSBAND 3/20 fs to 1/4 fs
	3, // tx
	-6, // tx_gain
	1, // tx_int
	{
		-4, -6, -37, 35, 186, 86, -284, -315,
			107, 219, -4, 271, 558, -307, -1182, -356,
			658, 157, 207, 1648, 790, -2525, -2553, 748,
			865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
			14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
			748, -2553, -2525, 790, 1648, 207, 157, 658,
			-356, -1182, -307, 558, 271, -4, 219, 107,
			-315, -284, 86, 186, 35, -37, -6, -4,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0
		}, // tx_coef[128]
	64, // tx_coef_size
	{0, 0, 0, 0, 0, 0}, // tx_path_clks[6]
	0 // tx_bandwidth
};
struct ad9361_rf_phy *ad9361_phy;
#ifdef FMCOMMS5
struct ad9361_rf_phy *ad9361_phy_b;
#endif


/***************************************************************************//**
 * @brief main
*******************************************************************************/
int main(void)
{
	int32_t status;
#ifdef XILINX_PLATFORM
	Xil_ICacheEnable();
	Xil_DCacheEnable();
	default_init_param.spi_param.extra = &xil_spi_param;
	default_init_param.spi_param.platform_ops = &xil_spi_ops;
#endif


	// NOTE: The user has to choose the GPIO numbers according to desired
	// carrier board.
	default_init_param.gpio_resetb.number = GPIO_RESET_PIN;

	default_init_param.rx_synthesizer_frequency_hz = 1030000000UL;

	default_init_param.gc_rx1_mode = 0; //manual gain
	default_init_param.gc_rx2_mode = 0; //manual gain

#ifdef FMCOMMS5
	default_init_param.gpio_sync.number = GPIO_SYNC_PIN;
//	default_init_param.gpio_cal_sw1.number = GPIO_CAL_SW1_PIN;
//	default_init_param.gpio_cal_sw2.number = GPIO_CAL_SW2_PIN;
	default_init_param.rx1rx2_phase_inversion_en = 1;
#else
	default_init_param.gpio_sync.number = -1;
	default_init_param.gpio_cal_sw1.number = -1;
	default_init_param.gpio_cal_sw2.number = -1;
#endif

	if (AD9364_DEVICE)
		default_init_param.dev_sel = ID_AD9364;
	if (AD9363A_DEVICE)
		default_init_param.dev_sel = ID_AD9363A;

#if defined FMCOMMS5 || defined ADI_RF_SOM || defined ADI_RF_SOM_CMOS
	default_init_param.xo_disable_use_ext_refclk_enable = 1;
#endif


	//0,6 is a good pair
	default_init_param.rx_data_clock_delay = 0;
	default_init_param.rx_data_delay = 6;


	uint8_t rx_reg = 0;
	uint8_t rx_delay = 0;
	uint8_t rx_fir_status = 1;





	ad9361_init(&ad9361_phy, &default_init_param);
	ad9361_tx_mute(ad9361_phy, 0);

	status = ad9361_get_rx_fir_en_dis(ad9361_phy, &rx_fir_status);
	printf("rx fir enable status: %u\n", rx_fir_status);


	ad9361_set_tx_fir_config(ad9361_phy, tx_fir_config);
	ad9361_set_rx_fir_config(ad9361_phy, rx_fir_config);

	status = ad9361_get_rx_fir_en_dis(ad9361_phy, &rx_fir_status);
	printf("rx fir enable status: %u\n", rx_fir_status);


	rx_reg = ad9361_spi_read(ad9361_phy->spi, 0x006); // RX Data/Clk Delay
	printf("Hardware RX Register (0x006) after phy init: 0x%02X\n", rx_reg);

	rx_delay = ad9361_phy->pdata->port_ctrl.rx_clk_data_delay;
	printf("RX Clock Delay after phy init: %d\n", rx_delay);




//	status = ad9361_bist_tone(ad9361_phy, BIST_INJ_RX, 0x02, 15, 0x00);
//	ad9361_dig_tune(ad9361_phy, 61440000, BE_MOREVERBOSE);



#ifdef FMCOMMS5
//	default_init_param.spi_param.chip_select = SPI_CS_2;
	default_init_param.spi_param.device_id = SPI_DEVICE_ID_1;
	default_init_param.gpio_resetb.number = GPIO_RESET_PIN_2;

	default_init_param.gpio_sync.number = -1;
	default_init_param.gpio_cal_sw1.number = -1;
	default_init_param.gpio_cal_sw2.number = -1;


	default_init_param.rx_adc_init->base = AD9361_RX_1_BASEADDR;
	default_init_param.tx_dac_init->base = AD9361_TX_1_BASEADDR;

	//Good pairs: 4/0, 8/0, 0/8
	default_init_param.rx_data_clock_delay = 0;
	default_init_param.rx_data_delay = 8;


	ad9361_init(&ad9361_phy_b, &default_init_param);
//	ad9361_dig_tune(ad9361_phy_b, 61440000, BE_MOREVERBOSE);
	ad9361_tx_mute(ad9361_phy_b, 0);

	rx_reg = ad9361_spi_read(ad9361_phy_b->spi, 0x006); // RX Data/Clk Delay
	printf("Hardware RX Register (0x006) after phy_b init: 0x%02X\n", rx_reg);
	rx_delay = ad9361_phy_b->pdata->port_ctrl.rx_clk_data_delay;
	printf("RX Clock Delay after phy_b init: %d\n\n", rx_delay);





	ad9361_set_tx_fir_config(ad9361_phy_b, tx_fir_config);
	ad9361_set_rx_fir_config(ad9361_phy_b, rx_fir_config);

//	status = ad9361_bist_tone(ad9361_phy_b, BIST_INJ_RX, 0x02, 15, 0x00);


//	ad9361_do_mcs(ad9361_phy, ad9361_phy_b);

//	ad9361_dig_tune(ad9361_phy, 61440000, BE_MOREVERBOSE);
//	ad9361_dig_tune(ad9361_phy_b, 61440000, BE_MOREVERBOSE);


//	status = ad9361_bist_tone(ad9361_phy,   BIST_INJ_RX, 0x02, 15, 0x00);
//	status = ad9361_bist_tone(ad9361_phy_b, BIST_INJ_RX, 0x02, 15, 0x00);



#endif

// Initialization of GPIOs
//--------------------------------------------------------------------------------
	printf("Initializing AXI GPIO 0...\n");

	// Initialize AXI GPIO 0
	if (XGpio_Initialize(&axi_gpio_inst_0, AXI_GPIO_DEVICE_ID_0) != XST_SUCCESS) {
		printf("Failed to initialize AXI GPIO 0!\n");
		return 1;
	}

	// Set GPIO channel as output
	XGpio_SetDataDirection(&axi_gpio_inst_0, GPIO_CHANNEL_1, 0x00000000);
	printf("Mode AC threshold GPIO (GPIO 0) Initialized Successfully.\n");
//--------------------------------------------------------------------------------
	printf("Initializing AXI GPIO 1...\n");

	// Initialize AXI GPIO 1
	if (XGpio_Initialize(&axi_gpio_inst_1, AXI_GPIO_DEVICE_ID_1) != XST_SUCCESS) {
		printf("Failed to initialize AXI GPIO 1!\n");
		return 1;
	}

	// Set GPIO channel as output
	XGpio_SetDataDirection(&axi_gpio_inst_1, GPIO_CHANNEL_1, 0x00000000);
	printf("Device ID GPIO (GPIO 1) Initialized Successfully.\n");
//--------------------------------------------------------------------------------
	printf("Initializing AXI GPIO 2...\n");

	// Initialize AXI GPIO 2
	if (XGpio_Initialize(&axi_gpio_inst_2, AXI_GPIO_DEVICE_ID_2) != XST_SUCCESS) {
		printf("Failed to initialize AXI GPIO 2!\n");
		return 1;
	}

	// Set GPIO channel as output
	XGpio_SetDataDirection(&axi_gpio_inst_2, GPIO_CHANNEL_1, 0x00000000);
	printf("Mode AC threshold GPIO (GPIO 2) Initialized Successfully.\n");
//--------------------------------------------------------------------------------
	printf("Initializing AXI GPIO 3...\n");

	// Initialize AXI GPIO 3
	if (XGpio_Initialize(&axi_gpio_inst_3, AXI_GPIO_DEVICE_ID_3) != XST_SUCCESS) {
		printf("Failed to initialize AXI GPIO 3!\n");
		return 1;
	}

	// Set GPIO channel as output
	XGpio_SetDataDirection(&axi_gpio_inst_3, GPIO_CHANNEL_1, 0x00000000);
	printf("Device ID GPIO (GPIO 3) Initialized Successfully.\n");
//--------------------------------------------------------------------------------



	printf("\n*******\n");
	printf("******* Get First SDR configuration   *******\n");
	printf("*******\n\n");
	uint32_t bandwidth_hz;
	status = ad9361_get_rx_rf_bandwidth(ad9361_phy, &bandwidth_hz);
	printf("rx_rf_bandwidth= %lu\n", bandwidth_hz);

	int32_t gain_db, gain_db1;
	status = ad9361_get_rx_rf_gain (ad9361_phy, 0, &gain_db);
	printf("rx1_rf_ch0_gain= %ld\n", gain_db);

	status = ad9361_get_rx_rf_gain (ad9361_phy, 1, &gain_db1);
	printf("rx1_rf_ch1_gain= %ld\n", gain_db1);

	uint32_t sampling_freq_hz;
	status =ad9361_get_tx_sampling_freq (ad9361_phy, &sampling_freq_hz);
	printf("tx_samp_freq= %lu\n", sampling_freq_hz);

	status =ad9361_get_rx_sampling_freq (ad9361_phy, &sampling_freq_hz);
	printf("rx_samp_freq= %lu\n", sampling_freq_hz);

	uint64_t lo_freq_hz;
	status = ad9361_get_rx_lo_freq (ad9361_phy, &lo_freq_hz);
	printf("rx_lo_freq= %llu\n", lo_freq_hz);

	status = ad9361_get_tx_lo_freq (ad9361_phy, &lo_freq_hz);
	printf("tx_lo_freq= %llu\n", lo_freq_hz);



	printf("******* Get Second SDR configuration   *******\n");
	printf("*******\n\n");
	status = ad9361_get_rx_rf_bandwidth(ad9361_phy_b, &bandwidth_hz);
	printf("rx_rf_bandwidth= %lu\n", bandwidth_hz);

	status = ad9361_get_rx_rf_gain (ad9361_phy_b, 0, &gain_db);
	printf("rx1_rf_ch0_gain= %ld\n", gain_db);

	status = ad9361_get_rx_rf_gain (ad9361_phy_b, 1, &gain_db1);
	printf("rx1_rf_ch1_gain= %ld\n", gain_db1);

	status =ad9361_get_tx_sampling_freq (ad9361_phy_b, &sampling_freq_hz);
	printf("tx_samp_freq= %lu\n", sampling_freq_hz);

	status =ad9361_get_rx_sampling_freq (ad9361_phy_b, &sampling_freq_hz);
	printf("rx_samp_freq= %lu\n", sampling_freq_hz);

	status = ad9361_get_rx_lo_freq (ad9361_phy_b, &lo_freq_hz);
	printf("rx_lo_freq= %llu\n", lo_freq_hz);

	status = ad9361_get_tx_lo_freq (ad9361_phy_b, &lo_freq_hz);
	printf("tx_lo_freq= %llu\n", lo_freq_hz);

//--------------------------------------------------------------------------------
	gain_db  = 30;  //rx1 channel gain
	gain_db1 = 30;  //rx2 channel gain


//first ad9361
	XGpio_DiscreteWrite(&axi_gpio_inst_0, GPIO_CHANNEL_1, 0x12C); // threshold
	XGpio_DiscreteWrite(&axi_gpio_inst_1, GPIO_CHANNEL_1, 0x1234); //device id


//second ad9361
	XGpio_DiscreteWrite(&axi_gpio_inst_2, GPIO_CHANNEL_1, 0x12C); // threshold
	XGpio_DiscreteWrite(&axi_gpio_inst_3, GPIO_CHANNEL_1, 0x6789); //device id
//--------------------------------------------------------------------------------


	// Setting first SDR config status
	printf("\n*******\n");
	printf("******* Set first SDR configuration   *******\n");
	printf("*******\n\n");


	sampling_freq_hz = 61440000;
	status = ad9361_set_rx_sampling_freq(ad9361_phy, sampling_freq_hz);usleep(2000);
	if(status < 0) {printf("Failed to set rx_samp_freq with error code : %d\n", status);}
	status = ad9361_get_rx_sampling_freq(ad9361_phy, &sampling_freq_hz);
	printf("rx_samp_freq set to : %lu\n", sampling_freq_hz);


	bandwidth_hz = 5000000UL;
	status = ad9361_set_rx_rf_bandwidth(ad9361_phy, bandwidth_hz);usleep(2000);
	if(status < 0) {printf("Failed to set rx_rf_bandwidth with error code : %d\n", status);}
	status = ad9361_get_rx_rf_bandwidth(ad9361_phy,&bandwidth_hz);
	printf("rx_rf_bandwidth set to : %lu\n", bandwidth_hz);

	/* Set the receive RF gain for the selected channel. */
	status = ad9361_set_rx_rf_gain(ad9361_phy, 0, gain_db);usleep(2000);
	if(status < 0) {printf("Failed to set rx_ch0_rf_gain with error code : %d\n", status);}
	status = ad9361_get_rx_rf_gain(ad9361_phy, 0, &gain_db); printf("rx_ch0_rf_gain set to : %ld\n", gain_db);

	status = ad9361_set_rx_rf_gain (ad9361_phy, 1, gain_db1);usleep(2000);
	if(status < 0) {printf("Failed to set rx_ch1_rf_gain with error code : %d\n", status);}
	status = ad9361_get_rx_rf_gain(ad9361_phy, 1, &gain_db1);
	printf("rx_ch1_rf_gain set to : %ld\n", gain_db1);


	lo_freq_hz =1030000000UL;
	status = ad9361_set_rx_lo_freq(ad9361_phy, lo_freq_hz);usleep(2000);
	if(status < 0) {printf("Failed to set rx_lo_freq with error code : %d\n", status);}
	status = ad9361_get_rx_lo_freq(ad9361_phy, &lo_freq_hz);
	printf("rx_lo_freq set to : %llu\n", lo_freq_hz);





	// Setting second SDR config status
	printf("\n*******\n");
	printf("******* Set second SDR configuration   *******\n");
	printf("*******\n\n");


	sampling_freq_hz = 61440000;
	status = ad9361_set_rx_sampling_freq(ad9361_phy_b, sampling_freq_hz);usleep(2000);
	if(status < 0) {printf("Failed to set rx_samp_freq with error code : %d\n", status);}
	status = ad9361_get_rx_sampling_freq(ad9361_phy_b, &sampling_freq_hz);
	printf("rx_samp_freq set to : %lu\n", sampling_freq_hz);


	bandwidth_hz = 5000000UL;
	status = ad9361_set_rx_rf_bandwidth(ad9361_phy_b, bandwidth_hz);usleep(2000);
	if(status < 0) {printf("Failed to set rx_rf_bandwidth with error code : %d\n", status);}
	status = ad9361_get_rx_rf_bandwidth(ad9361_phy_b, &bandwidth_hz);
	printf("rx_rf_bandwidth set to : %lu\n", bandwidth_hz);

	/* Set the receive RF gain for the selected channel. */
	status = ad9361_set_rx_rf_gain(ad9361_phy_b, 0, gain_db);usleep(2000);
	if(status < 0) {printf("Failed to set rx_ch0_rf_gain with error code : %d\n", status);}
	status = ad9361_get_rx_rf_gain(ad9361_phy_b, 0, &gain_db); printf("rx_ch0_rf_gain set to : %ld\n", gain_db);

	status = ad9361_set_rx_rf_gain (ad9361_phy_b, 1, gain_db1);usleep(2000);
	if(status < 0) {printf("Failed to set rx_ch1_rf_gain with error code : %d\n", status);}
	status = ad9361_get_rx_rf_gain(ad9361_phy_b, 1, &gain_db1);
	printf("rx_ch1_rf_gain set to : %ld\n", gain_db1);


	lo_freq_hz =1030000000UL;
	status = ad9361_set_rx_lo_freq(ad9361_phy_b, lo_freq_hz);usleep(2000);
	if(status < 0) {printf("Failed to set rx_lo_freq with error code : %d\n", status);}
	status = ad9361_get_rx_lo_freq(ad9361_phy_b, &lo_freq_hz);
	printf("rx_lo_freq set to : %llu\n", lo_freq_hz);


///////////////////////////////////////////////////Setup UART for GPS
		platform_setup_timer();


		struct uartIRQcallback uartIRQCallbackRef;
		uartIRQCallbackRef.XUartPsObj = &GPS_Uart_Ps;
		uartIRQCallbackRef.uart_RecvBufferPtr = uartRecvBufferMain;
		uartIRQCallbackRef.uart_RecvBuffer = uartRecvBufferMain;
		uartIRQCallbackRef.uart_BUFFER_SIZE = GPS_RX_BUFF_SIZE;
		uartIRQCallbackRef.TotalRecvCnt = 0;
		uartIRQCallbackRef.uart_time_out_flag = 0;

		XUartPsFormat uart_format;
		uart_format.BaudRate = 9600;
		uart_format.DataBits = XUARTPS_FORMAT_8_BITS;
		uart_format.Parity = XUARTPS_FORMAT_NO_PARITY;
		uart_format.StopBits = XUARTPS_FORMAT_1_STOP_BIT;

		status =  Uart_Init(&uartIRQCallbackRef, GPS_UART_DEVICE_ID, &uart_format);
		XScuGic_Config *GicConfig;
		GicConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
		if (NULL == GicConfig) return XST_FAILURE;

		status = XScuGic_CfgInitialize(&InterruptController, GicConfig, GicConfig->CpuBaseAddress);
		if (status != XST_SUCCESS) return XST_FAILURE;

		Xil_ExceptionInit();

		// Connect the GIC handler to the processor
		Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler,
					 &InterruptController);
		XScuGic_SetPriorityTriggerType(&InterruptController, GPS_UART_INT_IRQ_ID, 0xA0, 0x3);

		// Connect UART handler
		status = XScuGic_Connect(&InterruptController, GPS_UART_INT_IRQ_ID,
					 (Xil_ExceptionHandler)uart_irq_Handler,
					 (void *)&uartIRQCallbackRef);
		if (status != XST_SUCCESS) return XST_FAILURE;

		// Enable the UART interrupt in the GIC
		XScuGic_Enable(&InterruptController, GPS_UART_INT_IRQ_ID);





//////////////////// Timer Interrupt/////////////////////////////////////////////////////////////////////////////
		xil_printf("Connecting Timer interrupt to GIC...\r\n");

		status = XScuGic_Connect(&InterruptController, TIMER_IRPT_INTR,
					 (Xil_ExceptionHandler)timer_callback,
					 (void *)&TimerInstance);
		if (status != XST_SUCCESS) {xil_printf("Failed to connect timer interrupt\r\n");return XST_FAILURE;}

		XScuGic_SetPriorityTriggerType(&InterruptController, TIMER_IRPT_INTR, 0xA0, 0x3);

		// Enable the Timer interrupt in the GIC
		XScuGic_Enable(&InterruptController, TIMER_IRPT_INTR);



////////////////////PL Interrupt/////////////////////////////////////////////////////////////////////////////////
		XScuGic_Disable(&InterruptController, PL_INTERRUPT_ID);
		Xil_Out32(INTR_IP_BASEADDR + IER_OFFSET, 0);
		Xil_Out32(INTR_IP_BASEADDR + GIE_OFFSET, 0);
		Xil_Out32(INTR_IP_BASEADDR + IAR_OFFSET, PL_INTR_MASK);  // W1C

		status = XScuGic_Connect(&InterruptController, PL_INTERRUPT_ID, (Xil_ExceptionHandler)AXI4_ISR, (void *)&data_ready_flag);
		if (status != XST_SUCCESS) {xil_printf("Failed to connect PL IRQ.\r\n"); return XST_FAILURE;}
		XScuGic_SetPriorityTriggerType(&InterruptController, PL_INTERRUPT_ID, 0xA0, 0x1);
		XScuGic_Enable(&InterruptController, PL_INTERRUPT_ID);

		Xil_Out32(INTR_IP_BASEADDR + IER_OFFSET, PL_INTR_MASK);
		Xil_Out32(INTR_IP_BASEADDR + GIE_OFFSET, PL_INTR_MASK);
		u32 gie  = Xil_In32(INTR_IP_BASEADDR + GIE_OFFSET);
		u32 ier  = Xil_In32(INTR_IP_BASEADDR + IER_OFFSET);
		u32 isr  = Xil_In32(INTR_IP_BASEADDR + ISR_OFFSET);

		xil_printf("PL IRQ regs: GIE=0x%08lX IER=0x%08lX ISR=0x%08lX\r\n", gie, ier, isr);

		// Helper: read GIC pending & active bits for PL_INTERRUPT_ID
		u32 grp = PL_INTERRUPT_ID / 32;
		u32 bit = PL_INTERRUPT_ID % 32;
		u32 dist = InterruptController.Config->DistBaseAddress;
		u32 pend = XScuGic_ReadReg(dist, 0x200 + 4*grp);
		u32 active = XScuGic_ReadReg(dist, 0x300 + 4*grp);
		xil_printf("GIC: PEND[grp%lu]=0x%08lX ACTV[grp%lu]=0x%08lX (bit %lu)\n", (u32)grp, pend, (u32)grp, active, (u32)bit);


////////////////////////////////pl interrupt for the second ad9361
		XScuGic_Disable(&InterruptController, PL_INTERRUPT_ID_2);
		Xil_Out32(INTR_IP_BASEADDR_2 + IER_OFFSET, 0);
		Xil_Out32(INTR_IP_BASEADDR_2 + GIE_OFFSET, 0);
		Xil_Out32(INTR_IP_BASEADDR_2 + IAR_OFFSET, PL_INTR_MASK);  // W1C

		status = XScuGic_Connect(&InterruptController, PL_INTERRUPT_ID_2, (Xil_ExceptionHandler)AXI4_ISR_2, (void *)&data_ready_flag_2);
		if (status != XST_SUCCESS) {xil_printf("Failed to connect PL IRQ.\r\n"); return XST_FAILURE;}
		XScuGic_SetPriorityTriggerType(&InterruptController, PL_INTERRUPT_ID_2, 0xA0, 0x1);
		XScuGic_Enable(&InterruptController, PL_INTERRUPT_ID_2);

		Xil_Out32(INTR_IP_BASEADDR_2 + IER_OFFSET, PL_INTR_MASK);
		Xil_Out32(INTR_IP_BASEADDR_2 + GIE_OFFSET, PL_INTR_MASK);
		u32 gie_2  = Xil_In32(INTR_IP_BASEADDR_2 + GIE_OFFSET);
		u32 ier_2  = Xil_In32(INTR_IP_BASEADDR_2 + IER_OFFSET);
		u32 isr_2  = Xil_In32(INTR_IP_BASEADDR_2 + ISR_OFFSET);

		xil_printf("PL IRQ regs: GIE=0x%08lX IER=0x%08lX ISR=0x%08lX\r\n", gie_2, ier_2, isr_2);

		// Helper: read GIC pending & active bits for PL_INTERRUPT_ID
		u32 grp_2 = PL_INTERRUPT_ID_2 / 32;
		u32 bit_2 = PL_INTERRUPT_ID_2 % 32;
		dist = InterruptController.Config->DistBaseAddress;
		u32 pend_2 = XScuGic_ReadReg(dist, 0x200 + 4*grp_2);
		u32 active_2 = XScuGic_ReadReg(dist, 0x300 + 4*grp_2);
		xil_printf("GIC: PEND[grp%lu]=0x%08lX ACTV[grp%lu]=0x%08lX (bit %lu)\n", (u32)grp_2, pend_2, (u32)grp_2, active_2, (u32)bit_2);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		// Enable interrupts in the Processor
		Xil_ExceptionEnable();

		// Enable the Timeout interrupt on the UART peripheral itself
		XUartPs_SetInterruptMask(uartIRQCallbackRef.XUartPsObj, XUARTPS_IXR_TOUT);

		// Start the lwIP timer
		platform_enable_interrupts(); // This starts the timer

		printf("Interrupt system initialized successfully.\n");

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		xil_printf("\r\n--- INTERRUPT SANITY CHECK ---\r\n");

		// 1. Read the mask back from the UART peripheral
		u32 actual_imr = XUartPs_ReadReg(uartIRQCallbackRef.XUartPsObj->Config.BaseAddress, XUARTPS_IMR_OFFSET);
		xil_printf("UART IMR (Actual):   0x%08lX\r\n", actual_imr);
		xil_printf("UART IMR (Expected): 0x%08lX\r\n", (u32)XUARTPS_IXR_TOUT);

		if (actual_imr != (u32)XUARTPS_IXR_TOUT) {
			xil_printf("!!! ERROR: UART Interrupt Mask is NOT set correctly!\r\n");
		}

		// 2. Check if the GIC has the UART interrupt enabled
		u32 reg_offset = XSCUGIC_EN_DIS_OFFSET_CALC(XSCUGIC_ENABLE_SET_OFFSET, GPS_UART_INT_IRQ_ID);
		u32 gic_enable_reg = XScuGic_ReadReg(InterruptController.Config->DistBaseAddress, reg_offset);

		int gic_is_enabled = (gic_enable_reg >> (GPS_UART_INT_IRQ_ID % 32)) & 1;

		if (gic_is_enabled) {
			xil_printf("GIC check: Interrupt %d is ENABLED.\r\n", GPS_UART_INT_IRQ_ID);
		} else {
			xil_printf("!!! ERROR: GIC Interrupt %d is NOT enabled!\r\n", GPS_UART_INT_IRQ_ID);
		}
		xil_printf("------------------------------\r\n\r\n");
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////GPS settings
		u8 modify_GPS_settings[] = {0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00,
				0x00, 0x00, 0x20, 0x07,
				0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, //gps, enabled, L1C/A
				0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, //sbas, enabled, L1C/A
				0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, //galileo, not enabled
				0x03, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, //beidou, enabled, B1l
				0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, //?, not enabled
				0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, //qzss, enabled, L1C/A
				0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, //glonass, not enabled
				0x2F, 0xA1};
		uart_send(uartIRQCallbackRef.XUartPsObj,modify_GPS_settings,sizeof(modify_GPS_settings));usleep(2000);

		u8 ublox_messages_config[] ={0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F, //GGA, disabled
		   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11, //GLL, disabled
		   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13, //GSA, disabled
		   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15, //GSV, disabled
		   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17, //RMC, disabled
		   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19, //VTG, disabled
		   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF1, 0x04, 0x00, 0xFF, 0x1A, //PUBX-TIME, disabled
		   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x00, 0x02, 0x1F, //ZDA, disabled

		   //0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF1, 0x04, 0x01, 0x00, 0x1B, //PUBX-TIME, enabled
		   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x01, 0x03, 0x20, //ZDA, enabled
		   //0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0xFB, 0x10, //GGA, enabled
		   //0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x01, 0xFF, 0x18, //RMC, enabled
		   };
		uart_send(uartIRQCallbackRef.XUartPsObj,ublox_messages_config,sizeof(ublox_messages_config));usleep(2000);

		u8 disable_ubx_time[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x20, 0x00, 0x2B, 0x82};
		uart_send(uartIRQCallbackRef.XUartPsObj, disable_ubx_time, sizeof(disable_ubx_time));usleep(2000);

		u8 save_data[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF};
		uart_send(uartIRQCallbackRef.XUartPsObj,save_data,sizeof(save_data));usleep(2000);


///////////////////////////////////////////////////////////////////////////////////////////////
		int rx2tx2 = ad9361_phy->pdata->rx2tx2 ;
		if (rx2tx2)
			printf("rx2-tx2\r\n");
		else
			printf("no rx2-tx2\r\n");





///////////////////////////////////////////////////////////////////////////////////////////////
	struct netif *netif;
	/* the mac address of the board. this should be unique per board */
	unsigned char mac_ethernet_address[] = {0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };

	netif = &server_netif;
	//init_platform();
	lwip_init();



	/* Add network interface to the netif_list, and set it as default */
	if (!xemac_add(netif, NULL, NULL, NULL, mac_ethernet_address,
				PLATFORM_EMAC_BASEADDR)) {
		xil_printf("Error adding N/W interface\r\n");
		return -1;
	}
	netif_set_default(netif);
	netif_set_up(netif);







#if (LWIP_DHCP==1)
	/* Create a new DHCP client for this interface.
	 * Note: you must call dhcp_fine_tmr() and dhcp_coarse_tmr() at
	 * the predefined regular intervals after starting the client.
	 */
	dhcp_start(netif);
	dhcp_timoutcntr = 24;
	while (((netif->ip_addr.addr) == 0) && (dhcp_timoutcntr > 0))
		xemacif_input(netif);

	if (dhcp_timoutcntr <= 0) {
		if ((netif->ip_addr.addr) == 0) {
			xil_printf("ERROR: DHCP request timed out\r\n");
			assign_default_ip(&(netif->ip_addr),
					&(netif->netmask), &(netif->gw));
		}
	}

	/* print IP address, netmask and gateway */
#else
	assign_default_ip(&(netif->ip_addr), &(netif->netmask), &(netif->gw));
#endif
	print_ip_settings(&(netif->ip_addr), &(netif->netmask), &(netif->gw));
	xil_printf("\r\n");
	xil_printf("UDP client connecting to %s on port %d\r\n",
			UDP_SERVER_IP_ADDRESS, UDP_CONN_PORT);

	start_udp_application();



	uint8_t sec=0;
//	size_t lastDataSize = UDP_SEND_BUFSIZE;

	uint32_t data_192bit[6];
	uint32_t data_192bit_2[6];

	int gps_valid_cnt = 0;
	uint8_t gps_valid_flag = 0;

	uint8_t localUartBuffer[GPS_RX_BUFF_SIZE];
	unsigned int localRecvCnt = 0;
	int process_uart_data = 0; // Flag to process data outside critical section

	uint8_t sec_from_pl = 0;
	uint8_t sec_from_pl_2 = 0;

	uint8_t sec_sent_to_LAN = 0;
	uint8_t sec_sent_to_LAN_2 = 0;

	int second_offset = 0;
	int second_offset_2 = 0;

	uint8_t second_offset_done = 0;
	uint8_t second_offset_done_2 = 0;

	uint8_t second_sum = 0;
	uint8_t second_sum_2 = 0;

	int process_pl_data = 0;
	int process_pl_data_2 = 0;



	while (1) {
			if (TcpFastTmrFlag) {tcp_fasttmr(); TcpFastTmrFlag = 0;}
			if (TcpSlowTmrFlag) {tcp_slowtmr(); TcpSlowTmrFlag = 0;}

			xemacif_input(netif);

	        if (data_ready_flag == 1) {
	        	// Clear the flag to re-arm for the next interrupt
	        	data_ready_flag = 0;
	        	Xil_ExceptionDisable();
	            // Read the 192-bit data from the 6 registers
	            Read192BitData(data_192bit);

	            process_pl_data = 1;
	            Xil_ExceptionEnable();
	        }

	        if (data_ready_flag_2 == 1) {
	        	// Clear the flag to re-arm for the next interrupt
	            data_ready_flag_2 = 0;
	        	Xil_ExceptionDisable();
	            // Read the 192-bit data from the 6 registers
	            Read192BitData_2(data_192bit_2);

	            process_pl_data_2 = 1;
	            Xil_ExceptionEnable();
	        }


	        if (process_pl_data == 1) {

//	            xil_printf("\nReceived message from PL = %04X %02X %02X%08X%06X\r\n", (data_192bit[5] >> 16), (data_192bit[5] >> 8) & 0xFF, data_192bit[5] & 0xFF, data_192bit[4], (data_192bit[3] >> 8));
	            sec_from_pl = (data_192bit[5] >> 10) & 0x3F;
	//            xil_printf("Second count from PL message = %u\n", sec_from_pl);
	            process_pl_data = 0;

				if (gps_valid_flag == 1){
					if (second_offset_done != 1) {
						second_offset = sec - sec_from_pl;
						xil_printf("\nSecond offset for sdr 1 is %i, this message should appear just once.\n", second_offset);
						second_offset_done = 1;
					}
					if (second_offset_done == 1) {
						second_sum = ((sec_from_pl + second_offset) % 60 + 60) % 60;
						data_192bit[5] = set_seconds(data_192bit[5], second_sum);
					}

				}
				pack_data_to_sendbuf(send_buf, data_192bit);
	//			xil_printf("Message sent to LAN = %02X%02X %02X %02X%02X%02X%02X%02X%02X%02X%02X\r\n", send_buf[0], send_buf[1], send_buf[2], send_buf[3],
	//					send_buf[4], send_buf[5], send_buf[6], send_buf[7], send_buf[8], send_buf[9], send_buf[10]);

				sec_sent_to_LAN = (send_buf[2] >> 2) & 0x3F;
	//			xil_printf("UTC Second from message sent to LAN = %u\n", sec_sent_to_LAN);


				transfer_udp_data(UDP_SEND_BUFSIZE);

	        }



	        if (process_pl_data_2 == 1) {

//	            xil_printf("\nReceived message from PL = %04X %02X %02X%08X%06X\r\n", (data_192bit[5] >> 16), (data_192bit[5] >> 8) & 0xFF, data_192bit[5] & 0xFF, data_192bit[4], (data_192bit[3] >> 8));
	            sec_from_pl_2 = (data_192bit_2[5] >> 10) & 0x3F;
	//            xil_printf("Second count from PL message = %u\n", sec_from_pl);
	            process_pl_data_2 = 0;

				if (gps_valid_flag == 1){
					if (second_offset_done_2 != 1) {
						second_offset_2 = sec - sec_from_pl_2;
						xil_printf("\nSecond offset for sdr 2 is %i, this message should appear just once.\n", second_offset_2);
						second_offset_done_2 = 1;
					}
					if (second_offset_done_2 == 1) {
						second_sum_2 = ((sec_from_pl_2 + second_offset_2) % 60 + 60) % 60;
						data_192bit_2[5] = set_seconds(data_192bit_2[5], second_sum_2);
					}

				}
				pack_data_to_sendbuf(send_buf_2, data_192bit_2);
	//			xil_printf("Message sent to LAN = %02X%02X %02X %02X%02X%02X%02X%02X%02X%02X%02X\r\n", send_buf[0], send_buf[1], send_buf[2], send_buf[3],
	//					send_buf[4], send_buf[5], send_buf[6], send_buf[7], send_buf[8], send_buf[9], send_buf[10]);

				sec_sent_to_LAN_2 = (send_buf_2[2] >> 2) & 0x3F;
	//			xil_printf("UTC Second from message sent to LAN = %u\n", sec_sent_to_LAN);


				transfer_udp_data_2(UDP_SEND_BUFSIZE);

	        }




		    if (uartIRQCallbackRef.uart_time_out_flag == 1) {

		        Xil_ExceptionDisable();

		        memcpy(localUartBuffer, uartRecvBufferMain, uartIRQCallbackRef.TotalRecvCnt);
		        localRecvCnt = uartIRQCallbackRef.TotalRecvCnt;
		        // Reset the flag and buffer for the next message
		        uartIRQCallbackRef.uart_time_out_flag = 0;
		        uartIRQCallbackRef.TotalRecvCnt = 0;
		        uartIRQCallbackRef.uart_RecvBufferPtr = uartRecvBufferMain;
		        process_uart_data = 1;

		        Xil_ExceptionEnable();
		    }


			if (process_uart_data == 1) {
				// Null-terminate the received data to treat it as a string
				if (localRecvCnt > 0 && localRecvCnt < GPS_RX_BUFF_SIZE) {
					localUartBuffer[localRecvCnt] = '\0';
					//xil_printf("\n%s", localUartBuffer);

				// Debug print
//				for (size_t i = 0; i < localRecvCnt; i++) {xil_printf("%02X ", (uint8_t)localUartBuffer[i]);}
//				xil_printf("\n");


				if (gps_extract_second((const char*)localUartBuffer, &sec)) {
//					xil_printf("Second: %u\r\n", sec);

//					send_buf[0] = sec;
//					transfer_udp_data(1);

					if (sec != 0 && gps_valid_cnt < 5) {gps_valid_cnt = gps_valid_cnt + 1;}
					if (gps_valid_cnt >= 5) {gps_valid_flag = 1;}
				}
				}

				process_uart_data = 0;
			}

		}







#ifdef XILINX_PLATFORM
	Xil_DCacheDisable();
	Xil_ICacheDisable();
#endif

	return 0;
}
