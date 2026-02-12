/*
 * Copyright (C) 2009 - 2019 Xilinx, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/tcp.h"
#if defined (__arm__) || defined (__aarch64__)
#include "xil_printf.h"
#endif

struct tcp_pcb *global_pcb;

int transfer_data() {
	return 0;
}

void print_app_header()
{
#if (LWIP_IPV6==0)
	xil_printf("\n\r\n\r-----lwIP TCP echo server ------\n\r");
#else
	xil_printf("\n\r\n\r-----lwIPv6 TCP echo server ------\n\r");
#endif
	xil_printf("TCP packets sent to port 6001 will be echoed back\n\r");
}

err_t recv_callback(void *arg, struct tcp_pcb *tpcb,
                               struct pbuf *p, err_t err)
{
	/* do not read the packet if we are not in ESTABLISHED state */
	if (!p) {
		tcp_close(tpcb);
		tcp_recv(tpcb, NULL);
		return ERR_OK;
	}

	/* indicate that the packet has been received */
	tcp_recved(tpcb, p->len);

	/* echo back the payload */
	/* in this case, we assume that the payload is < TCP_SND_BUF */
	if (tcp_sndbuf(tpcb) > p->len) {
		err = tcp_write(tpcb, p->payload, p->len, 1);
	} else
		xil_printf("no space in tcp_sndbuf\n\r");

	/* free the received pbuf */
	pbuf_free(p);

	return ERR_OK;
}

err_t accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
	static int connection = 1;

	/* set the receive callback for this connection */
	tcp_recv(newpcb, recv_callback);

	/* just use an integer number indicating the connection id as the
	   callback argument */
	tcp_arg(newpcb, (void*)(UINTPTR)connection);

	/* increment for subsequent accepted connections */
	connection++;

	return ERR_OK;
}



/**
 * This function is called by lwIP when an ACK is received for data we sent.
 */
static err_t sent_callback(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    xil_printf("Message of length %d sent successfully.\n\r", len);

    // We can close the connection now that our message is sent and acknowledged.
    // tcp_close(tpcb); // Uncomment if you want to close after sending.

    return ERR_OK;
}



static err_t connect_callback(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    if (err == ERR_OK) {
        xil_printf("Connection to server established.\n\r");

        // --- PREPARE TO SEND DATA ---

        // 1. Register a callback for when data is successfully sent
        tcp_sent(tpcb, sent_callback);

        // 2. The message we want to send
        char *message = "Hello from PZ101!";
        err_t write_err = ERR_OK;

        // 3. Send the message
        write_err = tcp_write(tpcb, message, strlen(message), TCP_WRITE_FLAG_COPY);
        if (write_err != ERR_OK) {
            xil_printf("Error writing data to TCP buffer: %d\n\r", write_err);
            return write_err;
        }

        // 4. Push the data out to the network
        tcp_output(tpcb);

    } else {
        xil_printf("Failed to connect to server. err = %d\n\r", err);
    }

    return ERR_OK;
}




/**
 * This function is called by lwIP on fatal errors.
 */
static void error_callback(void *arg, err_t err)
{
    xil_printf("TCP Error detected, code %d\n\r", err);
    // You could attempt to reconnect here if desired
}



int start_application()
{
	struct tcp_pcb *pcb;
	err_t err;
	unsigned port = 1234;

	/* create new TCP PCB structure */
	pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
	if (!pcb) {
		xil_printf("Error creating PCB. Out of Memory\n\r");
		return -1;
	}

	/* bind to specified @port */
	err = tcp_bind(pcb, IP_ANY_TYPE, port);
	if (err != ERR_OK) {
		xil_printf("Unable to bind to port %d: err = %d\n\r", port, err);
		return -2;
	}

	/* we do not need any arguments to callback functions */
	tcp_arg(pcb, NULL);

	/* listen for connections */
	pcb = tcp_listen(pcb);
	if (!pcb) {
		xil_printf("Out of memory while tcp_listen\n\r");
		return -3;
	}

	/* specify callback to use for incoming connections */
	tcp_accept(pcb, accept_callback);

	xil_printf("TCP echo server started @ port %d\n\r", port);

	return 0;
}

int start_application_client2server()
{
	struct tcp_pcb *pcb;
	err_t err;
	unsigned port = 1234;

    // The IP address of your PC (the server)
    // !!! IMPORTANT: YOU MUST CHANGE THIS !!!
    ip_addr_t server_ip;
    IP4_ADDR(&server_ip, 192, 168, 1, 11); // Replace with your PC's IP address

    /* create new TCP PCB structure */
    pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        xil_printf("Error creating PCB. Out of Memory\n\r");
        return -1;
    }

    // Keep a global reference
    global_pcb = pcb;

    /* register error callback */
    tcp_err(pcb, error_callback);

    /* connect to the server */
    xil_printf("Attempting to connect to %d.%d.%d.%d on port %d...\n\r",
        ip4_addr1(&server_ip), ip4_addr2(&server_ip),
        ip4_addr3(&server_ip), ip4_addr4(&server_ip), port);

    err = tcp_connect(pcb, &server_ip, port, connect_callback);
    if (err != ERR_OK) {
        xil_printf("Error initiating connection: err = %d\n\r", err);
        return -1;
    }

    return 0;
}
