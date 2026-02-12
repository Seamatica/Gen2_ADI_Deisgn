/*
 * uart_com.c
 *
 *  Created on: Apr 3, 2025
 *      Author: Chronos
 */


#include "uart_com.h"
#include "xuartps.h"
//struct uartIRQcallback uartIRQCallbackRef;

int Uart_Init(struct uartIRQcallback* uartIRQCB, u16 DeviceId, XUartPsFormat* uart_format)
{
	int Status;
	XUartPs_Config *Config;

	//uart_RecvBufferPtr = uart_RecvBuffer;
	//TotalRecvCnt = 0;

	Config = XUartPs_LookupConfig(DeviceId);
	if (NULL == Config) {
		return XST_FAILURE;
	}
	Status = XUartPs_CfgInitialize(uartIRQCB->XUartPsObj, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = XUartPs_SelfTest(uartIRQCB->XUartPsObj);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	XUartPs_SetOperMode(uartIRQCB->XUartPsObj, XUARTPS_OPER_MODE_NORMAL);
	XUartPs_SetDataFormat(uartIRQCB->XUartPsObj, uart_format);
	XUartPs_SetFifoThreshold(uartIRQCB->XUartPsObj, 8);
	XUartPs_SetRecvTimeout(uartIRQCB->XUartPsObj, 80);


	//XUartPs_SetInterruptMask(uartIRQCB->XUartPsObj, UART_RX_IRQ_MASK);


	return XST_SUCCESS;
}


//void uart_irq_Handler(void *CallBackRef)
//{
//	struct uartIRQcallback* uartIRQcallbackPtr = (struct uartIRQcallback* )CallBackRef;
//
//	XUartPs *UartInstancePtr = uartIRQcallbackPtr->XUartPsObj;
//	u32 ReceivedCount = 0 ;
//	u32 IsrStatus ;
//
//	IsrStatus = XUartPs_ReadReg(UartInstancePtr->Config.BaseAddress,
//				   XUARTPS_IMR_OFFSET);
//	IsrStatus &= XUartPs_ReadReg(UartInstancePtr->Config.BaseAddress,
//				   XUARTPS_ISR_OFFSET);
//
//	if (IsrStatus & (u32)XUARTPS_IXR_RXOVR)
//	{
//		if(uartIRQcallbackPtr->TotalRecvCnt +8 + 1 < uartIRQcallbackPtr->uart_BUFFER_SIZE){
//			ReceivedCount = XUartPs_Recv(UartInstancePtr, uartIRQcallbackPtr->uart_RecvBufferPtr, (uartIRQcallbackPtr->uart_BUFFER_SIZE - uartIRQcallbackPtr->TotalRecvCnt)) ;
//			uartIRQcallbackPtr->TotalRecvCnt += ReceivedCount ;
//			uartIRQcallbackPtr->uart_RecvBufferPtr += ReceivedCount ;
//		}
//
//		XUartPs_WriteReg(UartInstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET, XUARTPS_IXR_RXOVR) ;
//
//	}
//
//	if(IsrStatus & (u32)XUARTPS_IXR_TOUT){
//		uartIRQcallbackPtr->uart_time_out_flag = 1;
//		if(uartIRQcallbackPtr->TotalRecvCnt +8 + 1 < uartIRQcallbackPtr->uart_BUFFER_SIZE){
//			ReceivedCount = XUartPs_Recv(UartInstancePtr, uartIRQcallbackPtr->uart_RecvBufferPtr, (uartIRQcallbackPtr->uart_BUFFER_SIZE - uartIRQcallbackPtr->TotalRecvCnt)) ;
//			uartIRQcallbackPtr->TotalRecvCnt += ReceivedCount ;
//			uartIRQcallbackPtr->uart_RecvBufferPtr += ReceivedCount ;
//		}
//
//		XUartPs_WriteReg(UartInstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET, XUARTPS_IXR_TOUT) ;
//		//uartIRQcallbackPtr->uart_RecvBufferPtr = uartIRQcallbackPtr->uart_RecvBuffer;
//		//uartIRQcallbackPtr->TotalRecvCnt = 0;
//	}
//
//}



void uart_irq_Handler(void *CallBackRef)
{
	struct uartIRQcallback* uartIRQcallbackPtr = (struct uartIRQcallback* )CallBackRef;
	XUartPs *UartInstancePtr = uartIRQcallbackPtr->XUartPsObj;
	u32 ReceivedCount = 0;
	u32 IsrStatus;
    u32 MaskedStatus;

    //xil_printf("\r\n*** !!! UART INTERRUPT FIRED !!! ***\r\n");

	/*
	 * Read the Interrupt Status Register to determine which
	 * interrupt is active.
	 */
	IsrStatus = XUartPs_ReadReg(UartInstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET);

	u32 imr = XUartPs_ReadReg(UartInstancePtr->Config.BaseAddress, XUARTPS_IMR_OFFSET);
	//xil_printf("ISR_Handler: Raw ISR=0x%08lX, Raw IMR=0x%08lX\r\n", IsrStatus, imr);
	/*
	 * VERY IMPORTANT:
	 * Immediately clear all the pending interrupt flags by writing the status
	 * value back to the Interrupt Status Register.
	 */
	XUartPs_WriteReg(UartInstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET, IsrStatus);

    /*
     * Now, figure out which of the interrupts that fired are the ones
     * we have enabled (i.e., are not masked).
     */
    MaskedStatus = IsrStatus & XUartPs_ReadReg(UartInstancePtr->Config.BaseAddress, XUARTPS_IMR_OFFSET);
    //xil_printf("ISR_Handler: MaskedStatus=0x%08lX\r\n", MaskedStatus);


//	if (MaskedStatus & (u32)XUARTPS_IXR_RXOVR)
//	{
//		/*
//		 * The Receive FIFO is full. Read the data from the FIFO.
//		 */
////		if(uartIRQcallbackPtr->TotalRecvCnt + 8 + 1 < uartIRQcallbackPtr->uart_BUFFER_SIZE){
////			ReceivedCount = XUartPs_Recv(UartInstancePtr, uartIRQcallbackPtr->uart_RecvBufferPtr, (uartIRQcallbackPtr->uart_BUFFER_SIZE - uartIRQcallbackPtr->TotalRecvCnt));
////			uartIRQcallbackPtr->TotalRecvCnt += ReceivedCount;
////			uartIRQcallbackPtr->uart_RecvBufferPtr += ReceivedCount;
////		}
//// NOTE: The flag was already cleared above. No need to clear it here again.
//
//
//		// Clear the FIFO by reading until it's empty
//		while (XUartPs_IsReceiveData(UartInstancePtr->Config.BaseAddress)) {
//			XUartPs_ReadReg(UartInstancePtr->Config.BaseAddress, XUARTPS_FIFO_OFFSET);
//		}
//
//		// Reset the main buffer
//		uartIRQcallbackPtr->TotalRecvCnt = 0;
//		uartIRQcallbackPtr->uart_RecvBufferPtr = uartIRQcallbackPtr->uart_RecvBuffer;
//
//		// Clear the timeout flag just in case
//		uartIRQcallbackPtr->uart_time_out_flag = 0;
//
//	}
//
//	if (MaskedStatus & (u32)XUARTPS_IXR_RXFULL)
//		{
//			/*
//			 * The Receive FIFO is full. Read the data to prevent an overflow.
//			 * Set the flag so the main loop processes this "full" packet.
//			 */
//			if(uartIRQcallbackPtr->TotalRecvCnt + 8 + 1 < uartIRQcallbackPtr->uart_BUFFER_SIZE){
//				ReceivedCount = XUartPs_Recv(UartInstancePtr, uartIRQcallbackPtr->uart_RecvBufferPtr, (uartIRQcallbackPtr->uart_BUFFER_SIZE - uartIRQcallbackPtr->TotalRecvCnt));
//				uartIRQcallbackPtr->TotalRecvCnt += ReceivedCount;
//				uartIRQcallbackPtr->uart_RecvBufferPtr += ReceivedCount;
//			}
//
//			// Set the flag to process this full buffer
//			uartIRQcallbackPtr->uart_time_out_flag = 1;
//		}


	if(MaskedStatus & (u32)XUARTPS_IXR_TOUT){
		/*
		 * A receive timeout has occurred. This likely means we have received a
		 * complete message. Set the flag for the main loop to process it.
		 */
		uartIRQcallbackPtr->uart_time_out_flag = 1;
		if(uartIRQcallbackPtr->TotalRecvCnt + 8 + 1 < uartIRQcallbackPtr->uart_BUFFER_SIZE){
			ReceivedCount = XUartPs_Recv(UartInstancePtr, uartIRQcallbackPtr->uart_RecvBufferPtr, (uartIRQcallbackPtr->uart_BUFFER_SIZE - uartIRQcallbackPtr->TotalRecvCnt));
			uartIRQcallbackPtr->TotalRecvCnt += ReceivedCount;
			uartIRQcallbackPtr->uart_RecvBufferPtr += ReceivedCount;
		}
        // NOTE: The flag was already cleared above. No need to clear it here again.
	}
}







void uart_send(XUartPs *InstancePtr,u8 *data, int length){
	int SentCount = 0;
	while (SentCount < length) {
		/* Transmit the data */
		SentCount += XUartPs_Send(InstancePtr,
					   &data[SentCount], 1);
	}
}
