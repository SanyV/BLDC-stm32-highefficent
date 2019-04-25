/*
    This file is part of AutoQuad ESC32.

    AutoQuad ESC32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad ESC32 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad ESC32.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012  Bill Nesbitt
*/

#include "serial.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

serialPort_t serialPort;

void serialStartTxDMA() {
    serialPort_t *s = &serialPort;

    SERIAL_TX_DMA->M0AR = (uint32_t)&s->txBuf[s->txTail];
    if (s->txHead > s->txTail) {
	SERIAL_TX_DMA->NDTR = s->txHead - s->txTail;
	s->txTail = s->txHead;
    }
    else {
	SERIAL_TX_DMA->NDTR = SERIAL_TX_BUFSIZE - s->txTail;
	s->txTail = 0;
    }
    /* Enable DMA Channel Tx */
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
}

void serialWrite(int ch) {
    serialPort_t *s = &serialPort;

    s->txBuf[s->txHead] = ch;
    s->txHead = (s->txHead + 1) % SERIAL_TX_BUFSIZE;

    if (!(SERIAL_TX_DMA->CR & 1))
	serialStartTxDMA();
}

unsigned char serialAvailable() {
    return (SERIAL_RX_DMA->NDTR != serialPort.rxPos);
}

// only call after a affirmative return from serialAvailable()
int serialRead() {
    serialPort_t *s = &serialPort;
    int ch;

    ch = s->rxBuf[SERIAL_RX_BUFSIZE - s->rxPos];
    if (--s->rxPos == 0)
	s->rxPos = SERIAL_RX_BUFSIZE;

    return ch;
}

void serialPrint(const char *str) {
    while (*str)
	serialWrite(*(str++));
}

void serialOpenPort(int baud) {
/*
	USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = SERIAL_FLOW_CONTROL;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(SERIAL_UART, &USART_InitStructure);

    USART_InitStruct.BaudRate = 230400;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART1);

    LL_USART_Enable(USART1);
    */
}
/*
void PacketlWrite(uint8_t *buf, uint16_t length) {
    uint32_t Dest,Source;
	serialPort_t *s = &serialPort;

    memcpy((uint32_t)s->txBuf+s->txHead,(uint32_t*)buf,length);

    s->txHead = ((s->txHead) + length) % SERIAL_TX_BUFSIZE;

    if (!(SERIAL_TX_DMA->CR & 1))
	serialStartTxDMA();
}
*/
void serialPrintF(const char * fmt, ...)
{
	char buf[4096];
	uint16_t i,len;

	va_list  vlist;
	va_start (vlist, fmt);

	len = vsnprintf(buf, sizeof(buf), fmt, vlist);
	i=0;
	//PacketlWrite(buf,len);

	while (i<len+1)
		{
	serialWrite(buf[i]);
	i++;
		}

	va_end(vlist);
}

void serialInit(void) {


    serialPort_t *s = &serialPort;

    s->rxHead = s->rxTail = 0;
    s->txHead = s->txTail = 0;

    SERIAL_TX_DMA->NDTR = 0;
    SERIAL_TX_DMA->PAR = (uint32_t)&(USART1->DR);

    SERIAL_RX_DMA->PAR = (uint32_t)&(USART1->DR);
    SERIAL_RX_DMA->M0AR= (uint32_t)s->rxBuf;

    /* Enable DMA RX Interrupt */
    LL_USART_EnableDMAReq_RX(USART1);

    /* Enable DMA Channel Rx */
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
    s->rxPos  = SERIAL_RX_DMA->NDTR;

    /* Enable DMA TX Interrupt */
    LL_USART_EnableDMAReq_TX(USART1);

    /* Enable DMA Channel Tx */
  //  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
    /* USER CODE END 2 */

}

void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */
	  if(LL_DMA_IsActiveFlag_TC7(DMA2))
	  {
	    LL_DMA_ClearFlag_TC7(DMA2);
	    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);

	    if (serialPort.txHead != serialPort.txTail)
	    serialStartTxDMA();
	    /* Call function Reception complete Callback */
	//    LL_GPIO_TogglePin(LED1_GPIO_Port,LED2_Pin);
	  }
	  else if(LL_DMA_IsActiveFlag_TE7(DMA2))
	  {
		  LL_DMA_ClearFlag_TE7(DMA2);
		  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
	    /* Call Error function */
	//	LL_GPIO_TogglePin(LED1_GPIO_Port,LED3_Pin);
	  }
  /* USER CODE END DMA2_Stream7_IRQn 0 */

  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
//не используется
void DMA2_Stream2_IRQHandler(void)
{
	  if(LL_DMA_IsActiveFlag_TC2(DMA2))
	  {
	    LL_DMA_ClearFlag_TC2(DMA2);
	    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);

	    if (serialPort.txHead != serialPort.txTail)
	   // serialStartTxDMA();
	    /* Call function Reception complete Callback */
	    LL_GPIO_TogglePin(LED1_GPIO_Port,LED2_Pin);
	  }
	  else if(LL_DMA_IsActiveFlag_TE2(DMA2))
	  {
		  LL_DMA_ClearFlag_TE2(DMA2);
		  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
	    /* Call Error function */
		LL_GPIO_TogglePin(LED1_GPIO_Port,LED3_Pin);
	  }
}
