#ifndef _USART_H_
#define _USART_H_

#ifdef __cplusplus
	extern "C"{
#endif
		#include "sys.h"
		
		#define COM1		0			//GPS
		#define COM2		1			//TELEM1
		#define COM4		2			//UART4
		#define COM6		3			//TELEM2
		#define COM7		4			//DEBUG
		#define COM8		5			//SUBS
		
		#define USART_PARITY_NONE				0
		#define USART_PARITY_EVEN				2
		#define USART_PARITY_ODD				3

		#define USART_ONESTOP						0
		#define USART_HALFSTOP					1
		#define USART_TWOSTOP						2
		#define USART_ONEANDHALFSTOP		3
		
		#define USART_BUFFER_SIZE			1024

		typedef
			struct {				
				unsigned short TxHead,TxTail;
				
				unsigned char TxBuffer[USART_BUFFER_SIZE];
				
				unsigned short RxHead,RxTail;
				
				unsigned char RxBuffer[USART_BUFFER_SIZE];
			}
			USART_REGISTER_DEF;
			
		extern USART_REGISTER_DEF Usart_reg[6];
		
		void usart_init(void);
		void usart1_dma_init(unsigned long baud,unsigned char par,unsigned char stop);
		void usart2_dma_init(unsigned long baud,unsigned char par,unsigned char stop);
		void uart4_init(unsigned long baud,unsigned char par,unsigned char stop);
		void usart6_dma_init(unsigned long baud,unsigned char par,unsigned char stop);
		void uart7_init(unsigned long baud,unsigned char par,unsigned char stop);
		void uart8_dma_init(unsigned long baud,unsigned char par,unsigned char stop);
		
		int Usart_GetRxLen(unsigned char ch);
		
		int Usart_GetTxLen(unsigned char ch);
		
		int Usart_Tx(unsigned char com, unsigned char *buf, int len);
		
		int Usart_Rx(unsigned char com, unsigned char *buf, int maxlen);
		
		/***************************************************************/
		void Usart1_DMA_TxStart(void);
		
		void Usart1_DMA_RxStart(void);
		
		void Usart2_DMA_TxStart(void);
		
		void Usart2_DMA_RxStart(void);
		
		void Uart4_DMA_RxStart(void);
		
		void Usart6_DMA_TxStart(void);
		
		void Usart6_DMA_RxStart(void);
		
		void Uart8_DMA_RxStart(void);
		
#ifdef __cplusplus
	}
#endif

#endif