#ifndef _DMA_H_
#define _DMA_H_
#ifdef __cplusplus
 extern "C" {
#endif
	 #include "sys.h"
	 #define DIR_P2M				0
	 #define DIR_M2P				1
	 #define DIR_M2M				2
	 
	 #define DATASIZE8BITS	0
	 #define DATASIZE16BITS	1
	 #define DATASIZE32BITS	2
	 
	 void DMAstream_Init(DMA_Stream_TypeDef *stream,unsigned long par,unsigned char dir,unsigned char msize,unsigned char psize);
	 
#ifdef __cplusplus
 }
#endif
#endif