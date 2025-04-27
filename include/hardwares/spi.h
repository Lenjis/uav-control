#ifndef _SPI_H_
#define _SPI_H_

#ifdef __cplusplus
	extern "C"{
#endif
		
		void spi_init(void);
		void SPI1_DMA_Init();
		void SPI2_DMA_Init();
		void SPI4_DMA_Init();
		
		unsigned char SPI1_ReadWrite_DMA(unsigned char *buf,int len);
		void SPI1_RxManager(unsigned char *buf);
		void SPI1_Tx(unsigned char id, unsigned char *buf, int len, void (*_fun)(unsigned char*));
		
		unsigned char SPI2_ReadWrite_DMA(unsigned char *buf,int len);
		void SPI2_RxManager(unsigned char *buf);
		void SPI2_Tx(unsigned char id, unsigned char *buf, int len, void (*_fun)(unsigned char*));
		
		unsigned char SPI4_ReadWrite_DMA(unsigned char *buf,int len);
		void SPI4_RxManager(unsigned char *buf);
		void SPI4_Tx(unsigned char id, unsigned char *buf, int len, void (*_fun)(unsigned char *));
		
#ifdef __cplusplus
	}
#endif

#endif