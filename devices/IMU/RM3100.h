#ifndef _RM3100_H_
#define _RM3100_H_

#define RM3100_READ		0x80
#define RM3100_WRITE	0x00

#define RM3100_POLL		0x00
#define RM3100_CMM		0x01

#define RM3100_CCX_M	0x04
#define RM3100_CCX_L	0x05
#define RM3100_CCY_M	0x06
#define RM3100_CCY_L	0x07
#define RM3100_CCZ_M	0x08
#define RM3100_CCZ_L	0x09

#define RM3100_TMRC		0x0b

#define RM3100_MX			0x24
#define RM3100_MY			0x27
#define RM3100_MZ			0x2a

#define RM3100_BIST		0x33
#define RM3100_STATUS	0x34
#define RM3100_HSHAKE 0x35
#define RM3100_REVID	0x36

#ifdef __cplusplus
	extern "C"{
#endif		
		void RM3100_init(void);
			
		void RM3100_Get(unsigned char *buf);
		
		void RM3100_Ask(unsigned char cmd);
			
		void RM3100_ReadMagn(float *magx,float *magy,float *magz);
		
		extern unsigned short magn_cnt,magn_freq;
		
#ifdef __cplusplus
	}
#endif

#endif