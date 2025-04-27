#ifndef _ADIS16470_H_
#define _ADIS16470_H_
#ifdef __cplusplus
	extern "C"{
#endif		
	extern int ADIS_cnt,adis_freq;
	int ADIS_Init(void);
	void ADIS_Read(float *a, float *g, float *temp);
	void ADIS_Ask(unsigned char id);
	void ADIS_Get(unsigned char *buf);
#ifdef __cplusplus
	}
#endif		
#endif