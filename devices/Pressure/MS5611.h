#ifndef _MS5611_H_
#define _MS5611_H_

#ifdef __cplusplus
	extern "C"{
#endif
	void MS5611_init(void);
	float MS5611_GetPress(float *temp, float *press);
	void MS5611_task(void);
#ifdef __cplusplus
	}
#endif
#endif
