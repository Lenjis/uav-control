#ifndef _INIT_H_
#define _INIT_H_

#ifdef __cplusplus
 extern "C" {
#endif

	#include "FRAM/FM25Vx.h"
	#include "IMU/RM3100.h"
	#include "IMU/BMI088.h"
	#include "IMU/ADIS16470.h"
	#include "Pressure/MS5611.h"

	void init(void);
	extern unsigned short  WD_cnt;
#ifdef __cplusplus
 }
#endif

#endif
 