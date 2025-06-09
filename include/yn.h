#ifndef _YN_H_
#define _YN_H_

#include "data_struct/data_struct.h"
#include "TELE/state_define.h"	
#include "TELE/teleH.h"	
#include "FRAM/FM25Vx.h"
#include "IMU/RM3100.h"
#include "IMU/BMI088.h"
#include "IMU/ADIS16470.h"
#include "Pressure/MS5611.h"
#include "hardwares/usart.h"
#include "hardwares/gpio.h"
#include "TCP/tcp.h"
#include "GPS_message/get_gps.h"
#include "startup/init.h"
#include "hardwares/pwm.h"
#include "hardwares/Sbus.h"

#ifdef __cplusplus
extern "C" {
#endif

    typedef signed short   INT16S;
    typedef unsigned short INT16U;
    typedef signed long    INT32S;

    extern float runtime;

#ifdef __cplusplus
}
#endif

#endif
