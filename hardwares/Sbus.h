#ifndef _SBUS_H_
#define _SBUS_H_

#ifdef  __cplusplus
extern "C" {
#endif

#include "yn.h"

extern SensorStruc ss_SBUS;
extern BYTE on_safe,on_sky,on_reset, on_takeoff,OnStop_eng;
void  SBUS_Rev(void);
void  SBUS_Decode(unsigned char buffer[]);
int   GetChannel(int channel);
void  PWM_sbus(void);
void  SBUS_Monitor(void) ;

typedef struct{
    float pwmper;
    float pwmhi; 
	unsigned long pwmper_cnt;
	unsigned long pwmhi_cnt;
} mpwm_t;

extern AnalogStruc            PWMin[16];
extern AnalogStruc            PWMout[12];

void get_flightstate();
float  MidVal (float x, float y,float z);
#ifdef  __cplusplus
}  
#endif

#endif