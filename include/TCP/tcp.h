#ifndef __tcp__
#define __tcp__

#include "data_struct.h"

#ifdef  __cplusplus
    extern "C" {
#endif

typedef unsigned char BOOL;

extern  SensorStruc  mems;
extern  BOOL GPS_OK_flag;
extern  BOOL IMU_OK_flag;
extern  BOOL YAW_OK_flag;
extern  BOOL truecourse_flag;
extern  int Ublox_freq,Ublox_freq_cnt;
extern float AP_temp,AP_press,AP_height;
/*---------------------------------------------------------------------------*/
void  MemsRx (unsigned char tmp[]);
void  TaskMemsRev(void);
void  MemsMonitor(void);
void  MemsInit(void);  
int Data_updata(void);
extern uint8_t Usart1_out_DATA[100];
extern SensorStruc state_NovAtel718D;
extern GPS_Data_value NovAtel718D_data;

void Task_NovAtel_718D(void);
unsigned short MemsRead(unsigned char addr);
void  Monitor_NovAtel_718D(void);
unsigned short MemsWrite(unsigned char addr, unsigned short data);
float  Psi_360(float angle);

void TELE_TaskTx(void);    
void TELE_TaskTx2(void);
void TaskSBGE(void);
void TELE_Frame(unsigned char* buf);
void MakeCheckSum(unsigned char* Buf, unsigned short  len);
//void GetSensorsData(void);

void Task_NovAtel_718D(void);
void Monitor_NovAtel_718D(void);
#ifdef  __cplusplus
}  
#endif
#endif