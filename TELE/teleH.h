#ifndef _TELE_H_
#define _TELE_H_

#ifdef __cplusplus
	extern "C"{
#endif

#include "yn.h"
		
extern  unsigned char    YEAR,MONTH,DAY,HOUR,MIN,SEC;
extern  unsigned char           RemoteCmd,                 /*[离散遥控指令]*/
                       RemoteCode;                /*[遥控指令回报]*/
extern  unsigned char           echo_tele;                 /*[回馈标志]*/
extern  SensorStruc         tele;

void        TELE_TaskRx(void);     /*[遥控和地面检测上行链接收任务][Task]*/

void        TELE_RxMan(unsigned char Buf[]);       /*[对遥控数据帧进行综合处理]*/
void        TELE_Monitor(void);           /*[遥控数据链监控][100ms调度]*/

void        TELE_TaskTx(void);            /*[遥测和地面检测下行发送任务][Task]*/
void        TELE_TxMan(void);             /*[遥测数据帧发送管理]*/
void        TELE_FrameA (unsigned char Buf[]);      /*[遥测数据A帧]*/
void        TELE_FrameB(unsigned char Buf[]);      /*[遥测数据B帧]*/

void        Ublox_Monitor(); /*Ublox GPS频率计算*/
void        TaskUblox();  /*Ublox GPS信号获取*/


typedef struct {
   
    double r1, r2;
    
} navUkfStruct_t;

#ifdef __cplusplus
}
#endif    

#endif
