#ifndef _TELE_H_
#define _TELE_H_

#ifdef __cplusplus
	extern "C"{
#endif

#include "yn.h"
		
extern  unsigned char    YEAR,MONTH,DAY,HOUR,MIN,SEC;
extern  unsigned char           RemoteCmd,                 /*[��ɢң��ָ��]*/
                       RemoteCode;                /*[ң��ָ��ر�]*/
extern  unsigned char           echo_tele;                 /*[������־]*/
extern  SensorStruc         tele;

void        TELE_TaskRx(void);     /*[ң�غ͵�������������������][Task]*/

void        TELE_RxMan(unsigned char Buf[]);       /*[��ң������֡�����ۺϴ���]*/
void        TELE_Monitor(void);           /*[ң�����������][100ms����]*/

void        TELE_TaskTx(void);            /*[ң��͵��������з�������][Task]*/
void        TELE_TxMan(void);             /*[ң������֡���͹���]*/
void        TELE_FrameA (unsigned char Buf[]);      /*[ң������A֡]*/
void        TELE_FrameB(unsigned char Buf[]);      /*[ң������B֡]*/

void        Ublox_Monitor(); /*Ublox GPSƵ�ʼ���*/
void        TaskUblox();  /*Ublox GPS�źŻ�ȡ*/


typedef struct {
   
    double r1, r2;
    
} navUkfStruct_t;

#ifdef __cplusplus
}
#endif    

#endif
