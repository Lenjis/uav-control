#include "init.h"
#include "yn.h"
#include "hardwares/usart.h"
#include "navigation_ESKF/psins.hpp"
#include "navigation_ESKF/insgps.hpp"
#include "Sim.h"

unsigned char s=0, len = 0;
unsigned char str[200];
int main(void)
{
 	init();
	ubloxInit();
	simu_init();
	while(1)
	{
		if(Timer2ms)
		{
			Timer2ms=0;
		}
		if(Timer5ms)
		{
			Timer5ms=0;
			
			/***Sbus �źŻ�ȡ***/
			SBUS_Rev();
			
			/***PWM out***/
			PWM_sbus(); 
			
			/***UBLOX GPS�źŻ�ȡ***/
			TaskUblox();
			
			/***IMU ������&&���ٶȼ��źŻ�ȡ***/
			get_IMU();
			
			/***ESKF��������***/
			ESKF();
			
			/***������Ϣ��ȡ����̬�ǣ��ٶȣ��³��ʣ�λ�ã�PQR.....***/
			get_flightstate();
		}
		if(Timer10ms)
		{
			Timer10ms = 0;
			LAW_Act();
			LAW_Out();
		}
		if(Timer100ms)
		{
			Timer100ms=0;
			
			/***flight task***/
			ctrl_flytask();
			
			/***Sbus frequency***/
			SBUS_Monitor();
			/***Sbus frequency***/
			
			/***UBLOX frequency***/
      Ublox_Monitor();
			/***UBLOX frequency***/
			
			/***���ڵ���***/
//			len=sprintf((char*)str,"Attitude: roll=%f,pitch=%f,yaw=%f\r\n",ac_phi*Rad2Deg,ac_theta*Rad2Deg,ac_psi*Rad2Deg);
			len=sprintf((char*)str,"Sbus_freq: %d\r\n", ss_SBUS.freq);
			len+=sprintf((char*)str+len,"att: %f %f %f\r\n", ac_phi*Rad2Deg, ac_theta*Rad2Deg, ac_psi*Rad2Deg);
//			len+=sprintf((char*)str+len,"pwmCh: %ld %ld %ld %ld\r\n", test[0],test[1],test[2],test[3]);
//			len+=sprintf((char*)str+len,"PWMin: %f, %f, %f, %f, %f\r\n",PWMin[0].vol,PWMin[1].vol,PWMin[2].vol,PWMin[3].vol,PWMin[4].vol);
//		  len+=sprintf((char*)str+len,"AP_height: %f\r\n",AP_height);

				
			len+=sprintf((char*)str+len,"Gyro: %f,%f,%f\r\n",ac_P*Rad2Deg,ac_Q*Rad2Deg,ac_R*Rad2Deg);
			len+=sprintf((char*)str+len,"Accel: %f,%f,%f\r\n",ac_ax,ac_ay,ac_az);
		//	Usart_Tx(COM7,str,len);
			/***���ڵ���***/
			
			TELE_TxMan();  //ң������
		}
		if(Timer1s)
		{
			Timer1s=0;
			
			/***X7 �¶�***/
			BMI088_TempAsk();
			/***X7 �¶�***/
			
			/***LED��˸***/
			set_pin(GPIOI,7,(s++)&1);
			/***LED��˸***/
			
			/***���Ź�***/
			WD_cnt++;
		}
	}
	return 0;
}

