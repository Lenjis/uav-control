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
			
			/***Sbus 信号获取***/
			SBUS_Rev();
			
			/***PWM out***/
			PWM_sbus(); 
			
			/***UBLOX GPS信号获取***/
			TaskUblox();
			
			/***IMU 陀螺仪&&加速度计信号获取***/
			get_IMU();
			
			/***ESKF导航解算***/
			ESKF();
			
			/***导航信息获取：姿态角，速度，下沉率，位置，PQR.....***/
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
			
			/***串口调试***/
//			len=sprintf((char*)str,"Attitude: roll=%f,pitch=%f,yaw=%f\r\n",ac_phi*Rad2Deg,ac_theta*Rad2Deg,ac_psi*Rad2Deg);
			len=sprintf((char*)str,"Sbus_freq: %d\r\n", ss_SBUS.freq);
			len+=sprintf((char*)str+len,"att: %f %f %f\r\n", ac_phi*Rad2Deg, ac_theta*Rad2Deg, ac_psi*Rad2Deg);
//			len+=sprintf((char*)str+len,"pwmCh: %ld %ld %ld %ld\r\n", test[0],test[1],test[2],test[3]);
//			len+=sprintf((char*)str+len,"PWMin: %f, %f, %f, %f, %f\r\n",PWMin[0].vol,PWMin[1].vol,PWMin[2].vol,PWMin[3].vol,PWMin[4].vol);
//		  len+=sprintf((char*)str+len,"AP_height: %f\r\n",AP_height);

				
			len+=sprintf((char*)str+len,"Gyro: %f,%f,%f\r\n",ac_P*Rad2Deg,ac_Q*Rad2Deg,ac_R*Rad2Deg);
			len+=sprintf((char*)str+len,"Accel: %f,%f,%f\r\n",ac_ax,ac_ay,ac_az);
		//	Usart_Tx(COM7,str,len);
			/***串口调试***/
			
			TELE_TxMan();  //遥测下行
		}
		if(Timer1s)
		{
			Timer1s=0;
			
			/***X7 温度***/
			BMI088_TempAsk();
			/***X7 温度***/
			
			/***LED闪烁***/
			set_pin(GPIOI,7,(s++)&1);
			/***LED闪烁***/
			
			/***看门狗***/
			WD_cnt++;
		}
	}
	return 0;
}

