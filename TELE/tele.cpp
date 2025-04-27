#include "Sbus.h"
#include "math.h"
#include "yn.h"

#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters
#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))
#define DEG_TO_RAD		(3.14159265f / 180.0f)

static INT16S  TimerTick;
SensorStruc	 ss_AHRS;
SensorStruc	 ss_Mpxv;
SensorStruc	 ss_Gps;
unsigned char gps_ini = 0;
navUkfStruct_t navUkfData;

void navCalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat/Rad2Deg);
    sinLat2 = sinLat2 * sinLat2;

    navUkfData.r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    navUkfData.r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

void  WP_Pos2XY (float lon_ref,float lat_ref, float lon,float lat, float *x,float *y)//两点经纬度转东北天坐标
{
	*y=(lat -lat_ref) * navUkfData.r1;
  *x=(lon -lon_ref) * navUkfData.r2;
}

/*UBLOX 获取*/
void TaskUblox()
{
	static unsigned char readbuf[200];
	unsigned char ret = 0;
  int     numread, idx;
	if(Usart_GetRxLen(COM6) > 0)
	{
		numread = Usart_Rx(COM6,readbuf,sizeof(readbuf));
		for (idx = 0; idx<numread; idx++) 
		{ 
			ret = ubloxCharIn(readbuf[idx]);
			if (ret == 1)
				ss_Gps.counter++;
			else if (ret == 2)
				ss_Gps.counter++;
			
		}
		
	  if(used_gps>=5 && !gps_ini) 
		{
			navCalcEarthRadius(lat_gps);
			gps_ini = 1;							
		}
    WP_Pos2XY (lon_ref,lat_ref,lon_gps,lat_gps,&ac_PE,&ac_PN);							
	}
}

//放入100ms的任务
void  Ublox_Monitor()                                 
{
  static INT16U	cnt=0, flag1s=false, old_counter;
	
	ss_Gps.invalid +=1;  if (ss_Gps.invalid>=20)   ss_Gps.fail=1;
	cnt=(cnt+1)%100;     if ((cnt%10)==0)          flag1s=true;                                    

	if (flag1s) {
		flag1s = false;
		ss_Gps.freq = ss_Gps.counter - old_counter;		   
		old_counter = ss_Gps.counter;
	}
}

/*[遥测数据帧发送管理]*/
void  TELE_TxMan (void)
{
  unsigned char   Buf[100];
   
	TELE_FrameA(Buf);
	MakeCheckSum(Buf, 50); 
	TELE_FrameB(&(Buf[50]));
	MakeCheckSum(&(Buf[50]), 50); 
  Usart_Tx(COM7,Buf,sizeof(Buf));	
 	//UartCom5Out(Buf, sizeof(Buf));
}


/*[遥测数据A帧]*/
void  TELE_FrameA (unsigned char Buf[])
{

    dWordStruc  src;

    Buf[0]=0xEB;
    Buf[1]=0x90;
    Buf[2]='A';
	
		src.D[0]=(INT16S)(ac_theta*Rad2Deg*10);      Buf[ 3]=src.B[0];  Buf[ 4]=src.B[1];     //[俯仰角theta_gyo
    src.D[0]=(INT16S)(ac_phi*Rad2Deg*10);       Buf[ 5]=src.B[0];  Buf[ 6]=src.B[1];     /*[滚转角]*/
    src.W[0]=(INT16U)(psi_hmr*10);        Buf[ 7]=src.B[0];  Buf[ 8]=src.B[1];     /*[真航向]*/
    src.D[0]=(INT16S)(ac_P*Rad2Deg*10);         Buf[ 9]=src.B[0];  Buf[10]=src.B[1];     //[滚转角速率Wx_gyo
    src.D[0]=(INT16S)(ac_R*Rad2Deg*10);         Buf[11]=src.B[0];  Buf[12]=src.B[1];     //[偏航角速率Wy_gyo
    src.D[0]=(INT16S)(ac_Q*Rad2Deg*10);         Buf[13]=src.B[0];  Buf[14]=src.B[1];     //[俯仰角速率Wz_gyo

    src.W[0]=(INT16U)(ac_vt*2);          Buf[15]=src.B[0];                        /*[指示空速]*/

    src.D[0]=(INT16S)(height_adc*10);     Buf[16]=src.B[0];  Buf[17]=src.B[1];     /*[气压高度]*/
    src.D[0]=(INT16S)(height_ini*10);     Buf[18]=src.B[0];  Buf[19]=src.B[1];     /*[初始气压高度]*/
   

    src.D[0]=(INT16S)(ac_ail*2);         Buf[20]=src.B[0];                        /*[副翼舵机]*/
    src.W[0]=(INT16U)(engine_var);        Buf[21]=src.B[0];                        /*[油门开度]*/
    src.D[0]=(INT16S)(ac_ele*2);         Buf[22]=src.B[0];                        /*[升降舵机]*/
	  src.D[0]=(INT16S)(rud_var*2);         Buf[23]=src.B[0];                        /*[方向舵机]*/
	  src.W[0]=(INT16U)(psi_cmd);           Buf[24]=src.B[0];                        /*[航向给定]*/
		

    src.D[0]=(INT16S)(theta_var*2);       Buf[25]=src.B[0];                        /*[俯仰角指令]*/
    src.D[0]=(INT16S)((gama_var)*2);        Buf[26]=src.B[0];                        /*[滚转角指令]*/
    src.D[0]=(INT16S)((height_var)*10);     Buf[27]=src.B[0];  Buf[28]=src.B[1];     /*[高度给定量]*/   
    
    
    /////////////////////////////////////////////////////////////////////////////////////////////       			       
    src.D[0]=(INT16S)(H_int*10);             Buf[29]=src.B[0];                        /*[高度积分]*/
    src.D[0]=(INT16S)(ac_dH*10);             Buf[30]=src.B[0];                        /*[高度差]*/
    
    
    /////////////////////////////////////////////////////////////////////////////////////////////

    
    src.D[0]=(INT16S)(ac_PE*10); 			Buf[31]=src.B[0];  Buf[32]=src.B[1];    /*东向距离*/ 
    src.D[0]=(INT16S)(ac_PN*10);            Buf[33]=src.B[0];  Buf[34]=src.B[1];    /*北向距离*/
       
    
    src.D[0]=(INT16S)(PWMin[0].val*2);    Buf[35]=src.B[0];                        /*[左副翼遥控]*/
    src.D[0]=(INT16S)(PWMin[3].val*2);    Buf[36]=src.B[0];                        /*[V尾2遥控]*/
    src.W[0]=(INT16U)(PWMin[2].val);      Buf[37]=src.B[0];                        /*[油门遥控]*/
    src.D[0]=(INT16S)(PWMin[1].val*2);    Buf[38]=src.B[0];                        /*[升降舵遥控]*/
    src.W[0]=(INT16U)(WD_cnt);      	  Buf[39]=src.B[0];                        //[看门狗计数]);
    src.W[0]=(INT16U)(theta_int*2);       Buf[40]=src.B[0];  Buf[41]=src.B[1];     //[俯仰角积分]);   
	  src.W[0]=(INT16U)(battery_volt*600);  Buf[42]=src.B[0];  Buf[43]=src.B[1];     //[KaWx]//(INT16U)(K.P.KaWx*100)
    src.W[0]=(INT16S)(theta_V*2);         Buf[44]=src.B[0];  Buf[45]=src.B[1];     //空速控制俯仰角积分
    src.D[0]=(INT16U)(landing_high*10);  Buf[46]=src.B[0];  Buf[47]=src.B[1];     //着陆起始高度


}

static  unsigned char    Twos[8] = { 1, 2, 4, 8, 16, 32, 64, 128 };

void    SetBit( unsigned char  *B, unsigned char  idx )
{   if (idx<8) *B |= Twos[idx]; }          // Set desired bit 


void    ClearBit( unsigned char  *B, unsigned char  idx )
{   if (idx<8) *B &= (255 - Twos[idx]); }   // Clear desired bit 
                                   
INT16S  BitStatus( unsigned char  B, unsigned char  idx )
{   if (idx<8) return((B&Twos[idx])/Twos[idx]);
    else return(0);
}


/*[遥测数据B帧]*/
void  TELE_FrameB (unsigned char Buf[])
{
    dWordStruc  src;

    Buf[0]=0xEB;
    Buf[1]=0x90;
    Buf[2]='B';

		src.DW  =(INT32S)(lon_gps*1e6);      Buf[ 3]=src.B[0];  Buf[ 4]=src.B[1];     /*[经度]*/
                                         Buf[ 5]=src.B[2];  Buf[ 6]=src.B[3];
    src.DW  =(INT32S)(lat_gps*1e6);      Buf[ 7]=src.B[0];  Buf[ 8]=src.B[1];     /*[纬度]*/
                                         Buf[ 9]=src.B[2];  Buf[10]=src.B[3];
    src.W[0]=(INT16U)(Vd_gps*2);         Buf[11]=src.B[0];                        /*[地速]*/
    src.W[0]=(INT16U)(psi_gps*10);       Buf[12]=src.B[0];  Buf[13]=src.B[1];     /*[航迹角]*/
    src.D[0]=(INT16S)(alt_gps*10);       Buf[14]=src.B[0];  Buf[15]=src.B[1];     /*[海拔高度]*/
    src.D[0]=(INT16S)(Hdot_gps*10);      Buf[16]=src.B[0];  Buf[17]=src.B[1];     /*[升降速度]*/
    src.W[0]=used_gps;                   Buf[18]=used_gps;                        /*[可见星数]*/
	
	                                       Buf[19]=0;
	                                       Buf[20]=0;
    if (!ss_AHRS.fail)                   SetBit(&Buf[20],4);                      /*[AHRS数据链通]*/
        else                             ClearBit(&Buf[20],4); 
    if (!ss_Mpxv.fail)                   SetBit(&Buf[20],5);                      /*[空速数据链通]*/
        else                             ClearBit(&Buf[20],5);
    if (!ss_Gps.fail)                    SetBit(&Buf[20],6);                      /*[GPS数据链通]*/  
        else                             ClearBit(&Buf[20],6);                      
    if (tag_DGPS)                        SetBit(&Buf[20],7);                      /*[DGPS]*/
        else                             ClearBit(&Buf[20],7);

    Buf[21]=0;                                                                    /*[系统状态字3]*/
    if (on_reset)                        SetBit(&Buf[21],0);                      /*[复位]*/

    if (on_takeoff)                      SetBit(&Buf[21],1);                      /*[起飞]*/

    if (OnStop_eng)                      SetBit(&Buf[21],2);                      /*[发动机预停]*/

    if (on_safe)                         SetBit(&Buf[21],3);                      /*[安控状态]*/
                                                                                /*[Buf[3].5~7=自主飞行模态]*/

    Buf[22]=0;                                                                    /*[系统状态字4]*/
    if (on_sky)                          SetBit(&Buf[22],0);                      /*[飞机在空中]*/
     
    if (tag_opt)                         SetBit(&Buf[22],2);                      /*[辅助控制状态]*/
   


    ////////////////////////////////////////////////////////////////////////////////////////////
    src.D[0]=(INT16S)(ac_dpsi*10);          Buf[23]=src.B[0];  Buf[24]=src.B[1];     /*[偏航角]*/
                                            Buf[25]=ac_dot;                          /*[航段号]*/
    src.D[0]=(INT16S)(ac_dL*10.0f);         Buf[26]=src.B[0];  Buf[27]=src.B[1];     /*[待飞距离]*/
    src.D[0]=(INT16S)(ac_dZ*10);            Buf[28]=src.B[0];  Buf[29]=src.B[1];     /*[侧偏距]*/
    ////////////////////////////////////////////////////////////////////////////////////////////
    
    src.D[0]=(INT16S)(0*10);             Buf[30]=src.B[0];  Buf[31]=src.B[1];     /*[机场高度]*/    
    
    src.DW  =(INT32S)(ac_ax*1e6);
                                         Buf[32]=src.B[0];  Buf[33]=src.B[1];     /*[前向过载]*/
                                         Buf[34]=src.B[2];  Buf[35]=src.B[3];      
	                                     Buf[36]=0;                                   
	  src.W[0]=(INT16U)(ac_psi*10);        Buf[37]=src.B[0];  Buf[38]=src.B[1];     /*[航向]*/
    src.D[0]=(INT16S)(ss_Gps.freq);      Buf[39]=src.B[0];                        /*[gps帧频率]*/
	  src.DW  =(INT32S)(H_acc*1000);       Buf[40]=src.B[0];  Buf[41]=src.B[1];     //水平误差
                                         Buf[42]=src.B[2];  Buf[43]=src.B[3];  
    src.DW  =(INT32S)(V_acc*1000);       Buf[44]=src.B[0];  Buf[45]=src.B[1];     //高度误差
                                         Buf[46]=src.B[2];  Buf[47]=src.B[3];
                                                                              
}

