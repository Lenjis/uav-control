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

void  WP_Pos2XY (float lon_ref,float lat_ref, float lon,float lat, float *x,float *y)//���㾭γ��ת����������
{
	*y=(lat -lat_ref) * navUkfData.r1;
  *x=(lon -lon_ref) * navUkfData.r2;
}

/*UBLOX ��ȡ*/
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

//����100ms������
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

/*[ң������֡���͹���]*/
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


/*[ң������A֡]*/
void  TELE_FrameA (unsigned char Buf[])
{

    dWordStruc  src;

    Buf[0]=0xEB;
    Buf[1]=0x90;
    Buf[2]='A';
	
		src.D[0]=(INT16S)(ac_theta*Rad2Deg*10);      Buf[ 3]=src.B[0];  Buf[ 4]=src.B[1];     //[������theta_gyo
    src.D[0]=(INT16S)(ac_phi*Rad2Deg*10);       Buf[ 5]=src.B[0];  Buf[ 6]=src.B[1];     /*[��ת��]*/
    src.W[0]=(INT16U)(psi_hmr*10);        Buf[ 7]=src.B[0];  Buf[ 8]=src.B[1];     /*[�溽��]*/
    src.D[0]=(INT16S)(ac_P*Rad2Deg*10);         Buf[ 9]=src.B[0];  Buf[10]=src.B[1];     //[��ת������Wx_gyo
    src.D[0]=(INT16S)(ac_R*Rad2Deg*10);         Buf[11]=src.B[0];  Buf[12]=src.B[1];     //[ƫ��������Wy_gyo
    src.D[0]=(INT16S)(ac_Q*Rad2Deg*10);         Buf[13]=src.B[0];  Buf[14]=src.B[1];     //[����������Wz_gyo

    src.W[0]=(INT16U)(ac_vt*2);          Buf[15]=src.B[0];                        /*[ָʾ����]*/

    src.D[0]=(INT16S)(height_adc*10);     Buf[16]=src.B[0];  Buf[17]=src.B[1];     /*[��ѹ�߶�]*/
    src.D[0]=(INT16S)(height_ini*10);     Buf[18]=src.B[0];  Buf[19]=src.B[1];     /*[��ʼ��ѹ�߶�]*/
   

    src.D[0]=(INT16S)(ac_ail*2);         Buf[20]=src.B[0];                        /*[������]*/
    src.W[0]=(INT16U)(engine_var);        Buf[21]=src.B[0];                        /*[���ſ���]*/
    src.D[0]=(INT16S)(ac_ele*2);         Buf[22]=src.B[0];                        /*[�������]*/
	  src.D[0]=(INT16S)(rud_var*2);         Buf[23]=src.B[0];                        /*[������]*/
	  src.W[0]=(INT16U)(psi_cmd);           Buf[24]=src.B[0];                        /*[�������]*/
		

    src.D[0]=(INT16S)(theta_var*2);       Buf[25]=src.B[0];                        /*[������ָ��]*/
    src.D[0]=(INT16S)((gama_var)*2);        Buf[26]=src.B[0];                        /*[��ת��ָ��]*/
    src.D[0]=(INT16S)((height_var)*10);     Buf[27]=src.B[0];  Buf[28]=src.B[1];     /*[�߶ȸ�����]*/   
    
    
    /////////////////////////////////////////////////////////////////////////////////////////////       			       
    src.D[0]=(INT16S)(H_int*10);             Buf[29]=src.B[0];                        /*[�߶Ȼ���]*/
    src.D[0]=(INT16S)(ac_dH*10);             Buf[30]=src.B[0];                        /*[�߶Ȳ�]*/
    
    
    /////////////////////////////////////////////////////////////////////////////////////////////

    
    src.D[0]=(INT16S)(ac_PE*10); 			Buf[31]=src.B[0];  Buf[32]=src.B[1];    /*�������*/ 
    src.D[0]=(INT16S)(ac_PN*10);            Buf[33]=src.B[0];  Buf[34]=src.B[1];    /*�������*/
       
    
    src.D[0]=(INT16S)(PWMin[0].val*2);    Buf[35]=src.B[0];                        /*[����ң��]*/
    src.D[0]=(INT16S)(PWMin[3].val*2);    Buf[36]=src.B[0];                        /*[Vβ2ң��]*/
    src.W[0]=(INT16U)(PWMin[2].val);      Buf[37]=src.B[0];                        /*[����ң��]*/
    src.D[0]=(INT16S)(PWMin[1].val*2);    Buf[38]=src.B[0];                        /*[������ң��]*/
    src.W[0]=(INT16U)(WD_cnt);      	  Buf[39]=src.B[0];                        //[���Ź�����]);
    src.W[0]=(INT16U)(theta_int*2);       Buf[40]=src.B[0];  Buf[41]=src.B[1];     //[�����ǻ���]);   
	  src.W[0]=(INT16U)(battery_volt*600);  Buf[42]=src.B[0];  Buf[43]=src.B[1];     //[KaWx]//(INT16U)(K.P.KaWx*100)
    src.W[0]=(INT16S)(theta_V*2);         Buf[44]=src.B[0];  Buf[45]=src.B[1];     //���ٿ��Ƹ����ǻ���
    src.D[0]=(INT16U)(landing_high*10);  Buf[46]=src.B[0];  Buf[47]=src.B[1];     //��½��ʼ�߶�


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


/*[ң������B֡]*/
void  TELE_FrameB (unsigned char Buf[])
{
    dWordStruc  src;

    Buf[0]=0xEB;
    Buf[1]=0x90;
    Buf[2]='B';

		src.DW  =(INT32S)(lon_gps*1e6);      Buf[ 3]=src.B[0];  Buf[ 4]=src.B[1];     /*[����]*/
                                         Buf[ 5]=src.B[2];  Buf[ 6]=src.B[3];
    src.DW  =(INT32S)(lat_gps*1e6);      Buf[ 7]=src.B[0];  Buf[ 8]=src.B[1];     /*[γ��]*/
                                         Buf[ 9]=src.B[2];  Buf[10]=src.B[3];
    src.W[0]=(INT16U)(Vd_gps*2);         Buf[11]=src.B[0];                        /*[����]*/
    src.W[0]=(INT16U)(psi_gps*10);       Buf[12]=src.B[0];  Buf[13]=src.B[1];     /*[������]*/
    src.D[0]=(INT16S)(alt_gps*10);       Buf[14]=src.B[0];  Buf[15]=src.B[1];     /*[���θ߶�]*/
    src.D[0]=(INT16S)(Hdot_gps*10);      Buf[16]=src.B[0];  Buf[17]=src.B[1];     /*[�����ٶ�]*/
    src.W[0]=used_gps;                   Buf[18]=used_gps;                        /*[�ɼ�����]*/
	
	                                       Buf[19]=0;
	                                       Buf[20]=0;
    if (!ss_AHRS.fail)                   SetBit(&Buf[20],4);                      /*[AHRS������ͨ]*/
        else                             ClearBit(&Buf[20],4); 
    if (!ss_Mpxv.fail)                   SetBit(&Buf[20],5);                      /*[����������ͨ]*/
        else                             ClearBit(&Buf[20],5);
    if (!ss_Gps.fail)                    SetBit(&Buf[20],6);                      /*[GPS������ͨ]*/  
        else                             ClearBit(&Buf[20],6);                      
    if (tag_DGPS)                        SetBit(&Buf[20],7);                      /*[DGPS]*/
        else                             ClearBit(&Buf[20],7);

    Buf[21]=0;                                                                    /*[ϵͳ״̬��3]*/
    if (on_reset)                        SetBit(&Buf[21],0);                      /*[��λ]*/

    if (on_takeoff)                      SetBit(&Buf[21],1);                      /*[���]*/

    if (OnStop_eng)                      SetBit(&Buf[21],2);                      /*[������Ԥͣ]*/

    if (on_safe)                         SetBit(&Buf[21],3);                      /*[����״̬]*/
                                                                                /*[Buf[3].5~7=��������ģ̬]*/

    Buf[22]=0;                                                                    /*[ϵͳ״̬��4]*/
    if (on_sky)                          SetBit(&Buf[22],0);                      /*[�ɻ��ڿ���]*/
     
    if (tag_opt)                         SetBit(&Buf[22],2);                      /*[��������״̬]*/
   


    ////////////////////////////////////////////////////////////////////////////////////////////
    src.D[0]=(INT16S)(ac_dpsi*10);          Buf[23]=src.B[0];  Buf[24]=src.B[1];     /*[ƫ����]*/
                                            Buf[25]=ac_dot;                          /*[���κ�]*/
    src.D[0]=(INT16S)(ac_dL*10.0f);         Buf[26]=src.B[0];  Buf[27]=src.B[1];     /*[���ɾ���]*/
    src.D[0]=(INT16S)(ac_dZ*10);            Buf[28]=src.B[0];  Buf[29]=src.B[1];     /*[��ƫ��]*/
    ////////////////////////////////////////////////////////////////////////////////////////////
    
    src.D[0]=(INT16S)(0*10);             Buf[30]=src.B[0];  Buf[31]=src.B[1];     /*[�����߶�]*/    
    
    src.DW  =(INT32S)(ac_ax*1e6);
                                         Buf[32]=src.B[0];  Buf[33]=src.B[1];     /*[ǰ�����]*/
                                         Buf[34]=src.B[2];  Buf[35]=src.B[3];      
	                                     Buf[36]=0;                                   
	  src.W[0]=(INT16U)(ac_psi*10);        Buf[37]=src.B[0];  Buf[38]=src.B[1];     /*[����]*/
    src.D[0]=(INT16S)(ss_Gps.freq);      Buf[39]=src.B[0];                        /*[gps֡Ƶ��]*/
	  src.DW  =(INT32S)(H_acc*1000);       Buf[40]=src.B[0];  Buf[41]=src.B[1];     //ˮƽ���
                                         Buf[42]=src.B[2];  Buf[43]=src.B[3];  
    src.DW  =(INT32S)(V_acc*1000);       Buf[44]=src.B[0];  Buf[45]=src.B[1];     //�߶����
                                         Buf[46]=src.B[2];  Buf[47]=src.B[3];
                                                                              
}

