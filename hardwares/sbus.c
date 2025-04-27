#include "yn.h"
#include "Sbus.h"
#include "data_struct.h"
#include <stdint.h>

SensorStruc ss_SBUS;
unsigned short RC_channels[18];
BYTE on_safe=0,on_sky=0,on_reset=0, on_takeoff=0,OnStop_eng=0;
float   switch_old;
double psi_0;

float  MidVal (float x, float y,float z)
{
    float   min, max, middle;
    
    if (x>=y)   { max = x;  min = y; }
    else        { max = y;  min = x; }
    
    if      (z<min)     middle = min;
    else if (z>max)     middle = max;
    else                middle = z;

    return (middle);
}

void SBUS_init(void)  //????????,??????
{
    unsigned char i;
	  SBUS_Rev();
    switch_old=GetChannel(8)/1000.0;
}

void SBUS_Decode(unsigned char buffer[])
{
	unsigned char idx;
	uint16_t h = 0;
	RC_channels[0]  = ((buffer[1]    |buffer[2]<<8)                 & 0x07FF);
	RC_channels[1]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
	RC_channels[2]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF);
	RC_channels[3]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
	RC_channels[4]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
	RC_channels[5]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
	RC_channels[6]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
	RC_channels[7]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
	RC_channels[8]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
	RC_channels[9]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
	RC_channels[10] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
	RC_channels[11] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
	RC_channels[12] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
	RC_channels[13] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
	RC_channels[14] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
	RC_channels[15] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);
	
}


int GetChannel(int channel)
{
	int temp;
	if((channel>=1)&&(channel<=18))
	{
		temp = ((int)((RC_channels[channel - 1] -1000)*0.625 + 1505)); 
		return temp;
	}
	else return 0;
}



void  SBUS_Rev(void)
{
	volatile  static  unsigned char    buf[256], head=0, tail=0,tmp=0;
	static          unsigned char    ptr, len, check, FCCdata[25];
	static          int     RxQueLen, numread, idx;
			
    RxQueLen=Usart_GetRxLen(COM8);
    numread=RxQueLen>25?25:RxQueLen;
    if (numread>0) {
				Usart_Rx(COM8,FCCdata,numread);  
        for(idx=0;idx<numread;idx++) { buf[head++]=FCCdata[idx]; }
    }

	len=head-tail;
    while (len>=sizeof(FCCdata)) {
		tmp =(unsigned char)(tail+24);
        if((buf[tail]==0x0F) && ((buf[tmp]==0x00)||(buf[tmp]==0x04)||(buf[tmp]==0x14)||(buf[tmp]==0x24)||(buf[tmp]==0x34)))
        {
            ptr=tail; for(idx=0;idx<sizeof(FCCdata);idx++) FCCdata[idx]=buf[ptr++];
						SBUS_Decode(FCCdata); 
						tail += sizeof(FCCdata); 		
            ss_SBUS.counter++;					
        }
        else tail ++;
        len=head-tail;
    }
}





void PWM_sbus(void)
{
	  unsigned char idx;
    static unsigned char cnt_safe=0, cnt_auto=0,switch_cnt=0; 
    static unsigned char cnt_V2L=0, cnt_L2V=0;   
    
    for(idx=0;idx<16;idx++) 
	{  
		PWMin[idx].vol= (float)GetChannel(idx+1)/1000.0;
		if(idx!=4) 	 PWMin[idx].vol=MidVal(PWMin[idx].vol,1.0,2.0);
	}

    if((PWMin[4].vol>=1.6)&&(PWMin[4].vol<=2.2))        //Safe                   ??C14
    {cnt_safe++; cnt_auto=0;	}
    else if((PWMin[4].vol<=1.4)&&(PWMin[4].vol>=0.8))   //Auto
    {cnt_auto++; cnt_safe=0;	}
    else 
    {cnt_safe=0; cnt_auto=0;	}
    
	if(cnt_safe>=3) 
    {	
    	on_safe=true;
    }
    else if(cnt_auto>=3) 
    {	
    	if(on_safe) 
    	{
    	   tag_SaveEleSwitch=true; tag_SaveEngSwitch=true;	tag_opt=0;

        lon_ref=lon_gps;
    		lat_ref=lat_gps;

    		height_cmd=alt_gps;
    		height_var=alt_gps;
    	//	step_long=0;  ??????case??
    		ac_ail=PWMin[0].val;
    		ac_ele=PWMin[1].val;
    	//	ac_eng=40;  ????

    	}
    	on_safe=false;  
    }
    
    PWMin[0].val = (PWMin[0].vol-1.5-PWMout[0].offset)/PWMout[0].ratio; //副翼   
    PWMin[1].val = (PWMin[1].vol-1.5-PWMout[1].offset)/PWMout[1].ratio; //升降
    PWMin[2].val = (PWMin[2].vol-1.0-PWMout[2].offset)/PWMout[2].ratio; //油门
    PWMin[3].val = (PWMin[3].vol-1.5-PWMout[3].offset)/PWMout[3].ratio; //方向
    PWMin[4].val = (PWMin[4].vol-1.5-PWMout[4].offset)/PWMout[4].ratio; //安控
}


void SBUS_Monitor(void)                       //放入100ms的任务中
{

    static uint8_t	 cnt = 0;
    static uint8_t   flag1s = false;
    static uint16_t	 old_counter;
	
		ss_SBUS.invalid +=1;    
    if (ss_SBUS.invalid>=5) ss_SBUS.fail=1;
    
    
	  cnt ++;
	  cnt %= 100;
	  if((cnt%10) == 0)
	       flag1s = true;                                  
    	    
	  if(flag1s) 
	  {
	       flag1s = false;
	       ss_SBUS.freq = ss_SBUS.counter - old_counter;		   
		   old_counter  = ss_SBUS.counter;
	  }
	  
	
}
