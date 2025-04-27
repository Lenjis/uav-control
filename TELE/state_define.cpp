#include "yn.h"

/*********״̬�������� simu state define*********/     
float     ac_theta, ac_phi, ac_psi;   //attitude ��̬��
float     ac_alpha, ac_beta;          //alpha, beta ӭ�ǣ��໬��
float     ac_P,ac_Q,ac_R;             //PQR ���������
float     ac_ax, ac_ay, ac_az;        //accelx �������
float     ac_PE, ac_PN, ac_height;    //XYZ ����ϵλ��
float     ac_vt;   //����
float     ac_dH;   //�³���
float     ac_dZ;   //��ƫ��
float     ac_dL;   //���ɾ���
float     ac_dpsi; //ƫ����
float     ac_dot;  //���κ�
float     psi_hmr; //�溽��
float     ac_mass = 17;

/*********���Ʊ������� sim control para define*********/
float     theta_cmd, gama_cmd, engine_cmd,  height_cmd,  Vt_cmd, Hdot_cmd;  //ָ�����
float     theta_var, gama_var, engine_var,  height_var,  Vt_var;             //ָ����������󣿣�
float     ele_var, ail_var, rud_var;                                //����ָ��
float   	ail_trim, ele_trim, rud_trim, eng_trim;                   //��ʼƽ����ƽֵ
float     psi_cmd;                                                  //��������

float     theta_int, theta_int_min, theta_int_max;                  //�����ǿ��ƻ��֣������޷�
float     gama_int, gama_int_min, gama_int_max;                     //��ת�ǿ��ƻ��֣������޷�
float     H_int , H_int_min, H_int_max;                             //�߶ȿ��ƻ��֣������޷�
float     KpV, IpV, engine_V=0,theta_V=0;                           //���ٿ������棬���Ż��֣������ǻ���
float     Igama=0;                                                  //��ת�ǿ��ƻ���           
float     landing_high;                                             //��½��ʼ�߶�

/***********ң�������Ʋ���***********/
short     tag_LongSwitch=0, tag_SaveEleSwitch=0, tag_SaveEngSwitch=0, tag_ViHoldON=0, tag_opt=0; 

/***********��ѹ�Ʋ���***********/
float  	  height_ini=0,   //��ʼ��ѹ�߶�
	        height_adc;     //��ѹ�߶�

/***********GPS�źŲ���***********/
float       lon_gps,   //����        
						lat_gps,   //γ�� 
						lon_ref,   //XYZ����ϵԭ�㾭�� 
						lat_ref,   //XYZ����ϵԭ��γ�� 
						psi_init,  //��ʼ����
	          psi_gps,   //GPS������       
            alt_gps,   //GPS�߶�      
						Hdot_gps,  //�����ٶ�
						H_acc,     //GPSˮƽ����
            V_acc,     //GPS��ֱ���� 
            Vd_gps;    //GPS����
      
unsigned char	used_gps;  //GPS������    
unsigned char tag_DGPS; 
            
/***********KaWx***********/
unsigned short        battery_volt;		
		 
		
		
		