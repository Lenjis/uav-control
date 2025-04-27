#ifndef _STATE_H_
#define _STATE_H_

#ifdef __cplusplus
	extern "C"{
#endif

#define ac_rud 	rud_var      //�����
#define ac_eng  engine_var   //����
#define ac_ail  ail_var      //�����
#define ac_ele  ele_var      //������
		 
/*********״̬�������� simu state define*********/     
extern float     ac_theta, ac_phi, ac_psi;   //attitude ��̬��
extern float     ac_alpha, ac_beta;          //alpha, beta ӭ�ǣ��໬��
extern float     ac_P,ac_Q,ac_R;             //PQR ���������
extern float     ac_ax, ac_ay, ac_az;        //accelx �������
extern float     ac_PE, ac_PN, ac_height;    //XYZ ����ϵλ��
extern float     ac_vt;   //����
extern float     ac_dH;   //�³���
extern float     ac_dZ;   //��ƫ��
extern float     ac_dL;   //���ɾ���
extern float     ac_dpsi; //ƫ����
extern float     ac_dot;  //���κ�
extern float     psi_hmr;
extern float     ac_mass;

/*********���Ʊ������� sim control para define*********/
extern float     theta_cmd, gama_cmd, engine_cmd,  height_cmd,  Vt_cmd,  Hdot_cmd;  //ָ�����
extern float     theta_var, gama_var, engine_var,  height_var,  Vt_var;             //ָ����������󣿣�
extern float     ele_var, ail_var, rud_var;                                //����ָ��
extern float   	 ail_trim, ele_trim, rud_trim, eng_trim;                   //��ʼƽ����ƽֵ
extern float     psi_cmd;                                                  //��������

extern float     theta_int, theta_int_min, theta_int_max;                  //�����ǿ��ƻ��֣������޷�
extern float     gama_int, gama_int_min, gama_int_max;                     //��ת�ǿ��ƻ��֣������޷�
extern float     H_int , H_int_min, H_int_max;                             //�߶ȿ��ƻ��֣������޷�
extern float     KpV, IpV, engine_V,theta_V;                               //���ٿ������棬���Ż��֣������ǻ���
extern float     Igama;                                                  //��ת�ǿ��ƻ���           
extern float     landing_high;                                             //��½��ʼ�߶�

/***********ң�������Ʋ���***********/
extern short     tag_LongSwitch, tag_SaveEleSwitch, tag_SaveEngSwitch, tag_ViHoldON, tag_opt; 

/***********��ѹ�Ʋ���***********/
extern float  	 height_ini,     //��ʼ��ѹ�߶�
	               height_adc;     //��ѹ�߶�

/***********GPS�źŲ���***********/
extern float     lon_gps,   //����        
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
      
extern unsigned char	used_gps;  //GPS������    
extern unsigned char tag_DGPS; 
            
/***********KaWx***********/
extern unsigned short        battery_volt;		

#ifdef __cplusplus
}
#endif    

#endif
