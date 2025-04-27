#ifndef _STATE_H_
#define _STATE_H_

#ifdef __cplusplus
	extern "C"{
#endif

#define ac_rud 	rud_var      //方向舵
#define ac_eng  engine_var   //油门
#define ac_ail  ail_var      //副翼舵
#define ac_ele  ele_var      //升降舵
		 
/*********状态变量定义 simu state define*********/     
extern float     ac_theta, ac_phi, ac_psi;   //attitude 姿态角
extern float     ac_alpha, ac_beta;          //alpha, beta 迎角，侧滑角
extern float     ac_P,ac_Q,ac_R;             //PQR 三轴角速率
extern float     ac_ax, ac_ay, ac_az;        //accelx 机体过载
extern float     ac_PE, ac_PN, ac_height;    //XYZ 坐标系位置
extern float     ac_vt;   //空速
extern float     ac_dH;   //下沉率
extern float     ac_dZ;   //侧偏距
extern float     ac_dL;   //待飞距离
extern float     ac_dpsi; //偏航角
extern float     ac_dot;  //航段号
extern float     psi_hmr;
extern float     ac_mass;

/*********控制变量定义 sim control para define*********/
extern float     theta_cmd, gama_cmd, engine_cmd,  height_cmd,  Vt_cmd,  Hdot_cmd;  //指令变量
extern float     theta_var, gama_var, engine_var,  height_var,  Vt_var;             //指令变量（软化后？）
extern float     ele_var, ail_var, rud_var;                                //舵面指令
extern float   	 ail_trim, ele_trim, rud_trim, eng_trim;                   //初始平飞配平值
extern float     psi_cmd;                                                  //给定航向

extern float     theta_int, theta_int_min, theta_int_max;                  //俯仰角控制积分，上下限幅
extern float     gama_int, gama_int_min, gama_int_max;                     //滚转角控制积分，上下限幅
extern float     H_int , H_int_min, H_int_max;                             //高度控制积分，上下限幅
extern float     KpV, IpV, engine_V,theta_V;                               //空速控制增益，油门积分，俯仰角积分
extern float     Igama;                                                  //滚转角控制积分           
extern float     landing_high;                                             //着陆起始高度

/***********遥控器控制参数***********/
extern short     tag_LongSwitch, tag_SaveEleSwitch, tag_SaveEngSwitch, tag_ViHoldON, tag_opt; 

/***********气压计参数***********/
extern float  	 height_ini,     //初始气压高度
	               height_adc;     //气压高度

/***********GPS信号参数***********/
extern float     lon_gps,   //经度        
								 lat_gps,   //纬度 
								 lon_ref,   //XYZ坐标系原点经度 
								 lat_ref,   //XYZ坐标系原点纬度 
								 psi_init,  //初始航向
								 psi_gps,   //GPS航迹角       
								 alt_gps,   //GPS高度      
								 Hdot_gps,  //升降速度
								 H_acc,     //GPS水平经度
								 V_acc,     //GPS垂直经度 
								 Vd_gps;    //GPS地速
      
extern unsigned char	used_gps;  //GPS收星数    
extern unsigned char tag_DGPS; 
            
/***********KaWx***********/
extern unsigned short        battery_volt;		

#ifdef __cplusplus
}
#endif    

#endif
