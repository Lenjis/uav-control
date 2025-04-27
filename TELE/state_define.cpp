#include "yn.h"

/*********状态变量定义 simu state define*********/     
float     ac_theta, ac_phi, ac_psi;   //attitude 姿态角
float     ac_alpha, ac_beta;          //alpha, beta 迎角，侧滑角
float     ac_P,ac_Q,ac_R;             //PQR 三轴角速率
float     ac_ax, ac_ay, ac_az;        //accelx 机体过载
float     ac_PE, ac_PN, ac_height;    //XYZ 坐标系位置
float     ac_vt;   //空速
float     ac_dH;   //下沉率
float     ac_dZ;   //侧偏距
float     ac_dL;   //待飞距离
float     ac_dpsi; //偏航角
float     ac_dot;  //航段号
float     psi_hmr; //真航向
float     ac_mass = 17;

/*********控制变量定义 sim control para define*********/
float     theta_cmd, gama_cmd, engine_cmd,  height_cmd,  Vt_cmd, Hdot_cmd;  //指令变量
float     theta_var, gama_var, engine_var,  height_var,  Vt_var;             //指令变量（软化后？）
float     ele_var, ail_var, rud_var;                                //舵面指令
float   	ail_trim, ele_trim, rud_trim, eng_trim;                   //初始平飞配平值
float     psi_cmd;                                                  //给定航向

float     theta_int, theta_int_min, theta_int_max;                  //俯仰角控制积分，上下限幅
float     gama_int, gama_int_min, gama_int_max;                     //滚转角控制积分，上下限幅
float     H_int , H_int_min, H_int_max;                             //高度控制积分，上下限幅
float     KpV, IpV, engine_V=0,theta_V=0;                           //空速控制增益，油门积分，俯仰角积分
float     Igama=0;                                                  //滚转角控制积分           
float     landing_high;                                             //着陆起始高度

/***********遥控器控制参数***********/
short     tag_LongSwitch=0, tag_SaveEleSwitch=0, tag_SaveEngSwitch=0, tag_ViHoldON=0, tag_opt=0; 

/***********气压计参数***********/
float  	  height_ini=0,   //初始气压高度
	        height_adc;     //气压高度

/***********GPS信号参数***********/
float       lon_gps,   //经度        
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
      
unsigned char	used_gps;  //GPS收星数    
unsigned char tag_DGPS; 
            
/***********KaWx***********/
unsigned short        battery_volt;		
		 
		
		
		