#ifndef _SIM_H_
#define _SIM_H_

#ifdef __cplusplus
	extern "C"{
#endif
		
typedef unsigned char	  BYTE;
typedef unsigned short	WORD;
typedef unsigned long	  DWORD;
typedef unsigned int	  UINT;

#define	Rad2Deg 	57.2957795130823208767981548141052

void  ctrl_flytask(void);
void  simu_init(void);
 
//---[7]---飞机模型仿真参数定义
//
//extern double H_int, Ivt, DdH, Iphi, Omega, deltaH, initposPN, initposPE, Pos_forward;
//extern double theta_cmd, theta_var, phi_cmd, phi_var, H_cmd, H_var, 
//PE_cmd, PE_var, psi_cmd, psi_var, vt_cmd, vt_var,
//eng_cmd, eng_var, theta_pid, ele_cmd, ele_var;


#ifdef __cplusplus
}
#endif   
#endif