#include <math.h>
#include <stdio.h>
#include "Sim.h"
#include "yn.h"

//void		(*aircraft)();
double	T, state_x[13], u[4], t=0, alphi_xtrim;
int			sim_step,sim_status;
short   step_long=0, psi_count = 0;
short flag_Stop=1;
#define freq 200
#define dt 1/freq
#define Re 6378137.0

double Maxmin(double x, double max, double min)
{
	if (x > max)
		x = max;
	if (x < min)
		x = min;
	return x;
}

double Psi_180(double psi)
{
	int n = psi / 360;
	psi -= 360 * n;
	if (psi > 180)
		psi -= 360;
	if (psi < -180)
	{
		psi += 360;
	}
	return psi;
}

/*飞行器纵向控制*/
void  ctrl_long(void)
{
	static  double   KthataPH = 1.6, KthetaIH = 0.50, KthetaDH = -0.2; 
  static  double   KeTHETA  = 1.2, KeQ      = 0.24, KeALPHA  =  0.35; //ele control
	static  double   Kev      = 0.1, KeIv     = 0.01; //eng control
	
	double  deltaH,  theta_pid;
	/*ac_eng = eng_var;
	ac_ele = ele_var;*/
	/********高度控制********/
	deltaH = Maxmin((height_cmd - ac_height), 0.5, -0.5);
	H_int += KthetaIH * deltaH * (T * 2);
	H_int = Maxmin(H_int, 0.5, -0.5);
	theta_pid = KthataPH * (height_cmd - ac_height)
           	+ H_int + KthetaDH * ac_dH;;
	/********俯仰角控制********/
	ac_ele = KeTHETA * (ac_theta * Rad2Deg - theta_var - 0.0);// + KeQ * ac_Q * Rad2Deg
		//+ KeALPHA * (ac_alpha - alphi_xtrim) * Rad2Deg;
	ac_ele = Maxmin(ac_ele, 30, -30);
	/********速度控制********/
	engine_V += KeIv * (Vt_cmd - ac_vt) * (T * 2);
	engine_V = Maxmin(engine_V, 50, -50);
//	ac_eng = engine_var +Kev * (Vt_cmd - ac_vt) + engine_V;
//	ac_eng = Maxmin(ac_eng, 100, 0);
}
/*飞行器横侧向控制*/
void  ctrl_late (void)
{
	static double    K_phipsi  =  2.5, KphiZ = 0.00;
	static double    Kaphi = 1.40,  KaP       =  0.6,   KaIphi = 0.1;
	static double    KrudR = 0.28,  K_rudbeta = -2.05;

	gama_cmd = K_phipsi * Psi_180(psi_cmd - ac_psi);
	gama_cmd = Maxmin(gama_cmd, 30.0, -30.0);

	Igama += KaIphi * (ac_phi * Rad2Deg - gama_var) * 2 * T;
	Igama = Maxmin(Igama, 0.61, -0.61);
	ac_ail = Kaphi * (ac_phi * Rad2Deg - gama_var);// + KaP * ac_P * Rad2Deg +Igama;
	ac_ail = Maxmin(ac_ail, 30, -30);
	ac_rud = KrudR * ac_R + K_rudbeta * ac_beta;
	ac_rud = Maxmin(ac_rud, 30, -30);
}
/*飞行器控制指令软化*/
void ctrl_cmdSmooth(void)
{
	static const double theta_unit = 0.1,
						phi_unit = 1.0,
						ele_unit = 0.1,
						ail_unit = 0.1,
						rud_unit = 0.1,
						eng_unit = 0.1,
						vt_unit = 0.1,
						H_unit = 0.1;
	double theta_total,
		phi_total,
		psi_total,
		ele_total,
		ail_total,
		rud_total,
		eng_total,
		vt_total,
		H_total;

	/*---[smoothing theta_cmd]---*/
	theta_total = theta_cmd;
	if (theta_var < theta_total)
	{
		theta_var += theta_unit;
		if (theta_var > theta_total)
			theta_var = theta_total;
	}
	else
	{
		theta_var -= theta_unit;
		if (theta_var < theta_total)
			theta_var = theta_total;
	}

	/*---[smoothing phi_cmd]---*/
	phi_total = gama_cmd;
	if (gama_var < phi_total)
	{
		gama_var += phi_unit;
		if (gama_var > phi_total)
			gama_var = phi_total;
	}
	else
	{
		gama_var -= phi_unit;
		if (gama_var < phi_total)
			gama_var = phi_total;
	}
	/*---[smoothing ele_cmd]---*/
	ele_total = ac_ele;
	if (ele_var < ele_total)
	{
		ele_var += ele_unit;
		if (ele_var > ele_total)
			ele_var = ele_total;
	}
	else
	{
		ele_var -= ele_unit;
		if (ele_var < ele_total)
			ele_var = ele_total;
	}

	/*---[smoothing ail_cmd]---*/
	ail_total = ac_ail;
	if (ail_var < ail_total)
	{
		ail_var += ail_unit;
		if (ail_var > ail_total)
			ail_var = ail_total;
	}
	else
	{
		ail_var -= ail_unit;
		if (ail_var < ail_total)
			ail_var = ail_total;
	}

	/*---[smoothing ele_cmd]---*/
	rud_total = ac_rud;
	if (rud_var < rud_total)
	{
		rud_var += rud_unit;
		if (rud_var > rud_total)
			rud_var = rud_total;
	}
	else
	{
		rud_var -= rud_unit;
		if (rud_var < rud_total)
			rud_var = rud_total;
	}

	/*---[smoothing eng_cmd]---*/
	eng_total = engine_cmd;
	if (engine_var < eng_total)
	{
		engine_var += eng_unit;
		if (engine_var > eng_total)
			engine_var = eng_total;
	}
	else
	{
		engine_var -= eng_unit;
		if (engine_var < eng_total)
			engine_var = eng_total;
	}

	/*---[smoothing vt_cmd]---*/
	vt_total = Vt_cmd;
	if (Vt_var < vt_total)
	{
		Vt_var += vt_unit;
		if (Vt_var > vt_total)
			Vt_var = vt_total;
	}
	else
	{
		Vt_var -= vt_unit;
		if (Vt_var < vt_total)
			Vt_var = vt_total;
	}

	/*---[smoothing H_cmd]---*/
	H_total = height_cmd;
	if (height_var < H_total)
	{
		height_var += H_unit;
		if (height_var > H_total)
			height_var = H_total;
	}
	else
	{
		height_var -= H_unit;
		if (height_var < H_total)
			height_var = H_total;
	}
}
/*飞行器着陆控制模块*/
void ctrl_landingtask(void)
{
	switch (step_long)
	{
	case 0:
		theta_cmd = 2.32;
		engine_cmd = 36;
		Vt_cmd = 25;
		height_cmd = 55; // 平飞进场
		if (ac_PN >= -1139.6)
			step_long++;
		break;
	case 1:
		theta_cmd = -1.2;
		engine_cmd = 11;
		Vt_cmd = 22;
		height_cmd = (-11 / tan(1.5 / Rad2Deg) - ac_PN) * tan(3.5 / Rad2Deg) + 11; // 陡下滑:height > 11m
		if (ac_height <= 11)
			step_long++;
		break;
	case 2:
		theta_cmd = 4.0;
		engine_cmd = 0.0;
		Vt_cmd = 16;
		height_cmd = -ac_PN * tan(1.5 / Rad2Deg); // 拉飘:height < 11m
		if (ac_height <= 0)
			step_long++;
		break;
	case 3:
		flag_Stop = 0; // 触地
		break;
	default:
		break;
	}

	ctrl_cmdSmooth();
	ctrl_long();
	ctrl_late();
}

/*飞行器四边航路控制模块*/
void ctrl_flytask(void)
{
	switch (step_long)
	{
	case 0:
		theta_cmd = 2.32;
		psi_cmd = Psi_360(psi_count * 90);
		engine_cmd = 36.23;
		Vt_cmd = 25;
		height_cmd = 55; // 北向航路
		// PE_cmd = 0;
		if (ac_PN >= 1200)
		{
			psi_count++;
			step_long++;
		}
		break;
	case 1:
		theta_cmd = 2.32;
		psi_cmd = Psi_360(psi_count * 90);
		engine_cmd = 36.23;
		Vt_cmd = 25;
		height_cmd = 55; // 东向航路
		// PE_cmd = ac_PE;
		if (ac_PE >= 430)
		{
			psi_count++;
			step_long++;
		}
		break;
	case 2:
		theta_cmd = 2.32;
		psi_cmd = Psi_360(psi_count * 90);
		engine_cmd = 36.23;
		Vt_cmd = 25;
		height_cmd = 55; // 南向航路
		// PE_cmd = 1569;
		if (ac_PN <= -1300)
		{
			psi_count++;
			step_long++;
		}
		break;
	case 3:
		theta_cmd = 2.32;
		psi_cmd = Psi_360(psi_count * 90);
		engine_cmd = 36.23;
		Vt_cmd = 25;
		height_cmd = 55; // 西向航路
		// PE_cmd = ac_PE;
		if (ac_PE <= 330.0)
		{
			psi_count++;
			step_long = 0;
		}
		break;
	default:
		break;
	}

	ctrl_cmdSmooth();
	ctrl_long();
	ctrl_late();
}

/*飞行器模型解算模块，无需看懂*/
void simu_run(void)
{
	static double g=9.81;
	static double Hn_x,Hn_y,Hn_z, matrix[3][3];
	static double stheta,ctheta, sphi,cphi, spsi,cpsi;	

	/*dx[0] = dVt;       dx[3] = dPN;    dx[6] = dP;    dx[9] = dphi;
	dx[1] = dalpha;    dx[4] = dPE;    dx[7] = dQ;    dx[10] = dtheta;
	dx[2] = dbeta;     dx[5] = dH;     dx[8] = dR;    dx[11] = dpsi;*/

	state_x[0]=ac_vt;    state_x[3]= ac_P;   state_x[6]= ac_theta; state_x[9] = ac_PN;
	state_x[1]=ac_alpha; state_x[4]= ac_Q;   state_x[7]= ac_phi;  state_x[10]= ac_PE;
	state_x[2]=ac_beta;  state_x[5]= ac_R;   state_x[8]= ac_psi;   state_x[11]= ac_height;
	state_x[12]=ac_mass;

	u[0] = ac_ele;   u[1] = ac_ail;  u[2] = ac_rud;  u[3] = ac_eng;

	//rk4(aircraft, t, state_x, u, 13, 2*T, state_x, &t);

	ac_vt=state_x[0];    ac_P =state_x[3];  ac_theta =state_x[6]; ac_PN =state_x[9];
	ac_alpha=state_x[1]; ac_Q =state_x[4];  ac_phi =state_x[7];  ac_PE =state_x[10];
	ac_beta=state_x[2];  ac_R =state_x[5];  ac_psi =state_x[8];   ac_height =state_x[11];  ac_mass=state_x[12];
	//gps_truecourse = Psi_360(ac_psi * Rad2Deg);
//	ac_ax= -g*sin(ac_theta);
//	ac_ay=  g*cos(ac_theta)*sin(ac_phi);
//	ac_az=  g*cos(ac_theta)*cos(ac_phi);
}
/*飞行器模型解算初始化，无需看懂*/
void simu_init(void)
{
	ac_vt=25;         ac_alpha=2.3240/Rad2Deg;   ac_beta=0/Rad2Deg;
	ac_P =0/Rad2Deg;      ac_R=0/Rad2Deg;      ac_Q=0/Rad2Deg;
	ac_theta= 2.3240 /Rad2Deg;    ac_phi=0.0/Rad2Deg;  ac_psi=0.0/Rad2Deg;
	ac_height = 55.0;  ac_PE =0.0/Rad2Deg;   
	ac_PN = -1300;
	ac_mass=17;
	Vt_var = ac_vt;
	height_var = ac_height;
	sim_step=5;   //5ms
	sim_status =0;
	T = sim_step / 1000.0f; 	t = 0;
	//aircraft= model6dof;
	alphi_xtrim = 2.32 / Rad2Deg;
	ac_ele=-0.2559;     ac_ail= -1.58906625570490e-14;
	ac_rud = 1.93814906058248e-14;       ac_eng = 36.23;
	engine_var = ac_eng;
}

