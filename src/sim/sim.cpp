#include <math.h>
#include <stdio.h>
#include "Sim.h"
#include "yn.h"

// void		(*aircraft)();
double T, state_x[13], u[4], t = 0, alphi_xtrim;
int sim_step, sim_status;
short step_long = 0, psi_count = 0;
short flag_Stop = 1;
double H_out = 0, H_int1 = 0;

#define freq 200
#define dt 1 / freq
#define Re 6378137.0

#define DT 0.01

#define KP_H 1.2
#define KI_H 0.1
#define KD_H 0

#define KP_THETA 0.7
#define KI_THETA 0.3
#define KD_THETA 0.1

#define KP_PHI 3
#define KI_PHI 0.5
#define KD_PHI 0.1

#define KP_SPEED 1000
#define KI_SPEED 1500
#define KD_SPEED 0

#define THETA_LEVEL 0.993401430622199  // [deg]
#define ENG_LEVEL 46.262948302356023   // [%]

double Maxmin(double x, double max, double min) {
    if(x > max)
        x = max;
    if(x < min)
        x = min;
    return x;
}

double Psi_180(double psi) {
    int n = psi / 360;
    psi -= 360 * n;
    if(psi > 180)
        psi -= 360;
    if(psi < -180) {
        psi += 360;
    }
    return psi;
}

void ctrl_alt(void) {
    static double H_i = 0, H_e = 0, H_prev = 0;

    H_e = height_cmd - ac_height;  // 高度误差

    H_prev = ac_height;

    if(H_e > 20)
        H_i += 20 * DT;
    else if(H_e < -20)
        H_i += -20 * DT;
    else
        H_i += H_e * DT;  // 积分项, dt=0.01s

    // if (H_i > 20)
    //     H_i = 20;
    // else if (H_i < -20)
    //     H_i = -20;

    H_out = KP_H * H_e + KI_H * H_i + KD_H * ac_dH;  // 高度控制
}

void ctrl_long(void) {  // incremental PID
    static double theta_e = 0, theta_e1 = 0, theta_e2 = 0;  // 当前、上一次、上上次误差
    static double du = 0;

    theta_e = theta_cmd + H_out - ac_theta * Rad2Deg;

    du = KP_THETA * (theta_e - theta_e1) + KI_THETA * theta_e * DT +
        KD_THETA * (theta_e - 2 * theta_e1 + theta_e2) / DT;

    ac_ele -= du;  // 舵量输入相反

    theta_e2 = theta_e1;
    theta_e1 = theta_e;
}

/*飞行器速度控制*/
void ctrl_speed() {
    static double speed = 0, speed_e1 = 0,
        speed_e2 = 0;  // 当前、上一次、上上次误差
    static double du = 0;

    speed = Vt_cmd - ac_vt;

    du = KP_SPEED * (speed - speed_e1) + KI_SPEED * speed * DT +
        KD_SPEED * (speed - 2 * speed_e1 + speed_e2) / DT;

    ac_eng += du;

    // ac_eng = Maxmin(ac_eng, 0, 100);

    speed_e2 = speed_e1;
    speed_e1 = speed;
}

/*飞行器横侧向控制*/
void ctrl_late(void) {
    const double Kp_phi = KP_PHI, Ki_phi = KI_PHI, Kd_phi = KD_PHI;
    static double phi_e = 0, phi_e1 = 0, phi_e2 = 0;  // 当前、上一次、上上次误差
    static double du = 0;

    phi_e = gama_cmd - ac_phi * Rad2Deg;

    du = Kp_phi * (phi_e - phi_e1) + Ki_phi * phi_e * DT +
        Kd_phi * (phi_e - 2 * phi_e1 + phi_e2) / DT;

    ac_ail -= du;  // 舵量输入相反
    ac_ail = Maxmin(ac_ail, 30, -30);
    ac_rud = Maxmin(ac_rud, 30, -30);

    phi_e2 = phi_e1;
    phi_e1 = phi_e;
}

/*矩形轨迹巡航*/
void ctrl_rectangular(void) {
    static double dpsi = 0, psi_cmd;

    switch(step_long) {
    case 0:
        psi_cmd = 0;
        if(ac_PN >= 10) step_long++;
        break;
    case 1:
        psi_cmd = 90;
        if(ac_PE >= 10) step_long++;
        break;
    case 2:
        psi_cmd = 180;
        if(ac_PN <= 0) step_long++;
        break;
    case 3:
        psi_cmd = 270;
        if(ac_PE <= 0) step_long = 0;
        break;
    default:
        break;
    }

    // if(t > 200) flag_Stop = 0;
    Vt_cmd = 20;  // 巡航速度
    dpsi = psi_cmd - ac_psi * Rad2Deg;
    while(dpsi > 180) dpsi = dpsi - 360;
    while(dpsi < -180) dpsi = dpsi + 360;
    gama_cmd = 1.0 * dpsi;
    if(gama_cmd > 45) gama_cmd = 45;
    if(gama_cmd < -45) gama_cmd = -45;
    theta_cmd = THETA_LEVEL;
    // height_cmd = 50;

    ctrl_cmdSmooth();
    ctrl_alt();
    ctrl_long();
    ctrl_late();
}

void ctrl_approach(void) {
    const double H2 = 2, H1 = 50, path2 = 0.8, path1 = 3.5;
    static double L1, L2;
    L2 = H2 / tan(path2 / Rad2Deg);              // 拉飘开始距离
    L1 = L2 + (H1 - H2) / tan(path1 / Rad2Deg);  // 陡下滑开始距离
    static double Vt_slope, Vt_0;

    switch(step_long) {
    case 0:
        theta_cmd = THETA_LEVEL;
        height_cmd = H1;
        Vt_cmd = 30;
        if(ac_PN > -L1) {
            theta_cmd = -1;
            ac_eng = 5;
            Vt_slope = (ac_vt - 18) / (H1 - H2);
            Vt_0 = ac_vt;
            step_long++;
        }
        break;
    case 1:
        height_cmd = H2 + (-ac_PN - L2) * tan(path1 / Rad2Deg);
        Vt_cmd = Vt_0 - Vt_slope * (H1 - ac_height);
        if(ac_PN > -L2) {
            theta_cmd = 4;
            ac_eng = 0;
            step_long++;
        }
        break;
    case 2:
        height_cmd = -ac_PN * tan(path2 / Rad2Deg);
        Vt_cmd = 17;
        if(ac_height <= 0 || t > 50) flag_Stop = 0;
        break;
    default:
        break;
    }
    ctrl_cmdSmooth();
    ctrl_alt();
    ctrl_long();
    ctrl_speed();
}

/*飞行器控制指令软化*/
void ctrl_cmdSmooth(void) {
    static const double theta_unit = 0.1, phi_unit = 1.0,
        ele_unit = 0.1, ail_unit = 0.1, rud_unit = 0.1, eng_unit = 0.1,
        vt_unit = 0.1, H_unit = 0.1;
    double theta_total, phi_total, psi_total,
        ele_total, ail_total, rud_total, eng_total,
        vt_total, height_total;

    /*---[smoothing theta_cmd]---*/
    theta_total = theta_cmd;
    if(theta_var < theta_total) {
        theta_var += theta_unit;
        if(theta_var > theta_total)
            theta_var = theta_total;
    }
    else {
        theta_var -= theta_unit;
        if(theta_var < theta_total)
            theta_var = theta_total;
    }

    /*---[smoothing phi_cmd]---*/
    phi_total = gama_cmd;
    if(gama_var < phi_total) {
        gama_var += phi_unit;
        if(gama_var > phi_total)
            gama_var = phi_total;
    }
    else {
        gama_var -= phi_unit;
        if(gama_var < phi_total)
            gama_var = phi_total;
    }
    /*---[smoothing ele_cmd]---*/
    ele_total = ac_ele;
    if(ele_var < ele_total) {
        ele_var += ele_unit;
        if(ele_var > ele_total)
            ele_var = ele_total;
    }
    else {
        ele_var -= ele_unit;
        if(ele_var < ele_total)
            ele_var = ele_total;
    }

    /*---[smoothing ail_cmd]---*/
    ail_total = ac_ail;
    if(ail_var < ail_total) {
        ail_var += ail_unit;
        if(ail_var > ail_total)
            ail_var = ail_total;
    }
    else {
        ail_var -= ail_unit;
        if(ail_var < ail_total)
            ail_var = ail_total;
    }

    /*---[smoothing ele_cmd]---*/
    rud_total = ac_rud;
    if(rud_var < rud_total) {
        rud_var += rud_unit;
        if(rud_var > rud_total)
            rud_var = rud_total;
    }
    else {
        rud_var -= rud_unit;
        if(rud_var < rud_total)
            rud_var = rud_total;
    }

    /*---[smoothing eng_cmd]---*/
    eng_total = engine_cmd;
    if(engine_var < eng_total) {
        engine_var += eng_unit;
        if(engine_var > eng_total)
            engine_var = eng_total;
    }
    else {
        engine_var -= eng_unit;
        if(engine_var < eng_total)
            engine_var = eng_total;
    }

    /*---[smoothing vt_cmd]---*/
    vt_total = Vt_cmd;
    if(Vt_var < vt_total) {
        Vt_var += vt_unit;
        if(Vt_var > vt_total)
            Vt_var = vt_total;
    }
    else {
        Vt_var -= vt_unit;
        if(Vt_var < vt_total)
            Vt_var = vt_total;
    }

    /*---[smoothing height_cmd]---*/
    height_total = height_cmd;
    if(height_var < height_total) {
        height_var += H_unit;
        if(height_var > height_total)
            height_var = height_total;
    }
    else {
        height_var -= H_unit;
        if(height_var < height_total)
            height_var = height_total;
    }
}

/*飞行器模型解算模块，无需看懂*/
void simu_run(void) {
    static double g = 9.81;
    static double Hn_x, Hn_y, Hn_z, matrix[3][3];
    static double stheta, ctheta, sphi, cphi, spsi, cpsi;

    /*dx[0] = dVt;       dx[3] = dPN;    dx[6] = dP;    dx[9] = dphi;
    dx[1] = dalpha;    dx[4] = dPE;    dx[7] = dQ;    dx[10] = dtheta;
    dx[2] = dbeta;     dx[5] = dH;     dx[8] = dR;    dx[11] = dpsi;*/

    state_x[0] = ac_vt;    state_x[3] = ac_P;   state_x[6] = ac_theta; state_x[9] = ac_PN;
    state_x[1] = ac_alpha; state_x[4] = ac_Q;   state_x[7] = ac_phi;  state_x[10] = ac_PE;
    state_x[2] = ac_beta;  state_x[5] = ac_R;   state_x[8] = ac_psi;   state_x[11] = ac_height;
    state_x[12] = ac_mass;

    u[0] = ac_ele;   u[1] = ac_ail;  u[2] = ac_rud;  u[3] = ac_eng;

    //rk4(aircraft, t, state_x, u, 13, 2*T, state_x, &t);

    ac_vt = state_x[0];    ac_P = state_x[3];  ac_theta = state_x[6]; ac_PN = state_x[9];
    ac_alpha = state_x[1]; ac_Q = state_x[4];  ac_phi = state_x[7];  ac_PE = state_x[10];
    ac_beta = state_x[2];  ac_R = state_x[5];  ac_psi = state_x[8];   ac_height = state_x[11];  ac_mass = state_x[12];
    //gps_truecourse = Psi_360(ac_psi * Rad2Deg);
    //	ac_ax= -g*sin(ac_theta);
    //	ac_ay=  g*cos(ac_theta)*sin(ac_phi);
    //	ac_az=  g*cos(ac_theta)*cos(ac_phi);
}

/*飞行器模型解算初始化，无需看懂*/
void simu_init(void) {
    ac_vt = 25;         ac_alpha = 2.3240 / Rad2Deg;   ac_beta = 0 / Rad2Deg;
    ac_P = 0 / Rad2Deg;      ac_R = 0 / Rad2Deg;      ac_Q = 0 / Rad2Deg;
    ac_theta = 2.3240 / Rad2Deg;    ac_phi = 0.0 / Rad2Deg;  ac_psi = 0.0 / Rad2Deg;
    ac_height = 55.0;  ac_PE = 0.0 / Rad2Deg;
    ac_PN = 0;
    ac_mass = 17;
    Vt_var = ac_vt;
    height_var = ac_height;
    sim_step = 5;   //5ms
    sim_status = 0;
    T = sim_step / 1000.0f; 	t = 0;
    //aircraft= model6dof;
    alphi_xtrim = 2.32 / Rad2Deg;
    ac_ele = -0.2559;     ac_ail = 0;
    ac_rud = 0;       ac_eng = 36.23;
    engine_var = ac_eng;
}
