#include <math.h>
#include <stdio.h>
#include "Sim.h"
#include "yn.h"

#define dt 0.01

#define KP_H 0.8
#define KI_H 0.1
#define KD_H 0

#define KP_THETA 0.5
#define KI_THETA 0.05
#define KD_THETA 0

#define KP_PHI 1.6
#define KI_PHI 0
#define KD_PHI 0

#define KP_SPEED 1000
#define KI_SPEED 1500
#define KD_SPEED 0

#define THETA_LEVEL 2  // [deg]
#define ENG_LEVEL 55   // [%]

short step_long = 0;
double H_out = 0;

double Maxmin(double x, double max, double min) {
    if(x > max)
        x = max;
    if(x < min)
        x = min;
    return x;
}

double Psi_180(double psi) {
    while(psi > 180) psi -= 360;
    while(psi < -180) psi += 360;
    return psi;
}

void ctrl_alt(void) {
    static double H_e = 0, H_prev = 0;

    H_e = Maxmin(height_cmd - ac_height, 0.5, -0.5);  // 高度误差
    H_prev = ac_height;
    H_i += H_e * dt;
    H_i = Maxmin(H_i, 0.5, -0.5);
    H_out = KP_H * H_e + KI_H * H_i + KD_H * ac_dH;  // 高度控制
    // H_out = 0;
}

void ctrl_long(void) {  // 常规PID
    static double theta_e = 0, theta_prev = 0;
    double theta_d = 0, du = 0;

    theta_e = theta_cmd + H_out - ac_theta * Rad2Deg;
    theta_i += theta_e * dt;
    theta_i = Maxmin(theta_i, 1, -1);  // 积分限幅
    theta_d = (theta_e - theta_prev) / dt;

    ele_var = KP_THETA * theta_e + KI_THETA * theta_i + KD_THETA * ac_Q * Rad2Deg;

    theta_prev = theta_e;
}

/*飞行器速度控制*/
void ctrl_speed() {
    static double speed = 0, speed_e1 = 0, speed_e2 = 0;  // 当前、上一次、上上次误差
    static double du = 0;

    speed = Vt_cmd - ac_vt;

    du = KP_SPEED * (speed - speed_e1) + KI_SPEED * speed * dt +
        KD_SPEED * (speed - 2 * speed_e1 + speed_e2) / dt;

    engine_var += du;

    engine_var = Maxmin(engine_var, 0, 90);

    speed_e2 = speed_e1;
    speed_e1 = speed;
}

/*飞行器横侧向控制*/
void ctrl_late(void) {
    static double phi_i = 0, phi_e = 0, phi_prev = 0;
    double phi_d = 0, du = 0;

    psi_hmr = Psi_180(ac_psi * Rad2Deg);  // 真航向
    phi_e = gama_cmd - ac_phi * Rad2Deg;
    phi_i += phi_e * dt;
    phi_i = Maxmin(phi_i, 1, -1);  // 积分限幅
    phi_d = (phi_e - phi_prev) / dt;

    ail_var = KP_PHI * phi_e + KI_PHI * phi_i + KD_PHI * ac_P * Rad2Deg;

    phi_prev = phi_e;
}

/*矩形轨迹巡航*/
void ctrl_rectangular(void) {
    static double dpsi = 0, psi_cmd;

    switch(step_long) {
    case 0:
        psi_cmd = 0;
        if(ac_PN >= 100) step_long++;
        break;
    case 1:
        psi_cmd = 90;
        if(ac_PE >= 100) step_long++;
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
    // height_cmd = 50; // hold on alt_gps when switching to auto mode

    ctrl_cmdSmooth();
    ctrl_alt();
    ctrl_long();
    ctrl_late();
}

/*平飞巡航*/
void ctrl_level(void) {
    // if(t > 200) flag_Stop = 0;
    Vt_cmd = 20;  // 巡航速度
    gama_cmd = 0;
    theta_cmd = THETA_LEVEL;
    // height_cmd = 50; // hold on alt_gps when switching to auto mode

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
            engine_var = 5;
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
            engine_var = 0;
            step_long++;
        }
        break;
    case 2:
        height_cmd = -ac_PN * tan(path2 / Rad2Deg);
        Vt_cmd = 17;
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
    ele_total = ele_var;
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
    ail_total = ail_var;
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
    rud_total = rud_var;
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
    ele_var = -0.2559;     ail_var = 0;
    rud_var = 0;       engine_var = 36.23;
    engine_var = engine_var;
}
