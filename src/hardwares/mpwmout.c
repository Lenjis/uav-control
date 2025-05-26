#include "pwm.h"
#include "sys.h"
#include "yn.h"

#define PWMINHIGHOFFSET 4
#define PWMOUTHIGHOFFSET 4
#define MPWMOUTCH1 0
#define MPWMOUTCH2 1
#define MPWMOUTCH3 2
#define MPWMOUTCH4 3
#define MPWMOUTCH5 4
#define MPWMOUTCH6 5
#define MPWMOUTCH7 6
#define MPWMOUTCH8 7
#define MPWMOUTCH9    8
#define MPWMOUTCH10   9
#define MPWMOUTCH11   10
#define MPWMOUTCH12   11
unsigned long test[8];
static float unittime = 0;

AnalogStruc PWMin[16];
AnalogStruc PWMout[12];
mpwm_t MPWMIN[6];
mpwm_t MPWMOUT[12];

void LAW_Act(void) {
    PWMout[0].val = MidVal(ail_var, PWMout[0].min, PWMout[0].max);    //-2.5;     //ailerion
    PWMout[1].val = MidVal(ele_var, PWMout[1].min, PWMout[1].max);    // elevator
    PWMout[2].val = MidVal(engine_var, PWMout[2].min, PWMout[2].max); // engine
    PWMout[3].val = MidVal(rud_var, PWMout[3].min, PWMout[3].max);    // rudder

    PWMout[0].vol = PWMout[0].ratio * PWMout[0].val + 1.5 + PWMout[0].offset; // ailerion
    // PWMout[0].vol = -PWMout[0].ratio * PWMout[0].val + 1.5 + PWMout[0].offset; // ailerion
    // PWMout[1].vol = PWMout[1].ratio * PWMout[1].val + 1.5 + PWMout[1].offset; // elevator
    PWMout[1].vol = -PWMout[1].ratio * PWMout[1].val + 1.5 + PWMout[1].offset; // elevator
    PWMout[2].vol = PWMout[2].ratio * PWMout[2].val + 1.0 + PWMout[2].offset;  // engine
    PWMout[3].vol = PWMout[3].ratio * PWMout[3].val + 1.5 + PWMout[3].offset;  // rudder
}

void LAW_Task(void) // 40ms 25Hz
{
    if(on_safe) {
        MPWMOUT[0].pwmhi = MidVal(PWMin[0].vol, 1.00, 2.00); /*ailerion*/
        MPWMOUT[1].pwmhi = MidVal(PWMin[1].vol, 1.00, 2.00); /*elevator*/
        MPWMOUT[2].pwmhi = MidVal(PWMin[2].vol, 1.00, 2.00); /*engine*/
        MPWMOUT[3].pwmhi = MidVal(PWMin[3].vol, 1.00, 2.00); /*rudder*/
    }
    else {
        MPWMOUT[0].pwmhi = MidVal(PWMout[0].vol, 1.00, 2.00); /*ailerion*/
        MPWMOUT[1].pwmhi = MidVal(PWMout[1].vol, 1.00, 2.00); /*elevator*/
        MPWMOUT[2].pwmhi = MidVal(PWMout[2].vol, 1.00, 2.00); /*engine*/
        MPWMOUT[3].pwmhi = MidVal(PWMout[3].vol, 1.00, 2.00); /*rudder*/
    }
}

void LAW_Out(void) {
    unsigned char i = 0;
    LAW_Task();
    // test = (unsigned short)(MPWMOUT[i].pwmhi * 5 * 20000 /100);
    for(i = MPWMOUTCH1; i <= MPWMOUTCH8; i++)
        pwm_set(i, MPWMOUT[i].pwmhi * 5); // MpwmOut(i);
}

void LAW_ParmInit(void) {
    PWMout[0].min = -20.0f;
    PWMout[0].max = 20.0f; /*[ailerion]*/
    PWMout[1].min = -18.5f;
    PWMout[1].max = 18.5f; /*[elevator]*/
    PWMout[2].min = 0.0f;
    PWMout[2].max = 100.0f; /*[engine]*/
    PWMout[3].min = -22.0f;
    PWMout[3].max = 22.0f; /*[rudder]*/
    H_int_max = 5.0f;
    H_int_min = -5.0f;
    theta_int_max = 4.0f; /*俯仰角积分[-5~+5][deg]*/
    theta_int_min = -4.0f;
    PWMin[0].vol = PWMin[1].vol = PWMin[3].vol = PWMin[4].vol = PWMout[5].vol = 1.5;
    PWMin[2].vol = 2.0;
    PWMin[7].vol = 1.1;

    PWMout[0].ratio = 0.01613;
    PWMout[1].ratio = 0.0148;
    PWMout[2].ratio = 0.0082;
    PWMout[3].ratio = -0.0129;
    PWMout[0].offset = 0;
    PWMout[1].offset = 0;
    PWMout[2].offset = 0.11;
    PWMout[3].offset = 0;
}