#ifndef _PWM_H_
#define _PWM_H_

#ifdef __cplusplus
extern "C" {
#endif
#define PWM_PERIOD_MS(x)	1000*x
#define PWM_CH0_3_Init	TIM4_init
#define PWM_CH4_7_Init	TIM4_init
#define PWM_CH8_11_Init	TIM4_init
#define PWM_CH12_13_Init	TIM4_init
    void pwm_init(void);
    //PWM CH0~3 init
    void TIM4_init(unsigned long arr);
    //PWM CH4~7 init
    void TIM5_init(unsigned long arr);
    //PWM CH8~11 init
    void TIM1_init(unsigned long arr);
    //PWM CH12~13 init
    void TIM12_init(unsigned long arr);
    void pwm_set(unsigned char ch, float duty);

    typedef struct {
        float    min;
        float    max;
        float    ratio;           /*[比例因子][*100]*/
        float    offset;          /*[偏移量]  [*100]*/
        double   vol;             /*[电压][V] [BP=10]*/
        float    val;             /*[物理量]  [BP=7]*/
    } AnalogStruc;

    void LAW_ParmInit();
    void LAW_Out();
    void LAW_Act(void);

    extern unsigned long test[8];
#ifdef __cplusplus
}
#endif
#endif