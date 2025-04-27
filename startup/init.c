#include "init.h"
#include "Sbus.h"
#include "sys.h"
#include "timer.h"
#include "gpio.h"
#include "fdcan.h"
#include "spi.h"
#include "exti.h"
#include "usart.h"
#include "pwm.h"
//#include "get_gps.h"
unsigned short  WD_cnt;
void init(void)
{
	clock_init();
	delay_ms(200);
	
	gpio_init();
	
	timer_init();
	
	pwm_init();
	
	PWM_CH0_3_Init(PWM_PERIOD_MS(20));
	PWM_CH4_7_Init(PWM_PERIOD_MS(20));
	PWM_CH8_11_Init(PWM_PERIOD_MS(20));
	PWM_CH12_13_Init(PWM_PERIOD_MS(20));
	
	//COM1
	usart1_dma_init(460800,USART_PARITY_NONE,USART_ONESTOP);
	//COM2
	usart2_dma_init(460800,USART_PARITY_NONE,USART_ONESTOP);
	//COM4
	uart4_init(460800,USART_PARITY_NONE,USART_ONESTOP);
	//COM6
	usart6_dma_init(230400,USART_PARITY_NONE,USART_ONESTOP);
	//COM7
	uart7_init(115200,USART_PARITY_NONE,USART_ONESTOP);
	//COM8
	uart8_dma_init(100000,USART_PARITY_EVEN,USART_ONESTOP);
	
	//FDCAN1
	fdcan1_init(FDCAN_500KHz);
	//FDCAN2
	fdcan2_init(FDCAN_500KHz);
	
	spi_init();
	LAW_ParmInit();
	
  //FM25Vx_init();
	RM3100_init();
	ADIS_Init();
	MS5611_init();
	
	exti_init();
	
	exti_enable();
	WD_cnt = 0;
}

