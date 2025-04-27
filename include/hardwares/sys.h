#ifndef _SYS_H_
#define _SYS_H_

		
//#define _SYSTEM_SUPPORT_OS_

#define HSE_CLK			16000000	// hse clock = 16MHz

#define HSI_CLK			64000000	// hsi clock = 64MHz
	 
#define PLL1_M_DIV	4
#define PLL1_N_DIV	200
#define PLL1_P_DIV	2
#define PLL1_Q_DIV	4
#define PLL1_R_DIV	4
	 
#define PLL2_M_DIV	16
#define PLL2_N_DIV	440
#define PLL2_P_DIV	2
#define PLL2_Q_DIV	4
#define PLL2_R_DIV	4


#define PLL1_CLK		(HSE_CLK/PLL1_M_DIV*PLL1_N_DIV)		// pll1 clock = 800MHz
#define PLL1_P_CLK	(PLL1_CLK/PLL1_P_DIV)							// pll1 p clock = 400MHz
#define PLL1_Q_CLK	(PLL1_CLK/PLL1_Q_DIV) 						// pll1 q clock = 200MHz
#define Pll1_R_CLK	(PLL1_CLK/PLL1_R_DIV)							// pll1 r clock = 400MHz

#define PLL2_CLK		(HSE_CLK/PLL2_M_DIV*PLL1_N_DIV)		// pll2 clock = 440MHz
#define PLL2_P_CLK	(PLL2_CLK/PLL2_P_DIV) 						// pll2 p clock = 220MHz
#define PLL2_Q_CLK	(PLL2_CLK/PLL2_Q_DIV) 						// pll2 q clock = 110MHz
#define PLL2_R_CLK	(PLL2_CLK/PLL2_R_DIV)							// pll2 r clock = 110MHz

#define SYS_CLK			PLL1_P_CLK												// sys clock = 400MHz

#define D1CPRE_CLK	SYS_CLK														// D1cpre clock = 400MHz
#define HPRE_CLK		(D1CPRE_CLK/2)										// rcc ahb3 clock = 200MHz

#define APB1_CLK		(HPRE_CLK/2)											// rcc apb1 clock = 100MHz
#define APB2_CLK		(HPRE_CLK/2)											// rcc apb2 clock = 100MHz
#define APB3_CLK		(HPRE_CLK/2)											// rcc apb3 clock = 100MHz
#define APB4_CLK		(HPRE_CLK/2)											// rcc apb4 clock = 100MHz



#ifdef __cplusplus
 extern "C" {
#endif
	 
		#include "stm32h743xx.h"
		#include <stdint.h>
		#include <stdio.h>
		#include <string.h>
	 
		extern unsigned long timetick;
		extern unsigned char Timer1ms;
		extern unsigned char Timer2ms;
		extern unsigned char Timer5ms;
		extern unsigned char Timer10ms;
		extern unsigned char Timer20ms;
		extern unsigned char Timer25ms;
		extern unsigned char Timer50ms;
		extern unsigned char Timer100ms;
		extern unsigned char Timer200ms;
		extern unsigned char Timer250ms;
		extern unsigned char Timer500ms;
		extern unsigned char Timer1s;
	 
		void clock_init(void);
	 
		void NVIC_SetVectorTable(unsigned long NVIC_VectTab,unsigned long Offset);
	 
		void NVIC_PriorityGroupConfig(unsigned char NVIC_Group);
	 
		void NVIC_Init(unsigned char NVIC_PreemptionPriority,unsigned char NVIC_SubPriority,unsigned char NVIC_Channel,unsigned char NVIC_Group);
	 
		void Enable_Irq(void);
		void Disable_Irq(void);
	 
		void delay_init(void);
		void delay_us(unsigned short nus);
		void delay_ms(unsigned short nms);
	 
#ifdef __cplusplus
 }
#endif

#endif 
