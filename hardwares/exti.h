#ifndef _EXTI_H_
#define _EXTI_H_

#ifdef __cplusplus
	extern "C"{
#endif
		
		void exti_init(void);
		
		void exti_enable(void);
		
		void exti_disable(void);
		
#ifdef __cplusplus
	}
#endif

#endif