#ifndef _GPIO_H_
#define _GPIO_H_

#ifdef __cplusplus
 extern "C"{
#endif
	 
	 #include "sys.h"
	 
	 #define GPIO_MODE_INPUT		0
	 #define GPIO_MODE_OUTPUT		1
	 #define GPIO_MODE_AF				2
	 #define GPIO_MODE_ANALOG		3
	 
	 #define GPIO_OTYPE_PP			0
	 #define GPIO_OTYPE_OD			1
	 
	 #define GPIO_LOWSPEED			0
	 #define GPIO_MIDSPEED			1
	 #define GPIO_HIGHSPEED			2
	 #define GPIO_VERYHIGHSPEED 3
	 
	 #define GPIO_NOPULL				0
	 #define GPIO_PULLUP				1
	 #define GPIO_PULLDOWN			2
	 
	 #define GPIO_AF(x)							x
	 
	 void gpio_init(void);
	 
	 unsigned char set_pin(GPIO_TypeDef *_gpio, unsigned char pin, unsigned char level);
	 
	 unsigned char get_pin(GPIO_TypeDef *_gpio, unsigned char pin);

#ifdef __cplusplus
 }
#endif
	 
#endif
