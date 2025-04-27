#ifndef _FM25VX_H_
#define _FM25VX_H_

#ifdef __cplusplus
	extern "C"{
#endif

		#include "sys.h"
		
		void FM25Vx_Get(unsigned char *buf);
		
		void FM25Vx_init(void);
		
		void FM25Vx_write(unsigned short address, unsigned char *buf, int len);
		
		void FM25Vx_read(unsigned short address, unsigned char *buf, int len);
		
#ifdef __cplusplus
	}
#endif

#endif