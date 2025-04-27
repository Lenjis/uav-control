#ifndef _FDCAN_H_
#define _FDCAN_H_
#if defined ( __CC_ARM   )
#pragma anon_unions
#endif
#ifdef __cplusplus
	extern "C"{
#endif
		
		#define FDCAN_1MKHz		5
		#define FDCAN_500KHz	10
		#define FDCAN_250KHz	20
		#define FDCAN_125KHz	40
		#define FDCAN_100KHz	50
		
		
		void fdcan_init(void);
		void fdcan1_init(unsigned short nbrp);
		void fdcan2_init(unsigned short nbrp);
		
		int FDCAN1_getTxLen(void);
		int FDCAN1_getRxLen(void);
		int FDCAN1_ReadStandardMSG(unsigned short *id,unsigned char buf[8]);
		int FDCAN1_SendStandardMsg(unsigned short id,unsigned char buf[8]);
		
		int FDCAN2_getTxLen(void);
		int FDCAN2_getRxLen(void);
		int FDCAN2_ReadStandardMSG(unsigned short *id,unsigned char buf[8]);
		int FDCAN2_SendStandardMsg(unsigned short id,unsigned char buf[8]);

#ifdef __cplusplus
	}
#endif
#endif
