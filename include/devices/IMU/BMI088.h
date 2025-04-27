#ifndef _BMI088_H_
#define _BMI088_H_

#ifdef __cplusplus
	extern "C"{
#endif

		#define BMI088_WRITE	0x00
		#define BMI088_READ		0x80

		#define BMI088_ACC_CHIP_ID        0x00
		#define BMI088_ACC_ERR_REG        0x02
		#define BMI088_ACC_STATUS         0x03
		#define BMI088_ACC_DATA           0x12
		#define BMI088_ACC_INT_STAT_1     0x1d
		#define BMI088_TEMP_DATA          0x22
		#define BMI088_ACC_FIFO_LENGTH		0x24
		#define BMI088_ACC_FIFO_DATA			0x26
		#define BMI088_ACC_CONF           0x40
		#define BMI088_ACC_RANGE          0x41
		#define BMI088_ACC_FIFO_CONFIG_0	0x48
		#define BMI088_ACC_FIFO_CONFIG_1	0x49
		#define BMI088_INT1_IO_CONF       0x53
		#define BMI088_INT2_IO_CONF       0x54
		#define BMI088_INT1_INT2_MAP_DATA 0x58
		#define BMI088_ACC_SELF_TEST      0x6D
		#define BMI088_ACC_PWR_CONF       0x7C
		#define BMI088_ACC_PWR_CTRL       0x7D
		#define BMI088_ACC_SOFTRESET      0x7E

		#define BMI088_GYR_CHIP_ID         		0x00
		#define BMI088_GYR_DATA             	0x02
		#define BMI088_GYR_INT_STAT_1					0x0A
		#define BMI088_GYR_FIFO_STATUS				0x0E
		#define BMI088_GYR_RANGE            	0x0F
		#define BMI088_GYR_BANDWIDTH        	0x10
		#define BMI088_GYR_LPM1              	0x11
		#define BMI088_GYR_SOFTRESET         	0x14
		#define BMI088_GYR_INT_CTRL          	0x15
		#define BMI088_GYR_INT3_INT4_IO_CONF 	0x16
		#define BMI088_GYR_INT3_INT4_IO_MAP  	0x18
		#define BMI088_GYR_SELF_TEST		  		0x3C
		#define BMI088_GYR_FIFO_CONFIG_0			0x3D
		#define BMI088_GYR_FIFO_CONFIG_1			0x3E
		#define BMI088_GYR_FIFO_DATA					0x3F
		
		
		extern unsigned short bmi088_accl_cnt,bmi088_accl_freq;
		
		extern unsigned short bmi088_gyro_cnt,bmi088_gyro_freq;
		
		void BMI088_init(void);
		
		void BMI088_ReadAccl(float *x,float *y,float *z);
		
		void BMI088_ReadGyro(float *x,float *y,float *z);
		
		float BMI088_ReadTemp(void);
		
		void BMI088_Accl_Get(unsigned char *buf);
		void BMI088_Gyro_Get(unsigned char *buf);
		void BMI088_Temp_Get(unsigned char *buf);
		void BMI088_Accl_Ask(unsigned char id);
		void BMI088_Gyro_Ask(unsigned char id);
		void BMI088_TempAsk(void);
		
#ifdef __cplusplus
	}
#endif
	
#endif