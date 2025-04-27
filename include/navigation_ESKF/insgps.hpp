#ifndef _INSGPS_H
#define _INSGPS_H
			
#include "psins.hpp"
#include "data_struct.h"
			
#define stdandmean 100
#define TS 0.005
#define G_xmu 9.78928723367847
			
class CKFApp:public CSINSTDKF, public LowPassFilter2p
{
public:
	unsigned char meanflag, num, atrest, mearloss;
  unsigned int meanflags;
	double tmeas, sl, sl2, sl4, last_rest_yaw;
  double std[6], std2[6], inputdata[stdandmean+1][6], vibesum[6], vibesum2[6], vibemean[6], vibemean2[6];	
	CVect3 measGPSVn, measGPSPos, measaccel, initaccel, initmag, initpos, gyrolpf, accellpf;
  CVect3 wm, vm, att, gpsvn, gpspos, eb;
  CVect3 gyro_base, accel_base;
  CVect3 mean, sum, pos_acc;
  CQuat quanb;
  CEarth eth;

	CKFApp(void);
	void Init16(const CSINS &sins0);
//	float  p(){return 1.0;}
  virtual void Measup(double hacc, double vacc, const CVect3 &refaccel);
  void AHRS(const CVect3 &refaccel);
	void SetMeas(CVect3 *vnm, CVect3 *posm, double tm);
  void Eulerinit(const CVect3 &accel, const CVect3 &mag, const CVect3 &pos);
	void Update(void);
  void gps_message();
  void gps_sbgmessage();
	void mean_std(int num, int meanflag);
	void at_rest();
	void IMUData_compensate(IMU_Data_value &imu_data);
};

class INITapp:public CAligni0
{
public:
	CVect3 gpspos0;
  CQuat quanb;
	INITapp(void);
  CQuat attinit(const CVect3 &wm, const CVect3 &vm, int nSamples, double ts);
  //virtual CQuat Update(void);
};

extern CKFApp kf;  
extern INITapp *insinit;
extern double ts, yaw0, g_xmu;
extern CVect3 euler_init;
extern double g_xmu;
extern unsigned short hzflag , ahrsflag , gpsflag , AHRS_model , GPS_model, testflag;
extern IMU_Data_value bmi088;
extern IMU_Data_value adis16488;
void INSUartOut(const CSINS &sins, const CKFApp &kf);

void get_IMU(void);
void ESKF(void);
void IMU_correct(IMU_Data_value &imu_data);

//void ESKF(const CKFApp &kf, const INITapp &insinit, IMU_Data_value &imu_data);

#endif


