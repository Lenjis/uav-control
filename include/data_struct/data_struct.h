#ifndef _data_struct_
#define _data_struct_

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include <stdbool.h>

#define ANALOG_BIT_ERROR_COUNT_LIMIT 1000 // analog hardware soft bit limits
#define COM_BIT_ERROR_COUNT_LIMIT    1000 // com soft bit limits
			
typedef unsigned char   BYTE;
typedef unsigned short  WORD;
typedef unsigned long   DWORD;
typedef float real;
#define NUM_AXIS 3
#define Rad2Deg 57.295779513
#define Deg2Rad 1/Rad2Deg			
typedef struct  {
    unsigned          char         gpsFixType;    // GPS solution type: invalid, spp, dgps, spp, rtk_float, rtk_fix
    unsigned          char         numSatellites;

    unsigned          char         GPSmonth;      // mm
    unsigned          char         GPSday;        // dd
    unsigned          char         GPSyear;       // yy last two digits of year
    char            GPSHour;       // hh
    char            GPSMinute;     // mm
    char            GPSSecond;     // ss
    double          GPSSecondFraction; // FIXME used?

    unsigned          int        itow;           ///< gps milisecond Interval Time Of Week
    int             updateFlagForEachCall; /// changed to 16 bits

    int             totalGGA;
    int             totalVTG;
    
    double          lat;           // [deg], latitude
    double          lon;           // [lon], longitude
    double          alt;          // [m] above WGS84 ellipsoid
    double          vNed[3];       // NED North East Down [m/s] x, y, z
    double          trueCourse; // [deg]
    double          rawGroundSpeed; // [m/s]

    float                geoidAboveEllipsoid;    // [m] Height of geoid (mean sea level) above WGS84 ellipsoid

    /// compatible with Ublox driver FIXME should these be seperate data structure?
    unsigned char        ubloxClassID;
    unsigned char        ubloxMsgID;
    signed long          LonLatH[3]; // SiRF Lat Lon[deg] * 10^7 Alt ellipse [m]*100 <-- UNUSED
    float                HDOP;       // Horizontal Dilution Of Precision x.x
    double               GPSVelAcc;
    unsigned short       GPSStatusWord;  /// will replace GPSfix
    unsigned char        isGPSFWVerKnown;
    unsigned char        isGPSBaudrateKnown;
    unsigned long        Timer100Hz10ms;
    unsigned char        ubloxOldVersion;
    float                UbloxSoftwareVer;
    unsigned int         navCFGword;
    unsigned int         nav2CFGword;
    char                 GPSConfigureOK; /// always needs to be initialized as -1

    unsigned char        reClassID;
    unsigned char        reMsgID;

    unsigned long        LLHCounter;
    unsigned long        VELCounter;
    unsigned long        STATUSCounter; // UBLOX - or first flag SiRF
    unsigned long        SBASCounter;
    unsigned long        firewallCounter;
    unsigned long        firewallRunCounter;
    unsigned long        reconfigGPSCounter;

    /// GPS Baudrate and protocal: -1, 0,1, 2, 3 corresponding to
    int                  GPSbaudRate;    /// 4800, 9600, 19200, 38400, 57600, 115200, etc
    /// AutoDect, Ublox Binary, NovAtel binary, NovAtel ASCII, NMEA
//    enumGPSProtocol      GPSProtocol;

//    universalMSGSpec     GPSMsgSignature;
    unsigned char        GPSAUTOSetting;
    unsigned char        GPSTopLevelConfig; // UBLOX
    unsigned char        resetAutoBaud;
    unsigned char        autoBaudCounter;

    float                GPSHorizAcc;
    float                GPSVertAcc;
	
		float				 heading;
		float				 hdg_std_dev;
		unsigned char hdg_update;
} GpsData_t;

typedef struct {
    double oldValues[3]; // moving window of delta values
    double sum;
    double last; // Last whole value
} gpsDeltaStruct;

typedef struct {
    // Sensor readings in the body-frame (B)
    real accel_B[NUM_AXIS];         // [m/s/s]
    real angRate_B[NUM_AXIS];       // [rad/s]
    real magField_B[NUM_AXIS];      // [G]

    // GPS information
    uint32_t itow;
    double llaAnt[3];               // Antenna Lat, Lon, ellipsoid Altitude, [rad, rad, meter]
    double vNedAnt[NUM_AXIS];       // Antenna NED velocity, [m/s, m/s, m/s]
    double lla[3];                  // IMU Lat, Lon, ellipsoid Altitude, [rad, rad, meter]
    double vNed[3];                 // IMU NED velocity, [m/s, m/s, m/s]
    float geoidAboveEllipsoid;      // [m]
    real trueCourse;                // Antenna heading, [deg]
    real rawGroundSpeed;            // IMU ground speed, calculated from vNed, [m/s]
    float GPSHorizAcc;              // [m]
    float GPSVertAcc;               // [m]
    float HDOP;
    uint8_t gpsFixType;             // Indicate if this GNSS measurement is valid
    uint8_t numSatellites;          /* Num of satellites in this GNSS measurement.
                                     * This is valid only when there is gps udpate.
                                     */
    bool gpsUpdate;                 // Indicate if GNSS measurement is updated.
		
		float				 heading;
		float				 hdg_std_dev;
		unsigned char hdg_update;

    // odometer
    bool odoUpdate;                 // indicate if odo measurement is updated
    real odoVelocity;               // velocity from odo, [m/s]

    // 1PPS from GNSS receiver
    bool ppsDetected;
} EKF_InputDataStruct;

typedef struct{
 float Frame_head;
 float OUT_cnt;
 float Gyro[3];
 float Accel[3];
 float Magn[3];
 float mBar;
 float Att[3];
 float Vn[3];
 float Pos[5];
 float GPS_Vn[3];
 float GPS_Pos[5];
 float GPS_status;
 float GPS_delay;
 float Temp;
 float pk[19];
}Out_Data;

extern Out_Data out_data;

typedef struct{
  unsigned int GPS_ITOW, pos_type;
  double GPS_Vn[3];
  double GPS_Pos[3];
  double GPS_Mot;
	double lat;
	double lon;
	double alt;
	double epv;
	double eph;
	double Vg, psi;
	double vel_neu[3];
	float yaw, yaw_acc;	
  unsigned char GPS_fixType;
  unsigned char GPS_flags;
  unsigned char GPS_numSV, stars_num;
  float GPS_pDOP;
  double GPS_hAcc;
  double GPS_vAcc;
  double GPS_headAcc;
  double GPS_sAcc;
  double GPS_gSpeed;
	double truecourse;
	double groundspeed;
}GPS_Data_value;

typedef struct{
	float Accel[3];//Accel X,Y,Z
	float Temp;
	float Gyro[3];//Gyro X,Y,Z
	float Mag[3];	//Mag X,Y,Z	
	float Pressure;
  float Altitude;
}IMU_Data_value;

typedef struct  {
    WORD        invalid;
    WORD        counter;
    int         fail;       
    WORD        freq;
} SensorStruc;

typedef union {
    BYTE    B[4];
    short   D[2];
    WORD    W[2];
    long    DW;
    unsigned long UDW;
	  float   f;
} dWordStruc;

#ifdef  __cplusplus
}  
#endif
#endif