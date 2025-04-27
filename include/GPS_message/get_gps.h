#ifndef _SBG_NEW_H_
#define _SBG_NEW_H_

#include "data_struct.h"

#ifdef  __cplusplus
    extern "C" {
#endif
			
#define GOT_GGA_MSG  			 0
#define GOT_VTG_MSG  			 1

#define gnssFixOK				1<<0
#define diffSoln				1<<1
#define relPosValid				1<<2
#define carrSolnFloat			1<<3
#define carrSolnFixed			1<<4
#define isMoving				1<<5
#define refPosMiss				1<<6
#define refObsMiss				1<<7
#define relPosHeadingValid		1<<8
#define relPosNormalized		1<<9
	
#define UBLOX_MAX_PAYLOAD   512
#define UBLOX_WAIT_MS	    20
			
typedef struct{
	/*device status*/
	unsigned long time_us;//(us)time since sensor is powered up.
	
	unsigned short status;//general status bitmask and enums.
	/* if true or OK bit set 1,if fail or error bit set 0.
	 * bit0: main power
	 * bit1: imu power
	 * bit2: gps power
	 * bit3: settings
	 * bit4: temperature
	 * bit5: datalogger
	 * bit6: cpu
	 */
	 
	unsigned long com_status;//communication status bitmask and enums.
	/* bit0: port A valid
	 * bit1: port B valid
	 * bit2: port C valid
	 * bit3: port D valid
	 * bit4: port E valid
	 * bit5: saturation on port A input
	 * bit6: saturation on port A output
	 * bit7: saturation on port B input
	 * bit8: saturation on port B output
	 * bit9: saturation on port C input
	 * bit10: saturation on port C output
	 * bit11: saturation on port D input
	 * bit12: saturation on port D output
	 * bit13: saturation on port E input
	 * bit14: saturation on port E output
	 * bit15: saturation on port ETH0
	 * bit16: saturation on port ETH1
	 * bit17: saturation on port ETH2
	 * bit18: saturation on port ETH3
	 * bit19: saturation on port ETH4
	 * bit25: low level communication error
	 * bit26: saturation on CAN bus input buffer
	 * bit27: saturation on CAN bus output buffer
	 * bit28-30: CAN bus status 
	 ** 0: bus off operation due to too much errors
	 ** 1: transmit or received error
	 ** 2: the CAN bus is working correctly
	 ** 3: a general error has occurred on the CAN bus
	 */
	
	unsigned long aiding_status;//aiding equipment status bitmask and enums.
	/* bit0: valid GPS 1 position data is received
	 * bit1: valid GPS 1 velocity data is received
	 * bit2: valid GPS 1 true heading data is received
	 * bit3: valid GPS 1 UTC time is received
	 * bit4: valid GPS 2 position data is received
	 * bit5: valid GPS 2 velocity data is received
	 * bit6: valid GPS 2 true heading data is received
	 * bit7: valid GPS 2 UTC time data is received
	 * bit8: valid magnetometer data is received
	 * bit9: odometer pulse is received
	 * bit10: valid DVL data is received
	 * bit11: valid USBL data is received
	 * bit12: valid Depth sensor data is received
	 * bit13: valid altitude and/or airspeed is received
	 */
	
	unsigned long up_time_us;//(s)system up time since the power on
	
	/*UTC time and GPS time*/
	unsigned short clock_status;//general UTC time and clock sync status
	/* bit0: set to 1 when a clock input can be used to synchronize the internal clock
	 * bit1-4: the internal clock estimation status
	 ** 0: an error has occured on the clock estimation
	 ** 1: the clock is only based on the internal crystal
	 ** 2: a PPS has ben detected and the clock is converging to it
	 ** 3: the clock has converged to the PPS and is within 500ns
	 * bit5: set to 1 if UTC time is synchronized with a PPS
	 * bit6-9: the UTC validity status
	 ** 0: the UTC time is not known,we are just propagating the UTC time internally
	 ** 1: we have received valid UTC time information bur we don't have the leap seconds information
	 ** 2:we have received valid UTC time data with valid leap seconds
	 */
	
	unsigned short year;//year
	
	unsigned char month;//month in year
	
	unsigned char day;//day in month
	
	unsigned char hour;//hour in day
	
	unsigned char min;//minute in hour
	
	unsigned char sec;//second in minute
	
	unsigned long nanosec;//nanosecond of second
	
	unsigned long gps_tow;//(ms)GPS time of week
	
	/*internal IMU data*/	
	unsigned short imu_status;//IMU status bitmask
	/* if true or OK bit set 1,if fail or error bit set 0.
	 * bit0: imu com
	 * bit1: imu status
	 * bit2: imu accel x
	 * bit3: imu accel y
	 * bit4: imu accel z
	 * bit5: imu gyro x
	 * bit6: imu gyro y
	 * bit7: imu gyro z
	 * bit8: imu accels in range
	 * bit9: imu gyros in range
	 */
	 
	float acclx,accly,acclz;//(m/s^2)accelerometer samples
	
	float gyrox,gyroy,gyroz;//(rad/s)gyroscope samples
	
	float temp;//internal temperature
	
	float dvelx,dvely,dvelz;//sculling output
	
	float dangx,dangy,dangz;//coning output
	
	/*ekf output*/	
	unsigned long solution_status;//global solution status
	/* bit0-3: the kalman filter computation mode
	 ** 0: the kalman filter is not initialized and the returned data are all invalid
	 ** 1: the kalman filter only rely on a vertical reference to compute roll and pitch angles,heading and navigation data drift freely
	 ** 2: a heading reference is available,the kalman filter provides full orientation but navigation data drift freely
	 ** 3: the kalman filter computes orientation and velocity.position is freely integrated from velocity estimation
	 ** 4: nominal mode,the kalman filter computes all parameters.absolute position is provided.
	 * bit4: attitude data is reliable (<0.5deg)
	 * bit5: heading data is reliable (<1deg)
	 * bit6: velocity data is reliable (<1.5m/s)
	 * bit7: position data is reliable (<10m)
	 * bit8: vertical reference is used in solution
	 * bit9: magnetometer is used in solution
	 * bit10: GPS velocity is used in solution
	 * bit11: GPS position is used in solution
	 * bit13: GPS true heading is used in solution
	 * bit14: GPS2 velocity is used in solution
	 * bit15: GPS2 position is used in solution
	 * bit17: GPS2 true heading is used in solution
	 * bit18: odometer is used in solution
	 * bit19: DVL bottom tracking is used in solution
	 * bit20: DVL water layer is used in solution
	 * bit24: USBL/LBL is used in solution
	 * bit25: an altitude or true airspeed is used in solution
	 * bit26: a ZUPT is used in solution
	 * bit27: sensor alignment and calibration parameters are valid
	 * bit28: depth sensor is used in solution
	 */
	 
	/*euler angles*/
	float roll,pitch,yaw;//(deg)euler angles
	
	float roll_acc,pitch_acc,yaw_acc;//(rad)1�� angles accuracy	 
	
	/*quaternion attitude*/
	float Q[4];//quaternion parameters
	
	/*navigation,position,velocity*/
	float veln,vele,veld;//(m/s)velocity in directions
	
	float veln_acc,vele_acc,veld_acc;//(m/s)1�� velocity in directions accuracy
	
	double lat,lon;//(1e-7*deg)GPS postion
	
	double alt;//altitude above mean sea level
	
	float undulation;//altitude difference between the geoid and the ellipsoid(WGS-84 altitude=MSL altitude + undulation)
	
	float lat_acc,lon_acc,alt_acc;//(m)1�� position accuracy
	
	/*magnetometer*/
	unsigned short mag_status;//magnetometer status bitmask
	/* bit0: the magnetometer X has passed the self test
	 * bit1: the magnetometer Y has passed the self test
	 * bit2: the magnetometer Z has passed the self test
	 * bit3: the accelerometer X has passed the self test
	 * bit4: the accelerometer Y has passed the self test
	 * bit5: the accelerometer Z has passed the self test
	 * bit6: magnetometer is mot saturated
	 * bit7: accelerometer is not saturated
	 * bit8: magnetometer seems to be calibrated
	 */
	
	float magx,magy,magz;//(a.u)magnetometer output
	
	/*GNSS velocity*/
	unsigned long GPS_vel_status;//GPS velocity fix and status bitmask
	/* bit0-5: the raw GPS velocity status
	 ** 0: a valid solution has been computed
	 ** 1: not enough valid SV to compute a solution
	 ** 2: an internal error has occurred
	 ** 3: velocity limit exceeded
	 * bit6-11: the raw GPS velocity type
	 ** 0: no valid velocity solution available
	 ** 1: an unknown solution type has been computed
	 ** 2: a doppler velocity has been computed
	 ** 3: a velocity has been computed between two positions
	 */
	
	float course;//(deg)true direction of motion over ground(0-360deg)
	
	float course_acc;//(deg)1�� course accuracy
	
	/*GNSS position*/
	unsigned long GPS_pos_status;//GPS position fix and atatus bitmask
	/* bit0-5: the raw GPS position status
	 ** 0: a valid solution has been computed
	 ** 1: not enough valid SV to compute a solution
	 ** 2: an internal error has occurred
	 ** 3: the height limit has been exceeded
	 * bit6-11: the raw GPS position type
	 ** 0: no valid solution available
	 ** 1: an unknown solution type has been computed
	 ** 2: single point solution position
	 ** 3: standard pseudorange differential solution
	 ** 4: SBAS satellite used for differential corrections
	 ** 5: omnistar VBS position
	 ** 6: floating RTK ambiguity solution
	 ** 7: integer RTK ambiguity solution
	 ** 8: precise point positioning with float ambiguities
	 ** 9: precise point positioning with fixed ambiguities
	 ** 10: fixed location solution position
	 * bit12: GPS L1CA/L1P is used in solution
	 * bit13: GPS L2P/L2C is used in solution
	 * bit14: GPS L5 is used in solution
	 * bit15: GLONASS L1CA is used in solution
	 * bit16: GLONASS L2C/L2P is used in solution
	 * bit17: GLONASS L3 is used in solution
	 * bit18: Galileo E1 is used in solution
	 * bit19: Galileo E5a is used in solution
	 * bit20: Galileo E5b is used in solution
	 * bit21: Galileo E5 AltBoc is used in solution
	 * bit22: Galileo E6  is used in solution
	 * bit23: BeiDou B1 is used in solution
	 * bit24: BeiDou B2 is used in solution
	 * bit25: BeiDou B3 is used in solution
	 * bit26: QZSS L1CA is used in solution
	 * bit27: QZSS L2C is used in solution
	 * bit28: QZSS L5 is used in solution
	 */
	
	unsigned char num_sv_used;//number of space vehicles used in GNSS solution
	
	unsigned short base_station_id;//ID of the DGPS/RTK base station in use
	
	float diff_age;//(s)differential data age
	
	/*GNSS true heading*/
	unsigned short GPS_HDT_status;//GPS true heading status
	/* bit0-5: the raw GPS true heading status
	 ** 0: a valid solution has been computed
	 ** 1: not enough valid SV to compute a solution
	 ** 2: an imternal error has occurred
	 ** 3: the height limit has been exceeded
	 * bit6: the baseline length field is filled and valid
	 */
	
	float gps_hdt;//(deg)GPS true heading
	
	float gps_hdt_acc;//(deg)1�� GPS true heading acuracy
	
	float gps_pitch;//(deg)pitch angle from the master to the rover
	
	float gps_pitch_acc;//(deg)1�� pitch estimated accuracy
	
	float gps_baseline;//(m)distance between main and aux antenna
	
	/*airdata,altitude and true airspeed*/
	unsigned short airdata_struct;//airdata information status
	/* bit0: the time stamp field if a measurement delay instead of an absolute time stamping information
	 * bit1: the absolute pressure field is filled and valid
	 * bit2: the barometric altitude field is filled and valid
	 * bit3: the differential pressure field is filled and valid
	 * bit4: the true airspeed field is filled and valid
	 * bit5: the output air temperature field is filled and valid
	 */
	
	float pressure_abs;//raw absolute pressure measured by the barometer sensor
	
	float baro_alt;//altitude computed from barometric altimeter
	
	float pressure_diff;//raw differential pressure measured by the pitot tube
	
	float true_airspeed;//true airspeed measured by the pitot tube
	
	float air_temp;//outside air temperature used for airspeed computations
} SBGN_MSG;

typedef struct  {
    uint8_t     gpsFixType;     // 1 if data is valid
    uint8_t     gpsUpdate;      // 1 if contains new data
    uint8_t     numSatellites;  // num of satellites in the solution    
    uint32_t    itow;           // gps Time Of Week, miliseconds

    double      latitude;       // latitude ,  degrees 
    double      longitude;      // longitude,  degrees 
    double      vNed[3];        // velocities,  m/s  NED (North East Down) x, y, z
	  double      pDOP;
    double      trueCourse;     // [deg]
	  double      trueCourse_acc; 
    double      rawGroundSpeed; // [m/s]
	  double      rawGroundSpeed_acc;
    double      altitude;       // above WGS84 ellipsoid [m]
    double      GPSSecondFraction; 

 
    uint8_t     GPSmonth;     // mm
    uint8_t     GPSday;       // dd
    uint8_t     GPSyear;      // yy last two digits of year
    char        GPSHour;      // hh
    char        GPSMinute;    // mm
    char        GPSSecond;    // ss

    float       GPSHorizAcc, GPSVertAcc;
    float       HDOP;
    float       geoidAboveEllipsoid;    // [m] Height of geoid (mean sea level) above WGS84 ellipsoid
	uint32_t    headingiTOW;           // gps Time Of Week, miliseconds
	float		heading;
	float		hdg_std_dev;
	unsigned char hdg_update;		
    bool        heading_update;		
} gpsDataStruct_t;

typedef struct {
	unsigned char gpsTask;
	unsigned char gpsVelFlag;
	unsigned char gpsPosFlag;
	unsigned int baudCycle[7];
	signed char baudSlot;
	unsigned int logHandle;

	unsigned short year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char min;
	unsigned char sec;

	unsigned char fixType;
	unsigned char numSV;
	unsigned long iTOW;
	double lat;
	double lon;
	float height;   // above mean sea level (m)
	float hAcc;     // horizontal accuracy est (m)
	float vAcc;     // vertical accuracy est (m)
	float velN;     // north velocity (m/s)
	float velE;     // east velocity (m/s)
	float velD;     // down velocity (m/s)
	float speed;    // ground speed (m/s)
	float heading;  // deg
	float sAcc;     // speed accuracy est (m/s)
	float cAcc;     // course accuracy est (deg)
	float pDOP;     // position Dilution of Precision
	float hDOP;
	float vDOP;


	unsigned long TPtowMS;    // timepulse time of week (ms)
	unsigned long lastReceivedTPtowMS;

	unsigned long lastTimepulse;
	unsigned long lastPosUpdate;
	unsigned long lastVelUpdate;
	unsigned long lastMessage;
	signed long microsPerSecond;
} gpsStruct_t;

typedef struct{
	unsigned long iTOW;
	float relPosN;
	float relPosE;
	float relPosD;
	float relPosLength;
	float relPosHeding;
	float accLength;
	float accHeading;
	unsigned char have_gps_yaw_accuracy;
}gpsheadingStruct_t;

typedef struct 
{
    unsigned long iTOW;      // ms
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
    unsigned char valid;      // Validity flags
    unsigned long tAcc;      // time accuracy estimate (UTC), ns
    signed long nano;       // fraction of second, range -1e9..1e9 (UTC), ns
    unsigned char fixType;    /* GNSS fix type
                         *  0: no fix
                         *  1: dead reckoning only
                         *  2: 2D-fix
                         *  3: 3D-fix
                         *  4: GNSS + dead reckoning combined
                         *  5: time only fix
                         */
    unsigned char flags;      /* Fix status flags
                         *  bit0:   gnssFixOK, 1 = valid fix (i.e within DOP & accuracy masks)
                         *  bit1:   diffSoln, 1 = differential corrections were applied
                         *  bit2-4: psmState, Power Save Mode state
                         *  bit5:   headVehValid, 1 = heading of vehicle is valid
                         *  bit6-7: carrSoln, Carrier phase range solution status:
                         *      0: no carrier phase range solution
                         *      1: float solution
                         *      2: fixed solution 
                         */
    unsigned char flags2;
    unsigned char numSV;      // number of satellites in Nav solution;
    signed long lon;        // deg, scaling is 1e-7
    signed long lat;        // deg, scaling is 1e-7
    signed long height;     // mm, height above ellipsoid;
    signed long hMSL;       // mm, hegiht above mean seal level
    unsigned long hAcc;      // mm, horizontal accuracy estimate
    unsigned long vAcc;      // mm, vertical accuracy estimate
    signed long velN;       // mm/s, north velocity
    signed long velE;       // mm/s, east velocity
    signed long velD;       // mm/s, dow velocity
    signed long gSpeed;     // mm/s, groud speed (2-D)
    signed long headMot;    // deg, scaling is 1e-5, heading of motion (2-D)
    unsigned long sAcc;      // mm/s, speed accuracy estimate
    unsigned long headAcc;   // deg, scaling is 1e-5, heading accuracy estimate
    unsigned short pDOP;      // scaling is 0.01, position DOP
    unsigned char reserved1[6];
    signed long headVeh;    // deg, scaling is 1e-5, heading of vehicle (2-D)
    signed short magDec;     // deg, scaling is 1e-2, magnetic declination
    unsigned short magAcc;    // deg, scaling is 1e-2, magnetic declination accuracy
} ubloxStructPVT_t;

typedef struct
{
	unsigned char version;
	unsigned char reserved1;
	unsigned short refStationId;
	unsigned long iTOW;			//ms, GPS time of week of the navigation epoch
	signed long relPosN;		//cm, North component of the relative position vector
	signed long relPosE;		//cm, East component of the relative position vector
	signed long relPosD;		//cm, Down component of the relative position vector
	signed long relPosLength;	//cm, Length of the relative position vector
	signed long relPosHeading;	//deg, scaling is 1e-5, Heading of the relative position vector
	unsigned char reserved2[4];
	signed char relPosHPN;
	signed char relPosHPE;
	signed char relPosHPD;
	signed char relPosHPLength;
	unsigned long accN;
	unsigned long accE;
	unsigned long accD;
	unsigned long accLength;	//mm, Accuracy of the length of the relative position vector
	unsigned long accHeading;	//deg, scaling is 1e-5, Accuracy of the relative position vector
	unsigned char reserved3[4];
	unsigned long flags;

}ubloxStructRELPOSNED_t;

typedef struct {
    int hwVer;

    signed long lastLat, lastLon;
    union {
		//ubloxStructPOSLLH_t posllh;
		//ubloxStructVALNED_t valned;
		//ubloxStructDOP_t dop;
		//ubloxStructTP_t tp;
		//ubloxStructTIMEUTC_t timeutc;
		//ubloxStructVER_t ver;
		ubloxStructPVT_t pvt;
		ubloxStructRELPOSNED_t relposned;
		char other[UBLOX_MAX_PAYLOAD];
    } payload;

    unsigned char state;
    unsigned int count;
    unsigned char charsum;
    unsigned char id;
    unsigned int length;
    unsigned int checksumErrors;

    unsigned char ubloxRxCK_A;
    unsigned char ubloxRxCK_B;

} ubloxStruct_t;

unsigned char ubloxCharIn(unsigned char c);

extern gpsDataStruct_t gGPS, gCanGps;

extern SBGN_MSG sbge_data;

extern SensorStruc state_sbge;

unsigned char decode_sbg(unsigned char *readbuf,int readlen);

void TaskSBGE();

void Monitor_SBGE();

void GetGPSData(gpsDataStruct_t *data);

void GetUbloxGPSData(gpsDataStruct_t *data);

void ubloxInit(void);

#ifdef  __cplusplus
}  
#endif
#endif //_SBG_N_H_