#ifndef ESP_LSM9DS0_UDP_H
#define ESP_LSM9DS0_UDP_H

#include <Wire.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>

#include <ESPSensorUDP.h>
// I2C setup 
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -11.55 // Declination (degrees) in Irvine, CA

#define DEFAULT_ACC_SCALE 4
#define DEFAULT_GYRO_SCALE 245
#define DEFAULT_MAG_SCALE 4
#define DEFAULT_ACC_SAMPLERATE 3
#define DEFAULT_GYRO_SAMPLERATE 3
#define DEFAULT_MAG_SAMPLERATE 7

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f


struct IMUDatagram{	
  unsigned int time;
  unsigned int idx;	
	SensorType type;
	byte mac[6];
	byte buffer[2]; //alignment issues
  float gx,gy,gz,ax,ay,az,mx,my,mz;	
	float w,x,y,z;
};

class ESP_LSM9DS1: public ESPSensor{
  public:
    ESP_LSM9DS1();
    ~ESP_LSM9DS1();
    void begin();
    void printSensorData();    
    bool read();
		void send();
    void setInterruptPins(byte gryoInt, byte accelInt, byte magInt);
		void setInterrupts();
		void setGyroScale(gyro_scale gscale){m_gscale = gscale;};		
		void setAccScale(accel_scale ascale){m_ascale = ascale;};		
		void setMagScale(mag_scale mscale){m_mscale = mscale;};
		void setGyroRate(gyro_odr grate){m_grate = grate;};
		void setAccRate(accel_odr arate){m_arate = arate;};		
		void setMagRate(mag_odr mrate){m_mrate = mrate;};
		void connectToBaseStation(){return;};
		
		
    IMUDatagram imuData;
    LSM9DS1 *imu;
    bool usingInterrupts;		
		
  private:
    byte m_INT1XM, m_INT2XM, m_DRDYM;
		float deltat = 0.0f;        // integration interval for both filter schemes
		uint32_t lastUpdate = 0;    // used to calculate integration interval
		uint32_t curTime = 0;       // used to calculate integration interval
		float abias[3];
		float gbias[3];
		float q[4];
		gyro_scale m_gscale;
		accel_scale m_ascale;
		mag_scale m_mscale;
		accel_odr m_arate;
		gyro_odr m_grate;
		mag_odr m_mrate;
		
		
		void MadgwickQuaternionUpdate();
    
};




#endif
