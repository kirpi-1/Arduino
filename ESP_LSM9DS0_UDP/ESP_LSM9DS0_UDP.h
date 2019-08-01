#ifndef ESP_LSM9DS0_UDP_H
#define ESP_LSM9DS0_UDP_H
#include <ESPSensorUDP.h>
#include <SFE_LSM9DS0.h>

// definitions for LSM9DS0
///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both grounded, therefore our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

#define MAX_IMU_BUFF_SIZE 128
#define HEADER_SYMBOL '['
#define FOOTER_SYMBOL ']'
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

#define ASCALE LSM9DS0::A_SCALE_6G
#define MSCALE LSM9DS0::M_SCALE_2GS
#define GSCALE LSM9DS0::G_SCALE_245DPS
#define GODR LSM9DS0::G_ODR_95_BW_125
#define AODR LSM9DS0::A_ODR_100
#define MODR LSM9DS0::M_ODR_100



struct IMUData{
  unsigned int time;
  unsigned int idx;	
  float gx,gy,gz,ax,ay,az,mx,my,mz;
	float w,x,y,z;
};

struct IMUDatagram{
	//unsigned char header[4];
  unsigned int time;
  unsigned int idx;	
	SensorType type;
  float gx,gy,gz,ax,ay,az,mx,my,mz;	
	float w,x,y,z;
	//unsigned char footer[4];
};

struct IMUBuffer{
	unsigned int size;
	IMUData data[MAX_IMU_BUFF_SIZE];
};

class ESP_LSM9DS0: public ESPSensor{
  public:
    ESP_LSM9DS0();
    ~ESP_LSM9DS0();
    void begin();
    void printSensorData();    
    bool read();
		void send();
    void setInterrupts(byte accelInt, byte magInt, byte gryoInt);

    IMUDatagram imuData;
		unsigned int dataSize;
		IMUBuffer imuBuff;
    LSM9DS0 *dof;
    bool usingInterrupts;
  private:
    byte m_INT1XM, m_INT2XM, m_DRDYG;
		float deltat = 0.0f;        // integration interval for both filter schemes
		uint32_t lastUpdate = 0;    // used to calculate integration interval
		uint32_t curTime = 0;           // used to calculate integration interval
		float abias[3];
		float gbias[3];
		float q[4];
		
		void MadgwickQuaternionUpdate();
    
};

#endif