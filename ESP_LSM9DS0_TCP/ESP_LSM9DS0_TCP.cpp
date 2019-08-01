#include "ESP_LSM9DS0_TCP.h"

ESP_LSM9DS0::ESP_LSM9DS0(){
  usingInterrupts = 0;
  m_INT1XM = 0;
  m_INT2XM = 0;
  m_DRDYG = 0;
	memset(abias,0,sizeof(float)*3);
	memset(gbias,0,sizeof(float)*3);
	memset(q,0,sizeof(float)*4);
	q[0] = 1.0f;
	q[1] = 0;
	q[2] = 0;
	q[3] = 0;
	lastUpdate = micros()/1000.0f;
	curTime = micros()/1000.0f;  
	dataSize = 0;
	data = new IMUData[MAX_IMU_BUFF_SIZE];

}
ESP_LSM9DS0::~ESP_LSM9DS0(){
  delete dof;
	delete[] data;
}

void ESP_LSM9DS0::printSensorData(){
  Serial.print(dof->gx);
  Serial.print(dof->gy);
  Serial.print(dof->gz);
  Serial.print("\t")   ; 
  Serial.print(dof->ax);
  Serial.print(dof->ay);
  Serial.print(dof->az);
  Serial.print("\t");
  Serial.print(dof->mx);
  Serial.print(dof->my);
  Serial.println(dof->mz);
}

bool ESP_LSM9DS0::connectToBaseStation(){
  return ESPSensor::connectToBaseStation(IMU);  
}


bool ESP_LSM9DS0::read(){  
  bool accelReady = digitalRead(m_INT1XM);
	bool magReady = digitalRead(m_INT2XM);
	bool gyroReady = digitalRead(m_DRDYG);
	newData = false;
  if (!usingInterrupts || (accelReady && magReady && gyroReady)){
    dof->readGyro();
    dof->readAccel();
    dof->readMag();
				
		if(dataSize<MAX_IMU_BUFF_SIZE){
			lastUpdate = curTime;
			curTime = micros();			
			deltat = (curTime-lastUpdate)/1000000.0f;
			
			data[dataSize].time = millis();					
			data[dataSize].idx = idx++;		
			data[dataSize].gx = dof->calcGyro(dof->gx) - gbias[0];
			data[dataSize].gy = dof->calcGyro(dof->gy) - gbias[1];
			data[dataSize].gz = dof->calcGyro(dof->gz) - gbias[2];    
			data[dataSize].ax = dof->calcAccel(dof->ax) - abias[0];
			data[dataSize].ay = dof->calcAccel(dof->ay) - abias[1];
			data[dataSize].az = dof->calcAccel(dof->az) - abias[2];
			data[dataSize].mx = dof->calcMag(dof->mx);
			data[dataSize].my = dof->calcMag(dof->my);
			data[dataSize].mz = dof->calcMag(dof->mz);
			MadgwickQuaternionUpdate();
			data[dataSize].w = q[0];
			data[dataSize].x = q[1];
			data[dataSize].y = q[2];
			data[dataSize].z = q[3];
			//Serial.println(data[dataSize].time);		
			
			dataSize++;
			newData = true;			
		}
		else{
/* 			Serial.print(millis());
			Serial.println(": Buffer is full!"); */			
			dataSize = 0;		
		}

    /* frame is 
     *  current time
     *  index
     *  Grav X, Y, Z
     *  Accel X, Y, Z
     *  Mag X, Y, Z
		 *  Quaternion W, X, Y, Z
     */

    return newData;
  }
  else{
    return false;  
	}
}
void ESP_LSM9DS0::setInterrupts(byte accelInt, byte magInt, byte gryoInt){
  m_INT1XM = accelInt;
  m_INT2XM = magInt;
  m_DRDYG = gryoInt;
  usingInterrupts = true;
// Set up interrupt pins as inputs:
  pinMode(m_INT1XM, INPUT);
  pinMode(m_INT2XM, INPUT);
  pinMode(m_DRDYG, INPUT);
}
void ESP_LSM9DS0::begin(){
  ESPSensor::begin();
  dof = new LSM9DS0(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
  // Use the begin() function to initialize the LSM9DS0 library.
  // You can either call it with no parameters (the easy way):
  // TODO determine proper scales
  uint16_t status = dof->begin(dof->G_SCALE_245DPS, dof->A_SCALE_2G, dof->M_SCALE_2GS,
															dof->G_ODR_95_BW_125, dof->A_ODR_100, dof->M_ODR_100);
  // Or call it with declarations for sensor scales and data rates:  
  //uint16_t status = dof.begin(dof.G_SCALE_2000DPS, 
  //                            dof.A_SCALE_6G, dof.M_SCALE_2GS);
 
 /*  	This is the default call:
	uint16_t begin(gyro_scale gScl = G_SCALE_245DPS, 
					accel_scale aScl = A_SCALE_2G, mag_scale mScl = M_SCALE_2GS,
					gyro_odr gODR = G_ODR_95_BW_125, accel_odr aODR = A_ODR_50, 
					mag_odr mODR = M_ODR_50); */
	// Use the FIFO mode to average accelerometer and gyro readings to calculate the biases, which can then be removed from
	// all subsequent measurements.
  dof->calLSM9DS0(gbias, abias);
	
	Serial.println();
  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println(); 
	
	
}

void ESP_LSM9DS0::send(){
  if(client.connected()){         
    if(!client.isSendWaiting()){      
      if(newData){   						
				imuStream[0] = HEADER_SYMBOL;
				imuStream[1] = HEADER_SYMBOL;
				imuStream[2] = HEADER_SYMBOL;
				imuStream[3] = HEADER_SYMBOL;
				//memcpy(imuStream+sizeof(char)*2,imuBuff.data,imuBuff.size*sizeof(IMUData));
				memcpy(imuStream+4,data,dataSize*sizeof(IMUData));
			  imuStream[HEADER_SIZE+dataSize*sizeof(IMUData)] = FOOTER_SYMBOL;
				imuStream[HEADER_SIZE+dataSize*sizeof(IMUData)+1] = FOOTER_SYMBOL;
				imuStream[HEADER_SIZE+dataSize*sizeof(IMUData)+2] = FOOTER_SYMBOL;
				imuStream[HEADER_SIZE+dataSize*sizeof(IMUData)+3] = FOOTER_SYMBOL;
				client.write((const uint8_t *)imuStream,sizeof(char)*8+dataSize*sizeof(IMUData));
					
				Serial.print("sent ");
				Serial.print(HEADER_SIZE+FOOTER_SIZE+dataSize*sizeof(IMUData));
				Serial.print(" bytes (");
				Serial.print(dataSize);
				Serial.println(" frames)");        
				memset(imuStream,0,MAX_IMU_BUFF_SIZE*sizeof(IMUData)+sizeof(char)*(HEADER_SIZE+FOOTER_SIZE));			
        newData = false;
        dataSize = 0;
      }      
    }
  }
  else{//reconnect
    newData = false;
		dataSize = 0;
    connectToBaseStation();    
  }

}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void ESP_LSM9DS0::MadgwickQuaternionUpdate()
{
		float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
		float ax = dof->ax;
		float ay = dof->ay;
		float az = dof->az;
		float gx = dof->gx;
		float gy = dof->gy;
		float gz = dof->gz;
		float mx = dof->mx;
		float my = dof->my;
		float mz = dof->mz;
		
		float norm;
		float hx, hy, _2bx, _2bz;
		float s1, s2, s3, s4;
		float qDot1, qDot2, qDot3, qDot4;

		// Auxiliary variables to avoid repeated arithmetic
		float _2q1mx;
		float _2q1my;
		float _2q1mz;
		float _2q2mx;
		float _4bx;
		float _4bz;
		float _2q1 = 2.0f * q1;
		float _2q2 = 2.0f * q2;
		float _2q3 = 2.0f * q3;
		float _2q4 = 2.0f * q4;
		float _2q1q3 = 2.0f * q1 * q3;
		float _2q3q4 = 2.0f * q3 * q4;
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f/norm;
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f/norm;
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		_2q1mx = 2.0f * q1 * mx;
		_2q1my = 2.0f * q1 * my;
		_2q1mz = 2.0f * q1 * mz;
		_2q2mx = 2.0f * q2 * mx;
		hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
		hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
		norm = 1.0f/norm;
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
		s4 *= norm;

		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

		// Integrate to yield quaternion
		q1 += qDot1 * deltat;
		q2 += qDot2 * deltat;
		q3 += qDot3 * deltat;
		q4 += qDot4 * deltat;
		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		norm = 1.0f/norm;
		q[0] = q1 * norm;
		q[1] = q2 * norm;
		q[2] = q3 * norm;
		q[3] = q4 * norm;

}
