#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "Kalman.h"

#include <SparkFunMPU9250-DMP.h>

class IMU {

public:

	float pitch, yaw, roll, heading;
	float gyro[3];
	float accel[3];
	float mag[3];

private:

	MPU9250_DMP imu;
  
public:
  
	// Constructors
	IMU();

	// Statics

	// Methods
	void init();
	void update();

private:
  
};

#endif // IMU_H
