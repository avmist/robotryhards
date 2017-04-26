#include "IMU.h"

IMU::IMU() {
  
}

void IMU::init() {

  // Adapted from SparkFun IMU example code
  
  // Call imu.begin() to verify communication and initialize
  if(imu.begin() != INV_SUCCESS) {
    
    while(true) {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
    
  }

  // Enable all sensors
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  //imu.setGyroFSR(250);

  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2);

  // DMP_FEATURE_GYRO_CAL - Enable gyro cal
  // DMP_FEATURE_SEND_CAL_GYRO - Send cal'd gyro values
  // Set DMP rate to 10 Hz
  //imu.dmpBegin(DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO, 200);
  imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_6X_LP_QUAT, 200);

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10 or 5 Hz
  //imu.setLPF(5);

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  //imu.setSampleRate(1000);

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  //imu.setCompassSampleRate(100);
  
}

void IMU::update() {

  // Adapted from SparkFun IMU example code
  
  // Check for new data in the FIFO
  if(imu.fifoAvailable()) {
    
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if(imu.dmpUpdateFifo() == INV_SUCCESS) {
      
      // Gyro
      gyro[0] = imu.calcGyro(imu.gx);
      gyro[1] = imu.calcGyro(imu.gy);
      gyro[2] = imu.calcGyro(imu.gz);

      // Accelerometer
      accel[0] = imu.calcAccel(imu.ax);
      accel[1] = imu.calcAccel(imu.ay);
      accel[2] = imu.calcAccel(imu.az);

      // Magnetometer
      mag[0] = imu.calcMag(imu.mx);
      mag[1] = imu.calcMag(imu.my);
      mag[2] = imu.calcMag(imu.mz);

      imu.computeEulerAngles();

      pitch = imu.pitch;
      yaw = imu.yaw;
      roll = imu.roll;

      heading = imu.computeCompassHeading();
      
    }
    
  }
  
}
