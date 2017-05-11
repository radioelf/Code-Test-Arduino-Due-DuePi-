// Test MPU9250 (SPI)->DuePi
// MPU9250 Giroscopio/Acelerómetro/Compás interno/9-AXIS (Bus SPI, max. 20Mhz) 
/* ******************************************************************* 
 * Based https://github.com/kriswiner/MPU-9250 and https://github.com/brianc118/MPU9250
 *  
  Radioelf - May 2017
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2017 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and  MPU9250.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

// MPU9250  Magnetic flux density [μT] Min.-4800  Max.4800 (-48,00 a 48,00 Gauss)
#include <SPI.h>                                                      // SPI para sensores
#include <MPU9250.h>                                                  // https://github.com/brianc118/MPU9250 (mod. use 2G and 250GPS)

#if defined(ARDUINO_SAM_DUE)
  #define duepi "TEST MPU9250"
#else
  #error  !!ERROR board Due!! 
#endif

//#define debug
#define SerialPlotter

 
#define ENABL_PIN 31                                                  // PA7  PIN SAM3X 26,  DronPi PIN-> 1,  Max. output current 15mA
#define CS_MPU_PIN 87                                                 // PA29 PIN SAM3X 112, DronPi PIN-> 26, Max. output current 15mA
#define INT_PIN 22                                                    // PB26 PIN SAM3X 1,   DronPi PIN-> 16, Max. output current 3mA!!
#define SPI_CLOCK 1000000                                             // 1MHz clock para bus SPI
//MPU9250 mpu(SPI_CLOCK, CS_MPU_PIN);                                 // 1Mhz, pin 87->PA29, low pass filter 184hz, low_pass_filter_acc 184hz
MPU9250 mpu(SPI_CLOCK, CS_MPU_PIN, BITS_DLPF_CFG_42HZ, BITS_DLPF_CFG_42HZ); // 8Mhz, pin 87->PA29, low pass filter 42hz, low_pass_filter_acc 42hz

#define Kp 2.0f * 5.0f                                                // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);                          // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);                          // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;                       // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;                       // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};                                // vector to hold quaternion
float ax, ay, az, gx, gy, gz, mx, my, mz;                             // variables to hold latest sensor data values 
float magbias[3] = {0.0f , 0.0f, 0.0f};
uint32_t lastUpdate = 0;                                              // used to calculate integration interval
uint32_t  sumCount = 0;                                               // used to control display output rate
float pitch, yaw, roll;
int pitch_ =0, yaw_ =0, roll_ =0, _pitch =0, _yaw =0, _roll =0;
float deltat = 0.0f, sum = 0.0f;                                      // integration interval for both filter schemes
float eInt[3] = {0.0f, 0.0f, 0.0f};                                   // vector to hold integral error for Mahony method
                                        
int8_t wai =0;
uint32_t Now = 0;                                                     // used to calculate integration interval

void setup() {
    
  pinMode(ENABL_PIN, OUTPUT);
  digitalWrite(ENABL_PIN, HIGH);                                       // Enable board DronPi
  pinMode(CS_MPU_PIN, OUTPUT);
  digitalWrite(CS_MPU_PIN, HIGH);
  Serial.begin(115200);
  delay(150);
  pinMode(INT_PIN, INPUT);
  SPI.begin();
  #if !defined (SerialPlotter)
    delay(100);
    Serial.println(duepi);
  #endif
  if (initializeMPU9250()){
    delay(2500);
  }else{
    while (true){
      delay(250);
        #ifdef SerialPlotter
          Serial.println(pitch_);
        #endif
    }
  }
  magbias[0] = +470.0;                                                // User environmental x-axis correction in milliGauss, should be automatically calculated
  magbias[1] = +120.0;                                                // User environmental x-axis correction in milliGauss
  magbias[2] = +125.0;                                                // User environmental x-axis correction in milliGauss
  
  mpu.read_all();
  delay(100);
  mpu.read_all();                                                     // We despise first reading
}

void loop() {  
  ReadMPU9250(1);                                                     // We get the accelerometer readings MPU9250
  #if !defined (SerialPlotter)
    delay(33);                                                        // 10ms-> 100Hz
  #endif
  ReadMPU9250(2);                                                     // We get the compass readings MPU9250  
  #if !defined (SerialPlotter)                                                
    delay(33);
  #endif
  ReadMPU9250(3);                                                     // We get the gyroscope readings MPU9250 
  #if !defined (SerialPlotter)                                                  
    delay(33); 
  #endif                                                   
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f);                          // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  sum += deltat;                                                     // sum for averaging filter update rate
  ReadMPU9250(4);
  #ifdef debug
    Serial.println("****************************************************");
  #endif
}

boolean  initializeMPU9250() {
  mpu.init(true);
  delay(10);
  uint8_t wai = mpu.whoami();
  if (wai == 0x71){
    #if !defined (SerialPlotter)
      Serial.println("TEST: MPU9250 SPI OK");
    #endif
    mpu.calib_acc();
    mpu.calib_mag();
    return true;
  }
  else{
    #if !defined (SerialPlotter)
      Serial.print("TEST: Error  MPU9250 0x");
      Serial.println(wai, HEX);
    #endif  
    return false;
  }
}
void ReadMPU9250(byte readTo){
  switch (readTo){
    case 1:
      mpu.read_acc();
        #ifdef debug
          Serial.print("RX rawData aaccelerometer: ");
          Serial.print(mpu.accel_data_raw[0]);  Serial.print('\t');
          Serial.print(mpu.accel_data_raw[1]);  Serial.print('\t');
          Serial.println(mpu.accel_data_raw[2]); 
        #endif
        // Now we'll calculate the accleration value into actual g's 
        // Set accelerometer full-scale to 2 g, 2.0/32768.0 = 0,00006103515625
        ax = (float)mpu.accel_data_raw[0]*0.00006103515625f;                          // get actual g value, this depends on scale being set
        ay = (float)mpu.accel_data_raw[1]*0.00006103515625f;    
        az = (float)mpu.accel_data_raw[2]*0.00006103515625f;   
        #ifdef debug
          Serial.print("ax = "); Serial.print((int)1000*ax);  
          Serial.print(" ay = "); Serial.print((int)1000*ay); 
          Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg"); Serial.println();
       #endif
    break;
    case 2:
      mpu.read_mag();
      #ifdef debug
        Serial.print("RX rawData compass: ");
        Serial.print(mpu.mag_data_raw[0]);    Serial.print('\t');
        Serial.print(mpu.mag_data_raw[1]);    Serial.print('\t');
        Serial.println(mpu.mag_data_raw[2]);  
      #endif

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental corrections
      // 16bits, 0.15 mG per LSB -> 10.0*4912.0/32760.0 = 1,499389499389499
      mx = (float)mpu.mag_data_raw[0] *1.4993895f *mpu.Magnetometer_ASA[0] - magbias[0];// get actual magnetometer value, this depends on scale being set
      my = (float)mpu.mag_data_raw[1] *1.4993895f *mpu.Magnetometer_ASA[1] - magbias[1];  
      mz = (float)mpu.mag_data_raw[2] *1.4993895f *mpu.Magnetometer_ASA[2] - magbias[2];   
      #ifdef debug
        Serial.print("mx = "); Serial.print( (int)mx ); 
        Serial.print(" my = "); Serial.print( (int)my ); 
        Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG"); Serial.println(); 
      #endif  
    break;
    case 3:
      mpu.read_gyro();
      #ifdef debug
        Serial.print("RX rawData gyroscope: ");
        Serial.print(mpu.gyro_data_raw[0]);   Serial.print('\t');
        Serial.print(mpu.gyro_data_raw[1]);   Serial.print('\t');
        Serial.println(mpu.gyro_data_raw[2]); 
      #endif
      // Calculate the gyro value into actual degrees per second
      // 250DPS-> 250.0/32768.0 = 0,00762939453125
      gx = (float)mpu.gyro_data_raw[0]*0.00763f;                                       // get actual gyro value, this depends on scale being set
      gy = (float)mpu.gyro_data_raw[1]*0.00763f;  
      gz = (float)mpu.gyro_data_raw[2]*0.00763f;  
      #ifdef debug
        Serial.print("gx = "); Serial.print( gx, 2); 
        Serial.print(" gy = "); Serial.print( gy, 2); 
        Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s"); Serial.println();
      #endif
    break;
    case 4:
      // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
      // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
      // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
      // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
      // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
      // This is ok by aircraft orientation standards!  
      // Pass gyro rate as rad/s
      //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
      MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
      #ifdef debug
        Serial.print("q0 = "); Serial.print(q[0]);
        Serial.print(" qx = "); Serial.print(q[1]); 
        Serial.print(" qy = "); Serial.print(q[2]); 
        Serial.print(" qz = "); Serial.println(q[3]); 
      #endif
      // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
      // In this coordinate system, the positive z-axis is down toward Earth. 
      // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
      // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
      // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
      // applied in the correct order which for this configuration is yaw, pitch, and then roll.
      // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI; 
      yaw   -= 0.59;                                                                          // Declination 
      roll  *= 180.0f / PI;
      #ifdef SerialPlotter
        Serial.print(yaw, 2); Serial.print(" "); Serial.print(pitch, 2); Serial.print(" "); Serial.println(roll, 2);
      #else
        #ifdef debug
          Serial.print("Yaw, Pitch, Roll: ");  
          yaw_ = _yaw+1;
        #else
          yaw_ = int (yaw);
          pitch_ = int (pitch);
          roll_ = int (roll); 
        #endif 
          if (yaw_ != _yaw || pitch_ != _pitch || roll_ != _roll){ 
          Serial.print(yaw, 2); Serial.print(", "); Serial.print(pitch, 2); Serial.print(", "); Serial.println(roll, 2);
          _yaw = yaw_;
          _pitch = pitch_;
          _roll = roll_;
        }
      Serial.print("rate = "); Serial.print((float)sumCount / sum, 2); Serial.println(" Hz");
      #endif
      sum = 0; 
      sumCount = 0;
    break;
    default:
      mpu.read_all();
      Serial.print("RX all rawData MPU:       ");
      Serial.print(mpu.gyro_data_raw[0]);   Serial.print('\t');
      Serial.print(mpu.gyro_data_raw[1]);   Serial.print('\t');
      Serial.print(mpu.gyro_data_raw[2]);   Serial.print('\t');
      Serial.print(mpu.accel_data_raw[0]);  Serial.print('\t');
      Serial.print(mpu.accel_data_raw[1]);  Serial.print('\t');
      Serial.print(mpu.accel_data_raw[2]);  Serial.print('\t');
      Serial.print(mpu.mag_data_raw[0]);    Serial.print('\t');
      Serial.print(mpu.mag_data_raw[1]);    Serial.print('\t');
      Serial.print(mpu.mag_data_raw[2]);    Serial.print('\t');
      Serial.println(mpu.temperature);      Serial.println();  
    break;
  }
}
//*****************************************************************************************************************
// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];                         // short name local variable for readability
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
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
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
  norm = 1.0f / norm;
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
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}

// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
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
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;
  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;
 
  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;


}
