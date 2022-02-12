/*
 This sketch shows to perform vector products and rotate heading (yaw angle) of the estimated orientation.
 */

#include <imuFilter.h>
#include <basicMPU6050.h>       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include "Wire.h" // This library allows you to communicate with I2C devices.

#define SMALL_ANG false 

// Sensor fusion
constexpr float GAIN = 0.1;     // Fusion gain, value between 0 and 1 - Determines response of heading correction with respect to gravity.
imuFilter <&GAIN> fusion;

// Imu sensor
basicMPU6050<> imu;

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

// raw data values
int16_t og_accelerometer_x, og_accelerometer_y, og_accelerometer_z; //variables for displacement
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t og_gyro_x, og_gyro_y, og_gyro_z; // variables for gyro raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function
float tmp_vec[3]; //temp array to hold last vector of movement

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

// Display function:
void printVector( float r[] ) {
  Serial.print( r[0], 2 );
  Serial.print( "," );
  Serial.print( r[1], 2 );
  Serial.print( "," );
  Serial.print( r[2], 2 );
  Serial.println();
}

void setup() {
  // get vals
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //get wire values
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  og_accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  og_accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  og_accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  
  // Initialize filter: 
  fusion.setup( og_accelerometer_x, og_accelerometer_y, og_accelerometer_z );     

  // Calibrate imu
  imu.setup();
  imu.setBias();

  Serial.begin(9600);

  // Vector operations:
  float v1[3] = { 3, 1, -1 };                    // Input Vector 
  float axis_y[3], axis_z[3];
  
  fusion.getYaxis( true, axis_y );              // Vectors to operate on [global axes] 
  fusion.getZaxis( true, axis_z );              

  fusion.crossProduct( v1, axis_y );            // Cross product: V = V cross R ; Output is stored in V 
  float v2[3] = { v1[0], v1[1], v1[2] };         // Store product
  
  fusion.normalizeVector( v1 );                 // Norm: V = V/|V| ; Output is stored in V 

  // stre V in temp
  for(int i=0;i<3;i++)
  {
    tmp_vec[i] = v1[i];
  }
  
  
  float dot = fusion.dotProduct( v1, axis_z );  // Dot product: Input order does not matter   
                 
  // Rotate heading: 
                          // Small angle approximation = true 
                          // Exact angle rotation = false 
  fusion.rotateHeading( SMALL_ANG, dot );

  // Display results:
  Serial.print( "y = " ); 
  printVector( axis_y );
  Serial.print( "v2 = " ); 
  printVector( v2 );
  Serial.print( "v1 = " ); 
  printVector( v1 );
  Serial.print( "dot = ");
  Serial.println( dot );

  // Wait for output to be read
  delay(10000);
}

void loop() {  

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  // Update filter:
  fusion.update( gyro_x, gyro_y, gyro_z, accelerometer_x, accelerometer_y, accelerometer_z );    

  // Vector operations:
  float v1[3];
  for(int i=0;i<3;i++)
  {
    v1[i] = tmp_vec[i];
  }
  float axis_y[3], axis_z[3];
  
  fusion.getYaxis( true, axis_y );              // Vectors to operate on [global axes] 
  fusion.getZaxis( true, axis_z );              

  fusion.crossProduct( v1, axis_y );            // Cross product: V = V cross R ; Output is stored in V 
  float v2[] = { v1[0], v1[1], v1[2] };         // Store product
  
  fusion.normalizeVector( v1 );                 // Norm: V = V/|V| ; Output is stored in V 
  
  float dot = fusion.dotProduct( v1, axis_z );  // Dot product: Input order does not matter   
                 
  // Rotate heading: 
                          // Small angle approximation = true 
                          // Exact angle rotation = false 
  fusion.rotateHeading( SMALL_ANG, dot );

  // Display results:
  Serial.print( "y = " ); 
  printVector( axis_y );
  Serial.print( "v2 = " ); 
  printVector( v2 );
  Serial.print( "v1 = " ); 
  printVector( v1 );
  Serial.print( "dot = ");
  Serial.println( dot );

  // Display angles:
  Serial.print( fusion.pitch() );
  Serial.print( " " );
  Serial.print( fusion.yaw() );
  Serial.print( " " );
  Serial.print( fusion.roll() );
  Serial.println();
}
