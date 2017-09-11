// Problem: 
// 1. Don't know why acc_z raw data is positive. I thought it might be the data type, but it still not working. 
//    Thus, I changed all raw data into opposite direction; now it will provide correct X and Y angle, but opposite Z angle
// 2. Don't really know what's difference between different baud rate. I tried 115200 and 57600, basically no difference.

// What this program can do:
// 1. It will provide good enough euler rotation angles.
// 2. Yaw drift is not significant.


// Open source:
// I read this article and write code based on it:
// http://iopscience.iop.org/article/10.1088/0957-0233/26/12/125102#mstaa038deqn002







#include <MatrixMath.h>
#include <Wire.h>

float gyro_w_x, gyro_w_y, gyro_w_z; // angluar velocity measured in body fixed coordinate 
float gyro_w_x_cal, gyro_w_y_cal, gyro_w_z_cal; // angluar velocity offset in body fixed coordinate 
float acc_x, acc_y, acc_z; // acceleration measured in body fixed coordinate 
float acc_abs; // magnitude of acceleration vector
float temperature; // temperature measured by MPU 6050
float dt; // time interval
float angle_roll_previous, angle_pitch_previous, angle_yaw_previous; // previous euler angles: first rotate about z by angle_yaw, then rotate about x by anlge_pitch, finally rotate about y by angle_roll
float angle_roll_current, angle_pitch_current, angle_yaw_current; // previous euler angles
float angle_roll_acc, angle_pitch_acc, angle_yaw_acc; // euler angles calculated by acceleration vector
float W_current[3][1], R_inv_previous[3][3]; // [angle_roll_dot; angle_pitch_dot; angle_yaw_dot]; euler transform matrix;
long int previousTime, currentTime; // timer 

void setup() {
  Wire.begin();
  Serial.begin(115200); // Don't really know what's difference between different baud rate

  setup_mpu_6050();
  calibrate_mpu_6050();

  previousTime = 0;
}





void loop() {
  read_mpu_6050();
  currentTime = micros(); // get time value right after read the data from MPU-6050

  dt = float (currentTime - previousTime)/1000000; // time interval
  
  //Serial.print("dt: "); // use when need to test if dt is right
  //Serial.print(dt,6); Serial.print(" \t ");
  
  complimentary_filter();

  Serial.print("angle_roll_current: ");
  Serial.print(angle_roll_current);
  Serial.print(" \t angle_pitch_current: ");
  Serial.print(angle_pitch_current);
  Serial.print(" \t angle_yaw_current: ");
  Serial.println(angle_yaw_current);

  previousTime = currentTime; // reset timer

}






void setup_mpu_6050(){
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Register 107 – Power Management 1
  Wire.write(0x00); // (0x00000000)
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Register 28 - Accelerometer Configuration
  Wire.write(0x10); // ± 2g (0x00000000)
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Register 27 - Gyroscope Configuration
  Wire.write(0x08); // ± 500 °/s (- 0x00001000)
  Wire.endTransmission();
}






void calibrate_mpu_6050(){
  Serial.println("Start calibrating...");
  for(int i = 0; i < 2000; i++){
    read_mpu_6050();
    gyro_w_x_cal += gyro_w_x;
    gyro_w_y_cal += gyro_w_y;
    gyro_w_z_cal += gyro_w_z;
    
  }
  gyro_w_x_cal /= 2000; // use first 2000 data to calculate the angular offsets of gyroscope
  gyro_w_y_cal /= 2000;
  gyro_w_z_cal /= 2000;
  
  
  angle_roll_previous = (atan2(acc_x,-acc_z)*180/PI); // use acceleration vector to find the euler angle
  angle_pitch_previous = asin(-acc_y/sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z))*180/PI;
  angle_yaw_previous = 0;
  
  //Serial.println(acc_x); // use only when need to test acc_x acc_z and angle_roll_previous
  //Serial.println(acc_z);
  //Serial.println(angle_roll_previous);
  
  float R_inv_previous[3][3] = { // inverse rotation matrix, transfer back to world coordinate from body fixed coordinate
    {sin(angle_roll_previous*PI/180)*tan(angle_pitch_previous*PI/180), 1, -cos(angle_roll_previous*PI/180)*tan(angle_pitch_previous*PI/180)},
    {cos(angle_roll_previous*PI/180), 0, sin(angle_roll_previous*PI/180)},
    {sin(angle_roll_previous*PI/180)/cos(angle_pitch_previous*PI/180), 0, -cos(angle_roll_previous*PI/180)}
    }; 
    
  //Matrix.Print((float*)R_inv_previous, 3, 3, "R_inv_previous");

  Serial.println("Done calibrating.");
}






void read_mpu_6050(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start from ACCEL_XOUT_H
  Wire.endTransmission();

  Wire.requestFrom(0x68,14); // Read 14 data start from ACCEL_XOUT_H
  while(Wire.available() < 14); // raw data from mpu 6050
    long acc_x_m = (Wire.read() << 8) | Wire.read(); 
    long acc_y_m = (Wire.read() << 8) | Wire.read(); 
    long acc_z_m = (Wire.read() << 8) | Wire.read(); 
    temperature =  (Wire.read() << 8) | Wire.read(); // I didn't deactive temperature
    long gyro_w_x_m = (Wire.read() << 8) | Wire.read(); 
    long gyro_w_y_m = (Wire.read() << 8) | Wire.read(); 
    long gyro_w_z_m = (Wire.read() << 8) | Wire.read(); 

  gyro_w_x = (float)gyro_w_x_m/65.5; // sensitivity datasheet pg.12
  gyro_w_y = (float)gyro_w_y_m/65.5;
  gyro_w_z = (float)gyro_w_z_m/65.5;
  
  acc_abs = sqrt((float) acc_x_m*acc_x_m + acc_y_m*acc_y_m + acc_z_m*acc_z_m); // calculating the magnitude of acceleration vector
  acc_x = -acc_x_m/acc_abs; // PROBLEM: don't know why acc_z is positive, in order to be consistent, I set all value into negative
  acc_y = -acc_y_m/acc_abs;
  acc_z = -acc_z_m/acc_abs;
  
}








void complimentary_filter(){
  float R_inv_previous[3][3] = { 
    {sin(angle_roll_previous*PI/180)*tan(angle_pitch_previous*PI/180), 1, -cos(angle_roll_previous*PI/180)*tan(angle_pitch_previous*PI/180)},
    {cos(angle_roll_previous*PI/180), 0, sin(angle_roll_previous*PI/180)},
    {sin(angle_roll_previous*PI/180)/cos(angle_pitch_previous*PI/180), 0, -cos(angle_roll_previous*PI/180)}
    };
  
  float w_current[3][1] = { // calculate net angular velocity
    {gyro_w_x - gyro_w_x_cal},
    {gyro_w_y - gyro_w_y_cal}, 
    {gyro_w_z - gyro_w_z_cal}
    };

  //Matrix.Print((float*)w_current, 3, 1, "w_current"); // use only when need to test the matrix
  //Matrix.Print((float*)R_inv_previous, 3, 3, "R_inv_previous");
  
  Matrix.Multiply((float*)R_inv_previous, (float*)w_current, 3, 3, 1, (float*)W_current); // W_current = R_inv_previous*w_current

  //Matrix.Print((float*)W_current, 3, 1, "W_current");
  
  angle_roll_current = angle_roll_previous + W_current[0][0]*dt; // calculate current euler angle
  angle_pitch_current = angle_pitch_previous + W_current[1][0]*dt;
  angle_yaw_current = angle_yaw_previous + W_current[2][0]*dt;

  angle_roll_acc = (atan2(acc_x,-acc_z)*180/PI); 
  angle_pitch_acc = asin(-acc_y/sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z))*180/PI;
  
  angle_roll_current = 0.98*angle_roll_current + 0.02*(angle_roll_acc); // complimentary filter
  angle_pitch_current = 0.98*angle_pitch_current + 0.02*(angle_pitch_acc);
  angle_yaw_current = angle_yaw_current;

  angle_roll_previous = angle_roll_current; // update euler angles
  angle_pitch_previous = angle_pitch_current;
  angle_yaw_previous = angle_yaw_current;
}

