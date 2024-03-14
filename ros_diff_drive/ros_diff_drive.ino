#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include "Arduino_BMI270_BMM150.h"

// min amount of PWM the motors need to move
#define PWM_MIN 75
// PWM ragnes from 0 to 255
#define PWM_MAX 125
 
// encoder
#define ENCA_R 2 // RED
#define ENCB_R 3 // BLACK
#define ENCA_L 4 // RED
#define ENCB_L 5 // BLACK

volatile int posi_R = 0; // specify posi_R as volatile
volatile int posi_L = 0; // specify posi_L as volatile

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// right motors connections
int R_PWM = 7;
int R_1 = 8;
int R_2 = 9;

// left motors connections
int L_PWM = 12;
int L_1 = 10;
int L_2 = 11;

void velCallback(const geometry_msgs::Twist &msg){
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  float lvel = (x-z)/2;
  float rvel = (x+z)/2;

  uint16_t lpwm = mapPwm(fabs(lvel), PWM_MIN, PWM_MAX);
  uint16_t rpwm = mapPwm(fabs(rvel), PWM_MIN, PWM_MAX);

  digitalWrite(R_1, rvel > 0);
  digitalWrite(R_2, rvel < 0);
  digitalWrite(L_1, lvel > 0);
  digitalWrite(L_2, lvel < 0);

  analogWrite(R_PWM, rpwm);
  analogWrite(L_PWM, lpwm);
}

float mapPwm(float x, float out_min, float out_max){
  return x * (out_max - out_min) + out_min;
}

void readEncoder_R() {
  int b = digitalRead(ENCB_R);
  if (b > 0) {
    if (b == encoder_maximum){
      posi_R = encoder_minimum;
    } else{
      posi_R++;
    }
  } else {
    if (b == encoder_minimum){
      posi_R = encoder_maximum;
    } else{
      posi_R--;
    }
  }
}

void readEncoder_L() {
  int b = digitalRead(ENCB_L);
  if (b > 0) {
    if (b == encoder_minimum){
      posi_L = encoder_maximum;
    } else{
      posi_L--;
    }
  } else {
    if (b == encoder_maximum){
      posi_L = encoder_minimum;
    } else{
      posi_L++;
    }
  }
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", velCallback);

std_msgs::Int16 posR_msg;
std_msgs::Int16 posL_msg;
ros::Publisher posR_pub("pos_R", &posR_msg);
ros::Publisher posL_pub("pos_L", &posL_msg);

std_msgs::String imu_msg;
ros::Publisher imu_pub("imu_raw", &imu_msg);

void pub_odom(){
  noInterrupts();
  int pos_R = posi_R;
  int pos_L = posi_L;
  interrupts();
  
  posR_msg.data = pos_R;
  posR_pub.publish(&posR_msg);

  posL_msg.data = pos_L;
  posL_pub.publish(&posL_msg);
}

void pub_imu(){
  float ax,ay,az;
  float gx,gy,gz;
  float mx,my,mz;

  if (IMU.accelerationAvailable()){
    IMU.readAcceleration(ax,ay,az);
  }
  if (IMU.gyroscopeAvailable()){
    IMU.readGyroscope(gx,gy,gz);
  }
  if (IMU.magneticFieldAvailable()){
    IMU.readMagneticField(mx,my,mz);
  }

  // Calculate the length required for the string
  int required_length = snprintf(NULL, 0, "%f,%f,%f,%f,%f,%f,%f,%f,%f",
                                 gx, gy, gz, ax, ay, az, mx, my, mz);
  
  char imu_data[required_length + 1]; // Add 1 for null terminator
  
  // Format the string with the desired pattern
  sprintf(imu_data, "%f,%f,%f,%f,%f,%f,%f,%f,%f",
          gx, gy, gz, ax, ay, az, mx, my, mz);

  imu_msg.data = imu_data;
  
  imu_pub.publish(&imu_msg);
}

void setup() {
  // set all motor control pins to outputs
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_1, OUTPUT);
  pinMode(R_2, OUTPUT);
  pinMode(L_1, OUTPUT);
  pinMode(L_2, OUTPUT);

  // turn off motors - initial state
  digitalWrite(R_1, LOW);
  digitalWrite(R_2, LOW);
  digitalWrite(L_1, LOW);
  digitalWrite(L_2, LOW);

  // encoder
  pinMode(ENCA_R,INPUT);
  pinMode(ENCB_R,INPUT);
  pinMode(ENCA_L,INPUT);
  pinMode(ENCB_L,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_R),readEncoder_R,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_L),readEncoder_L,RISING);
  
  // imu
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    // stop here if you can't access the IMU:
    while (true);
  }

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(posR_pub);
  nh.advertise(posL_pub);
  nh.advertise(imu_pub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
 
  pub_odom();

  pub_imu();
  
  delay(10);
}
