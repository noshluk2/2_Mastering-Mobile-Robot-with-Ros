#include <math.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ros.h>
#include <Wire.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

#define PWMRANGEs 1020
#define PWM_MIN 300

void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
float mapPwm(float x, float out_min, float out_max);



const uint8_t R_PWM =  D4;
const uint8_t R_BACK = D7;
const uint8_t R_FORW = D8;
const uint8_t L_BACK = D5;
const uint8_t L_FORW = D6;
const uint8_t L_PWM =  D3;
int r_enc=D0;
int l_enc=A0;
int r_interruptCounter = 0;
int l_interruptCounter = 0;

const uint8_t scl = D1;
const uint8_t sda = D2;
const uint8_t MPU6050SlaveAddress = 0x68;
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;



const char* ssid     = "LuqmanA_D";
const char* password = "qwertyuiop";

IPAddress server(192,168,43,225);
const uint16_t serverPort = 11411;

ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);
std_msgs::Int16 lwheel_msg;
std_msgs::Int16 rwheel_msg;
ros::Publisher lwheel_pub("lwheel", &lwheel_msg);
ros::Publisher rwheel_pub("rwheel", &rwheel_msg);

sensor_msgs::Imu msg;
ros::Publisher IMU("IMU_data", &msg);


bool _connected = false;

void setup()
{
  
  setupPins();
  Serial.begin(115200);
  setupWiFi();
  Wire.begin(sda, scl);
  node.getHardware()->setConnection(server);
  node.initNode();
  node.subscribe(sub);
  node.advertise(IMU);
  node.advertise(lwheel_pub);
  node.advertise(rwheel_pub);
  MPU6050_Init();
}

void setupPins()
{ 
  pinMode(l_enc, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(r_enc), r_handleInterrupt, RISING);
  pinMode(l_enc, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(l_enc), l_handleInterrupt, RISING); 
  pinMode(L_PWM,  OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_PWM,  OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  stop();
}

void setupWiFi()
{
   WiFi.begin(ssid, password);
   Serial.println("Connecting to Wifi");
   while (WiFi.status() != WL_CONNECTED) {
     Serial.print('.');
     delay(500);
   }

   Serial.println("\nWiFi connected");
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());

}

void stop()
{
  digitalWrite(L_FORW, 0);
  digitalWrite(L_BACK, 0);
  digitalWrite(R_FORW, 0);
  digitalWrite(R_BACK, 0);
  analogWrite(L_PWM,   0);
  analogWrite(R_PWM,   0);
}

void onTwist(const geometry_msgs::Twist &msg)
{ 

  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);
  
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  uint16_t lPwm = mapPwm(fabs(l), 450, 1050);
  uint16_t rPwm = mapPwm(fabs(r), 450, 1050);
  Serial.print("Pwms : ");
  Serial.print(lPwm);
  Serial.println(rPwm);
  digitalWrite(L_FORW, l > 0);
  digitalWrite(L_BACK, l < 0);
  digitalWrite(R_FORW, r > 0);
  digitalWrite(R_BACK, r < 0);
  analogWrite(L_PWM, lPwm);
  analogWrite(R_PWM, rPwm);
}

void loop()
{ double Ax, Ay, Az, T, Gx, Gy, Gz;
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;
  
  lwheel_msg.data = l_interruptCounter;
  rwheel_msg.data = r_interruptCounter;

  lwheel_pub.publish(&lwheel_msg);
  rwheel_pub.publish(&rwheel_msg);
  msg.angular_velocity.x=Gx;
  msg.angular_velocity.y=Gy;
  msg.angular_velocity.z=Gz;

  msg.linear_acceleration.x=Ax;
  msg.linear_acceleration.y=Ay;
  msg.linear_acceleration.z=Az;
  msg.header.frame_id="IMU_l";
  IMU.publish(&msg);
  node.spinOnce();
  delay(20);
}


float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

void r_handleInterrupt() {
  r_interruptCounter++;
}

void l_handleInterrupt() {
  l_interruptCounter++;
}


void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
