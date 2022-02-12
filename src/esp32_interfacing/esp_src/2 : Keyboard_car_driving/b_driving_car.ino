


// Drive with teleop key board or cmd gui 

#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>



                                          ///// Initializing the functions that we will be using through the code 
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
float mapPwm(float x, float out_min, float out_max);

                                          ///// All pins defining
const uint8_t R_PWM =  12;
const uint8_t R_BACK = 14;
const uint8_t R_FORW = 27;

const uint8_t L_PWM =  33;
const uint8_t L_BACK = 25;
const uint8_t L_FORW = 26;
const uint8_t channel_L =0;
const uint8_t channel_R= 1;
const char*  ssid = "robotics_wifi";
const char*  password = "123456789";

bool _connected = false;


IPAddress server(192,168,43,99); // this is the ip of computer on which ROSCORE is running
const uint16_t serverPort = 11411; // ip supplied with port for communication

//// Setting up arduino ROS communication handle 
ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist); // keyboard input subscriber


                                          

void setup()
{
    Serial.begin(115200);
    setupPins();
    setupWiFi();
    node.getHardware()->setConnection(server); // connecting esp8266 module to ROSCORE
    node.initNode();
    node.subscribe(sub);// starting to subscribe the desired topic
}

                                           //// Difining Functionality of all pins

void setupPins()
{ /// calling this function from setup();
  const int freq = 5000;
  const int res = 8;
  
  pinMode(L_PWM,  OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_PWM,  OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  
  ledcSetup(channel_R ,freq , res);
  ledcSetup(channel_L ,freq , res);
  ledcAttachPin(R_PWM,channel_R);
  ledcAttachPin(L_PWM,channel_L);
  stop(); // making robot to not move
}

                                            //// Connecting esp to common ROSCORE wifi                               
void setupWiFi()
{  /// calling this function from setup();
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());

}
                                           //// Stop function to stop the robot movement
void stop()
{
 
   ledcWrite(channel_R, 0);  
    ledcWrite(channel_L, 0);
}

                                           //// Starting the commands dealing mathemetically

void onTwist(const geometry_msgs::Twist &msg)
{ 
  
  float x = max(min(msg.linear.x, 1.0f), -1.0f);// minimum value (-1) maximum value (1)
  float z = max(min(msg.angular.z, 1.0f), -1.0f);// minimum value (-1) maximum value (1)
  
                                          /// Transforming linear and angular velocities to speed for the LEFT RIGHT MOTORS
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;
                                          ///According to the speed derived we get pwm for motors
  uint16_t lPwm = mapPwm(fabs(l), 100, 200);
  uint16_t rPwm = mapPwm(fabs(r), 100, 200);
  
  digitalWrite(L_FORW, l > 0);
  digitalWrite(R_FORW, r > 0);
  
  digitalWrite(L_BACK, l < 0);
  digitalWrite(R_BACK, r < 0);
   ledcWrite(channel_R, rPwm);  
    ledcWrite(channel_L, lPwm);
  Serial.print(l);Serial.print(" / ");Serial.print(r);Serial.print("  ");Serial.print(lPwm);Serial.print(" / ");Serial.println(rPwm);
}



                                           //// Main loop just spins on the back subscribing node gets the work done
void loop()
{
  node.spinOnce();
  delay(10);
}


float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
