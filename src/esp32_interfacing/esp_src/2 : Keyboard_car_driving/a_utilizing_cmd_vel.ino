                                          ///// Setting up all the libraries


#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

  
void setupWiFi();
void onTwist(const geometry_msgs::Twist &msg);
float mapPwm(float x, float out_min, float out_max);

const char*  ssid = "robotics_wifi";
const char*  password = "123456789";

IPAddress server(192,168,43,99); 
const uint16_t serverPort = 11411; 
ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist); 

void setup()
{
    Serial.begin(115200);
    
    setupWiFi();
    node.getHardware()->setConnection(server); // connecting esp8266 module to ROSCORE
    node.initNode();
    node.subscribe(sub);// starting to subscribe the desired topic
}
                        
void setupWiFi()
{  /// calling this function from setup();
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());
}
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
void onTwist(const geometry_msgs::Twist &msg)
{ 
  
  float x = max(min(msg.linear.x, 1.0f), -1.0f);// minimum value (-1) maximum value (1)
  float z = max(min(msg.angular.z, 1.0f), -1.0f);
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;
  uint16_t lPwm = mapPwm(fabs(l), 450, 1050);
  uint16_t rPwm = mapPwm(fabs(r), 450, 1050);
  Serial.print(l);Serial.print(" / ");Serial.print(r);Serial.print("  ");Serial.print(lPwm);Serial.print(" / ");Serial.println(rPwm);
}

void loop()
{
  node.spinOnce();
  delay(10);
}



