/* 
 * rosserial Planar Odometry Example
 */

#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define enc_R 14
#define enc_L 27
int count_R              = 0;                          //For Encoders
int count_L              = 0;  
int goal_distance =1;
int distance_traveled =10 ;
const char*  ssid = "robotics_wifi";
const char*  password = "123456789";

IPAddress server(192,168,43,99); // this is the ip of computer on which ROSCORE is running
const uint16_t serverPort = 11411; // ip supplied with port for communication



ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/base_link";
char odom[] = "/odom";

void setup()
{
  Serial.begin(115200);
  setupWiFi();
  nh.getHardware()->setConnection(server);
  nh.initNode();
  broadcaster.init(nh);
}

void loop()
{  
  // drive in a circle
  // double dx = 0.2;
  // double dtheta = 0.18;
  // x += cos(theta)*dx*0.1;
  // y += sin(theta)*dx*0.1;
  // theta += dtheta*0.1;
  // if(theta > 3.14)
  //   theta=-3.14;
    
  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = count_R/100;
  t.transform.translation.y = count_L/100;

  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
  nh.spinOnce();
  
  delay(10);
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
void setup_motor_enc(){
  
  pinMode(enc_R,INPUT);
  pinMode(enc_L,INPUT);
  attachInterrupt(digitalPinToInterrupt(enc_R),Update_encR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_L),Update_encL,CHANGE);

}

void Update_encR(){
    count_R++;  
}

void Update_encL(){
   count_L++; 
} 
