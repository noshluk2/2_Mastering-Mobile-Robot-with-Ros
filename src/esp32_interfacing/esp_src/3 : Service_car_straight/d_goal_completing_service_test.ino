/*
 *  * Send a Goal and test if it generates reponse
 */
#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <rosserial_arduino/Test.h>


void setupPins();
void setupWiFi();
void move_robot();
void robot_stop();
void reset_variables();
void distance_to_travel(int centimeters);


const uint8_t enc_R  = 5;
const uint8_t enc_L  = 18;
const uint8_t R_PWM  =  12;
const uint8_t R_BACK = 14;
const uint8_t R_FORW = 27;

const uint8_t L_PWM  = 33;
const uint8_t L_BACK = 25;
const uint8_t L_FORW = 26;
const uint8_t channel_L = 0;
const uint8_t channel_R = 1;
const char*  ssid = "robotics_wifi";
const char*  password = "123456789";
int count_R              = 0;                          //For Encoders
int count_L              = 0;  

int goal_distance =1;
int distance_traveled =10 ;
char buffer[12];
bool car_moving=true;
String print_variable="";
IPAddress server(192,168,43,99); // this is the ip of computer on which ROSCORE is running
const uint16_t serverPort = 11411; // ip supplied with port for communication

ros::NodeHandle  nh;
using rosserial_arduino::Test;

short one_rev_dist = 20.9 ;    
unsigned int ticks_for_360 = 2964 ;
short ticks_req = 0;
short rev_req = 0; 
unsigned int one_rev_ticks = 750 ; 

void distance_to_travel(int centimeters){
  reset_variables();                                // reseting variables so it may not count previous ticks into new task.'
  rev_req = centimeters / one_rev_dist;             // this will produced revolution required for specified distance to travel
  ticks_req = rev_req * one_rev_ticks ;             // this will produced ticks motor should do to complete required distance
  Serial.print("Starting_drive  ");Serial.println(ticks_req);
  delay(3000); 
  move_robot();                                        //Enable motors
  
  while(car_moving){
    Serial.print("In Loop  ");Serial.println(count_R);
    if(count_R>=ticks_req){
      robot_stop();
      Serial.println("Exiting while ");
      delay(2000);
      
                           } // so the car fully stop after inertia 
                         
                   }
  }


void robot_stop(){
  
  digitalWrite(L_FORW,LOW);
  digitalWrite(R_FORW,LOW);
  digitalWrite(L_FORW,LOW);
  digitalWrite(R_FORW,LOW);
  ledcWrite(channel_R , 0);                             // giving each motor 150 dutycycle resolution
  ledcWrite(channel_L , 0);
  car_moving=false;
}

void reset_variables(){
  ticks_req = 0 ;                                         // restting here because new counting should be started
  count_R =0;
}

void move_robot(){
  digitalWrite(L_FORW,HIGH);
  digitalWrite(R_FORW,HIGH);
  digitalWrite(R_BACK,LOW);
  digitalWrite(L_BACK,LOW);
  ledcWrite(channel_R , 200);                             // giving each motor 150 dutycycle resolution
  ledcWrite(channel_L , 200);
  car_moving=true;
}
void callback(const Test::Request & req, Test::Response & res){
  String value = req.input;
  goal_distance = value.toInt();
  distance_to_travel(goal_distance);
  res.output = "Goal Reached ";
}



ros::ServiceServer<Test::Request, Test::Response> server_("goal_setting_service",&callback);

void setup()
{ 
  
  Serial.begin(115200);
  setupPins();
  setupWiFi();
  
  nh.getHardware()->setConnection(server);
  nh.initNode();
  nh.advertiseService(server_);
}

void setupWiFi()
{ 
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());

}
void loop()
{
  
  nh.spinOnce();
  delay(1);
}


void setupPins(){
  const int freq = 5000;
  const int res = 8;
  
  pinMode(L_PWM,  OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_PWM,  OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  pinMode(enc_R,INPUT);
  pinMode(enc_L,INPUT);

  ledcSetup(channel_R ,freq , res);
  ledcSetup(channel_L ,freq , res);
  ledcAttachPin(R_PWM,channel_R);
  ledcAttachPin(L_PWM,channel_L);
  robot_stop();
  
  attachInterrupt(digitalPinToInterrupt(enc_R),Update_encR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_L),Update_encL,CHANGE);

}


void Update_encR(){
    count_R++;  
}

void Update_encL(){
   count_L++; 
} 
