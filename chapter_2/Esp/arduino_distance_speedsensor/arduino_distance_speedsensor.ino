#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/Int16.h>

const uint8_t R_PWM =  D2;
const uint8_t R_BACK = D7;
const uint8_t R_FORW = D8;

const uint8_t L_PWM =  D3;
const uint8_t L_BACK = D5;
const uint8_t L_FORW = D6;

int r_enc=D1;
int interruptCounter = 0;
int total_ticks_to_goal;
int ticks_per_cm=4.5;



//------- doing maths 
//Circumference = pi * Diameter or 2*pi*r
//wheel_dia=65mm
//cir=204.2 (mm) or 20.42(cm) as we want our distance in cm
//revolution = (Distance to travel / circumference) 
// Distance to travel = revolution * circumference
/// Distance to travel = 20.42 cm ~ 21cm  (through 90 ticks)
///ticks_per_cm=4.28 -> 4.5



void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();

const char* ssid     = "paradise1";
const char* password = "112233445";

IPAddress server(192,168,10,2); // this is the ip of computer on which ROSCORE is running
const uint16_t serverPort = 11411; // ip supplied with port for communication
ros::NodeHandle node;
ros::Subscriber<std_msgs::Int16> sub("/distance", &move_bot_callback); 


void setup() {
  
   Serial.begin(115200);
   setupPins();
   setupWiFi();
   node.getHardware()->setConnection(server); // connecting esp8266 module to ROSCORE
   node.initNode();
   node.subscribe(sub);// starting to subscribe the desired topic

 
 }
 void setupPins()
{ /// calling this function from setup();
  pinMode(r_enc, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(r_enc), handleInterrupt, RISING);
  pinMode(L_PWM,  OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_PWM,  OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  stop(); // making robot to not move
}
void setupWiFi()
{  /// calling this function from setup();
   WiFi.begin(ssid, password);
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());

}
void go_forward(){
  Serial.println("Going_forward");
  digitalWrite(L_FORW, HIGH);
  digitalWrite(L_BACK, LOW);
  digitalWrite(R_FORW, HIGH);
  digitalWrite(R_BACK, LOW);
  analogWrite(L_PWM,   1000);
  analogWrite(R_PWM,   1000);
}
void stop()
{Serial.println("Stopping");
  digitalWrite(L_FORW, 0);
  digitalWrite(L_BACK, 0);
  digitalWrite(R_FORW, 0);
  digitalWrite(R_BACK, 0);
  analogWrite(L_PWM,   0);
  analogWrite(R_PWM,   0);
  interruptCounter=0;
  Serial.println(total_ticks_to_goal);
}
 
void loop() {
     151 > 150
  if(interruptCounter > total_ticks_to_goal){
    stop();
    interruptCounter=0;
  }
  node.spinOnce();
   delay(10);
 }

void move_bot_callback(const std_msgs::Int16 &msg)
{ 
  total_ticks_to_goal = ticks_per_cm *msg.data;
  
  go_forward();
  

}

 void handleInterrupt() {
  interruptCounter++;
  Serial.print("Window number : ");
  Serial.println(interruptCounter);
}
