/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int16.h>

const int trig_Pin = 14;
const int echo_Pin = 12;

#define SPEED_OF_SOUND 0.034

long duration;
float dist_in_cm;

ros::NodeHandle  nh;

std_msgs::Int16 ultrasonic_values_msg;
ros::Publisher ultrasonic_values_node("ultrasonic_Values_Topic", &ultrasonic_values_msg);

void setup()
{ 
  pinMode(trig_Pin, OUTPUT); 
  pinMode(echo_Pin, INPUT);
  nh.initNode();
  nh.advertise(ultrasonic_values_node);
}

void loop()
{ 
  digitalWrite(trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_Pin, LOW);
  duration = pulseIn(echo_Pin, HIGH);
  dist_in_cm = duration * SPEED_OF_SOUND/2;

  ultrasonic_values_node.publish( &ultrasonic_values_msg );
  nh.spinOnce();
  delay(100);
}


