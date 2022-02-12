/*
 * rosserial Service Server
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <rosserial_arduino/Test.h>

ros::NodeHandle  nh;
using rosserial_arduino::Test;

int i;
void callback(const Test::Request & req, Test::Response & res){
    res.output=req.input;
    // res.output = itoa(count_R,buf,10);  ## integer to char conversion
//    res.output = "hello";
//  else
//    delay(4000);
//    res.output = "world";
}

ros::ServiceServer<Test::Request, Test::Response> server("test_srv",&callback);

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertiseService(server);
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(10);
}