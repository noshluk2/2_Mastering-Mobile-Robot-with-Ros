                                          ///// Setting up all the libraries


#include <ros.h>
#include <std_msgs/Int16.h>


void setup_motor_enc();


#define enc_R 14
#define enc_L 27

int count_R              = 0;                          //For Encoders
int count_L              = 0;  

ros::NodeHandle node;

std_msgs::Int16 left_enc_msg;
std_msgs::Int16 right_enc_msg;
ros::Publisher left_enc("left_encoder_ticks", &left_enc_msg);
ros::Publisher right_enc("right_encoder_ticks", &right_enc_msg);


                                
void setup()
{
    setup_motor_enc();
    node.initNode();
    node.advertise(left_enc);
    node.advertise(right_enc);
}

void setup_motor_enc(){
  
  pinMode(enc_R,INPUT);
  pinMode(enc_L,INPUT);
  attachInterrupt(digitalPinToInterrupt(enc_R),Update_encR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_L),Update_encL,CHANGE);

}

void Update_encR(){
    right_enc_msg.data=count_R++;  
}

void Update_encL(){
    left_enc_msg.data=count_L++; 
}              


void loop()
{
  right_enc.publish(&right_enc_msg);
  left_enc.publish(&left_enc_msg);
  node.spinOnce();
  delay(1);
}