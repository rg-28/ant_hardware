#include<ros.h>
#include<std_msgs/Float32.h>
#include<TimerOne.h>
#include<geometry_msgs/Twist.h>

ros::NodeHandle nh;

#define LOOP_TIME 200000
#define left_encoder_pin 2
#define right_encoder_pin 3

unsigned int counter_left=0;
unsigned int counter_right=0;
float L=0.17;
float radius=0.04;

std_msgs::Float32 left_wheel_vel;
ros::Publisher left_wheel_vel_pub("/left_wheel_velocity", &left_wheel_vel);

std_msgs::Float32 right_wheel_vel;
ros::Publisher right_wheel_vel_pub("/right_wheel_velocity", &right_wheel_vel);

geometry_msgs::Twist sensor_vel;
ros::Publisher sensor_vel_pub("/sensor_velocity", &sensor_vel);

void docount_left()
{
  counter_left++;
}

void docount_right()
{
  counter_right++;
}

void timerISR()
{
  Timer1.detachInterrupt();
  left_wheel_vel.data=counter_left;
  right_wheel_vel.data=counter_right;
  sensor_vel.linear.x = radius*(left_wheel_vel.data + right_wheel_vel.data)/2;
  sensor_vel.linear.y = 0;
  sensor_vel.linear.z = 0;
  sensor_vel.angular.x = 0;
  sensor_vel.angular.y = 0;
  sensor_vel.angular.z = radius*(left_wheel_vel.data + right_wheel_vel.data)/L;
  //left_wheel_vel_pub.publish(&left_wheel_vel);
  //right_wheel_vel_pub.publish(&right_wheel_vel);
  sensor_vel_pub.publish(&sensor_vel);
  counter_left=0;
  counter_right=0;
  Timer1.attachInterrupt(timerISR);
}

void setup()
{
  pinMode(left_encoder_pin, INPUT_PULLUP);
  pinMode(right_encoder_pin, INPUT_PULLUP);
  Timer1.initialize(LOOP_TIME);
  attachInterrupt(2,docount_left, CHANGE);
  attachInterrupt(3,docount_right, CHANGE);
  
  nh.initNode();
  //nh.advertise(left_wheel_vel_pub);
  //nh.advertise(right_wheel_vel_pub);
  nh.advertise(sensor_vel_pub);
  Timer1.attachInterrupt(timerISR);
}

void loop()
{
  nh.spinOnce();
}
