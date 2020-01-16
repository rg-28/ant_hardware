#include <ros.h>
#include <std_msgs/Int16.h>   //Include the required libraries.
#include <geometry_msgs/Twist.h>


ros::NodeHandle  nh;

#define ENA 5
#define ENB 6
#define IN1 4
#define IN2 7
#define IN3 3
#define IN4 8

float L=0.17;

/*void cmdMotorCB(const std_msgs::Int16& msg)
{
  if(msg.data>=0)
  {
    analogWrite(EN,msg.data);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    analogWrite(EN,-msg.data);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}*/

void cmdvelCB(const geometry_msgs::Twist& twist)
{
  int gain=10000;
  float left_wheel_data=gain*(twist.linear.x - twist.angular.z*L);
  float right_wheel_data=gain*(twist.linear.x + twist.angular.z*L);
  if(left_wheel_data>=0)
  {
    analogWrite(ENA,abs(left_wheel_data));
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    analogWrite(ENA,abs(left_wheel_data));
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  if(right_wheel_data>=0)
  {
    analogWrite(ENB,abs(right_wheel_data));
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else
  {
    analogWrite(ENB,abs(right_wheel_data));
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}

ros::Subscriber<geometry_msgs::Twist> subcmdvel("/cmd_vel",cmdvelCB);

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(ENA, 100);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(ENB, 100);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  
  nh.initNode();
  nh.subscribe(subcmdvel);
}

void loop()
{
  nh.spinOnce();
}

