#include <ros.h>
#include <std_msgs/Int16.h>   //Include the required libraries.
#include <geometry_msgs/Twist.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

ros::NodeHandle  nh;

float L=0.17;

void motion_pin_config(void)
{
  DDRA = DDRA | 0x0F;
  PORTA = PORTA & 0xF0;
  DDRL = DDRL | 0x18;
  PORTL = PORTL | 0x18;
}

void velocity(unsigned char left_wheel_data, unsigned char right_wheel_data)
{
  OCR5AL = (unsigned char)left_wheel_data;
  OCR5BL = (unsigned char)right_wheel_data;
}

void port_init()
{
  motion_pin_config();
}

void timer5_init()
{
  TCCR5B = 0x00;
  TCNT5H = 0xFF;
  TCNT5L = 0x01;
  OCR5AH = 0xFF;
  OCR5AL = 0xFF;
  OCR5BH = 0xFF;
  OCR5BL = 0xFF;
  TCCR5A = 0xA9;
  TCCR5B = 0x0B;
}

void init_devices()
{
  cli();
  port_init();
  timer5_init();
  sei();
}

void cmdvelCB(const geometry_msgs::Twist& twist)
{
  int gain=10000;
  float left_wheel_data=0;
  left_wheel_data=gain*(twist.linear.x - twist.angular.z*L);
  float right_wheel_data=0;
  right_wheel_data=gain*(twist.linear.x + twist.angular.z*L);
  if(left_wheel_data>0 && right_wheel_data>0)
  {
    velocity(abs(left_wheel_data),abs(right_wheel_data));
    PORTA = PORTA | 0x06;
    PORTA = PORTA & 0x6F;
  }
  else if(left_wheel_data<0 && right_wheel_data<0)
  {
    velocity(abs(left_wheel_data),abs(right_wheel_data));
    PORTA = PORTA | 0x09;
    PORTA = PORTA & 0xF9;
  }
  else if(left_wheel_data>=0 && right_wheel_data<0)
  {
    velocity(abs(left_wheel_data),abs(right_wheel_data));
    PORTA = PORTA | 0x0A;
    PORTA = PORTA & 0xFA;
  }
  else if(left_wheel_data<0 && right_wheel_data>=0)
  {
    velocity(abs(left_wheel_data),abs(right_wheel_data));
    PORTA = PORTA | 0x05;
    PORTA = PORTA & 0xF5;
  }
  else if(left_wheel_data==0 && right_wheel_data==0)
  {
    velocity(abs(left_wheel_data),abs(right_wheel_data));
    PORTA = PORTA & 0xF0;
  }
}

ros::Subscriber<geometry_msgs::Twist> subcmdvel("/cmd_vel",cmdvelCB);

void setup()
{
  init_devices();
  
  nh.initNode();
  nh.subscribe(subcmdvel);
}

void loop()
{
  nh.spinOnce();
}

