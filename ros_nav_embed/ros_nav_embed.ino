#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

ros::NodeHandle nh;

unsigned long int ShaftCountLeft=0;
unsigned long int ShaftCountRight=0;

float radius=0.04;
float L=0.17;

std_msgs::Float32 left_wheel_vel;
ros::Publisher left_wheel_vel_pub("/left_wheel_velocity", &left_wheel_vel);

std_msgs::Float32 right_wheel_vel;
ros::Publisher right_wheel_vel_pub("/right_wheel_velocity", &right_wheel_vel);

geometry_msgs::Twist sensor_vel;
ros::Publisher sensor_vel_pub("/sensor_velocity", &sensor_vel);

void motion_pin_config(void)
{
  DDRA = DDRA | 0x0F;
  PORTA = PORTA & 0xF0;
  DDRL = DDRL | 0x18;
  PORTL = PORTL | 0x18;
}

void left_encoder_pin_config()
{
  DDRE = DDRE & 0xEF;
  PORTE = PORTE | 0x10;
}

void right_encoder_pin_config()
{
  DDRE = DDRE & 0xDF;
  PORTE = PORTE | 0x20;
}

void left_position_encoder_interrupt_init(void)
{
  cli();
  EICRB = EICRB | 0x01;
  EIMSK = EIMSK | 0x10;
  sei();
}

void right_position_encoder_interrupt_init(void)
{
  cli();
  EICRB = EICRB | 0x04;
  EIMSK = EIMSK | 0x20;
  sei();
}

ISR(INT4_vect)
{
  ShaftCountLeft++;
}

ISR(INT5_vect)
{
  ShaftCountRight++;
}

ISR(TIMER4_OVF_vect)
{
  TCCR4B=0x00;
  left_wheel_vel.data=ShaftCountLeft;
  right_wheel_vel.data=ShaftCountRight;
  sensor_vel.linear.x=radius*(left_wheel_vel.data + right_wheel_vel.data)/2;
  sensor_vel.linear.y=0;
  sensor_vel.linear.z=0;
  sensor_vel.angular.x=0;
  sensor_vel.angular.y=0;
  sensor_vel.angular.z=radius*(left_wheel_vel.data + right_wheel_vel.data)/L;
  //left_wheel_vel_pub.publish(&left_wheel_vel);
  //right_wheel_vel_pub.publish(&right_wheel_vel);
  sensor_vel_pub.publish(&sensor_vel);
  ShaftCountRight=0;
  ShaftCountLeft=0;
  TCNT4H = 0xF3;
  TCNT4L = 0xCA;
  TCCR4B = 0x04;
}

void velocity(unsigned char left_wheel_data, unsigned char right_wheel_data)
{
  OCR5AL = (unsigned char)left_wheel_data;
  OCR5BL = (unsigned char)right_wheel_data;
}

void port_init()
{
  motion_pin_config();
  left_encoder_pin_config();
  right_encoder_pin_config();
}

void timer4_init()
{
  TCCR4B = 0x00; //stop
  TCNT4H = 0xF3; //Counter higher 8 bit value
  TCNT4L = 0xCA; //Counter lower 8 bit value
  OCR4AH = 0x00; //Output Compair Register (OCR)- Not used
  OCR4AL = 0x00; //Output Compair Register (OCR)- Not used
  OCR4BH = 0x00; //Output Compair Register (OCR)- Not used
  OCR4BL = 0x00; //Output Compair Register (OCR)- Not used
  OCR4CH = 0x00;
  OCR4CL = 0x00;
  ICR4H  = 0x00; //Input Capture Register (ICR)- Not used
  ICR4L  = 0x00; //Input Capture Register (ICR)- Not used
  TCCR4A = 0x00; 
  TCCR4C = 0x00;
  TCCR4B = 0x04; //start Timer
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
  timer4_init();
  TIMSK4 = 0x01;  //Timer4 overflow interrupt enable.
  timer5_init();
  left_position_encoder_interrupt_init();
  right_position_encoder_interrupt_init();
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
    PORTA = PORTA | 0x05;
    PORTA = PORTA & 0xF5;
  }
  else if(left_wheel_data<0 && right_wheel_data>=0)
  {
    velocity(abs(left_wheel_data),abs(right_wheel_data));
    PORTA = PORTA | 0x0A;
    PORTA = PORTA & 0xFA;
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
  //nh.advertise(left_wheel_vel_pub);
  //nh.advertise(right_wheel_vel_pub);
  nh.advertise(sensor_vel_pub);
  nh.subscribe(subcmdvel);
}

void loop()
{
  nh.spinOnce();
}
