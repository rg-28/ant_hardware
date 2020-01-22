#include <ros.h>
#include <std_msgs/Float32.h>
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

void left_encoder_pin_config()
{
  DDRD = DDRD & 0xFC;
  PORTD = PORTD | 0x04;
}

void right_encoder_pin_config()
{
  DDRD = DDRD & 0xF7;
  PORTD = PORTD | 0x08;
}

void left_position_encoder_interrupt_init(void)
{
  cli();
  EICRA = EICRA | 0x01;
  EIMSK = EIMSK | 0x01;
  sei();
}

void right_position_encoder_interrupt_init(void)
{
  cli();
  EICRA = EICRA | 0x04;
  EIMSK = EIMSK | 0x02;
  sei();
}

ISR(INT0_vect)
{
  ShaftCountLeft++;
  //Serial.println("Left Encoder");
  //Serial.println(ShaftCountLeft);
}

ISR(INT1_vect)
{
  ShaftCountRight++;
  //Serial.println("Right Encoder");
  //Serial.println(ShaftCountRight);
}

ISR(TIMER1_OVF_vect)
{
  TCCR1B=0x00;
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
  //Serial.print("Vel_Linear:");
  //Serial.println(sensor_vel.linear.x);
  //Serial.print("Vel_Angular:");
  //Serial.println(sensor_vel.angular.z);
  ShaftCountRight=0;
  ShaftCountLeft=0;
  TCNT1H= 0xF3;
  TCNT1L= 0xCA;
  TCCR1B=0x04;
}

void port_init()
{
  left_encoder_pin_config();
  right_encoder_pin_config();
}

void timer1_init()
{
  TCCR1B = 0x00; //stop
  TCNT1H = 0xF3; //Counter higher 8 bit value
  TCNT1L = 0xCA; //Counter lower 8 bit value
  OCR1AH = 0x00; //Output Compair Register (OCR)- Not used
  OCR1AL = 0x00; //Output Compair Register (OCR)- Not used
  OCR1BH = 0x00; //Output Compair Register (OCR)- Not used
  OCR1BL = 0x00; //Output Compair Register (OCR)- Not used
  ICR1H  = 0x00; //Input Capture Register (ICR)- Not used
  ICR1L  = 0x00; //Input Capture Register (ICR)- Not used
  TCCR1A = 0x00; 
  TCCR1C = 0x00;
  TCCR1B = 0x04; //start Timer
}

void init_devices()
{
  cli();
  port_init();
  timer1_init();
  TIMSK1 = 0x01;  //Timer1 overflow interrupt enable.
  left_position_encoder_interrupt_init();
  right_position_encoder_interrupt_init();
  sei();
}

void setup()
{
  init_devices();
  //Serial.begin(9600);
  //Serial.println("Data");
  
  nh.initNode();
  //nh.advertise(left_wheel_vel_pub);
  //nh.advertise(right_wheel_vel_pub);
  nh.advertise(sensor_vel_pub);
}

void loop()
{
  nh.spinOnce();
}
