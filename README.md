# ant_hardware
Trying to create an autonomous bot using Raspberry Pi and Arduino Mega with other required hardware.

Integrated various hardware modules with ROS (like motors, encoders, camera etc.)

Code for for the following cases:

* **ros_encoders_embed** - Embedded C code for integrating motor encoders with Arduino Mega and send the data to the server (or main computer) through ROS.

* **ros_motors** - Arduino IDE code for integrating two Geared DC motors with Arduino Mega.

* **ros_motors_embed** - Embedded C code for integrating two Geared DC motors with Arduino Mega and send the commands to the motor from the server (or main computer) through ROS.

* **ros_nav_embed** - Embedded C code for integrating both Geared DC motors and the encoders with each other and Arduino Mega and send/receive the data to/from the server (or main computer) through ROS.

* **ros_nav_pid** - Embedded C code for implementing **PID algorithm** to the motors to get accurate data from the encoders.

* **teleop_joystick_hardware** - C++ script for integrating a joystick controller with server (or main computer) and send the data to the motors through ROS.
