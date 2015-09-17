#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#undef serial

#ifdef serial
	#include "boot/asio.hpp"
#endif


const int leftStickX = 1;
const int leftStickY = 2;
const int rightStickX = 3;
const int rightStickY = 4;
const int leftTrigger = 5;
const int rightTrigger = 6;
const int xButton = 7;

class Controls
{
  public:
    Controls();
    void loop();
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
  private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber joy;
    ros::Subscriber imu;
    #ifdef serial
    	boost::asio::serial_port s_p;
    	s_p.open("/dev/ttyUSB0");
		s_p.set_option(boost::asio::serial_port_base::baud_rate(9600));
	#endif
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controls_node");
  Controls controls;
  controls.loop();
}

Controls::Controls() : nh()
{
  joy = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Controls::joy_callback, this);
}

void Controls::loop()
{
  ros::Rate rate(20);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void Controls::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  int leftServoAngle = 0;
  int rightServoAngle = 0;
  int rightMultiplier = 1;
  int leftMultiplier = 1;

  int leftPower = 0;
  int rightPower = 0;

  if(joy->buttons[rightTrigger] > 0){
    rightMultiplier = 0;
  }
  if(joy->buttons[leftTrigger] > 0){
    leftMultiplier = 0;
  }
  /* No Servos, because I said so */
  if(joy->buttons[xButton] > 0){
  	leftServoAngle = 90;
  	rightServoAngle = 90;
  }

  leftPower = joy->axes[leftStickY] * (100.0 / 127.0) * leftMultiplier;
  rightPower = joy->axes[leftStickY] * (100.0 / 127.0) * rightMultiplier;
  if (leftPower < 0){
  	leftPower = 1500-leftPower;
  }
  else{
  	leftPower = 1500 + leftPower;
  }
  if (rightPower < 0){
  	rightPower = 1500-rightPower;
  }
  else{
  	rightPower = 1500 + rightPower;
  }
  const int SIZE = 11;
  unsigned char packet[SIZE];
  packet[0] = '-';
  packet[1] = leftServoAngle >> 8;
  packet[2] = leftServoAngle;
  packet[3] = rightServoAngle >> 8;
  packet[4] = rightServoAngle;
  packet[5] = 0 >> 8;
  packet[6] = 0;
  packet[7] = leftPower >> 8;
  packet[8] = leftPower;
  packet[9] = rightPower >> 8;
  packet[10] = rightPower;
  #ifdef serial
	s_p.write_some(boost::asio::buffer(&packet, SIZE));
  #endif
}
