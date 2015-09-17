#include "ros/ros.h"
#include "sensor_msgs/Joy.h"


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
  int rightMultiplier = 1;
  if(joy->buttons[rightTrigger] > 0){
    rightMultiplier = 0;
  }
  int leftMultiplier = 1;
  if(joy->buttons[leftTrigger] > 0){
    leftMultiplier = 1;
  }
  /* No Servos, because I said so */


}
