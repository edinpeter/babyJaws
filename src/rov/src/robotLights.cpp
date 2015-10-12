//
// Created by peter on 10/11/15.
//

#include "ros/ros.h"
#include "boost/asio.hpp"
#include "sensor_msgs/Joy.h"

const int SIZE = 1;
unsigned char packet[SIZE];

class Lights
{
public:
    void callback(const sensor_msgs::Joy::ConstPtr& joy);
    void loop();
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    boost::asio::io_service i_o;
    boost::asio::serial_port s_p;
    std::string port_name;
    int baud_rate;


public:
    Lights() : nh(), i_o(), s_p(i_o)
    {
        s_p.open("/dev/ttyACM0");
        s_p.set_option(boost::asio::serial_port_base::baud_rate(9600));
        sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Lights::callback, this);
        packet[0] = '0';
    }
};
void Lights::callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    int start = 3;
    int lefttrigger = 8;
    int righttrigger = 9;

    if(joy->buttons[lefttrigger] > 0 && joy->buttons[righttrigger] > 0){
        packet[0] = '2';
        s_p.write_some(boost::asio::buffer(&packet, SIZE));
        ROS_INFO("%s","Written!");

    }
    else if(joy->buttons[lefttrigger] > 0){
        packet[0] = '1';
        s_p.write_some(boost::asio::buffer(&packet, SIZE));
        ROS_INFO("%s","Written!");

    }
    else if(joy->buttons[start] > 0){
        packet[0] = '0';
        s_p.write_some(boost::asio::buffer(&packet, SIZE));
        ROS_INFO("%s","Written!");

    }
}
void Lights::loop()
{
    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lights_node");
    Lights lights;
    lights.loop();
}