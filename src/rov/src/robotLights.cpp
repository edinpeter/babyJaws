//
// Created by peter on 10/11/15.
//

#include "ros/ros.h"
#include "boost/asio.hpp"
#include "sensor_msgs/Joy.h"
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
        s_p.open("/dev/ttyACM2");
        ROS_INFO("Serial port: %s ", port_name.c_str());
        s_p.set_option(boost::asio::serial_port_base::baud_rate(9600));
        ROS_INFO("Baud rate: %i", baud_rate);
        sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Lights::callback, this);
    }
};
void Lights::callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    const int SIZE = 3;
    unsigned char packet[SIZE];
    packet[0] = '-';
    packet[1] = 1;
    packet[2] = '$';
    ROS_INFO("%s","Written!");
    s_p.write_some(boost::asio::buffer(&packet, SIZE));
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