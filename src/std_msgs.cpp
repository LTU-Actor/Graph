#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Char.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_twist");
    ros::NodeHandle nh("~");

    ros::Time last_update;
    ros::Time last_print;
    ros::Time start_time;
    int period;
    std::string topic;
    std::string type;

    last_update = last_print = start_time = ros::Time::now();

    nh.param("period", period, 10);
    if (!nh.getParam("type", type))
    {
        ROS_ERROR_STREAM("Must set ~type");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("input", topic))
    {
        ROS_ERROR_STREAM("Must set ~input");
        return EXIT_FAILURE;
    }

    std::cout << "Twist" << std::endl;
    std::cout << "Time (s)," << topic << std::endl;

#define a(msg_type, val_type, access)                                                                                  \
    if (type == std::string(#msg_type))                                                                                \
    {                                                                                                                  \
        boost::function<void(const std_msgs::msg_type &)> cb = [&](const std_msgs::msg_type &msg) {                    \
            ros::Time now = ros::Time::now();                                                                          \
            if ((now - last_update).toSec() < period / 1000.0) return;                                                 \
            last_update = ros::Time::now();                                                                            \
            std::cout << (now - start_time).toSec() << ',' << msg.access << std::endl;                                 \
        };                                                                                                             \
        ros::Subscriber sub = nh.subscribe<std_msgs::msg_type>(topic, 1, cb);                                          \
        ros::spin();                                                                                                   \
        return EXIT_SUCCESS;                                                                                           \
    }

    a(Bool, bool, data);
    a(Byte, unsigned char, data);
    a(Char, char, data);
    a(Duration, double, data.toSec());
    a(Float32, float, data);
    a(Float64, double, data);
    a(Int16, int, data);
    a(Int32, int, data);
    a(Int8, int, data);
    a(String, std::string, data);
    a(Time, double, data.toSec());
    a(UInt16, unsigned int, data);
    a(UInt32, unsigned int, data);
    a(UInt8, unsigned int, data);

#undef a

    ROS_ERROR_STREAM("Unknown type " << type);
    return EXIT_FAILURE;
}
