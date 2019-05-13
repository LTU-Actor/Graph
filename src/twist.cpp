#include <cmath>
#include <iostream>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

static ros::Time last_update;
static int period;

static void
twist_cb(const geometry_msgs::Twist &msg)
{
    ros::Time now = ros::Time::now();
    if ((now - last_update).toSec() < period / 1000.0) return;
    std::cout << now.toSec() << ',' << msg.linear.x << ',' << msg.linear.y << ',' << msg.linear.z << ','
              << msg.angular.x << ',' << msg.angular.y << ',' << msg.angular.z << std::endl;
    last_update = now;
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_twist");
    ros::NodeHandle nh{"~"};

    std::cout << "Twist" << std::endl;
    std::cout << "Time (s),linear.x,linear.y,linear.z,angular.x,angular.y,angular.z" << std::endl;

    std::string topic;

    nh.param("period", period, 10);
    if(!nh.getParam("input", topic))
    {
        ROS_ERROR_STREAM("Must set input");
        return EXIT_FAILURE;
    }

    last_update = ros::Time::now();

    ros::Subscriber sub = nh.subscribe(topic, 1, &twist_cb);
    ros::spin();

    return EXIT_SUCCESS;
}
