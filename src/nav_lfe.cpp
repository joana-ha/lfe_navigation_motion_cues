#include "ros/ros.h"

#include "./../include/turtlebot2i_lfe_navigation/dynamic_obstacle_listener.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "lfe_nav");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    lfe_navigation::DynamicObstacleListener bodyListener(nh, pnh);

    return 0;
}
