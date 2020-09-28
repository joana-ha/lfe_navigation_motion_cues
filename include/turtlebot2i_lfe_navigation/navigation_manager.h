//
// Created by turtlebot on 06.09.20.
//

#ifndef SRC_NAVIGATION_MANAGER_H
#define SRC_NAVIGATION_MANAGER_H

//cpp
#include <iostream>
#include <unistd.h>

//ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

//ros navigation stack
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//internal


namespace lfe_navigation {

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    class NavigationManager {

    public:
        NavigationManager(ros::NodeHandle &nh);
        ~NavigationManager();

        void sendGoal();
        void stopGoal();
        void backOff(double back_velocity, double back_seconds);
        void continueGoal(int seconds);

        void setGoal1(double x, double y, double z, double orientation);
        void setGoal2(double x, double y, double z, double orientation);
        void setGoal3(double x, double y, double z, double orientation);

    private:
        int count_;
        bool paused_;
        bool sendingGoal_;
        ros::Publisher pub_vel_bo_;

        MoveBaseClient ac_;

        move_base_msgs::MoveBaseGoal goal1_;
        move_base_msgs::MoveBaseGoal goal2_;
        move_base_msgs::MoveBaseGoal goal3_;

    };
}




#endif //SRC_NAVIGATION_MANAGER_H
