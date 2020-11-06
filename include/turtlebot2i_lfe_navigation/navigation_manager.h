/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  TU Munich - Lehrstuhl fuer Ergonomie.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Joana Haase
 *********************************************************************/

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

    /**
     * @brief The NavigationManager contains all methods responsible for navigation.
     *
     * The NavigationManager class manages the communication with the actionLib server and executes the motion cues.
     * It is responsible to start, cancel and continue a navigation goal.
     */
    class NavigationManager {

    public:

        /**
        * @brief Overloaded constructor that takes in a nodehandle.
        * @param nh The nodehandle by which back-off backwards velocity is published for wheel control
        */
        NavigationManager(ros::NodeHandle &nh);


        /**
        * @brief Method sending goals to the actionlib server.
        */
        void sendGoal();

        /**
        * @brief Method setting paused_ to true. Paused_ stops the navigation in sendGoal() method.
        */
        void stopGoal();

        /**
        * @brief Method executing the back-off motion cue.
        * @param back_velocity Backwards velocity
        * @param back_seconds Back-off duration in seconds
        */
        void backOff(double back_velocity, double back_seconds);

        /**
        * @brief Method Setting sendingGoal_ to true to continue the navigation.
        * @param seconds Time to wait before continuing the navigation in seconds
        */
        void continueGoal(int seconds);

        /**
        * @brief Method setting new coordinates to goal1_ variable.
        * @param x x-coordinate
        * @param y y-coordinate
        * @param z z-coordinate
        * @param orientation Angular orientation around z-axis
        */
        void setGoal1(double x, double y, double z, double orientation);

        /**
        * @brief Method setting new coordinates to goal2_ variable.
        * @param x x-coordinate
        * @param y y-coordinate
        * @param z z-coordinate
        * @param orientation Angular orientation around z-axis
        */
        void setGoal2(double x, double y, double z, double orientation);

        /**
        * @brief Method setting new coordinates to goal3_ variable.
        * @param x x-coordinate
        * @param y y-coordinate
        * @param z z-coordinate
        * @param orientation Angular orientation around z-axis
        */
        void setGoal3(double x, double y, double z, double orientation);

    private:
        /** counter being increased when a goal is successfully reached */
        int count_;

        /** state variable indicating that navigation is paused */
        bool paused_;

        /** state variable indicating that navigation is running */
        bool sendingGoal_;

        /** publisher sending back-off backwards velocity to wheels */
        ros::Publisher pub_vel_bo_;

        /** move base client */
        MoveBaseClient ac_;

        /** 1st goal to be reached, storing x,y,z coords */
        move_base_msgs::MoveBaseGoal goal1_;

        /** 2nd goal to be reached, storing x,y,z coords */
        move_base_msgs::MoveBaseGoal goal2_;

        /** 3rd goal to be reached, storing x,y,z coords */
        move_base_msgs::MoveBaseGoal goal3_;

    };
}




#endif //SRC_NAVIGATION_MANAGER_H
