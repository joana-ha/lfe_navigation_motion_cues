/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Joana Haase
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

#ifndef SRC_DYNAMIC_OBSTACLE_LISTENER_H
#define SRC_DYNAMIC_OBSTACLE_LISTENER_H

//ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <turtlebot2i_lfe_navigation/LfeNavReconfigureConfig.h>

//ros_astra_sdk_wrapper
#include <ros_astra_sdk_wrapper/BodyTracking.h>

//internal
#include "./navigation_manager.h"
#include "./lfe_nav_config.h"
#include "./lfe_nav_logger.h"

namespace lfe_navigation{

typedef turtlebot2i_lfe_navigation::LfeNavReconfigureConfig Config;
typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

/**
 * @class DynamicObstacleListener
 * @brief Listens to /camera/body/skeleton messages and calculates distance to robot
 *
 * The DynamicObstacleListener listens to body and depth messages to calculate distance to robot and trigger motion cues, if conditions apply.
 */

    class DynamicObstacleListener {

    public:

        /**
         * @brief Construct DynamicObstacleListener and set up the subscriber Object
         * @param nh Nodehandle to subscribe to /camera/body/skeleton
         * @param pnh Private nodehandle to get configured parameters
         */
        DynamicObstacleListener(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    private:

        /** stores the configuration of whether a back-off shall be executed or a stop */
        bool backOff_;

        /** state variable indicating whether a body is currently perceived */
        bool body_received_;

        /** state variable indicating whether robot is ready to continue after motion cue execution is finished */
        bool ready_to_continue_;

        /** state variable indicating whether navigation is paused, because a motion cue is currently executed */
        bool paused_;

        /** variable indicating whether human distance to robot has decreased over last received body frames */
        bool human_dist_decrease_;

        /** counter to check, whether no body was received for a certain period of time */
        int body_not_received_count_;

        /** LfeNavLogger object responsible for logging */
        LfeNavLogger lfeNavLogger_;

        /** NavigationManager object responsible for navigation */
        NavigationManager navigationManager_;

        /** Distance of human to robot received via body messages */
        double human_robot_distance_;

        /** x-coordinate of knee in pixel coordinates (camera image)*/
        int pxl_lknee_x_;

        /** y-coordinate of knee in pixel coordinates (camera image)*/
        int pxl_lknee_y_;

        /** stores current robot velocity*/
        double current_robot_vel_;

        /** stores human approaching distances */
        std::vector<double> human_dist_seq_;

        /** stores human approaching distance time stamps */
        std::vector<double> human_dist_time_seq_;

        /** stores robot velocities at the same time as the human distances are tracked */
        std::vector<double> robot_vel_seq_;

        /** stores human distance to robot during motion cue execution */
        std::vector<double> human_mc_dist_seq_;

        /** stores human distance to robot time stamps during motion cue execution */
        std::vector<double> human_mc_dist_time_seq_;

        /** LfeNavConfig object to set all new configurations and retrieve existing configurations */
        LfeNavConfig cfg_;

        /** Instance of a reconfigure server */
        boost::shared_ptr<ReconfigureServer> reconfigure_server_;

        /** Variable tracking if reconfiguration is currently performed */
        bool config_init_;

        /** Sequence of calculated medians from depth image data, only used for internal calc */
        std::vector<double> avg_img_depth_seq_;

        /**
        * @brief Method setting which motion cue shall be performed based on configurations.
        * @param backOff True, if back-off shall be performed
        */
        void setMotionCue(bool backOff);

        /**
        * @brief Method starting the subscription to robot velocity, body messages and depth stream.
        * @param sub Nodehandle for subscription
        */
        void subscriberLoop(ros::NodeHandle& sub);

        /**
          * @brief Listens to messages from /tf
          */
        void loop();

        /**
         * @brief Callback to calculate distance, if bodymsg is received. Starts a thread to execute motion cue, if motion cue is triggered based on human distance.
         * @param bodyMsg Message received from subscriber
         */
        void newBodyMsgCallback(const ros_astra_sdk_wrapper::BodyTracking& bodyMsg);

        /**
         * @brief Callback to get robot velocity for logging purposes.
         * @param odom_msg Message received from subscriber
         */
        void newVelocityCallback(geometry_msgs::Twist odom_msg);

        /**
         * @brief Callback to calculate distance, if new depth frame is received.
         * @param depth_msg Pointer to depth frame received from nodelet
         * @param info_msg Pointer to info_msg received from nodelet
         */
        void newDepthCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                              const sensor_msgs::CameraInfoConstPtr& info_msg);

        /**
          * @brief Callback for the dynamic configuration.
          *
          * This callback allows to modify parameters dynamically at runtime without restarting the node
          * @param config Reference to the dynamic reconfigure config
          */
        void configCb(turtlebot2i_lfe_navigation::LfeNavReconfigureConfig& config);

        /**
         * @brief Method to call the stopGoal() function in the NavigationManager and to call the logging methods of the LfeNavLogger object
         * @param frame_id Frame ID of depth or body frame from which the motion cue was triggered, used for logging
         * @param human_approach_vel human approaching velocity, used for logging
         * @param robot_vel_avg robot average velocity over the last frames for logging
         */
        void executeMotionCue(std::string frame_id, double human_approach_vel, double robot_vel_avg);

    };

}




#endif //SRC_DYNAMIC_OBSTACLE_LISTENER_H
