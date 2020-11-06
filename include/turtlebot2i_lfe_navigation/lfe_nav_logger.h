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

#ifndef SRC_LFE_NAV_LOGGER_H
#define SRC_LFE_NAV_LOGGER_H

//ros
#include <ros/ros.h>

//tf
#include <tf/transform_listener.h>

#include <turtlebot2i_lfe_navigation/BOLogMsg.h>
#include <turtlebot2i_lfe_navigation/STLogMsg.h>

namespace lfe_navigation{

/**
 * @brief The LfeNavLogger is responsible for parameter logging and publishing of log data as messages.
 *
 * Methods of the LfeNavLogger are called by the DynamicObstacleListener to initialize or finalize a logging message.
 * Logging messages are published and can be recorded into bagfiles.
 */

class LfeNavLogger {

public:

    /**
    * @brief Overloaded constructor that takes in a nodehandle.
    * @param nh The nodehandle by which logging messages are published
    */
    LfeNavLogger(ros::NodeHandle& nh);

    /**
    * @brief Overloaded method to initialize a back-off log message in a frontal path-crossing situation.
    * @param back_velocity Configured back-off backwards velocity
    * @param back_duration Configured duration of the back-off
    * @param wait_duration Configured duration to wait before navigation is continued
    * @param human_dist_seq Stores the distances of the approaching human
    * @param human_dist_time_seq Stores the timestamp of the distances in human_dist_seq
    * @param human_approach_vel Stores the approaching velocity of the human
    * @param robot_vel Current robot velocity
    * @param frame_id Frame id of the image, from which the back-off was triggered
    */
    void bo_log_init(double back_velocity, double back_duration, int wait_duration, std::vector<double> human_dist_seq, std::vector<double> human_dist_time_seq, double human_approach_vel, double robot_vel,std::string frame_id);

    /**
    * @brief Overloaded method to initialize a stop motion cue log message in a frontal path-crossing situation.
    * @param wait_duration Configured duration to wait before navigation is continued
    * @param human_dist_seq Stores the distances of the approaching human
    * @param human_dist_time_seq Stores the timestamp of the distances in human_dist_seq
    * @param human_approach_vel Stores the approaching velocity of the human
    * @param robot_vel Current robot velocity
    * @param frame_id Frame id of the image, from which the back-off was triggered
    */
    void st_log_init(int wait_duration, std::vector<double> human_dist_seq, std::vector<double> human_dist_time_seq, double human_approach_vel, double robot_vel, std::string frame_id);

    /**
    * @brief Overloaded method to initialize a back-off log message in a lateral path-crossing situation.
    * @param back_velocity Configured back-off backwards velocity
    * @param back_duration Configured duration of the back-off
    * @param wait_duration Configured duration to wait before navigation is continued
    * @param robot_vel Current robot velocity
    * @param frame_id Frame id of the image, from which the back-off was triggered
    */
    void bo_log_init(double back_velocity, double back_duration, int wait_duration, double robot_vel, std::string frame_id);

    /**
    * @brief Overloaded method to initialize a stop motion cue log message in a lateral path-crossing situation.
    * @param wait_duration Configured duration to wait before navigation is continued
    * @param robot_vel Current robot velocity
    * @param frame_id Frame id of the image, from which the back-off was triggered
    */
    void st_log_init(int wait_duration, double robot_vel, std::string frame_id);

    /**
    * @brief Method finalizing the logging of both, back-off and stop motion cue, and publishing it as a message.
    * @param back_off True, if motion cue is back-off, False, if motion cue is stop
    * @param human_mc_dist_seq Stores the distances of the human during the motion cue execution
    * @param human_mc_dist_time_seq Stores the time stamps of the distances in the humanh_mc_dist_seq array
    * @param human_continue_goal True, if a human was still detected while continuing the navigation
    */
    void finalize_log(bool back_off, std::vector<double> human_mc_dist_seq, std::vector<double> human_mc_dist_time_seq, bool human_continue_goal);

private:
    /** message object for back-off logging message */
    turtlebot2i_lfe_navigation::BOLogMsg bo_log_msg_;

    /** message object for stop logging message */
    turtlebot2i_lfe_navigation::STLogMsg st_log_msg_;

    /** counter used for logging back-off logging parameter calculation */
    int bo_id_counter_;

    /** counter used for logging stop motion cue logging parameter calculation */
    int st_id_counter_;

    /** start time of the motion cue execution */
    ros::Time total_duration_start_;

    /** end time of the motion cue execution */
    ros::Time total_duration_end_;

    /** nodehandle to publish messages */
    ros::NodeHandle& nh_;

    /** Publisher object for back-off message publishing */
    ros::Publisher pub_bo_log_;

    /** Publisher object for stop message publishing */
    ros::Publisher pub_st_log_;

    /** Tranform listener to get current robot position */
    tf::TransformListener listener_;

    /** Tranform object */
    tf::StampedTransform tf_;
};

}




#endif //SRC_LFE_NAV_LOGGER_H
