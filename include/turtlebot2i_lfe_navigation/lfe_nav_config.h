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

#ifndef SRC_LFE_NAV_CONFIG_H
#define SRC_LFE_NAV_CONFIG_H

//ros
#include <ros/console.h>
#include <ros/ros.h>

//boost
#include <boost/thread/mutex.hpp>

//internal
#include <turtlebot2i_lfe_navigation/LfeNavReconfigureConfig.h>

namespace lfe_navigation {

    /**
     * @brief The LfeNavConfig class is used as a data type for dynamic configurations.
     */
    class LfeNavConfig {

    public:

        struct Navigation {
            double goal1_pos_x; //!< x coordinate of goal1 position
            double goal1_pos_y; //!< y coordinate of goal1 position
            double goal1_pos_z; //!< z coordinate of goal1 position
            double goal1_orientation; //!< goal1 orientation as radian for z axis
            double goal2_pos_x; //!< x coordinate of goal2 position
            double goal2_pos_y; //!< y coordinate of goal2 position
            double goal2_pos_z; //!< z coordinate of goal2 position
            double goal2_orientation; //!< goal2 orientation as radian for z axis
            double goal3_pos_x; //!< x coordinate of goal3 position
            double goal3_pos_y; //!< y coordinate of goal3 position
            double goal3_pos_z; //!< z coordinate of goal3 position
            double goal3_orientation; //!< goal3 orientation as radian for z axis
        } navigation; //!< Navigation related parameters

        struct HumanRobotInteraction {
            bool backOff; //!< Is true, if a Back Off (BO) motion cue shall be performed upon a human-robot encounter. Is false, if the Stop (ST) motion cue shall be performed.
            int wait_duration; //!< number of seconds to wait until the robot continues after the performed motion cue
            double motion_cue_distance; //!< desired distance between human and robot in meters, when the motion cue shall be triggered
            int human_motion_frame_distance; //!< The distance with which a person approaches the robot from frame to frame in cm. This value indicates the reliability with which a human is detected. A low value will detect humans more reliably at the cost of detecting 'fake' humans as well. The higher the value, the faster the people need to approach the robot in order to be detected.
        } hri; //!< HRI related parameters

        struct BackOff {
            double back_velocity; //!< Velocity of first backwards movement in m/s. Can be between -0.7 and 0. Value should be negative.
            double back_duration; //!< Duration of first backwards movement in seconds.
        } backOff; //!< BackOff related parameters

        /**
        * @brief Construct the LfeNavConfig using default values.
        * @warning If the \b rosparam server or/and \b dynamic_reconfigure (rqt_reconfigure) node are used,
        *	     the default variables will be overwritten: \n
        */
        LfeNavConfig() {

            //navigation
            navigation.goal1_pos_x = -4.04; //kraftraum: 0.5 //allraumtest: 0.33  //rtabmaphome: 2.0 //lfe: -17.3
            navigation.goal1_pos_y = 11.4; //kraftraum:-2.3 //allraumtest: -1.75 //rthabmaphome: -3.3 //lfe: -26.2
            navigation.goal1_pos_z = 0.0;
            navigation.goal1_orientation = 0.0;

            navigation.goal2_pos_x = 3.09; //kraftraum:1.56 //allraumtest: -2.4 //rtabmaphome: 0.4 //lfe: 1.13
            navigation.goal2_pos_y = 6.04; //kraftraum:2.6 //allraumtest: -3.77 //rtabmaphome: -0.95 //lfe: -0.59
            navigation.goal2_pos_z = 0.0;
            navigation.goal2_orientation = 0.0;

            navigation.goal3_pos_x = 0.11; //rtabmaphome: 1.5 //lfe:
            navigation.goal3_pos_y = -2.48; //rtabmaphome: -1.0  //lfe:
            navigation.goal3_pos_z = 0.0;
            navigation.goal3_orientation = 0.0;

            //human-robot-interaction
            hri.backOff = true;
            hri.wait_duration = 2;
            hri.motion_cue_distance = 1.6;
            hri.human_motion_frame_distance = 3;

            //back-off
            backOff.back_velocity = -0.4;
            backOff.back_duration = 1;

        }

        /**
         * @brief Load parameters from the configuration user interface via nodehandle.
         * @param nh const reference to the local ros::NodeHandle
         */
        void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

        /**
         * @brief Reconfigure parameters from the dynamic_reconfigure config.
         * Change parameters dynamically (e.g. with <c>rosrun rqt_reconfigure rqt_reconfigure</c>).
         * A reconfigure server needs to be instantiated that calls this method in it's callback.
         * In case of the package \e lfe_navigation default values are defined
         * in \e PROJECT_SRC/cfg/LfeNavReconfigure.cfg.
         * @param cfg Config class autogenerated by dynamic_reconfigure according to the cfg-file mentioned above.
         */
        void reconfigure(turtlebot2i_lfe_navigation::LfeNavReconfigureConfig& cfg);

        /**
         * @brief Return the internal config mutex
         */
        boost::mutex& configMutex() {return config_mutex_;}

    private:
        boost::mutex config_mutex_; //!< Mutex for config accesses and changes

    };
};


#endif //SRC_LFE_NAV_CONFIG_H
