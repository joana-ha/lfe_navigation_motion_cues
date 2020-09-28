//
// Created by turtlebot on 16.09.20.
//


#include "../include/turtlebot2i_lfe_navigation/lfe_nav_config.h"

namespace lfe_navigation{

    void LfeNavConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh) {

        //navigation
        nh.param("goal1_pos_x", navigation.goal1_pos_x, navigation.goal1_pos_x);
        nh.param("goal1_pos_y", navigation.goal1_pos_y, navigation.goal1_pos_y);
        nh.param("goal1_pos_z", navigation.goal1_pos_z, navigation.goal1_pos_z);
        nh.param("goal1_orientation", navigation.goal1_orientation, navigation.goal1_orientation);
        nh.param("goal2_pos_x", navigation.goal2_pos_x, navigation.goal2_pos_x);
        nh.param("goal2_pos_y", navigation.goal2_pos_y, navigation.goal2_pos_y);
        nh.param("goal2_pos_z", navigation.goal2_pos_z, navigation.goal2_pos_z);
        nh.param("goal2_orientation", navigation.goal2_orientation, navigation.goal2_orientation);
        nh.param("goal3_pos_x", navigation.goal3_pos_x, navigation.goal3_pos_x);
        nh.param("goal3_pos_y", navigation.goal3_pos_y, navigation.goal3_pos_y);
        nh.param("goal3_pos_z", navigation.goal3_pos_z, navigation.goal3_pos_z);
        nh.param("goal3_orientation", navigation.goal3_orientation, navigation.goal3_orientation);

        //human-robot-interaction
        nh.param("backOff", hri.backOff, hri.backOff);
        nh.param("wait_duration", hri.wait_duration, hri.wait_duration);
        nh.param("motion_cue_distance", hri.motion_cue_distance, hri.motion_cue_distance);
        nh.param("human_motion_frame_distance", hri.human_motion_frame_distance, hri.human_motion_frame_distance);

        //back-off
        nh.param("back_velocity", backOff.back_velocity, backOff.back_velocity);
        nh.param("back_duration", backOff.back_duration, backOff.back_duration);

    }

    void LfeNavConfig::reconfigure(turtlebot2i_lfe_navigation::LfeNavReconfigureConfig& cfg){
        boost::mutex::scoped_lock l(config_mutex_);

        //navigation
        navigation.goal1_pos_x = cfg.goal1_pos_x;
        navigation.goal1_pos_y = cfg.goal1_pos_y;
        navigation.goal1_pos_z = cfg.goal1_pos_z;
        navigation.goal1_orientation = cfg.goal1_orientation;
        navigation.goal2_pos_x = cfg.goal2_pos_x;
        navigation.goal2_pos_y = cfg.goal2_pos_y;
        navigation.goal2_pos_z = cfg.goal2_pos_z;
        navigation.goal2_orientation = cfg.goal2_orientation;
        navigation.goal3_pos_x = cfg.goal3_pos_x;
        navigation.goal3_pos_y = cfg.goal3_pos_y;
        navigation.goal3_pos_z = cfg.goal3_pos_z;
        navigation.goal3_orientation = cfg.goal3_orientation;

        //human-robot-interaction
        hri.backOff = cfg.backOff;
        hri.wait_duration = cfg.wait_duration;
        hri.motion_cue_distance = cfg.motion_cue_distance;
        hri.human_motion_frame_distance = cfg.human_motion_frame_distance;

        //back-off
        backOff.back_velocity = cfg.back_velocity;
        backOff.back_duration = cfg.back_duration;

    }
}
