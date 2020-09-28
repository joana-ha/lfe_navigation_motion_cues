//
// Created by turtlebot on 17.09.20.
//

#ifndef SRC_LFE_NAV_LOGGER_H
#define SRC_LFE_NAV_LOGGER_H

//ros
#include <ros/ros.h>

//tf
#include <tf/transform_listener.h>

#include <turtlebot2i_lfe_navigation/BOLogMsg.h>
#include <turtlebot2i_lfe_navigation/STLogMsg.h>

namespace lfe_navigation{

class LfeNavLogger {

public:

    LfeNavLogger(ros::NodeHandle& nh);

    /// LOGGING ///

    void bo_log_init(double back_velocity, double back_duration, int wait_duration, std::vector<double> human_dist_seq, std::vector<double> human_dist_time_seq, double human_approach_vel, double robot_vel,std::string frame_id);
    void st_log_init(int wait_duration, std::vector<double> human_dist_seq, std::vector<double> human_dist_time_seq, double human_approach_vel, double robot_vel, std::string frame_id);

    void finalize_log(bool back_off, std::vector<double> human_mc_dist_seq, std::vector<double> human_mc_dist_time_seq, bool human_continue_goal);


private:
    //logging objects
    turtlebot2i_lfe_navigation::BOLogMsg bo_log_msg_;
    turtlebot2i_lfe_navigation::STLogMsg st_log_msg_;

    //helpers for logging parameter calculation
    int bo_id_counter_;
    int st_id_counter_;
    ros::Time total_duration_start_;
    ros::Time total_duration_end_;

    //nh to retrieve robot velocity from odom
    ros::NodeHandle& nh_;
    ros::Publisher pub_bo_log_;
    ros::Publisher pub_st_log_;

    tf::TransformListener listener_;
    tf::StampedTransform tf_;
};

}




#endif //SRC_LFE_NAV_LOGGER_H
