//
// Created by turtlebot on 17.09.20.
//

#include "../include/turtlebot2i_lfe_navigation/lfe_nav_logger.h"

namespace lfe_navigation{

    LfeNavLogger::LfeNavLogger(ros::NodeHandle& nh) : bo_id_counter_(0), nh_(nh){
        ros::NodeHandle nh_log(nh_, "lfe_nav");
        pub_bo_log_ = nh_log.advertise<turtlebot2i_lfe_navigation::BOLogMsg>("bo_log", 10, true);
        pub_st_log_ = nh_log.advertise<turtlebot2i_lfe_navigation::STLogMsg>("st_log", 10, true);
    }


    /// LOGGING ///


    void LfeNavLogger::bo_log_init(double back1_velocity, double back1_duration, double forw_velocity, double forw_duration, double back2_velocity, double back2_duration, int wait_duration, double human_approach_vel, double robot_vel, std::string frame_id) {

        std::string interimCounter = "bo_" + std::to_string(bo_id_counter_);

        bo_log_msg_.id.data = interimCounter;

        bo_log_msg_.back1_velocity = back1_velocity;
        bo_log_msg_.back1_duration = back1_duration;
        bo_log_msg_.forw_velocity = forw_velocity;
        bo_log_msg_.forw_duration = forw_duration;
        bo_log_msg_.back2_velocity = back2_velocity;
        bo_log_msg_.back2_duration = back2_duration;

        bo_log_msg_.wait_seconds = (float) wait_duration;

        try{
            listener_.lookupTransform("map", "base_link", ros::Time(0), tf_);
            bo_log_msg_.robot_coord.x = tf_.getOrigin().x();
            bo_log_msg_.robot_coord.y = tf_.getOrigin().y();
            bo_log_msg_.robot_coord.z = tf_.getOrigin().z();
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }

        std::cout << "reached tf" << std::endl;

        bo_log_msg_.robot_lin_vel_x = robot_vel;

        bo_log_msg_.human_approach_vel = human_approach_vel;
        bo_log_msg_.header.stamp = ros::Time::now();
        bo_log_msg_.header.frame_id = frame_id;
        total_duration_start_ = ros::Time::now();

        bo_id_counter_++;
    }

    void LfeNavLogger::st_log_init(int wait_duration, double human_approach_vel, double robot_vel, std::string frame_id) {
        tf::TransformListener listener;
        tf::StampedTransform tf;

        st_log_msg_.id.data = "bo_" + st_id_counter_;

        st_log_msg_.wait_seconds = (float) wait_duration;

        listener.lookupTransform("/base_link", "/map", ros::Time(0), tf);
        st_log_msg_.robot_coord.x = tf.getOrigin().x();
        st_log_msg_.robot_coord.y = tf.getOrigin().y();
        st_log_msg_.robot_coord.z = tf.getOrigin().z();

        st_log_msg_.robot_lin_vel_x = robot_vel;

        st_log_msg_.header.stamp = ros::Time::now();
        st_log_msg_.header.frame_id = frame_id;
        total_duration_start_ = ros::Time::now();

        st_id_counter_++;
    }

    void LfeNavLogger::finalize_log(bool back_off){
        total_duration_end_ = ros::Time::now();
        if(back_off){
            bo_log_msg_.total_duration = total_duration_end_.toSec() - total_duration_start_.toSec();
            pub_bo_log_.publish(bo_log_msg_);
        }else{
            st_log_msg_.total_duration = total_duration_end_.toSec() - total_duration_start_.toSec();
            pub_st_log_.publish(st_log_msg_);
        }
    }






}


