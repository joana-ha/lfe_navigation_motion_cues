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


    void LfeNavLogger::bo_log_init(double back_velocity, double back_duration, int wait_duration, std::vector<double> human_dist_seq, std::vector<double> human_dist_time_seq, double human_approach_vel, double robot_vel, std::string frame_id) {

        std::string interimCounter = "bo_" + std::to_string(bo_id_counter_);

        bo_log_msg_.id.data = interimCounter;

        bo_log_msg_.cross_situation = false;

        bo_log_msg_.back_velocity = back_velocity;
        bo_log_msg_.back_duration = back_duration;

        bo_log_msg_.wait_seconds = (float) wait_duration;

        std::cout << "vel3 " << human_approach_vel << std::endl;

        try{
            listener_.lookupTransform("map", "base_link", ros::Time(0), tf_);
            bo_log_msg_.robot_coord.x = tf_.getOrigin().x();
            bo_log_msg_.robot_coord.y = tf_.getOrigin().y();
            bo_log_msg_.robot_coord.z = tf_.getOrigin().z();
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }

        bo_log_msg_.robot_lin_vel_x = robot_vel;

        //human robot interaction
        bo_log_msg_.human_approach_vel = human_approach_vel;

        int j = 0;

        for(int i = (human_dist_seq.size()-1); i >= 0; --i){
            if(j < human_dist_seq.size()) {
                bo_log_msg_.human_approach_dist_seq[j] = (float) human_dist_seq.at(i);
                bo_log_msg_.human_approach_dist_time_seq[j] = (float) human_dist_time_seq.at(i);
            }
            j++;
        }

        bo_log_msg_.header.stamp = ros::Time::now();
        bo_log_msg_.header.frame_id = frame_id;
        total_duration_start_ = ros::Time::now();

        bo_id_counter_++;
    }

    void LfeNavLogger::st_log_init(int wait_duration, std::vector<double> human_dist_seq, std::vector<double> human_dist_time_seq, double human_approach_vel, double robot_vel, std::string frame_id) {
        tf::TransformListener listener;
        tf::StampedTransform tf;

        std::string interimCounter = "st_" + std::to_string(st_id_counter_);

        st_log_msg_.id.data = interimCounter;

        st_log_msg_.cross_situation = false;

        st_log_msg_.wait_seconds = (float) wait_duration;

        try{
            listener_.lookupTransform("map", "base_link", ros::Time(0), tf_);
            st_log_msg_.robot_coord.x = tf_.getOrigin().x();
            st_log_msg_.robot_coord.y = tf_.getOrigin().y();
            st_log_msg_.robot_coord.z = tf_.getOrigin().z();
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }

        st_log_msg_.robot_lin_vel_x = robot_vel;

        //human robot interaction
        st_log_msg_.human_approach_vel = human_approach_vel;

        int j = 0;

        std::cout << "in log init before for" << std::endl;

        for(int i = (human_dist_seq.size()-1); i >= 0; --i){
            if(j < human_dist_seq.size()) {
                st_log_msg_.human_approach_dist_seq[j] = (float) human_dist_seq.at(i);
                st_log_msg_.human_approach_dist_time_seq[j] = (float) human_dist_time_seq.at(i);
            }
            j++;
        }

        std::cout << "in log init after for" << std::endl;


        st_log_msg_.header.stamp = ros::Time::now();
        st_log_msg_.header.frame_id = frame_id;
        total_duration_start_ = ros::Time::now();

        st_id_counter_++;

        std::cout << "in log init done" << std::endl;

    }

    void LfeNavLogger::bo_log_init(double back_velocity, double back_duration, int wait_duration, double robot_vel, std::string frame_id){
        std::string interimCounter = "bo_" + std::to_string(bo_id_counter_);

        bo_log_msg_.id.data = interimCounter;

        bo_log_msg_.cross_situation = true;

        bo_log_msg_.back_velocity = back_velocity;
        bo_log_msg_.back_duration = back_duration;

        bo_log_msg_.wait_seconds = (float) wait_duration;

        try{
            listener_.lookupTransform("map", "base_link", ros::Time(0), tf_);
            bo_log_msg_.robot_coord.x = tf_.getOrigin().x();
            bo_log_msg_.robot_coord.y = tf_.getOrigin().y();
            bo_log_msg_.robot_coord.z = tf_.getOrigin().z();
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }

        bo_log_msg_.robot_lin_vel_x = robot_vel;

        int j = 0;

        bo_log_msg_.header.stamp = ros::Time::now();
        bo_log_msg_.header.frame_id = frame_id;
        total_duration_start_ = ros::Time::now();

        bo_id_counter_++;
    }

    void LfeNavLogger::st_log_init(int wait_duration, double robot_vel, std::string frame_id){
        tf::TransformListener listener;
        tf::StampedTransform tf;

        std::string interimCounter = "st_" + std::to_string(st_id_counter_);

        st_log_msg_.id.data = interimCounter;

        st_log_msg_.cross_situation = true;

        st_log_msg_.wait_seconds = (float) wait_duration;

        try{
            listener_.lookupTransform("map", "base_link", ros::Time(0), tf_);
            st_log_msg_.robot_coord.x = tf_.getOrigin().x();
            st_log_msg_.robot_coord.y = tf_.getOrigin().y();
            st_log_msg_.robot_coord.z = tf_.getOrigin().z();
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }

        st_log_msg_.robot_lin_vel_x = robot_vel;

        st_log_msg_.header.stamp = ros::Time::now();
        st_log_msg_.header.frame_id = frame_id;
        total_duration_start_ = ros::Time::now();

        st_id_counter_++;


    }

    void LfeNavLogger::finalize_log(bool back_off, std::vector<double> human_mc_dist_seq, std::vector<double> human_mc_dist_time_seq, bool human_continue_goal){
        total_duration_end_ = ros::Time::now();
        if(back_off){
            bo_log_msg_.time4interaction = total_duration_end_.toSec() - total_duration_start_.toSec();
            bo_log_msg_.human_continue = human_continue_goal;

            for(int i = 0; i < bo_log_msg_.human_mc_dist_seq.size(); i++){
                if ( i < human_mc_dist_seq.size()) {
                    bo_log_msg_.human_mc_dist_seq[i] = human_mc_dist_seq.at(i);
                    bo_log_msg_.human_mc_dist_time_seq[i] = human_mc_dist_time_seq.at(i);
                }
            }


            pub_bo_log_.publish(bo_log_msg_);

            bo_log_msg_.human_mc_dist_seq = {0};
            bo_log_msg_.human_mc_dist_time_seq = {0};
            bo_log_msg_.human_approach_dist_seq = {0};
            bo_log_msg_.human_approach_dist_time_seq = {0};

            std::cout << "final vel" << bo_log_msg_.human_approach_vel << std::endl;
        }else{
            st_log_msg_.time4interaction = total_duration_end_.toSec() - total_duration_start_.toSec();
            st_log_msg_.human_continue = human_continue_goal;

            for(int i = 0; i < st_log_msg_.human_mc_dist_seq.size(); i++){
                if ( i < human_mc_dist_seq.size()) {
                    st_log_msg_.human_mc_dist_seq[i] = human_mc_dist_seq.at(i);
                    st_log_msg_.human_mc_dist_time_seq[i] = human_mc_dist_time_seq.at(i);
                }
            }

            pub_st_log_.publish(st_log_msg_);

            st_log_msg_.human_mc_dist_seq = {0};
            st_log_msg_.human_mc_dist_time_seq = {0};
            st_log_msg_.human_approach_dist_seq = {0};
            st_log_msg_.human_approach_dist_time_seq = {0};

        }
    }






}


