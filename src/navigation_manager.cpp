//
// Created by turtlebot on 06.09.20.
//

#include "../include/turtlebot2i_lfe_navigation/navigation_manager.h"

namespace lfe_navigation{

    NavigationManager::NavigationManager(ros::NodeHandle &nh) : count_(1), paused_(false), sendingGoal_(false), ac_("move_base", true) {

        goal1_.target_pose.header.frame_id = "map";
        goal1_.target_pose.header.stamp = ros::Time::now();

        goal1_.target_pose.pose.position.x = 0.0;
        goal1_.target_pose.pose.position.y = 0.0;
        goal1_.target_pose.pose.position.z = 0.0;
        goal1_.target_pose.pose.orientation.z = 0.0;
        goal1_.target_pose.pose.orientation.w = 1.0;

        goal2_.target_pose.header.frame_id = "map";
        goal2_.target_pose.header.stamp = ros::Time::now();

        goal2_.target_pose.pose.position.x = 0.0;
        goal2_.target_pose.pose.position.y = 0.0;
        goal2_.target_pose.pose.position.z = 0.0;
        goal2_.target_pose.pose.orientation.z = 0.0;
        goal2_.target_pose.pose.orientation.w = 1.0;

        pub_vel_bo_ = nh.advertise<geometry_msgs::Twist>("/navigation_velocity_smoother/raw_cmd_vel", 10);

    }

    NavigationManager::~NavigationManager() {

    }

    void NavigationManager::sendGoal(){

        while (!ac_.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        sendingGoal_ = true;
        ROS_INFO("Sending goal is true, count: ", count_);
        while (ros::ok()){
            if(sendingGoal_ == true){
                if (count_%2 != 0){
                    ac_.sendGoal(goal1_);
                    ROS_INFO("Goal 1 sent");
                }else {
                    ac_.sendGoal(goal2_);
                    ROS_INFO("Goal 2 sent");
                }

                std::cout << "interrupt 1 " << std::endl;

                while(ac_.getState() == actionlib::SimpleClientGoalState::PENDING){
                    if (paused_ == true){
                        ROS_INFO("jumped into paused_ true if after pending");
                        ac_.cancelGoal();
                        ac_.waitForResult();
                        /*while (ac_.getState() != actionlib::SimpleClientGoalState::REJECTED){
                            continue;
                        }*/
                        sendingGoal_ = false;
                    }else{
                        continue;
                    }
                }

                std::cout << "interrupt 2 " << std::endl;

                if(sendingGoal_ == true){
                    while(ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE){
                        if (paused_ == true){
                            ROS_INFO("jumped into paused_ true if after active");
                            ac_.cancelGoal();
                            ROS_INFO("goal is cancelled");
                            ac_.waitForResult();
                            ROS_INFO("stop done");
                            /*while (ac_.getState() != actionlib::SimpleClientGoalState::ABORTED){
                                continue;
                            }*/
                            sendingGoal_ = false;
                        }else{
                            continue;
                        }
                    }
                }

                std::cout << "interrupt 2 " << std::endl;

                if(paused_ == false && sendingGoal_ == true){
                    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                        ROS_INFO("Hooray, base reached goal");
                        count_++;
                    }
                }
            }else{
                continue;
            }
        }
    }

    void NavigationManager::stopGoal(){
        //if paused is set to true, the goal will cancel automatically
        if (sendingGoal_ == true){
            ROS_INFO("stopGoal called ");
            paused_ = true;
            ROS_INFO("paused_ is true");
            while (sendingGoal_ == true){
                boost::this_thread::yield();
                continue;
            }
            ROS_INFO("paused_ is false");
            paused_ = false;
        }
    }

    void NavigationManager::backOff(double back1_velocity, double back1_seconds, double forw_velocity, double forw_seconds, double back2_velocity, double back2_seconds){

        int count=0; //count seconds
        //TODO Geschwindigkeit einbauen, rotation im ersten goal
        geometry_msgs::Twist bo_goal_back1;
        geometry_msgs::Twist bo_goal_forward;
        geometry_msgs::Twist bo_goal_back2;
        geometry_msgs::Twist bo_goal_stop;

        bo_goal_back1.linear.x = back1_velocity;
        bo_goal_forward.linear.x = forw_velocity;
        bo_goal_back2.linear.x = back2_velocity;
        bo_goal_stop.linear.x = 0;

        if (sendingGoal_ == true){
            ROS_INFO("stopGoal called ");
            paused_ = true;
            ROS_INFO("paused_ is true");
            while (sendingGoal_ == true){
                boost::this_thread::yield();
                continue;
            }

            count = 0;

            while(count < back1_seconds*4){
                pub_vel_bo_.publish(bo_goal_back1);
                usleep(250*1000);
                count++;
            }

            ROS_INFO("first bo done ");

            count = 0;

            while(count < forw_seconds*4){
                pub_vel_bo_.publish(bo_goal_forward);
                usleep(250*1000);
                count++;
            }


            ROS_INFO("sec bo done ");

            count = 0;

            while(count < back2_seconds*2){
                pub_vel_bo_.publish(bo_goal_back2);
                usleep(250*1000);
                count++;
            }


            ROS_INFO("third bo done ");

            ROS_INFO("paused_ is false");
            paused_ = false;
        }
    }


    void NavigationManager::continueGoal(int seconds){
        if(!sendingGoal_ && !paused_){
            ROS_INFO("continueGoal called ");
            usleep(seconds*1000*1000);
            ROS_INFO("waited for 10 secs ");
            sendingGoal_ = true;
        }
    }

    void NavigationManager::setGoal1(double x, double y, double z, double orientation){
        goal1_.target_pose.pose.position.x = x;
        goal1_.target_pose.pose.position.y = y;
        goal1_.target_pose.pose.position.z = z;
        goal1_.target_pose.pose.orientation.z = orientation;
    }

    void NavigationManager::setGoal2(double x, double y, double z, double orientation){
        goal2_.target_pose.pose.position.x = x;
        goal2_.target_pose.pose.position.y = y;
        goal2_.target_pose.pose.position.z = z;
        goal2_.target_pose.pose.orientation.z = orientation;
    }

}