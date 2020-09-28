//
// Created by turtlebot on 06.09.20.
//

#include "../include/turtlebot2i_lfe_navigation/navigation_manager.h"

namespace lfe_navigation{

    NavigationManager::NavigationManager(ros::NodeHandle &nh) : count_(0), paused_(false), sendingGoal_(false), ac_("move_base", true) {

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

        goal3_.target_pose.header.frame_id = "map";
        goal3_.target_pose.header.stamp = ros::Time::now();

        goal3_.target_pose.pose.position.x = 0.0;
        goal3_.target_pose.pose.position.y = 0.0;
        goal3_.target_pose.pose.position.z = 0.0;
        goal3_.target_pose.pose.orientation.z = 0.0;
        goal3_.target_pose.pose.orientation.w = 1.0;

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

        std::cout << "goal 1:" << std::to_string(goal1_.target_pose.pose.position.x) << "," << std::to_string(goal1_.target_pose.pose.position.y) << "," << std::to_string(goal1_.target_pose.pose.position.z) << "," << std::to_string(goal1_.target_pose.pose.orientation.z) << std::endl;
        std::cout << "goal 2:" << std::to_string(goal2_.target_pose.pose.position.x) << "," << std::to_string(goal2_.target_pose.pose.position.y) << "," << std::to_string(goal2_.target_pose.pose.position.z) << "," << std::to_string(goal2_.target_pose.pose.orientation.z) << std::endl;
        std::cout << "goal 3:" << std::to_string(goal3_.target_pose.pose.position.x) << "," << std::to_string(goal3_.target_pose.pose.position.y) << "," << std::to_string(goal3_.target_pose.pose.position.z) << "," << std::to_string(goal3_.target_pose.pose.orientation.z) << std::endl;



        while (ros::ok()){
            if(sendingGoal_ == true){
                if (count_%3 == 0){
                    ac_.sendGoal(goal1_);
                    ROS_INFO("Goal 1 sent");
                }

                if (count_%3 == 1){
                    ac_.sendGoal(goal2_);
                    ROS_INFO("Goal 2 sent");
                }

                if (count_%3 == 2){
                    ac_.sendGoal(goal3_);
                    ROS_INFO("Goal 3 sent");
                }

                boost::this_thread::yield();

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

                boost::this_thread::yield();

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

                boost::this_thread::yield();

                if(paused_ == false && sendingGoal_ == true){
                    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                        ROS_INFO("Hooray, base reached goal");
                        count_++;
                    }
                }
            }else{
                boost::this_thread::yield();
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

    void NavigationManager::backOff(double back_velocity, double back_seconds){

        int count=0; //count seconds
        //TODO rotation einbauen ?
        geometry_msgs::Twist bo_goal_back;

        bo_goal_back.linear.x = back_velocity;

        if (sendingGoal_ == true){
            ROS_INFO("stopGoal called ");
            paused_ = true;
            ROS_INFO("paused_ is true");
            while (sendingGoal_ == true){
                boost::this_thread::yield();
                continue;
            }

            count = 0;

            std::cout << "back seconds " << back_seconds << std::endl;

            while(count < back_seconds*4){
                ROS_INFO("sending first bo ");
                pub_vel_bo_.publish(bo_goal_back);
                usleep(250*1000);
                count++;
            }

            ROS_INFO("first bo done ");

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

    void NavigationManager::setGoal3(double x, double y, double z, double orientation){
        goal3_.target_pose.pose.position.x = x;
        goal3_.target_pose.pose.position.y = y;
        goal3_.target_pose.pose.position.z = z;
        goal3_.target_pose.pose.orientation.z = orientation;
    }

}