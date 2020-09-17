//
// Created by turtlebot on 06.09.20.
//

#include "../include/turtlebot2i_lfe_navigation/dynamic_obstacle_listener.h"

namespace lfe_navigation {

    DynamicObstacleListener::DynamicObstacleListener(ros::NodeHandle &nh, ros::NodeHandle& pnh) : backOff_(true), config_init_(false), reconfigure_server_(NULL), navigationManager_(nh), human_dist_decrease_(false), ready_to_continue_(false), body_received_(false) {

        // get parameters of LfeNavConfig via the nodehandle and override the default config
        cfg_.loadRosParamFromNodeHandle(pnh);

        // Initialize dynamic reconfigure
        reconfigure_server_ = boost::make_shared<ReconfigureServer> (pnh);
        ReconfigureServer::CallbackType cb = boost::bind(&DynamicObstacleListener::configCb, this, _1);
        reconfigure_server_->setCallback(cb);

        while (!config_init_)
        {
            ROS_DEBUG("Waiting for dynamic reconfigure configuration.");
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
        ROS_DEBUG("Dynamic reconfigure configuration received.");

        std::cout << "Dynamic Reconfigure Server is up. Please make configurations now and press ENTER when you are ready." << std::endl;

        std::cin.get();

        std::cout << "Please remove all cables from the robot and press ENTER to start robot navigation." << std::endl;

        std::cin.get();

        boost::thread* bodyLoopThr = new boost::thread(boost::bind(&DynamicObstacleListener::bodyLoop, this, nh));

        boost::thread* tfLoopThr = new boost::thread(boost::bind(&DynamicObstacleListener::tfLoop, this));

        navigationManager_.sendGoal();

    };

    DynamicObstacleListener::~DynamicObstacleListener() {

    };

    void DynamicObstacleListener::configCb (turtlebot2i_lfe_navigation::LfeNavReconfigureConfig& config){
        cfg_.reconfigure(config);
        navigationManager_.setGoal1(cfg_.navigation.goal1_pos_x, cfg_.navigation.goal1_pos_y, cfg_.navigation.goal1_pos_z, cfg_.navigation.goal1_orientation);
        navigationManager_.setGoal2(cfg_.navigation.goal2_pos_x, cfg_.navigation.goal2_pos_y, cfg_.navigation.goal2_pos_z, cfg_.navigation.goal2_orientation);
        setMotionCue(cfg_.hri.backOff);
        config_init_ = true;
    }

    void DynamicObstacleListener::bodyLoop(ros::NodeHandle &nh){
        //std::cout << "jumped into Body Loop" << std::endl;
        //todo lock mutex ?
        ros_astra_sdk_wrapper::BodyTracking msg;
        //define cb function with boost ?
        ros::Subscriber bodySubscriber = nh.subscribe("/camera/body/skeleton", 10, &DynamicObstacleListener::newBodyMsgCallback, this);
        ros::spin();
    }

    void DynamicObstacleListener::tfLoop() {
        //std::cout << "jumped into tf Loop" << std::endl;

        ros::Rate rate(10.0);

        while (ros::ok()) {
            if (body_received_ == true){
                    boost::this_thread::yield();

                    if(human_robot_distance_ >=2 && ready_to_continue_ == true && !navigationManager_.getPaused()){
                        ready_to_continue_ = false;
                        human_dist_seq_.clear();
                        human_dist_decrease_ = false;
                        navigationManager_.continueGoal(cfg_.hri.wait_duration);

                    }

            }else{
                //testen ob lange kein body received
                if(body_received_ == false && ready_to_continue_ == true && !navigationManager_.getPaused()){
                    ready_to_continue_ = false;
                    human_dist_seq_.clear();
                    human_dist_decrease_ = false;
                    navigationManager_.continueGoal(cfg_.hri.wait_duration);
                }
            }
            body_received_ = false;
        }
    }

    //todo include BodyTracking
    //todo body cb function, ausgeben von body distance, coordinatensysteme ? mit in tf aufnehmen ?
    void DynamicObstacleListener::newBodyMsgCallback(const ros_astra_sdk_wrapper::BodyTracking& bodyMsg){
        bool validLeftKnee = (bodyMsg.joint_position_lknee.x != 0 || bodyMsg.joint_position_lknee.y != 0 || bodyMsg.joint_position_lknee.z !=0) ? true : false;
        bool validRightKnee = (bodyMsg.joint_position_rknee.x != 0 || bodyMsg.joint_position_rknee.y != 0 || bodyMsg.joint_position_rknee.z != 0) ? true : false;

        int decrease = 0;

        if( validLeftKnee && validRightKnee ){
            body_received_ = true;
            boost::this_thread::yield();
            //ROS_INFO("Distance is calculated... ");

            if(validLeftKnee){
                human_robot_distance_ = bodyMsg.joint_position_base_spine.x;

                if(!navigationManager_.getPaused()) std::cout << "distance: " << human_robot_distance_ << std::endl;

                if(human_dist_seq_.size() == 5){
                    human_dist_seq_.pop_back();
                }

                human_robot_distance_ = (int)(human_robot_distance_*100+0.5)/100.0; //round to two decimal places
                human_dist_seq_.insert(human_dist_seq_.begin(), human_robot_distance_);


                for(std::vector<double>::iterator it = human_dist_seq_.begin(); it != (human_dist_seq_.end()-1); ++it) {
                    if (((*it)+(cfg_.hri.human_motion_frame_distance/100)) < *(it+1)){
                        decrease++;
                    }
                }

                if (decrease > 3){
                    human_dist_decrease_ = true;
                }else{
                    human_dist_decrease_ = false;
                }

            }

            if(!navigationManager_.getPaused()) std::cout << "decrease ?: " << decrease << std::endl;
                //prev_human_robot_d
                // istance_ = human_robot_distance_;

            if(human_dist_decrease_ == true && human_robot_distance_ <= cfg_.hri.motion_cue_distance && !navigationManager_.getPaused()){
                ready_to_continue_ = false;
                ROS_INFO("goal stop is sent");

                if(backOff_){
                    navigationManager_.backOff(cfg_.backOff.back1_velocity, cfg_.backOff.back1_duration, cfg_.backOff.forw_velocity, cfg_.backOff.forw_duration, cfg_.backOff.back2_velocity, cfg_.backOff.back2_duration);
                }else{
                    navigationManager_.stopGoal();
                }

                ROS_INFO("stop goal finsihed");
                ready_to_continue_ = true;

            }

            boost::this_thread::yield();

            //std::cout << "Current robot position x: " << tf_.getOrigin().getX() << std::endl << "Current robot position y: " << std::endl << tf_.getOrigin().getY() << std::endl;


        }
        body_received_ = false;
        //ROS_INFO("body received is false");

    }

    void DynamicObstacleListener::setMotionCue(bool backOff){
        backOff_ = backOff;
    }

}