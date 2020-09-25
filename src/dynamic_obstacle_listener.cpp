//
// Created by turtlebot on 06.09.20.
//

#include "../include/turtlebot2i_lfe_navigation/dynamic_obstacle_listener.h"

namespace lfe_navigation {

    DynamicObstacleListener::DynamicObstacleListener(ros::NodeHandle &nh, ros::NodeHandle& pnh) : dist_array_idx_count_(0), body_not_received_count_(0), paused_(false), lfeNavLogger_(nh), backOff_(true), config_init_(false), reconfigure_server_(NULL), navigationManager_(nh), human_dist_decrease_(false), ready_to_continue_(false), body_received_(false) {

        ros::NodeHandle nh_body(nh);

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

        boost::thread* subscriberThr = new boost::thread(boost::bind(&DynamicObstacleListener::subscriberLoop, this, nh_body));

        boost::thread* loopThr = new boost::thread(boost::bind(&DynamicObstacleListener::loop, this));

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

    void DynamicObstacleListener::subscriberLoop(ros::NodeHandle &nh){
        ros::Subscriber robotVelSubscriber = nh.subscribe("/navigation_velocity_smoother/raw_cmd_vel", 10, &DynamicObstacleListener::newVelocityCallback, this);
        ros::Subscriber bodySubscriber = nh.subscribe("/camera/body/skeleton", 10, &DynamicObstacleListener::newBodyMsgCallback, this);
        image_transport::ImageTransport it_depth(nh);
        image_transport::CameraSubscriber depthSubscriber = it_depth.subscribeCamera("/camera/depth/image_raw", 10, &DynamicObstacleListener::newDepthCallback, this);
        ros::spin();
    }

    void DynamicObstacleListener::loop() {

        ros::Rate rate(10.0);

        while (ros::ok()) {
            if (body_received_ == true){

                    if(human_robot_distance_ >=2 && ready_to_continue_ == true && !paused_){
                        std::cout << "human continue goal" << std::endl;
                        ready_to_continue_ = false;
                        dist_array_idx_count_ = 0;
                        human_dist_seq_.clear();
                        human_dist_time_seq_.clear();
                        robot_vel_seq_.clear();
                        human_dist_decrease_ = false;
                        lfeNavLogger_.finalize_log(cfg_.hri.backOff, human_mc_dist_seq_, human_mc_dist_time_seq_);
                        human_mc_dist_seq_.clear();
                        human_mc_dist_time_seq_.clear();
                        navigationManager_.continueGoal(cfg_.hri.wait_duration);

                        body_received_ = false;

                    }

                    body_not_received_count_++;

                    if (body_not_received_count_ > 1600000){
                        body_received_ = false;
                        body_not_received_count_ = 0;
                    }

                    boost::this_thread::yield();

            }else{

                if(body_received_ == false && ready_to_continue_ == true && !paused_){
                    std::cout << "no huamn continue goal" << std::endl;
                    ready_to_continue_ = false;
                    dist_array_idx_count_ = 0;
                    human_dist_seq_.clear();
                    human_dist_time_seq_.clear();
                    robot_vel_seq_.clear();
                    human_dist_decrease_ = false;
                    lfeNavLogger_.finalize_log(cfg_.hri.backOff, human_mc_dist_seq_, human_mc_dist_time_seq_);
                    human_mc_dist_seq_.clear();
                    human_mc_dist_time_seq_.clear();
                    navigationManager_.continueGoal(cfg_.hri.wait_duration);
                }
            }
        }
    }

    void DynamicObstacleListener::newVelocityCallback(geometry_msgs::Twist odom_msg){
        current_robot_vel_ = odom_msg.linear.x;
        boost::this_thread::yield();
    }

    void DynamicObstacleListener::newBodyMsgCallback(const ros_astra_sdk_wrapper::BodyTracking& bodyMsg){
        bool validLeftKnee = (bodyMsg.joint_position_lknee.x != 0 || bodyMsg.joint_position_lknee.y != 0 || bodyMsg.joint_position_lknee.z !=0) ? true : false;
        bool validRightKnee = (bodyMsg.joint_position_rknee.x != 0 || bodyMsg.joint_position_rknee.y != 0 || bodyMsg.joint_position_rknee.z != 0) ? true : false;

        int decrease = 0;
        std::string frame_id;
        double time_stamp_nsec;
        double time_stamp_sec;
        double time_stamp;
        double human_approach_vel;

        double robot_vel_sum = 0.0;
        double robot_vel_avg = 0.0;

        if( validLeftKnee && validRightKnee ){
            body_received_ = true;
            body_not_received_count_ = 0;
            time_stamp_nsec = bodyMsg.header.stamp.nsec;
            time_stamp_sec = bodyMsg.header.stamp.sec;
            frame_id = bodyMsg.header.frame_id;
            boost::this_thread::yield();

            if(validLeftKnee){
                human_robot_distance_ = bodyMsg.joint_position_base_spine.x;
                pxl_lknee_x_ = (int) bodyMsg.pxl_joint_position_lknee_x;
                pxl_lknee_y_ = (int) bodyMsg.pxl_joint_position_lknee_y;

                human_robot_distance_ = (int)(human_robot_distance_*100+0.5)/100.0; //round to two decimal places

                std::cout << "distance: " << human_robot_distance_ << std::endl;

                if (paused_ == true || ready_to_continue_ == true){
                    if(dist_array_idx_count_ <= 500){
                        human_mc_dist_seq_.push_back(human_robot_distance_);
                        time_stamp = time_stamp_sec + (time_stamp_nsec/1000000000.0);
                        human_mc_dist_time_seq_.push_back(time_stamp);
                        dist_array_idx_count_++;
                    }else{
                        body_received_ = false;
                    }

                }else{
                    if(human_dist_seq_.size() == 14){
                        human_dist_seq_.pop_back();
                    }

                    if(human_dist_time_seq_.size() == 14){
                        human_dist_time_seq_.pop_back();
                    }

                    if(robot_vel_seq_.size() == 14){
                        robot_vel_seq_.pop_back();
                    }

                    //store distance
                    human_dist_seq_.insert(human_dist_seq_.begin(), human_robot_distance_);

                    //store time stamp in seconds for every distance
                    time_stamp = time_stamp_sec + (time_stamp_nsec/1000000000.0);
                    human_dist_time_seq_.insert(human_dist_time_seq_.begin(), time_stamp);

                    //store robot velocity for every distance
                    robot_vel_seq_.insert(robot_vel_seq_.begin(), current_robot_vel_);

                    //check, if human approaches by checking, if distance is decreasing
                    for(std::vector<double>::iterator it = human_dist_seq_.begin(); it != (human_dist_seq_.end()-1); ++it) {
                        if (((*it)+(cfg_.hri.human_motion_frame_distance/100)) < *(it+1)){
                            decrease++;
                        }
                    }

                    //calculate average velocity of robot during the sequence
                    for(std::vector<double>::iterator it = robot_vel_seq_.begin(); it != (robot_vel_seq_.end()-1); ++it) {
                        robot_vel_sum += *it;
                    }
                    robot_vel_avg = robot_vel_sum / robot_vel_seq_.size();

                    //calculate human velocity
                    human_approach_vel = (*(human_dist_seq_.end()-1) - *human_dist_seq_.begin()) / (*human_dist_time_seq_.begin() - *(human_dist_time_seq_.end()-1));
                    human_approach_vel = (int)(human_approach_vel*100+0.5)/100.0;

                    //optionally, robot velocity can be subtracted from human approach vel
                    human_approach_vel = human_approach_vel;//-robot_vel_avg;

                    //std::cout << "approach vel: " << human_approach_vel << std::endl;

                    if (decrease > 12){
                        human_dist_decrease_ = true;
                    }else{
                        human_dist_decrease_ = false;
                    }

                    if(human_dist_decrease_ == true && human_robot_distance_ <= cfg_.hri.motion_cue_distance){
                        paused_ = true;
                        boost::thread* motionCueThr = new boost::thread(boost::bind(&DynamicObstacleListener::executeMotionCue, this, frame_id, human_approach_vel, robot_vel_avg));
                    }
                }
            }

            boost::this_thread::yield();

        }

    }

    void DynamicObstacleListener::newDepthCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg){
        const std::vector<unsigned char> data = depth_msg->data;
        std::vector<int> image_patch;
        int median_idx;
        int median;

        double time_stamp_nsec;
        double time_stamp_sec;
        double time_stamp;

        unsigned char tmp_char1;
        unsigned char tmp_char2;
        unsigned int tmp_int;

        time_stamp_nsec = depth_msg->header.stamp.nsec;
        time_stamp_sec = depth_msg->header.stamp.sec;

        if(paused_ == true || ready_to_continue_ == true){

            image_patch.clear();

            //get a patch of 48x64 pixels around the knee pixel
            for(int i=-24; i < 24; i++){
                for(int j=-32; j < 32; j++){
                    if((((pxl_lknee_x_+j)+(pxl_lknee_y_+i)*depth_msg->width)*2) < data.size() && ((((pxl_lknee_x_+j)+(pxl_lknee_y_+i)*depth_msg->width)*2)+1) < data.size() ) {
                        tmp_char1 = data[((pxl_lknee_x_ + j) + (pxl_lknee_y_ + i) * depth_msg->width) * 2];
                        tmp_char2 = data[(((pxl_lknee_x_ + j) + (pxl_lknee_y_ + i) * depth_msg->width) * 2) + 1];
                        tmp_int = ((unsigned int) ((((unsigned int)tmp_char2) << 8) & 0xFF00)) + (unsigned int) (((unsigned int) tmp_char1) & 0x00FF);
                        image_patch.push_back(tmp_int);
                    }else{
                        //TODO write exception
                        std::cout << "index out of bounds error" << std::endl;
                    }
                }
            }

            //take median of that patch as distance to be resilient to outliers
            std::sort(image_patch.begin(), image_patch.end());
            median_idx = ((image_patch.size()+(image_patch.size()+1))/2)-1;
            median = image_patch.at(median_idx);
            std::cout << "depth median: " << median << std::endl;

            //use depth stream median instead of body tracking msg, if distance < 1 meter, is more accurate
            if(median < 1000){
                if(dist_array_idx_count_ <= 100){
                    human_mc_dist_seq_.push_back(median/1000);
                    time_stamp = time_stamp_sec + (time_stamp_nsec/1000000000.0);
                    human_mc_dist_time_seq_.push_back(time_stamp);
                    dist_array_idx_count_++;
                }else{
                    body_received_ = false;
                }



                if (median != 0){
                    body_not_received_count_ = 0;
                }
            }
        }

        //TODO nach plötzlicher Intensitätsveränderung schauen
    }

    void DynamicObstacleListener::executeMotionCue(std::string frame_id, double human_approach_vel, double robot_vel_avg){

        ready_to_continue_ = false;
        ROS_INFO("goal stop is sent");

        if(backOff_){
            //TODO vector dist time seq logging
            //TODO vector dist seq logging
            lfeNavLogger_.bo_log_init(cfg_.backOff.back1_velocity, cfg_.backOff.back1_duration, cfg_.backOff.forw_velocity, cfg_.backOff.forw_duration, cfg_.backOff.back2_velocity, cfg_.backOff.back2_duration, cfg_.hri.wait_duration, human_dist_seq_, human_dist_time_seq_, human_approach_vel, robot_vel_avg, frame_id);
            navigationManager_.backOff(cfg_.backOff.back1_velocity, cfg_.backOff.back1_duration, cfg_.backOff.forw_velocity, cfg_.backOff.forw_duration, cfg_.backOff.back2_velocity, cfg_.backOff.back2_duration);
        }else{

            navigationManager_.stopGoal();
        }

        ROS_INFO("stop goal finished");
        paused_ = false;
        ready_to_continue_ = true;
    }

    void DynamicObstacleListener::setMotionCue(bool backOff){
        backOff_ = backOff;
    }

}