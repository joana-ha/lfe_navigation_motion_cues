//
// Created by turtlebot on 06.09.20.
//

#ifndef SRC_DYNAMIC_OBSTACLE_LISTENER_H
#define SRC_DYNAMIC_OBSTACLE_LISTENER_H

//ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <turtlebot2i_lfe_navigation/LfeNavReconfigureConfig.h>

//ros_astra_sdk_wrapper
#include <ros_astra_sdk_wrapper/BodyTracking.h>

//internal
#include "./navigation_manager.h"
#include "./lfe_nav_config.h"
#include "./lfe_nav_logger.h"

namespace lfe_navigation{

typedef turtlebot2i_lfe_navigation::LfeNavReconfigureConfig Config;
typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

/**
 * @class DynamicObstacleListener
 * @brief Listens to /camera/body/skeleton messages and calculates distance to robot
 */

    class DynamicObstacleListener {

    public:

        /**
         * @brief Construct DynamicObstacleListener and set up the subscriber Object
         * @param sub Nodehandle to subscribe to /camera/body/skeleton
         * @todo initialize loop thread
         */
        DynamicObstacleListener(ros::NodeHandle& nh, ros::NodeHandle& pnh);

        ~DynamicObstacleListener();


    private:

        //config
        bool backOff_;

        //state variables
        bool body_received_;
        bool ready_to_continue_;
        bool paused_;
        bool human_dist_decrease_;

        //counters for state variables
        int body_not_received_count_;

        //logging
        LfeNavLogger lfeNavLogger_;

        NavigationManager navigationManager_;

        double human_robot_distance_;
        int pxl_lknee_x_;
        int pxl_lknee_y_;
        double current_robot_vel_;
        std::vector<double> human_dist_seq_;
        std::vector<double> human_dist_time_seq_;
        std::vector<double> robot_vel_seq_;

        std::vector<double> human_mc_dist_seq_;
        std::vector<double> human_mc_dist_time_seq_;

        //dynamic reconfigure
        LfeNavConfig cfg_;
        boost::shared_ptr<ReconfigureServer> reconfigure_server_;
        bool config_init_;

        //internal
        std::vector<double> avg_img_depth_seq_;

        void setMotionCue(bool backOff);

        /**
          * @brief Listens to messages from /camera/body/skeleton, sets body_received_ to true
          */
        void subscriberLoop(ros::NodeHandle& sub);

        /**
          * @brief Listens to messages from /tf
          */
        void loop();

        /**
          * @todo callback to calculate distance
          */

        /**
         * @brief Callback to calculate distance, if bodymsg is received
         * @param msg message received from subscriber
         */
        void newBodyMsgCallback(const ros_astra_sdk_wrapper::BodyTracking& bodyMsg);

        void newVelocityCallback(geometry_msgs::Twist odom_msg);

        void newDepthCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                              const sensor_msgs::CameraInfoConstPtr& info_msg);

        /**
          * @brief Callback for the dynamic configuration.
          *
          * This callback allows to modify parameters dynamically at runtime without restarting the node
          * @param config Reference to the dynamic reconfigure config
          */
        void configCb(turtlebot2i_lfe_navigation::LfeNavReconfigureConfig& config);

        void executeMotionCue(std::string frame_id, double human_approach_vel, double robot_vel_avg);

    };

}




#endif //SRC_DYNAMIC_OBSTACLE_LISTENER_H
