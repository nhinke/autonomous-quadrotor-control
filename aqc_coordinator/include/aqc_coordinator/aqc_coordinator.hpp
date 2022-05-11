#include <algorithm>
#include <ros/ros.h>
// #include <sensor_msgs/Imu.h>

// for aqc_mode_manager services
#include <aqc_msgs/ChangeMode.h>
#include <aqc_msgs/ArmOrDisarm.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <aqc_msgs/ArmAction.h>
#include <aqc_msgs/ChangeFlightModeAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

namespace rsp {

    class aqc_coordinator {

        private:

            typedef actionlib::SimpleActionServer<aqc_msgs::ArmAction> ArmServer;
            typedef actionlib::SimpleActionServer<aqc_msgs::ChangeFlightModeAction> CFMServer;
            std::unique_ptr<ArmServer> arm_server;
            std::unique_ptr<CFMServer> cfm_server;

            ros::NodeHandle nh;
            ros::ServiceClient arm_client;
            ros::ServiceClient cfm_client;

            ros::Subscriber sub_fcu_odom;
            ros::Subscriber sub_fcu_state;

            // nav_msgs::Odometry fcu_odom;
            mavros_msgs::State fcu_state;
            geometry_msgs::PoseStamped fcu_pose;
            geometry_msgs::TwistStamped fcu_twist;

        protected:

            // void fcu_imu_callback(const sensor_msgs::Imu::ConstPtr& fcu_imu_msg);
            void fcu_odom_callback(const nav_msgs::Odometry::ConstPtr& fcu_odom_msg);
            void fcu_state_callback(const mavros_msgs::State::ConstPtr& fcu_state_msg);
            void change_arm_status_callback(const aqc_msgs::ArmGoalConstPtr &goal_received);
            void change_flight_mode_callback(const aqc_msgs::ChangeFlightModeGoal::ConstPtr& goal_received);

        public:

            aqc_coordinator(ros::NodeHandle& nh);
            ~aqc_coordinator();

    };

}
