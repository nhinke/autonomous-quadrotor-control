#include <algorithm>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <aqc_msgs/PositionSetpoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>

#include <dynamic_reconfigure/server.h>
#include <aqc_pos_controller_px4/DrControllerConfig.h>

#include <aqc_msgs/ArmAction.h>
#include <aqc_msgs/ChangeFlightModeAction.h>
#include <actionlib/client/simple_action_client.h>

// future note: switch aqc_pos_controller_px4 (package name) and aqc_pos_controller_px4_dr (class and file names)

namespace rsp {

    class aqc_pos_controller_px4_dr {

        private:

            ros::NodeHandle nh;

            ros::Publisher pub_pos_setpoint;
            ros::Publisher pub_raw_pos_setpoint;
            ros::Subscriber sub_pos_setpoint;

            double setpoint_x, setpoint_y, setpoint_z, setpoint_yaw;

            bool first_call_flag;
            dynamic_reconfigure::Server<aqc_pos_controller_px4::DrControllerConfig> server;
            dynamic_reconfigure::Server<aqc_pos_controller_px4::DrControllerConfig>::CallbackType callback;

            typedef actionlib::SimpleActionClient<aqc_msgs::ArmAction> ArmClient;
            typedef actionlib::SimpleActionClient<aqc_msgs::ChangeFlightModeAction> CfmClient;
            std::unique_ptr<ArmClient> arm_client;
            std::unique_ptr<CfmClient> cfm_client;

        protected:

            void make_arm_request(const int& arm_code);
            void make_cfm_request(const int& cfm_code);
            void dr_server_callback(aqc_pos_controller_px4::DrControllerConfig& config, uint32_t level);
            void position_setpoint_callback(const aqc_msgs::PositionSetpoint::ConstPtr& pos_setpoint);

        public:

            aqc_pos_controller_px4_dr(ros::NodeHandle& nh);
            ~aqc_pos_controller_px4_dr();
            void publish_setpoint();

    };

}
