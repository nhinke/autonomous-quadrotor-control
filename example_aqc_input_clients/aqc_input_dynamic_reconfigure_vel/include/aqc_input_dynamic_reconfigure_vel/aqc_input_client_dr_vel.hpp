#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <dynamic_reconfigure/server.h>
#include <aqc_input_dynamic_reconfigure_vel/AqcInputClientVelConfig.h>

#include <aqc_msgs/ArmAction.h>
#include <aqc_msgs/ChangeFlightModeAction.h>
#include <actionlib/client/simple_action_client.h>

namespace rsp {

    class aqc_input_client_dr_vel {

        private:

            ros::NodeHandle nh;

            ros::Publisher pub_twist_setpoint;
            double setpoint_rate_x, setpoint_rate_y, setpoint_rate_z, setpoint_rate_yaw;

            bool first_call_flag;
            dynamic_reconfigure::Server<aqc_input_dynamic_reconfigure_vel::AqcInputClientVelConfig> server;
            dynamic_reconfigure::Server<aqc_input_dynamic_reconfigure_vel::AqcInputClientVelConfig>::CallbackType callback;

            typedef actionlib::SimpleActionClient<aqc_msgs::ArmAction> ArmClient;
            typedef actionlib::SimpleActionClient<aqc_msgs::ChangeFlightModeAction> CfmClient;
            std::unique_ptr<ArmClient> arm_client;
            std::unique_ptr<CfmClient> cfm_client;

        protected:

            void make_arm_request(const int& arm_code);
            void make_cfm_request(const int& cfm_code);
            void dr_server_callback(aqc_input_dynamic_reconfigure_vel::AqcInputClientVelConfig& config, uint32_t level);

        public:

            aqc_input_client_dr_vel(ros::NodeHandle& nh);
            ~aqc_input_client_dr_vel();    
            void publish_twist_setpoint();      

    };

}
