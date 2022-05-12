#include <algorithm>
#include <ros/ros.h>
#include <aqc_msgs/PositionSetpoint.h>

#include <dynamic_reconfigure/server.h>
#include <aqc_input_dynamic_reconfigure/AqcInputClientConfig.h>

#include <aqc_msgs/ArmAction.h>
#include <aqc_msgs/ChangeFlightModeAction.h>
#include <actionlib/client/simple_action_client.h>

namespace rsp {

    class aqc_input_client_dr {

        private:

            ros::NodeHandle nh;

            ros::Publisher pub_pos_setpoint;
            double setpoint_x, setpoint_y, setpoint_z, setpoint_yaw;
    
            bool first_call_flag;
            dynamic_reconfigure::Server<aqc_input_dynamic_reconfigure::AqcInputClientConfig> server;
            dynamic_reconfigure::Server<aqc_input_dynamic_reconfigure::AqcInputClientConfig>::CallbackType callback;

            typedef actionlib::SimpleActionClient<aqc_msgs::ArmAction> ArmClient;
            typedef actionlib::SimpleActionClient<aqc_msgs::ChangeFlightModeAction> CfmClient;
            std::unique_ptr<ArmClient> arm_client;
            std::unique_ptr<CfmClient> cfm_client;

        protected:

            void publish_position_setpoint();      
            void make_arm_request(const int& arm_code);
            void make_cfm_request(const int& cfm_code);
            void dr_server_callback(aqc_input_dynamic_reconfigure::AqcInputClientConfig& config, uint32_t level);

        public:

            aqc_input_client_dr(ros::NodeHandle& nh);
            ~aqc_input_client_dr();      

    };

}
