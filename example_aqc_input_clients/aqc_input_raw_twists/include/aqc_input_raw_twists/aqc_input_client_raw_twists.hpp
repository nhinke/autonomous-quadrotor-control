#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <aqc_msgs/FullTwistCommand.h>

#include <aqc_msgs/ArmAction.h>
#include <aqc_msgs/ChangeFlightModeAction.h>
#include <actionlib/client/simple_action_client.h>

namespace rsp {

    class aqc_input_client_raw_twists {

        private:

            ros::NodeHandle nh;

            ros::Subscriber sub_cmd_full;
            ros::Publisher pub_cmd_twist;

            geometry_msgs::Twist twist_setpoint;

            typedef actionlib::SimpleActionClient<aqc_msgs::ArmAction> ArmClient;
            typedef actionlib::SimpleActionClient<aqc_msgs::ChangeFlightModeAction> CfmClient;
            std::unique_ptr<ArmClient> arm_client;
            std::unique_ptr<CfmClient> cfm_client;

        protected:

            void make_arm_request(const int& arm_code);
            void make_cfm_request(const int& cfm_code);
            void new_command_setpoint_callback(const aqc_msgs::FullTwistCommand::ConstPtr& cmd_msg);
        
        public:

            aqc_input_client_raw_twists(ros::NodeHandle& nh);            
            ~aqc_input_client_raw_twists();
            void publish_twist_setpoint();      

    };

}
