#include <string.h>
#include <algorithm>
#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <aqc_msgs/ChangeMode.h>
#include <aqc_msgs/ArmOrDisarm.h>

// #include <aqc_msgs/ChangeFlightModeAction.h>
// #include <actionlib/server/simple_action_server.h>
// #include <actionlib/client/simple_action_client.h>

namespace rsp {

    class aqc_mode_manager {

        private:

            int mode_val_manual;
            int mode_val_acro;
            int mode_val_altctl;
            int mode_val_position;
            int mode_val_offboard;
            int mode_val_stabilized;
            int mode_val_rattitude;
            int mode_val_mission;
            int mode_val_loiter;
            int mode_val_rtl;
            int mode_val_land;
            int mode_val_rtgs;
            int mode_val_ready;
            int mode_val_takeoff;
        
            ros::NodeHandle nh;
            ros::Subscriber sub_fcu_state;
            ros::ServiceServer cfm_server;
            ros::ServiceServer arm_server;
            ros::ServiceClient mavros_cfm_client;
            ros::ServiceClient mavros_arm_client;

            bool fcu_armed;
            int fcu_state_value;
            mavros_msgs::State fcu_state;
            std::string get_fcu_state_string(const int &fcu_state_int);
            int get_fcu_state_value(const std::string &fcu_state_string);

        protected:

            void fcu_state_callback(const mavros_msgs::State::ConstPtr& fcu_state_msg);
            bool change_flight_mode_service(aqc_msgs::ChangeMode::Request &req, aqc_msgs::ChangeMode::Response &res);
            bool change_arm_status_service(aqc_msgs::ArmOrDisarm::Request &req, aqc_msgs::ArmOrDisarm::Response &res);

        public:

            aqc_mode_manager(ros::NodeHandle& nh);
            ~aqc_mode_manager();

    };

}
