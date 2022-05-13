#include <aqc_input_raw_twists/aqc_input_client_raw_twists.hpp>

rsp::aqc_input_client_raw_twists::aqc_input_client_raw_twists(ros::NodeHandle& nh) : nh(nh) {

    twist_setpoint.linear.x = 0.0;
    twist_setpoint.linear.y = 0.0;
    twist_setpoint.linear.z = 0.0;
    twist_setpoint.angular.x = 0.0;
    twist_setpoint.angular.y = 0.0;
    twist_setpoint.angular.z = 0.0;

    sub_cmd_full = nh.subscribe("/input_cmd_twist", 10, &rsp::aqc_input_client_raw_twists::new_command_setpoint_callback, this);
    pub_cmd_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    arm_client.reset( new ArmClient(nh, "change_arm_status_action") );
    cfm_client.reset( new CfmClient(nh, "change_flight_mode_action") );

    ros::Duration server_timeout(5);
    if (!arm_client->waitForServer(server_timeout)) { 
        ROS_ERROR("'arm_client' within 'aqc_input_client_raw_twists' node could not connect to its server. Please ensure that 'aqc_coordinator' node is alive and running.");
    }
    if (!cfm_client->waitForServer(server_timeout)) {
        ROS_ERROR("'cfm_client' within 'aqc_input_client_raw_twists' node could not connect to its server. Please ensure that 'aqc_coordinator' node is alive and running.");
    }

}

rsp::aqc_input_client_raw_twists::~aqc_input_client_raw_twists() {}

void rsp::aqc_input_client_raw_twists::new_command_setpoint_callback(const aqc_msgs::FullTwistCommand::ConstPtr& cmd_msg) {

    aqc_msgs::FullTwistCommand cmd = *cmd_msg;

    if (cmd.ignore_arm != cmd.IGNORE) {
        int arm_code = cmd.arm_code.arm_code;
        make_arm_request(arm_code);
    }

    if (cmd.ignore_cfm != cmd.IGNORE) {
        int cfm_code = cmd.flight_mode.flight_mode_code;
        make_cfm_request(cfm_code);
    }

    if (cmd.ignore_twist != cmd.IGNORE) {
        twist_setpoint = cmd.cmd_vel;
    }

}

void rsp::aqc_input_client_raw_twists::make_arm_request(const int& arm_code) {

    aqc_msgs::ArmGoal goal;
    if (arm_code == 0) {
        goal.arm_goal.arm_code = goal.arm_goal.DISARM;
    } else if (arm_code == 1) {
        goal.arm_goal.arm_code = goal.arm_goal.ARM;
    } else {
        goal.arm_goal.arm_code = goal.arm_goal.DISARM;
    }
    arm_client->sendGoal(goal);

}

void rsp::aqc_input_client_raw_twists::make_cfm_request(const int& cfm_code) {

    // codes can be found in aqcs_msgs/msg/FCUmode.msg
    aqc_msgs::ChangeFlightModeGoal goal;
    goal.cfm_goal.flight_mode.flight_mode_code = cfm_code;
    cfm_client->sendGoal(goal);

}

void rsp::aqc_input_client_raw_twists::publish_twist_setpoint() {

    pub_cmd_twist.publish(twist_setpoint);

}
