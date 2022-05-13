#include <aqc_input_dynamic_reconfigure_vel/aqc_input_client_dr_vel.hpp>

rsp::aqc_input_client_dr_vel::aqc_input_client_dr_vel(ros::NodeHandle& nh) : 
    nh(nh), setpoint_rate_x(0.0), setpoint_rate_y(0.0), setpoint_rate_z(0.0), setpoint_rate_yaw(0.0), first_call_flag(true) {

    pub_twist_setpoint = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    callback = boost::bind(&rsp::aqc_input_client_dr_vel::dr_server_callback, this, _1, _2);
    server.setCallback(callback);   

    arm_client.reset( new ArmClient(nh, "change_arm_status_action") );
    cfm_client.reset( new CfmClient(nh, "change_flight_mode_action") );

    ros::Duration server_timeout(5);
    if (!arm_client->waitForServer(server_timeout)) { 
        ROS_ERROR("'arm_client' within 'aqc_input_client_dr_vel' node could not connect to its server. Please ensure that 'aqc_coordinator' node is alive and running.");
    }
    if (!cfm_client->waitForServer(server_timeout)) {
        ROS_ERROR("'cfm_client' within 'aqc_input_client_dr_vel' node could not connect to its server. Please ensure that 'aqc_coordinator' node is alive and running.");
    }

}

rsp::aqc_input_client_dr_vel::~aqc_input_client_dr_vel() {}

void rsp::aqc_input_client_dr_vel::dr_server_callback(aqc_input_dynamic_reconfigure_vel::AqcInputClientVelConfig& config, uint32_t level) {

    // prevent any changes when node first launched
    if (first_call_flag) {
        first_call_flag = false;
        return;
    }

    // vel command
    if ((level & 1) == 1 || (level & 2) == 2 || (level & 4) == 4 || (level & 8) == 8) {
        setpoint_rate_x = config.east_rate_setpoint; 
        setpoint_rate_y = config.north_rate_setpoint;
        setpoint_rate_z = config.up_rate_setpoint;
        setpoint_rate_yaw = config.yaw_rate_setpoint;
    }

    // arm request
    if ((level & 16) == 16) {
        make_arm_request(config.arm_request);
    }

    // cfm request 
    if ((level & 32) == 32) {
        make_cfm_request(config.cfm_request);
    }

}

void rsp::aqc_input_client_dr_vel::publish_twist_setpoint() {

    geometry_msgs::Twist setpoint;
    setpoint.linear.x = setpoint_rate_x;
    setpoint.linear.y = setpoint_rate_y;
    setpoint.linear.z = setpoint_rate_z;
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = setpoint_rate_yaw;
    pub_twist_setpoint.publish(setpoint);

}

void rsp::aqc_input_client_dr_vel::make_arm_request(const int& arm_code) {

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

void rsp::aqc_input_client_dr_vel::make_cfm_request(const int& cfm_code) {

    aqc_msgs::ChangeFlightModeGoal goal;
    if (cfm_code == 0) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.TAKEOFF;
    } else if (cfm_code == 1) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.LAND;
    } else if (cfm_code == 2) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.RTL;
    } else if (cfm_code == 3) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.LOITER;
    } else if (cfm_code == 4) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.OFFBOARD;
    } else if (cfm_code == 5) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.MANUAL;
    } else if (cfm_code == 6) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.POSCTL;
    } else if (cfm_code == 7) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.STABILIZED;
    } else if (cfm_code == 8) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.ACRO;
    } else if (cfm_code == 9) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.ALTCTL;
    } else {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.LOITER;
    }
    cfm_client->sendGoal(goal);

}

