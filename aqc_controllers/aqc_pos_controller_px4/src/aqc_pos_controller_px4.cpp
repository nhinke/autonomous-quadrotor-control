#include <aqc_pos_controller_px4/aqc_pos_controller_px4.hpp>

rsp::aqc_pos_controller_px4_dr::aqc_pos_controller_px4_dr(ros::NodeHandle& nh) : 
    nh(nh), setpoint_x(0.0), setpoint_y(0.0), setpoint_z(5.0), setpoint_yaw(0.0), first_call_flag(true) {

    arm_client.reset( new ArmClient(nh, "change_arm_status_action") );
    cfm_client.reset( new CfmClient(nh, "change_flight_mode_action") );

    // sub_pos_setpoint = nh.subscribe<aqc_msgs::PositionSetpoint>("/cmd_position", 10, &rsp::aqc_pos_controller_px4_dr::position_setpoint_callback, this);
    pub_pos_setpoint = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    pub_raw_pos_setpoint = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);

    callback = boost::bind(&rsp::aqc_pos_controller_px4_dr::dr_server_callback, this, _1, _2);
    server.setCallback(callback);   

    ros::Duration server_timeout(5);
    if (!arm_client->waitForServer(server_timeout)) { 
        ROS_ERROR("'arm_client' within 'aqc_pos_controller_px4' node could not connect to its server. Please ensure that 'aqc_coordinator' node is alive and running.");
    }
    if (!cfm_client->waitForServer(server_timeout)) {
        ROS_ERROR("'cfm_client' within 'aqc_pos_controller_px4' node could not connect to its server. Please ensure that 'aqc_coordinator' node is alive and running.");
    }

}

rsp::aqc_pos_controller_px4_dr::~aqc_pos_controller_px4_dr() {}

void rsp::aqc_pos_controller_px4_dr::dr_server_callback(aqc_pos_controller_px4::DrControllerConfig& config, uint32_t level) {

    // prevent any changes when node first launched
    if (first_call_flag) {
        first_call_flag = false;
        return;
    }

    if ((level & 1) == 1) {
        // std::cout << "\nnew east setpoint: " << (config.east_setpoint) << std::endl;
        setpoint_x = config.east_setpoint;
    }

    if ((level & 2) == 2) {
        // std::cout << "\nnew north setpoint: " << (config.north_setpoint) << std::endl;
        setpoint_y = config.north_setpoint;
    }

    if ((level & 4) == 4) {
        // std::cout << "\nnew up setpoint: " << (config.up_setpoint) << std::endl;
        setpoint_z = config.up_setpoint;
    }

    if ((level & 8) == 8) {
        std::cout << "\nnew yaw setpoint: " << (config.yaw_setpoint) << std::endl;
        setpoint_yaw = config.yaw_setpoint;
    }

    if ((level & 16) == 16) {
        std::cout << "\nnew arm request: " << (config.arm_request) << std::endl;
        make_arm_request(config.arm_request);
    }

    if ((level & 32) == 32) {
        // std::cout << "\nnew cfm request: " << (config.cfm_request) << std::endl;
        make_cfm_request(config.cfm_request);
    }

}

void rsp::aqc_pos_controller_px4_dr::make_arm_request(const int& arm_code) {

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

void rsp::aqc_pos_controller_px4_dr::make_cfm_request(const int& cfm_code) {

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

void rsp::aqc_pos_controller_px4_dr::position_setpoint_callback(const aqc_msgs::PositionSetpoint::ConstPtr& pos_setpoint) {

    aqc_msgs::PositionSetpoint setpoint = *pos_setpoint;
    
    setpoint_x = setpoint.ENU_position.x;
    setpoint_y = setpoint.ENU_position.y;
    setpoint_z = setpoint.ENU_position.z;
    setpoint_yaw = setpoint.yaw_orientation;

}

void rsp::aqc_pos_controller_px4_dr::publish_setpoint() {

    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    // mavros_msgs::PositionTarget::IGNORE_YAW |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    msg.position.x = setpoint_x;
    msg.position.y = setpoint_y;
    msg.position.z = setpoint_z;
    msg.yaw = setpoint_yaw;

    msg.header.stamp = ros::Time::now();
    pub_raw_pos_setpoint.publish(msg);

}

// void rsp::aqc_pos_controller_px4_dr::publish_setpoint() {

//     geometry_msgs::PoseStamped setpoint;
//     setpoint.pose.position.x = setpoint_x;
//     setpoint.pose.position.y = setpoint_y;
//     setpoint.pose.position.z = setpoint_z;
//     setpoint.pose.orientation = tf::createQuaternionMsgFromYaw(setpoint_yaw);

//     setpoint.header.stamp = ros::Time::now();

//     pub_pos_setpoint.publish(setpoint);

// }
