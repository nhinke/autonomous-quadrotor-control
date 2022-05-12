#include <aqc_mode_manager/aqc_mode_manager.hpp>

rsp::aqc_mode_manager::aqc_mode_manager(ros::NodeHandle& nh) : nh(nh) {

    pub_fcu_state = nh.advertise<mavros_msgs::State>("/fcu_state", 10);
    sub_fcu_state = nh.subscribe("/mavros/state", 10, &rsp::aqc_mode_manager::fcu_state_callback, this); 
    
    cfm_server = nh.advertiseService("change_fcu_mode", &rsp::aqc_mode_manager::change_flight_mode_service, this);
    arm_server = nh.advertiseService("change_fcu_arm_status", &rsp::aqc_mode_manager::change_arm_status_service, this);

    mavros_cfm_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    mavros_arm_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    ros::Duration server_timeout(5);
    if (!mavros_arm_client.waitForExistence(server_timeout)) { 
        ROS_ERROR("'mavros_arm_client' within 'aqc_mode_manager' server class could not connect to its server. Please ensure that 'mavros_node' is alive and running.");
    }
    if (!mavros_cfm_client.waitForExistence(server_timeout)) { 
        ROS_ERROR("'mavros_cfm_client' within 'aqc_mode_manager' server class could not connect to its server. Please ensure that 'mavros_node' is alive and running.");
    }

    mode_val_manual = nh.param<int>("fcu_mode_value_manual", 1);
    mode_val_acro = nh.param<int>("fcu_mode_value_acro", 2);
    mode_val_altctl = nh.param<int>("fcu_mode_value_altctl", 3);
    mode_val_position = nh.param<int>("fcu_mode_value_position", 4);
    mode_val_offboard = nh.param<int>("fcu_mode_value_offboard", 5);
    mode_val_stabilized = nh.param<int>("fcu_mode_value_stabilized", 6);
    mode_val_rattitude = nh.param<int>("fcu_mode_value_rattitude", 7);
    mode_val_mission = nh.param<int>("fcu_mode_value_mission", 8);
    mode_val_loiter = nh.param<int>("fcu_mode_value_loiter", 9);
    mode_val_rtl = nh.param<int>("fcu_mode_value_rtl", 10);
    mode_val_land = nh.param<int>("fcu_mode_value_land", 11);
    mode_val_rtgs = nh.param<int>("fcu_mode_value_rtgs", 12);
    mode_val_ready = nh.param<int>("fcu_mode_value_ready", 13);
    mode_val_takeoff = nh.param<int>("fcu_mode_value_takeoff", 14);

}

rsp::aqc_mode_manager::~aqc_mode_manager() {}

void rsp::aqc_mode_manager::fcu_state_callback(const mavros_msgs::State::ConstPtr& fcu_state_msg) {
    
    fcu_state = *fcu_state_msg;
    // std::cout << "\naqc_mode_manager received fcu_state:" << std::endl;
    // std::cout << fcu_state << std::endl;

    fcu_state_value = get_fcu_state_value(fcu_state.mode);
    // std::cout << "Check conversions:  " << fcu_state_value << " " << get_fcu_state_string(fcu_state_value) << std::endl;

    fcu_armed = fcu_state.armed;

    pub_fcu_state.publish(fcu_state);

}

bool rsp::aqc_mode_manager::change_flight_mode_service(aqc_msgs::ChangeMode::Request &req, aqc_msgs::ChangeMode::Response &res) {
    
    // std::cout << "\naqc_mode_manager received ChangeMode request:" << std::endl;
    // std::cout << int(req.fcu_mode.flight_mode_code) << " " << get_fcu_state_string(req.fcu_mode.flight_mode_code) << std::endl;

    mavros_msgs::SetMode mode_req;
    mode_req.request.custom_mode = get_fcu_state_string(req.fcu_mode.flight_mode_code);

    if (!fcu_armed) {
        ROS_WARN("WARNING: changing FCU flight mode while FCU is disarmed; FCU will remain in this flight mode which may cause accident if FCU armed later");
    }

    if (mavros_cfm_client.call(mode_req)) {
        res.success.data = true;
        std::cout << "\naqc_mode_manager returned success when changing FCU flight mode" << std::endl;
        return true;
    } else {
        res.success.data = false;
        std::cout << "\naqc_mode_manager returned failure when changing FCU flight mode" << std::endl;
        return false;
    }

}

bool rsp::aqc_mode_manager::change_arm_status_service(aqc_msgs::ArmOrDisarm::Request &req, aqc_msgs::ArmOrDisarm::Response &res) {

    // std::cout << "\naqc_mode_manager received ArmOrDisarm request:" << std::endl;
    // std::cout << req.arm.arm_code << std::endl;

    // true=arm and false=disarm
    mavros_msgs::CommandBool arm_req;
    arm_req.request.value = req.arm.arm_code;

    // TODO add more logic about what modes will allow you to arm and when calls to disarm should be allowed (although px4 should already handle all the safety stuff just fine)

    if (mavros_arm_client.call(arm_req)) {
        res.success.data = true;
        std::cout << "\naqc_mode_manager returned success when changing FCU arm status" << std::endl;
        return true;
    } else {
        res.success.data = false;
        std::cout << "\naqc_mode_manager returned failure when changing FCU arm status" << std::endl;
        return false;
    }

}

int rsp::aqc_mode_manager::get_fcu_state_value(const std::string &fcu_state_string) {
    
    if (fcu_state_string.compare(std::string("MANUAL")) == 0) {
        return mode_val_manual;
    } else if (fcu_state_string.compare(std::string("ACRO")) == 0) {
        return mode_val_acro;
    } else if (fcu_state_string.compare(std::string("ALTCTL")) == 0) {
        return mode_val_altctl;
    } else if (fcu_state_string.compare(std::string("POSCTL")) == 0) {
        return mode_val_position;
    } else if (fcu_state_string.compare(std::string("OFFBOARD")) == 0) {
        return mode_val_offboard;
    } else if (fcu_state_string.compare(std::string("STABILIZED")) == 0) {
        return mode_val_stabilized;
    } else if (fcu_state_string.compare(std::string("RATTITUDE")) == 0) {
        return mode_val_rattitude;
    } else if (fcu_state_string.compare(std::string("AUTO.MISSION")) == 0) {
        return mode_val_mission;
    } else if (fcu_state_string.compare(std::string("AUTO.LOITER")) == 0) {
        return mode_val_loiter;
    } else if (fcu_state_string.compare(std::string("AUTO.RTL")) == 0) {
        return mode_val_rtl;
    } else if (fcu_state_string.compare(std::string("AUTO.LAND")) == 0) {
        return mode_val_land;
    } else if (fcu_state_string.compare(std::string("AUTO.RTGS")) == 0) {
        return mode_val_rtgs;
    } else if (fcu_state_string.compare(std::string("AUTO.READY")) == 0) {
        return mode_val_ready;
    } else if (fcu_state_string.compare(std::string("AUTO.TAKEOFF")) == 0) {
        return mode_val_takeoff;
    } 
    
    return -1;
}

std::string rsp::aqc_mode_manager::get_fcu_state_string(const int &fcu_state_int) {

    if (fcu_state_int == mode_val_manual) {
        return std::string("MANUAL");
    } else if (fcu_state_int == mode_val_acro) {
        return std::string("ACRO");
    } else if (fcu_state_int == mode_val_altctl) {
        return std::string("ALTCTL");
    } else if (fcu_state_int == mode_val_position) {
        return std::string("POSCTL");
    } else if (fcu_state_int == mode_val_offboard) {
        return std::string("OFFBOARD");
    } else if (fcu_state_int == mode_val_stabilized) {
        return std::string("STABILIZED");
    } else if (fcu_state_int == mode_val_rattitude) {
        return std::string("RATTITUDE");
    } else if (fcu_state_int == mode_val_mission) {
        return std::string("AUTO.MISSION");
    } else if (fcu_state_int == mode_val_loiter) {
        return std::string("AUTO.LOITER");
    } else if (fcu_state_int == mode_val_rtl) {
        return std::string("AUTO.RTL");
    } else if (fcu_state_int == mode_val_land) {
        return std::string("AUTO.LAND");
    } else if (fcu_state_int == mode_val_rtgs) {
        return std::string("AUTO.RTGS");
    } else if (fcu_state_int == mode_val_ready) {
        return std::string("AUTO.READY");
    } else if (fcu_state_int == mode_val_takeoff) {
        return std::string("AUTO.TAKEOFF");
    }

    return std::string("UNDEFINED");

}
