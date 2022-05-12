#include <aqc_coordinator/aqc_coordinator.hpp>

// rostopic pub -1 /change_flight_mode_action/goal aqc_msgs/ChangeFlightModeActionGoal <tab-tab>
// rostopic pub -1 /change_arm_status_action/goal aqc_msgs/ArmActionGoal <tab-tab>

rsp::aqc_coordinator::aqc_coordinator(ros::NodeHandle& nh) : nh(nh) {

    cfm_client = nh.serviceClient<aqc_msgs::ChangeMode>("change_fcu_mode");
    arm_client = nh.serviceClient<aqc_msgs::ArmOrDisarm>("change_fcu_arm_status");

    arm_server.reset( new ArmServer(nh, "change_arm_status_action", boost::bind(&rsp::aqc_coordinator::change_arm_status_callback, this, _1), false) );
    cfm_server.reset( new CFMServer(nh, "change_flight_mode_action", boost::bind(&rsp::aqc_coordinator::change_flight_mode_callback, this, _1), false) );

    pub_quad_yaw = nh.advertise<aqc_msgs::YawStamped>("/quad_state/yaw", 10);
    pub_quad_pos = nh.advertise<aqc_msgs::PositionStamped>("/quad_state/position", 10);

    sub_fcu_state = nh.subscribe("/mavros/state", 10, &rsp::aqc_coordinator::fcu_state_callback, this); 
    sub_fcu_odom = nh.subscribe("/mavros/global_position/local", 10, &rsp::aqc_coordinator::fcu_odom_callback, this); 

    ros::Duration server_timeout(5);
    if (!arm_client.waitForExistence(server_timeout)) { 
        ROS_ERROR("'arm_client' within 'aqc_coordinator' server class could not connect to its server. Please ensure that 'aqc_mode_manager_node' is alive and running.");
    }
    if (!cfm_client.waitForExistence(server_timeout)) { 
        ROS_ERROR("'cfm_client' within 'aqc_coordinator' server class could not connect to its server. Please ensure that 'aqc_mode_manager_node' is alive and running.");
    }

    arm_server->start();
    cfm_server->start();

}

rsp::aqc_coordinator::~aqc_coordinator() {}

void rsp::aqc_coordinator::fcu_state_callback(const mavros_msgs::State::ConstPtr& fcu_state_msg) {
    
    fcu_state = *fcu_state_msg;
    // std::cout << "\naqc_coordinator received fcu_state:" << std::endl;
    // std::cout << fcu_state << std::endl;

    // note: not as complete as identical method in aqc_mode_manager --> b/c this probably isn't necessary here

}

void rsp::aqc_coordinator::fcu_odom_callback(const nav_msgs::Odometry::ConstPtr& fcu_odom_msg) {

    nav_msgs::Odometry fcu_odom = *fcu_odom_msg;

    fcu_pose.header.stamp = fcu_odom.header.stamp;
    fcu_twist.header.stamp = fcu_odom.header.stamp;

    fcu_pose.header.frame_id = fcu_odom.child_frame_id;
    fcu_twist.header.frame_id = fcu_odom.child_frame_id;

    // fcu_odom has PoseWithCovariance and TwistWithCovariance
    fcu_pose.pose = fcu_odom.pose.pose;
    fcu_twist.twist = fcu_odom.twist.twist;

    aqc_msgs::PositionStamped pos_msg;
    pos_msg.header = fcu_odom.header;
    pos_msg.ENU_position = fcu_pose.pose.position;

    // double fcu_yaw = tf::getYaw(fcu_pose.pose.orientation);
    // std::cout << "\naqc_coordinator received yaw: " << fcu_yaw << std::endl;

    aqc_msgs::YawStamped yaw_msg;
    yaw_msg.header = fcu_odom.header;
    yaw_msg.yaw = tf::getYaw(fcu_pose.pose.orientation);

    pub_quad_pos.publish(pos_msg);
    pub_quad_yaw.publish(yaw_msg);

}

void rsp::aqc_coordinator::change_flight_mode_callback(const aqc_msgs::ChangeFlightModeGoalConstPtr &goal_received) {
    
    std::cout << "\naqc_coordinator received cfm_goal:" << std::endl;
    std::cout << *goal_received << std::endl;

    aqc_msgs::ChangeFlightModeGoal goal = *goal_received;
    aqc_msgs::ChangeFlightModeFeedback feedback;

    // TODO feedback

    aqc_msgs::ChangeMode cfm_srv;
    cfm_srv.request.fcu_mode.flight_mode_code = goal.cfm_goal.flight_mode.flight_mode_code;

    if (cfm_client.call(cfm_srv)) {
        // std_msgs::Bool res = cfm_srv.response.success;
        std::cout << "Received success from aqc_mode_manager change flight mode server"  << std::endl;
        cfm_server->setSucceeded();
    } else {
        std::cout << "Received failure from aqc_mode_manager change flight mode server"  << std::endl;
        cfm_server->setPreempted();
    }

}

void rsp::aqc_coordinator::change_arm_status_callback(const aqc_msgs::ArmGoalConstPtr &goal_received) {

    std::cout << "\naqc_coordinator received arm_goal:" << std::endl;
    std::cout << *goal_received << std::endl;

    aqc_msgs::ArmGoal goal = *goal_received;
    aqc_msgs::ArmFeedback feedback;

    // TODO feedback

    // 1=arm and 0=disarm
    aqc_msgs::ArmOrDisarm arm_srv;
    arm_srv.request.arm.arm_code = goal.arm_goal.arm_code;
    
    if (arm_client.call(arm_srv)) {
        // std_msgs::Bool res = arm_srv.response.success;
        std::cout << "Received success from aqc_mode_manager change arm status server"  << std::endl;
        arm_server->setSucceeded();
    } else {
        std::cout << "Received failure from aqc_mode_manager change arm status server"  << std::endl;
        arm_server->setPreempted();
    }

}
