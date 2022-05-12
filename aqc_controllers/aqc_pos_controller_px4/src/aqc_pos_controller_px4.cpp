#include <aqc_pos_controller_px4/aqc_pos_controller_px4.hpp>

rsp::aqc_pos_controller_px4::aqc_pos_controller_px4(ros::NodeHandle& nh) : 
    nh(nh), setpoint_x(0.0), setpoint_y(0.0), setpoint_z(5.0), setpoint_yaw(0.0) {

    // pub_twist_setpoint = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    // sub_twist_setpoint = nh.subscribe("/cmd_vel", 10, &rsp::aqc_vel_controller_teleop::twist_setpoint_callback, this);

    sub_pos_setpoint = nh.subscribe<aqc_msgs::PositionSetpoint>("/quad/cmd_setpoint", 10, &rsp::aqc_pos_controller_px4::position_setpoint_callback, this);
    pub_pos_setpoint = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    pub_raw_pos_setpoint = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);

}

rsp::aqc_pos_controller_px4::~aqc_pos_controller_px4() {}

void rsp::aqc_pos_controller_px4::position_setpoint_callback(const aqc_msgs::PositionSetpoint::ConstPtr& pos_setpoint) {

    aqc_msgs::PositionSetpoint setpoint = *pos_setpoint;
    
    setpoint_x = setpoint.ENU_position.x;
    setpoint_y = setpoint.ENU_position.y;
    setpoint_z = setpoint.ENU_position.z;
    setpoint_yaw = setpoint.yaw_orientation;

}

void rsp::aqc_pos_controller_px4::publish_setpoint() {

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

// void rsp::aqc_pos_controller_px4::publish_setpoint() {

//     geometry_msgs::PoseStamped setpoint;
//     setpoint.pose.position.x = setpoint_x;
//     setpoint.pose.position.y = setpoint_y;
//     setpoint.pose.position.z = setpoint_z;
//     setpoint.pose.orientation = tf::createQuaternionMsgFromYaw(setpoint_yaw);

//     setpoint.header.stamp = ros::Time::now();

//     pub_pos_setpoint.publish(setpoint);

// }
