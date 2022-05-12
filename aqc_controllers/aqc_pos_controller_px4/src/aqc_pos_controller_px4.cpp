#include <aqc_pos_controller_px4/aqc_pos_controller_px4.hpp>

rsp::aqc_pos_controller_px4::aqc_pos_controller_px4(ros::NodeHandle& nh) : 
    nh(nh), setpoint_x(0.0), setpoint_y(0.0), setpoint_z(4.0), setpoint_yaw(0.0) {

    sub_pos_setpoint = nh.subscribe<aqc_msgs::PositionSetpoint>("/cmd_position", 10, &rsp::aqc_pos_controller_px4::position_setpoint_callback, this);
    pub_pos_setpoint = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    pub_raw_pos_setpoint = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

}

rsp::aqc_pos_controller_px4::~aqc_pos_controller_px4() {}

// subscriber callback for input clients to utilize
void rsp::aqc_pos_controller_px4::position_setpoint_callback(const aqc_msgs::PositionSetpoint::ConstPtr& pos_setpoint) {

    aqc_msgs::PositionSetpoint setpoint = *pos_setpoint;
    
    setpoint_x = setpoint.ENU_position.x;
    setpoint_y = setpoint.ENU_position.y;
    setpoint_z = setpoint.ENU_position.z;
    setpoint_yaw = setpoint.yaw_orientation;

}

// sends position setpoint using SET_POSITION_TARGET_LOCAL_NED
void rsp::aqc_pos_controller_px4::publish_position_setpoint() {

    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = setpoint_x;
    setpoint.pose.position.y = setpoint_y;
    setpoint.pose.position.z = setpoint_z;
    setpoint.pose.orientation = tf::createQuaternionMsgFromYaw(setpoint_yaw);

    setpoint.header.stamp = ros::Time::now();

    pub_pos_setpoint.publish(setpoint);

}

// send RAW setpoint messages to FCU and provide loopback topics on PX4
void rsp::aqc_pos_controller_px4::publish_raw_position_setpoint() {

    mavros_msgs::PositionTarget setpoint;
    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                         mavros_msgs::PositionTarget::IGNORE_VY |
                         mavros_msgs::PositionTarget::IGNORE_VZ |
                         mavros_msgs::PositionTarget::IGNORE_AFX |
                         mavros_msgs::PositionTarget::IGNORE_AFY |
                         mavros_msgs::PositionTarget::IGNORE_AFZ |
                         mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    setpoint.position.x = setpoint_x;
    setpoint.position.y = setpoint_y;
    setpoint.position.z = setpoint_z;
    setpoint.yaw = setpoint_yaw;

    setpoint.header.stamp = ros::Time::now();
    pub_raw_pos_setpoint.publish(setpoint);

}
