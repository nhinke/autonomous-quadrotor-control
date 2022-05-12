#include <aqc_vel_controller_teleop/aqc_vel_controller_teleop.hpp>

rsp::aqc_vel_controller_teleop::aqc_vel_controller_teleop(ros::NodeHandle& nh) : nh(nh) {

    pub_twist_setpoint = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    sub_twist_setpoint = nh.subscribe("/cmd_vel", 10, &rsp::aqc_vel_controller_teleop::twist_setpoint_callback, this);

}

rsp::aqc_vel_controller_teleop::~aqc_vel_controller_teleop() {}

void rsp::aqc_vel_controller_teleop::twist_setpoint_callback(const geometry_msgs::Twist::ConstPtr& twist_setpoint) {

    geometry_msgs::Twist cmd_vel = *twist_setpoint;
    pub_twist_setpoint.publish(cmd_vel);

}
