#include <aqc_vel_controller_raw_twists/aqc_vel_controller_raw_twists.hpp>

rsp::aqc_vel_controller_raw_twists::aqc_vel_controller_raw_twists(ros::NodeHandle& nh) : nh(nh) {

    setpoint.linear.x = 0.0;
    setpoint.linear.y = 0.0;
    setpoint.linear.z = 0.0;
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;

    pub_twist_setpoint = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    sub_twist_setpoint = nh.subscribe("/cmd_vel", 10, &rsp::aqc_vel_controller_raw_twists::twist_setpoint_callback, this);

}

rsp::aqc_vel_controller_raw_twists::~aqc_vel_controller_raw_twists() {}

void rsp::aqc_vel_controller_raw_twists::twist_setpoint_callback(const geometry_msgs::Twist::ConstPtr& twist_setpoint) {

    setpoint = *twist_setpoint;

}

void rsp::aqc_vel_controller_raw_twists::publish_raw_twist_setpoint() {

    pub_twist_setpoint.publish(setpoint);

}
