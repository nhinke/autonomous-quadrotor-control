#include <aqc_vel_controller_raw_twists/aqc_vel_controller_raw_twists.hpp>

int main(int argc, char** argv) {

    ros::init(argc,argv,"aqc_vel_controller_raw_twists_node");
    ros::NodeHandle nh("~");

    int controller_rate_hz = nh.param<int>("controller_rate_hz", 30);

    ros::Rate controller_rate(controller_rate_hz);
    rsp::aqc_vel_controller_raw_twists vel_controller_raw_twists(nh);

    while (nh.ok()) {
        vel_controller_raw_twists.publish_raw_twist_setpoint();
        ros::spinOnce();
        controller_rate.sleep();
        ros::spinOnce();
    }

    return 0;

}
