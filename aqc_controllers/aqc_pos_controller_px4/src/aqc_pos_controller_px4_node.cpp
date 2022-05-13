#include <aqc_pos_controller_px4/aqc_pos_controller_px4.hpp>

int main(int argc, char** argv) {

    ros::init(argc,argv,"aqc_pos_controller_px4_node");
    ros::NodeHandle nh("~");

    int controller_rate_hz = nh.param<int>("controller_rate_hz", 30);
    bool use_raw_position_setpoints = nh.param<bool>("use_raw_position_target_setpoints", true);

    ros::Rate controller_rate(controller_rate_hz);
    rsp::aqc_pos_controller_px4 pos_controller_px4(nh);

    if (use_raw_position_setpoints) {
        while (nh.ok()) {
            pos_controller_px4.publish_raw_position_setpoint();
            ros::spinOnce();
            controller_rate.sleep();
            ros::spinOnce();
        }
    } else {
        while (nh.ok()) {
            pos_controller_px4.publish_position_setpoint();
            ros::spinOnce();
            controller_rate.sleep();
            ros::spinOnce();
        }
    }

    return 0;

}
