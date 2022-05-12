#include <aqc_pos_controller_px4/aqc_pos_controller_px4.hpp>

int main(int argc, char** argv) {

    ros::init(argc,argv,"aqc_pos_controller_px4_node");
    ros::NodeHandle nh;

    ros::Rate controller_rate(30); // TODO set from param
    rsp::aqc_pos_controller_px4 pos_controller_px4(nh);

    while (nh.ok()) {
        pos_controller_px4.publish_setpoint();
        ros::spinOnce();
        controller_rate.sleep();
    }

    return 0;

}
