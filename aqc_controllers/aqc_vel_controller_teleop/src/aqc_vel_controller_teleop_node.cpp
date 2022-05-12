#include <aqc_vel_controller_teleop/aqc_vel_controller_teleop.hpp>

int main(int argc, char** argv) {

    ros::init(argc,argv,"aqc_vel_controller_teleop_node");
    ros::NodeHandle nh;

    rsp::aqc_vel_controller_teleop vel_controller_teleop(nh);
    ros::spin();

    return 0;

}
