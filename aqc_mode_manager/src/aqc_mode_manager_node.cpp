#include <aqc_mode_manager/aqc_mode_manager.hpp>

int main(int argc, char** argv) {

    ros::init(argc,argv,"aqc_mode_manager_node");
    ros::NodeHandle nh;

    rsp::aqc_mode_manager mode_manager(nh);
    ros::spin();

    return 0;

}
