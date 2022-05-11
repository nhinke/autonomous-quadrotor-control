#include <aqc_coordinator/aqc_coordinator.hpp>

int main(int argc, char** argv) {

    ros::init(argc,argv,"aqc_coordinator_node");
    ros::NodeHandle nh;

    rsp::aqc_coordinator task_coordinator(nh);
    ros::spin();

    return 0;

}
