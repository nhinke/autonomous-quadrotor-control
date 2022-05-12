#include <aqc_input_dynamic_reconfigure/aqc_input_client_dr.hpp>

int main(int argc, char** argv) {

    ros::init(argc,argv,"aqc_input_client_dr_node");
    ros::NodeHandle nh;

    rsp::aqc_input_client_dr input_client_dr(nh);

    ros::spin();

    return 0;

}
