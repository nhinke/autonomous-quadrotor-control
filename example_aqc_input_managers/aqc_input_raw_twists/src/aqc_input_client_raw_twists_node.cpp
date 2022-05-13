#include <aqc_input_raw_twists/aqc_input_client_raw_twists.hpp>

int main(int argc, char** argv) {

    ros::init(argc,argv,"aqc_input_client_raw_twists_node");
    ros::NodeHandle nh;

    int cmd_rate_hz = nh.param<int>("/aqc_input_raw_twists/cmd_rate_hz", 30);

    ros::Rate cmd_rate(cmd_rate_hz);
    rsp::aqc_input_client_raw_twists input_client(nh);

    while (nh.ok()) {
        input_client.publish_twist_setpoint();
        ros::spinOnce();
        cmd_rate.sleep();
        ros::spinOnce();
    }

    return 0;

}