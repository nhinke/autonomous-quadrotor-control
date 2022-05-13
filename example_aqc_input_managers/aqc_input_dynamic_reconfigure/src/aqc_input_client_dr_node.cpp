#include <aqc_input_dynamic_reconfigure/aqc_input_client_dr.hpp>

int main(int argc, char** argv) {

    ros::init(argc,argv,"aqc_input_client_dr_node");
    ros::NodeHandle nh;

    int cmd_rate_hz = nh.param<int>("/aqc_input_dynamic_reconfigure/cmd_rate_hz", 30);
    bool use_relative_xy_setpoints = nh.param<bool>("/aqc_input_dynamic_reconfigure/use_relative_xy_setpoints", false);

    ros::Rate cmd_rate(cmd_rate_hz);
    rsp::aqc_input_client_dr input_client(nh, use_relative_xy_setpoints);

    while (nh.ok()) {
        input_client.publish_position_setpoint();
        ros::spinOnce();
        cmd_rate.sleep();
        ros::spinOnce();
    }

    return 0;

}
