#include <algorithm>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <aqc_msgs/PositionSetpoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <dynamic_reconfigure/server.h>
#include <aqc_pos_controller_px4/aqc_pos_controller_px4Config.h>

namespace rsp {

    class aqc_pos_controller_px4 {

        private:

            ros::NodeHandle nh;

            ros::Publisher pub_pos_setpoint;
            ros::Publisher pub_raw_pos_setpoint;
            ros::Subscriber sub_pos_setpoint;

            double setpoint_x, setpoint_y, setpoint_z, setpoint_yaw;

            // dynamic_reconfigure::Server<aqc_pos_controller_px4::aqc_pos_controller_px4Config> server;
            // dynamic_reconfigure::Server<aqc_pos_controller_px4::aqc_pos_controller_px4Config>::CallbackType callback;
            // dynamic_reconfigure::Server<aqc_pos_controller_px4::ControllerConfig> server;
            // dynamic_reconfigure::Server<aqc_pos_controller_px4::ControllerConfig>::CallbackType callback;

        protected:

            void position_setpoint_callback(const aqc_msgs::PositionSetpoint::ConstPtr& pos_setpoint);

        public:

            aqc_pos_controller_px4(ros::NodeHandle& nh);
            ~aqc_pos_controller_px4();
            void publish_setpoint();

    };

}
