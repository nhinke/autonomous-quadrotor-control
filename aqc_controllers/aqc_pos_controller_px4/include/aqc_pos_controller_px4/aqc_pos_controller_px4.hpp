#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <aqc_msgs/PositionSetpoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>

namespace rsp {

    class aqc_pos_controller_px4 {

        private:

            ros::NodeHandle nh;

            ros::Publisher pub_pos_setpoint;
            ros::Publisher pub_raw_pos_setpoint;
            ros::Subscriber sub_pos_setpoint;
            
            double setpoint_x, setpoint_y, setpoint_z, setpoint_yaw;

        protected:
        
            void position_setpoint_callback(const aqc_msgs::PositionSetpoint::ConstPtr& pos_setpoint);
        

        public:

            aqc_pos_controller_px4(ros::NodeHandle& nh);
            ~aqc_pos_controller_px4();
            void publish_position_setpoint();
            void publish_raw_position_setpoint();

    };

}
