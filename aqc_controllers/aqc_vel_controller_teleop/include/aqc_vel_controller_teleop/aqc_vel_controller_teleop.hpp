// #include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace rsp {

    class aqc_vel_controller_teleop {

        private:

            ros::NodeHandle nh;
            ros::Publisher pub_twist_setpoint;
            ros::Subscriber sub_twist_setpoint;

        protected:

            void twist_setpoint_callback(const geometry_msgs::Twist::ConstPtr& twist_setpoint);

        public:

            aqc_vel_controller_teleop(ros::NodeHandle& nh);
            ~aqc_vel_controller_teleop();

    };

}
