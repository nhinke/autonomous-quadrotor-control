#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace rsp {

    class aqc_vel_controller_raw_twists {

        private:

            ros::NodeHandle nh;
            ros::Publisher pub_twist_setpoint;
            ros::Subscriber sub_twist_setpoint;

            geometry_msgs::Twist setpoint; 

        protected:

            void twist_setpoint_callback(const geometry_msgs::Twist::ConstPtr& twist_setpoint);

        public:

            aqc_vel_controller_raw_twists(ros::NodeHandle& nh);
            ~aqc_vel_controller_raw_twists();
            void publish_raw_twist_setpoint();

    };

}
