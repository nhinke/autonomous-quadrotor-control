#include <algorithm>
#include <ros/ros.h>

namespace rsp {

    class aqc_pos_controller_pid {

        private:

            ros::NodeHandle nh;

        public:

            aqc_pos_controller_pid(ros::NodeHandle& nh);
            ~aqc_pos_controller_pid();

    };

}
