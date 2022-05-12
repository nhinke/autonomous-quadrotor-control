#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <aqc_msgs/ArmAction.h>
#include <aqc_msgs/ChangeFlightModeAction.h>
#include <actionlib/client/simple_action_client.h>

#include <map>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

// adapted from https://github.com/methylDragon/teleop_twist_keyboard_cpp

void arm_done_callback(const actionlib::SimpleClientGoalState& state, const aqc_msgs::ArmResultConstPtr& result) {
    std::cout << "\narm_client in aqc_teleop_twist_keyboard received result from aqc_coordinator:" << std::endl;
    std::cout << *result << std::endl;
}

void arm_active_callback() {
    // std::cout << "Active" << std::endl;
}

void arm_feedback_callback(const aqc_msgs::ArmFeedbackConstPtr& feedback) {
    std::cout << "\narm_client in aqc_teleop_twist_keyboard received feedback from aqc_coordinator:" << std::endl;
    std::cout << *feedback << std::endl;
}

void cfm_done_callback(const actionlib::SimpleClientGoalState& state, const aqc_msgs::ChangeFlightModeResultConstPtr& result) {
    std::cout << "\ncfm_client in aqc_teleop_twist_keyboard received result from aqc_coordinator:" << std::endl;
    std::cout << *result << std::endl;
}

void cfm_active_callback() {
    // std::cout << "Active" << std::endl;
}

void cfm_feedback_callback(const aqc_msgs::ChangeFlightModeFeedbackConstPtr& feedback) {
    std::cout << "\ncfm_client in aqc_teleop_twist_keyboard received feedback from aqc_coordinator:" << std::endl;
    std::cout << *feedback << std::endl;
}

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'y', {0, 0, 1, 0}},
  {'n', {0, 0, -1, 0}},
  {'k', {0, 0, 0, 0}},
  {'K', {0, 0, 0, 0}}
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};

// Map for arming keys
std::map<char, int> armBindings
{
  {'d', 0},
  {'a', 1}
};

// Map for mode keys
std::map<char, std::string> modeBindings
{
  {'t', "AUTO.TAKEOFF"},
  {'h', "AUTO.LOITER"}, 
  {'p', "POSCTL"},
  {'r', "AUTO.RTL"},
  {'v', "AUTO.LAND"}, // v looks like down arrow
  {'f', "OFFBOARD"} // f for "free at last!" or "wtf!"
};

// Reminder message
std::string msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Arm requests:
   a : arm
   d : disarm
Change flight mode requests:
   t : takeoff
   h : hold (loiter)
   p : position
   r : return to land
   v : land
   f : offboard
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
   U    I    O
   J    K    L
   M    <    >
y : up (+z)
n : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
)";

// Init variables
bool stop_robot_flag(false);
float speed(0.5); // Linear velocity (m/s)
float turn(1.0); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "aqc_teleop_twist_keyboard_cpp");
  ros::NodeHandle n;

  ros::Rate rate(30);

  // Init cmd_vel publisher
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  actionlib::SimpleActionClient<aqc_msgs::ArmAction> arm_client(n, "change_arm_status_action");
  actionlib::SimpleActionClient<aqc_msgs::ChangeFlightModeAction> cfm_client(n, "change_flight_mode_action");

  ros::Duration server_timeout(5);
  if (!arm_client.waitForServer(server_timeout)) { 
      ROS_ERROR("'arm_client' within 'aqc_teleop_twist_keyboard' node could not connect to its server. Please ensure that 'aqc_coordinator' is alive and running.");
  }
  if (!cfm_client.waitForServer(server_timeout)) { 
      ROS_ERROR("'cfm_client' within 'aqc_teleop_twist_keyboard' node could not connect to its server. Please ensure that 'aqc_coordinator' is alive and running.");
  }

  // Create Twist message
  geometry_msgs::Twist twist;

  // printf("%s", msg);
  std::cout << msg << std::endl;
  // printf("\rCurrent: speed %f\tturn %f | Awaiting command...\r", speed, turn);

  while(ros::ok()) {

    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in moveBindings
    if (moveBindings.count(key) == 1)
    {
      // Grab the direction data
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      z = moveBindings[key][2];
      th = moveBindings[key][3];

      // printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
    }

    // Otherwise if it corresponds to a key in speedBindings
    else if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      speed = speed * speedBindings[key][0];
      turn = turn * speedBindings[key][1];

      // printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
    }

    // Otherwise if it corresponds to a key in armBindings
    else if (armBindings.count(key) == 1)
    {
      stop_robot_flag = true;
      int arm_req = armBindings[key];

      aqc_msgs::ArmGoal goal;
      if (arm_req == 0) {
        goal.arm_goal.arm_code = goal.arm_goal.DISARM;
      } else if (arm_req == 1) {
        goal.arm_goal.arm_code = goal.arm_goal.ARM;
      }

      arm_client.sendGoal(goal, &arm_done_callback, &arm_active_callback, &arm_feedback_callback);
      std::cout << "Arm request: " << arm_req << std::endl;
    }    

    // Otherwise if it corresponds to a key in modeBindings
    else if (modeBindings.count(key) == 1)
    {
      stop_robot_flag = true;
      std::string cfm_req = modeBindings[key];
      // const char* cfm_req_cstr = cfm_req.c_str();

      aqc_msgs::ChangeFlightModeGoal goal;
      if (cfm_req.compare(std::string("AUTO.TAKEOFF")) == 0) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.TAKEOFF;
      } else if (cfm_req.compare(std::string("AUTO.LOITER")) == 0) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.LOITER;
      } else if (cfm_req.compare(std::string("POSCTL")) == 0) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.POSCTL;
      } else if (cfm_req.compare(std::string("AUTO.RTL")) == 0) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.RTL;
      } else if (cfm_req.compare(std::string("AUTO.LAND")) == 0) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.LAND;
      } else if (cfm_req.compare(std::string("OFFBOARD")) == 0) {
        goal.cfm_goal.flight_mode.flight_mode_code = goal.cfm_goal.flight_mode.OFFBOARD;
      }

      cfm_client.sendGoal(goal, &cfm_done_callback, &cfm_active_callback, &cfm_feedback_callback);
      // printf("\rChange flight mode request: ");
      std::cout << "Change flight mode request: " << cfm_req << std::endl;
      // printf("\rChange flight mode request: %s   ", cfm_req_cstr);
      // delete[] cfm_req_cstr;
    }  

    // Otherwise, set the robot to stop
    else
    {
      stop_robot_flag = true;

      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        std::cout << "\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n" << std::endl;
        break;
      }

      // printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed, turn, key);
      // delete[] msg;
    }

    if (stop_robot_flag) {
      x = 0;
      y = 0;
      z = 0;
      th = 0;
      stop_robot_flag = false;
    }

    // Update the Twist message
    twist.linear.x = x * speed;
    twist.linear.y = y * speed;
    twist.linear.z = z * speed;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn;

    std::cout << "\nhere" << std::endl;
    std::cout << twist << std::endl;

    // Publish it and resolve any remaining callbacks
    pub.publish(twist);

    ros::spinOnce();
    rate.sleep();
    // ros::spinOnce();

  }

  return 0;

}
