// The FullStateSimulator node.
//
///////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <crazyflie_simulator/full_state_simulator.h>

namespace crazyflie_simulator = cs;

int main(int argc, char** argv) {
  ros::init(argc, argv, "full_state_simulator");
  ros::NodeHandle n("~");

  cs::FullStateSimulator simulator;

  if (!simulator.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize full_state_simulator.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
