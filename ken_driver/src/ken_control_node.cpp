#include <memory>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "ken_driver/ken_interface.hpp"
#include "rclcpp/rclcpp.hpp"

static constexpr double SPIN_RATE = 100;  // Hz

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe) { exe->spin(); }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Logger
  const rclcpp::Logger logger = rclcpp::get_logger("ken robot logger");

  auto control_param_node = rclcpp::Node::make_shared("control_param_node");

  // create my_robot instance
  auto my_robot = std::make_shared<KenInterface>();

  std::vector<std::string> joint_name_list = {"arm_base_to_arm_link1", "arm_link1_to_arm_link2",
                                              "arm_link2_to_arm_link3", "arm_link3_to_arm_link4",
                                              "arm_link4_to_shinai_link"};
  std::vector<uint8_t> joint_id_list = {21, 22, 23, 24, 25};
  std::vector<uint16_t> p_gain = {1000, 1000, 1000, 800, 800};
  std::vector<uint16_t> i_gain = {200, 200, 200, 200, 100};
  std::vector<uint16_t> d_gain = {100, 200, 200, 100, 100};

  // initialize the robot
  if (
    my_robot->init("/dev/ttyUSB0", 57600, joint_id_list, joint_name_list, p_gain, i_gain, d_gain) !=
    hardware_interface::HW_RET_OK) {
    fprintf(stderr, "failed to initialized ken hardware\n");
    return -1;
  }

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // start the controller manager with the robot hardware
  controller_manager::ControllerManager cm(my_robot, executor);
  // load the joint state controller.
  // "ros_controllers" is the resource index from where to look for controllers
  // "ros_controllers::JointStateController" is the class we want to load
  // "my_robot_joint_state_controller" is the name for the node to spawn
  cm.load_controller(
    "ros_controllers", "ros_controllers::JointStateController", "ken_joint_state_controller");
  // load the trajectory controller
  cm.load_controller(
    "ros_controllers", "ros_controllers::JointTrajectoryController",
    "ken_joint_trajectory_controller");

  // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
  auto future_handle = std::async(std::launch::async, spin, executor);

  // we can either configure each controller individually through its services
  // or we use the controller manager to configure every loaded controller
  if (cm.configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
    RCLCPP_ERROR(logger, "at least one controller failed to configure");
    return -1;
  }
  // and activate all controller
  if (cm.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
    RCLCPP_ERROR(logger, "at least one controller failed to activate");
    return -1;
  }

  // main loop
  hardware_interface::hardware_interface_ret_t ret;
  rclcpp::Rate rate(SPIN_RATE);

  while (rclcpp::ok()) {
    ret = my_robot->read();
    if (ret != hardware_interface::HW_RET_OK) {
      fprintf(stderr, "read failed!\n");
    }

    cm.update();

    ret = my_robot->write();
    if (ret != hardware_interface::HW_RET_OK) {
      fprintf(stderr, "write failed!\n");
    }

    rate.sleep();
  }

  executor->cancel();
}
