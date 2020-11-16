#ifndef KEN_INTERFACE_HPP_
#define KEN_INTERFACE_HPP_
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/joint_command_handle.hpp"
#include "hardware_interface/joint_state_handle.hpp"
#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "ken_driver/ken_driver.hpp"
#include "rclcpp/rclcpp.hpp"

class KenInterface : public hardware_interface::RobotHardware
{
public:
  KenInterface();
  ~KenInterface();
  hardware_interface::hardware_interface_ret_t init();
  hardware_interface::hardware_interface_ret_t init(
    const std::string & port_name, const int baudrate, const std::vector<uint8_t> & dxl_id_list,
    const std::vector<std::string> & joint_name_list);
  hardware_interface::hardware_interface_ret_t read();
  hardware_interface::hardware_interface_ret_t write();

private:
  std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
  std::vector<hardware_interface::JointCommandHandle> joint_command_handles_;
  std::vector<hardware_interface::OperationModeHandle> joint_mode_handles_;

  std::vector<std::string> joint_names_;

  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> eff_;
  std::vector<double> cmd_;
  std::vector<hardware_interface::OperationMode> op_mode_;

  std::shared_ptr<KenInterface> driver_;
};

#endif  // KEN_INTERFACE_HPP_