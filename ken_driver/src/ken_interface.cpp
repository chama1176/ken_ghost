#include <memory>
#include <string>
#include <vector>

#include "ken_driver/ken_interface.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ken_interface");

KenInterface::KenInterface() {}

KenInterface::~KenInterface()
{
  driver_->torque_enable(false);
  driver_->close_port();
}

hardware_interface::hardware_interface_ret_t KenInterface::init()
{
  return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t KenInterface::init(
  const std::string & port_name, const int baudrate, const std::vector<uint8_t> & dxl_id_list,
  const std::vector<std::string> & joint_name_list, const std::vector<uint16_t> & p_gain,
  const std::vector<uint16_t> & i_gain, const std::vector<uint16_t> & d_gain)
{
  driver_ = std::make_shared<KenDriver>(port_name, baudrate, dxl_id_list);

  if (!driver_->open_port()) {
    RCLCPP_ERROR(LOGGER, driver_->get_last_error_log());
    return hardware_interface::HW_RET_ERROR;
  }

  if (!driver_->set_gain_all(p_gain, i_gain, d_gain)) {
    RCLCPP_ERROR(LOGGER, driver_->get_last_error_log());
    return hardware_interface::HW_RET_ERROR;
  }

  for (auto joint_name : joint_name_list) {
    joint_names_.push_back(joint_name);
    std::cout << joint_name << std::endl;
  }

  // Resize members
  pos_.resize(joint_names_.size());
  vel_.resize(joint_names_.size());
  eff_.resize(joint_names_.size());
  cmd_.resize(joint_names_.size());
  op_mode_.resize(joint_names_.size());
  joint_state_handles_.resize(joint_names_.size());
  joint_command_handles_.resize(joint_names_.size());
  joint_mode_handles_.resize(joint_names_.size());

  size_t i = 0;
  for (auto & joint_name : joint_names_) {
    hardware_interface::JointStateHandle state_handle(joint_name, &pos_[i], &vel_[i], &eff_[i]);
    joint_state_handles_[i] = state_handle;

    if (register_joint_state_handle(&joint_state_handles_[i]) != hardware_interface::HW_RET_OK) {
      RCLCPP_ERROR(LOGGER, "Unable to register %s.", joint_state_handles_[i].get_name());
      return hardware_interface::HW_RET_ERROR;
    }

    hardware_interface::JointCommandHandle command_handle(joint_name, &cmd_[i]);
    joint_command_handles_[i] = command_handle;
    if (
      register_joint_command_handle(&joint_command_handles_[i]) != hardware_interface::HW_RET_OK) {
      RCLCPP_ERROR(LOGGER, "Unable to register %s.", joint_command_handles_[i].get_name());
      return hardware_interface::HW_RET_ERROR;
    }

    joint_mode_handles_[i] = hardware_interface::OperationModeHandle(
      joint_name, reinterpret_cast<hardware_interface::OperationMode *>(&op_mode_[i]));
    if (register_operation_mode_handle(&joint_mode_handles_[i]) != hardware_interface::HW_RET_OK) {
      RCLCPP_ERROR(LOGGER, "Unable to register %s.", joint_mode_handles_[i].get_name());
      return hardware_interface::HW_RET_ERROR;
    }
    ++i;
  }

  if (!driver_->add_sync_read_param()) {
    RCLCPP_ERROR(LOGGER, driver_->get_last_error_log());
    return hardware_interface::HW_RET_ERROR;
  }

  if (!driver_->torque_enable(true)) {
    RCLCPP_ERROR(LOGGER, driver_->get_last_error_log());
    return hardware_interface::HW_RET_ERROR;
  }

  // set current joint positions to pos_
  read();
  for (size_t i = 0; i < cmd_.size(); i++) {
    cmd_[i] = pos_[i];  // set current joint positions to target positions cmd_.
  }

  return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t KenInterface::read()
{
  std::vector<double> joint_positions;
  if (!driver_->sync_read_present_positions(&joint_positions)) {
    RCLCPP_ERROR(LOGGER, driver_->get_last_error_log());
    return hardware_interface::HW_RET_ERROR;

  } else if (pos_.size() != joint_positions.size()) {
    RCLCPP_ERROR(
      LOGGER, "vectors size does not match. pos_:%d, joint_positions:%d", pos_.size(),
      joint_positions.size());
    return hardware_interface::HW_RET_ERROR;

  } else {
    for (size_t i = 0; i < pos_.size(); ++i) {
      pos_[i] = joint_positions[i];
    }
  }

  return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t KenInterface::write()
{
  if (!driver_->sync_write_goal_positions(cmd_)) {
    RCLCPP_ERROR(LOGGER, driver_->get_last_error_log());
    return hardware_interface::HW_RET_ERROR;
  }

  return hardware_interface::HW_RET_OK;
}