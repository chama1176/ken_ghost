#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ken_driver/ken_driver.hpp"

KenDriver::KenDriver(const std::string port_name, const int baudrate, std::vector<uint8_t> id_list)
: baudrate_(baudrate), id_list_(id_list)
{
  dxl_port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
    dynamixel::PortHandler::getPortHandler(port_name.c_str()));
  dxl_packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
    dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));

  dxl_group_sync_read_ = std::shared_ptr<dynamixel::GroupSyncRead>(new dynamixel::GroupSyncRead(
    dxl_port_handler_.get(), dxl_packet_handler_.get(), ADDR_PRESENT_POSITION,
    LEN_PRESENT_POSITION));

  dxl_group_sync_write_ = std::shared_ptr<dynamixel::GroupSyncWrite>(new dynamixel::GroupSyncWrite(
    dxl_port_handler_.get(), dxl_packet_handler_.get(), ADDR_GOAL_POSITION, LEN_GOAL_POSITION));
}

KenDriver::~KenDriver() { close_port(); }

bool KenDriver::open_port(void)
{
  if (!dxl_port_handler_->openPort()) {
    last_error_log_ = std::string(__func__) +
                      ": unable to open dynamixel port: " + dxl_port_handler_->getPortName();
    return false;
  }

  if (!dxl_port_handler_->setBaudRate(baudrate_)) {
    last_error_log_ = std::string(__func__) + ": unable to set baudrate" +
                      std::to_string(dxl_port_handler_->getBaudRate());
    return false;
  }

  return true;
}

void KenDriver::close_port(void) { dxl_port_handler_->closePort(); }

bool KenDriver::torque_enable(const bool enable)
{
  bool retval = true;
  for (auto dxl_id : id_list_) {
    uint8_t dxl_error = 0;
    int dxl_result = dxl_packet_handler_->write1ByteTxRx(
      dxl_port_handler_.get(), dxl_id, ADDR_TORQUE_ENABLE, enable, &dxl_error);

    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }
  }

  return retval;
}

bool KenDriver::add_sync_read_param(void)
{
  bool retval = true;

  for (auto dxl_id : id_list_) {
    if (!dxl_group_sync_read_->addParam(dxl_id)) {
      last_error_log_ =
        std::string(__func__) + ": unable to add sync read param " + std::to_string(dxl_id);
      retval = false;
    }
  }
  return retval;
}

bool KenDriver::sync_read_present_positions(std::vector<double> * joint_positions)
{
  bool retval = true;

  // Syncread present position
  int dxl_result = dxl_group_sync_read_->txRxPacket();
  if (!parse_dxl_error(std::string(__func__), dxl_result)) {
    retval = false;
  }
  joint_positions->resize(id_list_.size());

  for (size_t i = 0; i < id_list_.size(); ++i) {
    // Check if groupsyncread data of Dynamixel#1 is available
    bool dxl_getdata_result =
      dxl_group_sync_read_->isAvailable(id_list_[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    if (dxl_getdata_result != true) {
      last_error_log_ = "groupSyncRead getdata failed " + std::to_string(id_list_[i]);
      return false;
    }
    joint_positions->at(i) = dxl_pos2rad((uint16_t)dxl_group_sync_read_->getData(
      id_list_[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));
  }

  return retval;
}

bool KenDriver::sync_write_goal_positions(const std::vector<double> & goal_positions)
{
  if (goal_positions.size() != id_list_.size()) {
    last_error_log_ = std::string(__func__) + ": vectors size does not match: " +
                      " goal_positions:" + std::to_string(goal_positions.size()) +
                      ", id_list:" + std::to_string(id_list_.size());
    return false;
  }

  bool retval = true;

  // Add Dynamixel goal position value to the Syncwrite storage
  for (size_t i = 0; i < goal_positions.size(); i++) {
    uint16_t goal_position = rad2dxl_pos(goal_positions[i]);
    goal_position = std::max(goal_position, (uint16_t)0);
    goal_position = std::min(goal_position, (uint16_t)(DXL_RESOLUTION - 1));

    std::cout << i << " goal pos: " << goal_position << std::endl;
    auto dxl_id = id_list_[i];

    uint8_t param_goal_position[4];
    // Allocate goal position value into byte array
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

    if (!dxl_group_sync_write_->addParam(dxl_id, param_goal_position)) {
      last_error_log_ =
        std::string(__func__) + ": unable to add sync write param " + std::to_string(dxl_id);
      return false;
    }
  }

  // Syncwrite goal position
  int dxl_result = dxl_group_sync_write_->txPacket();
  if (!parse_dxl_error(std::string(__func__), dxl_result)) {
    retval = false;
  }

  // Clear syncwrite parameter storage
  dxl_group_sync_write_->clearParam();

  return retval;
}

std::string KenDriver::get_last_error_log(void) { return last_error_log_; }

bool KenDriver::parse_dxl_error(
  const std::string func_name, const uint8_t dxl_id, const int dxl_comm_result,
  const uint8_t dxl_packet_error)
{
  bool retval = true;

  if (dxl_comm_result != COMM_SUCCESS) {
    last_error_log_ = func_name + ": dxl_id: " + std::to_string(dxl_id) + " :" +
                      std::string(dxl_packet_handler_->getTxRxResult(dxl_comm_result));
    retval = false;
  }

  if (dxl_packet_error != 0) {
    last_error_log_ = func_name + ": dxl_id: " + std::to_string(dxl_id) + " :" +
                      std::string(dxl_packet_handler_->getRxPacketError(dxl_packet_error));
    retval = false;
  }

  return retval;
}

bool KenDriver::parse_dxl_error(const std::string func_name, const int dxl_comm_result)
{
  bool retval = true;

  if (dxl_comm_result != COMM_SUCCESS) {
    last_error_log_ = func_name + std::string(dxl_packet_handler_->getTxRxResult(dxl_comm_result));
    retval = false;
  }

  return retval;
}

double KenDriver::dxl_pos2rad(const uint16_t position)
{
  return (position - DXL_RESOLUTION / 2) / DXL_RESOLUTION * 2 * M_PI;
}

uint16_t KenDriver::rad2dxl_pos(const double position)
{
  return (position * DXL_RESOLUTION / 2 / M_PI) + DXL_RESOLUTION / 2;
}