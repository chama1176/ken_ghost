#include <algorithm>
#include <cmath>
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

bool KenDriver::torque_enable(const bool enable) {}

bool KenDriver::write_goal_positions(const std::vector<double> & goal_positions) {}

bool KenDriver::read_present_positions(std::vector<double> * joint_positions) {}

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

double KenDriver::dxl_pos2rad(const uint16_t position)
{
  return position / DXL_RESOLUTION * 2 * M_PI;
}

uint16_t KenDriver::rad2dxl_pos(const double position)
{
  return position * DXL_RESOLUTION / 2 / M_PI;
}