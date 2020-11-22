#ifndef KEN_DRIVER_HPP_
#define KEN_DRIVER_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <memory>
#include <string>
#include <vector>

class KenDriver
{
public:
  KenDriver(const std::string port_name, const int baudrate, std::vector<uint8_t> id_list);
  ~KenDriver();

  bool open_port(void);
  void close_port(void);
  bool torque_enable(const bool enable);
  bool add_sync_read_param(void);
  bool sync_read_present_positions(std::vector<double> * joint_positions);
  bool sync_write_goal_positions(const std::vector<double> & goal_positions);

  // TODO: Get voltage func
  // TODO: Get temperature func

  std::string get_last_error_log(void);

private:
  static constexpr double PROTOCOL_VERSION = 2.0;
  static constexpr double DXL_RESOLUTION = 4096.0;  // [pulse/rev]

  // Dynamixel XM430-W350 address table
  // Ref: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
  static constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
  static constexpr uint16_t ADDR_GOAL_POSITION = 116;
  static constexpr uint16_t ADDR_PRESENT_POSITION = 132;

  static constexpr uint16_t LEN_GOAL_POSITION = 4;
  static constexpr uint16_t LEN_PRESENT_POSITION = 4;

  std::shared_ptr<dynamixel::PortHandler> dxl_port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> dxl_packet_handler_;
  std::shared_ptr<dynamixel::GroupSyncRead> dxl_group_sync_read_;
  std::shared_ptr<dynamixel::GroupSyncWrite> dxl_group_sync_write_;

  int baudrate_;
  std::vector<uint8_t> id_list_;
  std::string last_error_log_;

  bool parse_dxl_error(
    const std::string func_name, const uint8_t dxl_id, const int dxl_comm_result,
    const uint8_t dxl_packet_error);
  bool parse_dxl_error(const std::string func_name, const int dxl_comm_result);

  inline double dxl_pos2rad(const uint16_t position);
  inline uint16_t rad2dxl_pos(const double position);
};

#endif  // KEN_DRIVER_HPP_