#ifndef POWER_HAL_CAN_INTERFACE_HPP
#define POWER_HAL_CAN_INTERFACE_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>

namespace power_hal
{

enum class BatteryType : uint8_t
{
  BATTERY_12V = 0,
  BATTERY_24V = 1,
  BATTERY_72V = 2
};

struct BatteryControlData
{
  BatteryType battery_type;
  bool switch_state;
};

struct BMSData
{
  BatteryType battery_type;
  
  uint16_t cell_voltages[32];
  int8_t temperatures[8];
  
  float total_voltage;
  float total_current;
  float remain_capacity;
  float full_capacity;
  
  float soc;
  uint16_t cycle_count;
  
  float max_cell_vol;
  uint8_t max_cell_id;
  float min_cell_vol;
  uint8_t min_cell_id;
  
  int8_t max_temp;
  int8_t min_temp;
  
  uint8_t cell_count;
  
  uint32_t protection_status;
  bool mos_charge_state;
  bool mos_discharge_state;
  
  bool updated;
  
  BMSData() : updated(false)
  {
    std::memset(cell_voltages, 0, sizeof(cell_voltages));
    std::memset(temperatures, 0, sizeof(temperatures));
    total_voltage = 0.0f;
    total_current = 0.0f;
    remain_capacity = 0.0f;
    full_capacity = 0.0f;
    soc = 0.0f;
    cycle_count = 0;
    max_cell_vol = 0.0f;
    max_cell_id = 0;
    min_cell_vol = 0.0f;
    min_cell_id = 0;
    max_temp = 0;
    min_temp = 0;
    cell_count = 0;
    protection_status = 0;
    mos_charge_state = false;
    mos_discharge_state = false;
  }
};

struct BatteryStatusData
{
  BatteryType battery_type;
  bool switch_state;
  float voltage;
  BMSData bms_data;
};

class CanInterface
{
public:
  CanInterface(const std::string & can_interface = "can4");
  ~CanInterface();

  bool init();
  void close();
  bool is_open() const {return socket_fd_ >= 0;}

  bool send_control_command(const BatteryControlData & control);
  bool receive_status(BatteryStatusData & status);

private:
  std::string can_interface_;
  int socket_fd_;
  
  bool set_nonblocking(bool enable);
};

}  
#endif