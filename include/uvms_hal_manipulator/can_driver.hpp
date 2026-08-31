#ifndef UVMS_HAL_MANIPULATOR_CAN_DRIVER_HPP
#define UVMS_HAL_MANIPULATOR_CAN_DRIVER_HPP

#include <string>
#include "types.hpp"

namespace uvms_hal_manipulator
{

class CanDriver
{
public:
    CanDriver();
    ~CanDriver();

    CanDriver(const CanDriver&) = delete;
    CanDriver& operator=(const CanDriver&) = delete;

    bool open(const std::string& if_name);
    void close();
    bool is_open() const;
    bool write_frame(const CanFrame& frame);
    bool read_frame(CanFrame& frame);
    void flush();

private:
    int socket_fd_;
    std::string if_name_;
};

}  // namespace uvms_hal_manipulator

#endif