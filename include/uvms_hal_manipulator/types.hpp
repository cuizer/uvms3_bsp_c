#ifndef UVMS_HAL_MANIPULATOR_TYPES_HPP
#define UVMS_HAL_MANIPULATOR_TYPES_HPP

#include <array>
#include <cstdint>
#include <vector>
#include <string>
#include <map>

namespace uvms_hal_manipulator
{

constexpr uint8_t APP_MSG_ID_ARMCABIN_MOTOR_DATA = 0x09;
constexpr uint8_t APP_MSG_ID_ARM_MOTOR_DATA      = 0x0A;
constexpr uint8_t APP_MSG_ID_ARM_CONTROLLER_DATA = 0x0B;
constexpr uint8_t APP_MSG_ID_ARM_CONTROL_CMD     = 0x32;

constexpr uint32_t CAN_ID_ARMCABIN_MOTOR_DATA = 0x09;
constexpr uint32_t CAN_ID_ARM_MOTOR_DATA      = 0x0A;
constexpr uint32_t CAN_ID_ARM_CONTROLLER_DATA = 0x0B;
constexpr uint32_t CAN_ID_ARM_CONTROL_CMD     = 0x10;

enum class ArmControlCommand : uint8_t
{
    CABIN_OPEN         = 0x01,
    CABIN_CLOSE        = 0x02,
    ARM_EXTEND         = 0x03,
    ARM_RETRACT        = 0x04,
    DATA_UPLOAD_ENABLE = 0x05,
    DATA_UPLOAD_DISABLE = 0x06
};

struct CanFrame
{
    uint32_t can_id{0};
    uint8_t dlc{0};
    std::array<uint8_t, 8> data{};
};

struct ArmCabinMotorState
{
    std::array<int16_t, 2> current{};
    std::array<int16_t, 2> speed{};
    std::array<int16_t, 2> position{};
    std::array<uint16_t, 2> temperature{};
    std::array<uint8_t, 2> error{};
};

struct ArmMotorState
{
    std::array<int16_t, 10> current{};
    std::array<int16_t, 10> speed{};
    std::array<int16_t, 10> position{};
    std::array<uint16_t, 10> temperature{};
    std::array<uint8_t, 10> error{};
};

struct ArmControllerState
{
    std::array<int16_t, 12> current{};
    std::array<int16_t, 12> speed{};
    std::array<int16_t, 12> position{};
    std::array<uint16_t, 12> temperature{};
    std::array<uint8_t, 12> error{};
};

enum class CompleteMessageType : uint8_t
{
    NONE = 0,
    ARMCABIN_MOTOR,
    ARM_MOTOR,
    ARM_CONTROLLER
};

struct CompleteMessage
{
    CompleteMessageType type{CompleteMessageType::NONE};
};

struct FragmentBuffer
{
    uint8_t app_msg_id{0};
    uint8_t total_frames{0};
    std::map<uint8_t, std::vector<uint8_t>> fragments;
};

struct JointLimitConfig
{
    std::vector<double> position_min;
    std::vector<double> position_max;
    std::vector<double> max_velocity;
};

struct MotorSafetyConfig
{
    double max_current{0.0};
    double max_temperature{0.0};
    double communication_timeout_sec{0.2};
};

struct SafetyCheckResult
{
    bool ok{true};
    std::string reason;
};

}  // namespace uvms_hal_manipulator

#endif
