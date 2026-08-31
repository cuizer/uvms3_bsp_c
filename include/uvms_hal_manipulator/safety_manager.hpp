#ifndef UVMS_HAL_MANIPULATOR_SAFETY_MANAGER_HPP
#define UVMS_HAL_MANIPULATOR_SAFETY_MANAGER_HPP

#include <vector>
#include <string>
#include "types.hpp"

namespace uvms_hal_manipulator
{

class SafetyManager
{
public:
    SafetyManager() = default;
    ~SafetyManager() = default;

    void set_joint_limit_config(const JointLimitConfig& config);
    void set_motor_safety_config(const MotorSafetyConfig& config);

    void set_estop(bool estop);
    bool is_estop() const;

    SafetyCheckResult validate_joint_command(
        const std::vector<double>& positions,
        const std::vector<double>& velocities) const;

    std::vector<double> clamp_velocity(
        const std::vector<double>& velocities) const;

    SafetyCheckResult check_motor_overload(
        const std::vector<double>& currents,
        const std::vector<double>& temperatures) const;

    SafetyCheckResult check_communication_timeout(
        double elapsed_sec) const;

private:
    bool check_size_match(
        const std::vector<double>& positions,
        const std::vector<double>& velocities) const;

private:
    JointLimitConfig joint_limit_config_;
    MotorSafetyConfig motor_safety_config_;
    bool estop_{false};
};

}  // namespace uvms_hal_manipulator

#endif