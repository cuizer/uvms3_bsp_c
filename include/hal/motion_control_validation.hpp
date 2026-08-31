#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace hal::motion {

struct PidConfig {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double max_i = 0.0;
    double max_out = 0.0;
};

struct MotionControlConfig {
    double control_rate_hz = 0.0;
    double command_timeout_s = 0.0;
    double thrust_min_pct = 0.0;
    double thrust_max_pct = 0.0;
    double servo_max_deg = 0.0;
    std::array<PidConfig, 6U> pid{};
    // linear/quad X, Y, Z, yaw followed by buoyancy trim.
    std::array<double, 9U> feedforward{};
    std::vector<double> allocation;
};

struct ConfigValidationResult {
    bool ok = false;
    std::string message;
};

inline bool allocationHasFullRank(const std::vector<double> &matrix)
{
    constexpr std::size_t DIMENSION = 6U;
    constexpr double PIVOT_EPSILON = 1.0e-9;
    if (matrix.size() != DIMENSION * DIMENSION) return false;

    std::array<double, DIMENSION * DIMENSION> work{};
    std::copy(matrix.begin(), matrix.end(), work.begin());
    std::size_t rank = 0U;
    for (std::size_t column = 0U; column < DIMENSION && rank < DIMENSION; ++column) {
        std::size_t pivot = rank;
        for (std::size_t row = rank + 1U; row < DIMENSION; ++row) {
            if (std::abs(work[row * DIMENSION + column]) >
                std::abs(work[pivot * DIMENSION + column])) {
                pivot = row;
            }
        }
        if (std::abs(work[pivot * DIMENSION + column]) <= PIVOT_EPSILON) {
            continue;
        }
        if (pivot != rank) {
            for (std::size_t index = 0U; index < DIMENSION; ++index) {
                std::swap(
                    work[rank * DIMENSION + index],
                    work[pivot * DIMENSION + index]);
            }
        }
        const double pivot_value = work[rank * DIMENSION + column];
        for (std::size_t row = rank + 1U; row < DIMENSION; ++row) {
            const double factor = work[row * DIMENSION + column] / pivot_value;
            for (std::size_t index = column; index < DIMENSION; ++index) {
                work[row * DIMENSION + index] -=
                    factor * work[rank * DIMENSION + index];
            }
        }
        ++rank;
    }
    return rank == DIMENSION;
}

inline ConfigValidationResult validateMotionControlConfig(
    const MotionControlConfig &config)
{
    const auto invalid = [](const std::string &message) {
        return ConfigValidationResult{false, message};
    };
    if (!std::isfinite(config.control_rate_hz) ||
        config.control_rate_hz <= 0.0 || config.control_rate_hz > 1000.0) {
        return invalid("control_rate_hz must be finite and in (0, 1000]");
    }
    if (!std::isfinite(config.command_timeout_s) ||
        config.command_timeout_s <= 0.0 || config.command_timeout_s > 60.0) {
        return invalid("cmd_timeout_s must be finite and in (0, 60]");
    }
    if (!std::isfinite(config.thrust_min_pct) ||
        !std::isfinite(config.thrust_max_pct) ||
        config.thrust_min_pct < -100.0 || config.thrust_max_pct > 100.0 ||
        config.thrust_min_pct >= config.thrust_max_pct) {
        return invalid("thrust limits must be ordered and inside [-100, 100]");
    }
    if (!std::isfinite(config.servo_max_deg) ||
        config.servo_max_deg <= 0.0 || config.servo_max_deg > 180.0) {
        return invalid("servo_max_deg must be finite and in (0, 180]");
    }
    for (std::size_t index = 0U; index < config.pid.size(); ++index) {
        const auto &pid = config.pid[index];
        if (!std::isfinite(pid.kp) || !std::isfinite(pid.ki) ||
            !std::isfinite(pid.kd) || !std::isfinite(pid.max_i) ||
            !std::isfinite(pid.max_out) || pid.kp < 0.0 || pid.ki < 0.0 ||
            pid.kd < 0.0 || pid.max_i < 0.0 || pid.max_out <= 0.0) {
            return invalid("PID parameters must be finite with non-negative gains/"
                "integral limit and a positive output limit");
        }
    }
    for (std::size_t index = 0U; index < config.feedforward.size(); ++index) {
        const double value = config.feedforward[index];
        if (!std::isfinite(value) || std::abs(value) > 10000.0 ||
            (index < 8U && value < 0.0)) {
            return invalid("feedforward parameters are non-finite or outside limits");
        }
    }
    if (config.allocation.size() != 36U) {
        return invalid("alloc_matrix must contain exactly 36 elements");
    }
    if (!std::all_of(config.allocation.begin(), config.allocation.end(),
            [](double value) { return std::isfinite(value) && std::abs(value) <= 1000.0; })) {
        return invalid("alloc_matrix contains a non-finite or excessive coefficient");
    }
    if (!allocationHasFullRank(config.allocation)) {
        return invalid("alloc_matrix must have full rank");
    }
    return ConfigValidationResult{true, "controller configuration is valid"};
}

}  // namespace hal::motion
