/*
 * velocity_limiter.hpp
 *
 *  Created on: May 30, 2019
 *      Author: Christof Dubs
 */
#pragma once

#include <limits>

namespace diff_drive_controller {

/**
 * \brief Class for limiting velocity commands of a differential drive type robot.
 *
 * Limiting the velocity command [vel_x, vel_theta] is based on a simple dynamic model.
 * Internally, it makes use of the estimated mass to rotational inertia ratio, which is
 * estimated from the `acc_x_max` and `acc_th_max` parameters. It is therefore important that
 * these parameters are set to the actual limits of the robot (or that at least the ratio
 * between the two should be identical to the ratio of the actual limits).
 * The `vel_x_max` parameter should be set conservatively, such that simultaneous large
 * forward and angular velocity commands can be down-scaled properly.
 */
class VelocityLimiter {
public:
    struct Vector {
        float x;
        float th;
    };

    struct Config {
        float wheel_separation = 0.0f;         ///< [m] wheel separation distance
        bool has_velocity_limits = false;      ///< [-] enable velocity limits
        bool has_acceleration_limits = false;  ///< [-] enable acceleration limits
        float vel_x_max = 0.0;                 ///< [m/s] robot's maximum achievable velocity in forward direction (>0)
        float vel_th_max =
                std::numeric_limits<float>::max();  ///< [rad/s] user-defined desired maximum angular velocity
        float acc_x_max = 0.0;   ///< [m/s^2] robot's maximum achievable acceleration in forward direction (>0)
        float acc_x_min = 0.0;   ///< [m/s^2] robot's maximum achievable deceleration in forward direction (<0)
        float acc_th_max = 0.0;  ///< [rad/s^2] robot's maximum achievable angular acceleration (>0)
        bool isValid() const;
    };

    VelocityLimiter();

    bool init(const Config& config);

    void limit(Vector& vel_cmd, const Vector& vel_cmd_prev, const float dt) const;

private:
    void limitVel(Vector& vel_des) const;
    void limitAcc(Vector& vel_cmd, const Vector& vel_cmd_prev, const float dt) const;

    void downscaleBoth(
            float& v1, const float v1_min, const float v1_max, float& v2, const float v2_min, const float v2_max) const;
    void downscaleBoth(float& v1, float& v2, const float min, const float max) const;
    float clip(float& val, const float min, const float max) const;

    static constexpr float _mass = 1.0f;
    float _wheel_separation;
    float _vel_x_max;
    float _vel_th_max;
    float _inertia;
    float _f_wheel_max;
    float _f_wheel_min;
    bool _has_velocity_limits;
    bool _has_acceleration_limits;
};

}  // namespace diff_drive_controller
