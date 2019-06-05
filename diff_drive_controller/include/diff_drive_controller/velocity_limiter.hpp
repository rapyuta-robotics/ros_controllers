/*
 * velocity_limiter.hpp
 *
 *  Created on: May 30, 2019
 *      Author: Christof Dubs
 */

#pragma once

namespace diff_drive_controller {

class VelocityLimiter {
public:
    struct Vector {
        float x;
        float th;
    };

    struct Config {
        float wheel_separation = 0.0f;
        bool has_velocity_limits = false;
        bool has_acceleration_limits = false;
        float vel_x_max = 0.0;
        float vel_th_max = 0.0;
        float acc_x_max = 0.0;
        float acc_x_min = 0.0;
        float acc_th_max = 0.0;
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
