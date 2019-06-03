/*
 * velocity_limiter.cpp
 *
 *  Created on: May 30, 2019
 *      Author: christof
 */
#include <diff_drive_controller/velocity_limiter.hpp>
#include <cassert>

namespace diff_drive_controller {

VelocityLimiter::VelocityLimiter()
        : _wheel_separation(0.0f)
        , _vel_x_max(0.0f)
        , _mass(1.0f)
        , _inertia(0.0f)
        , _f_wheel_max(0.0f)
        , _f_wheel_min(0.0f)
        , _has_velocity_limits(false)
        , _has_acceleration_limits(false){};

bool VelocityLimiter::init(float wheel_separation, bool has_velocity_limits, bool has_acceleration_limits,
        float vel_x_max, float vel_th_max, float acc_x_max, float acc_x_min, float acc_th_max) {
    if (wheel_separation <= 0.0f) {
        return false;
    }
    _wheel_separation = wheel_separation;
    _has_velocity_limits = has_velocity_limits;
    _has_acceleration_limits = has_acceleration_limits;

    if (_has_acceleration_limits) {
        if (acc_x_max <= 0 || acc_x_min >= 0) {
            return false;
        }

        _f_wheel_max = 0.5f * _mass * acc_x_max;
        _f_wheel_min = 0.5f * _mass * acc_x_min;

        // J * acc_th_max = 2 * f_wheel_max / (wheel_separation / 2)
        _inertia = _wheel_separation * _f_wheel_max / acc_th_max;
    }

    if (_has_velocity_limits) {
        if (vel_x_max <= 0 || vel_th_max <= 0) {
            return false;
        }
        _vel_x_max = vel_x_max;
        _vel_th_max = vel_th_max;
    }
    return true;
};

void VelocityLimiter::limit(Vector& vel_cmd, const Vector& vel_cmd_prev, float dt) const {
    // emergency stop
    if (vel_cmd.x == 0.0f && vel_cmd.th == 0.0f) {
        return;
    }

    if (_has_velocity_limits) {
        limit_vel(vel_cmd);
    }

    if (_has_acceleration_limits) {
        limit_acc(vel_cmd, vel_cmd_prev, dt);
    }
    return;
};

void VelocityLimiter::limit_vel(Vector& vel_cmd) const {
    // initial down-scaling for angular velocity limit
    clip_both(vel_cmd.x, -_vel_x_max, _vel_x_max, vel_cmd.th, -_vel_th_max, _vel_th_max);

    // convert to wheel velocities
    float v_left = vel_cmd.x - vel_cmd.th * _wheel_separation * 0.5f;
    float v_right = vel_cmd.x + vel_cmd.th * _wheel_separation * 0.5f;

    // clip
    clip_both(v_left, v_right, -_vel_x_max, _vel_x_max);

    // convert back to x/th velocities
    vel_cmd.x = 0.5 * (v_left + v_right);
    vel_cmd.th = (v_right - v_left) / _wheel_separation;
}

void VelocityLimiter::limit_acc(Vector& vel_cmd, const Vector& vel_cmd_prev, const float dt) const {
    Vector acc{(vel_cmd.x - vel_cmd_prev.x) / dt, (vel_cmd.th - vel_cmd_prev.th) / dt};

    // calculate wheel acceleration forces
    float f_left = 0.5f * _mass * acc.x - _inertia * acc.th / _wheel_separation;
    float f_right = 0.5f * _mass * acc.x + _inertia * acc.th / _wheel_separation;

    // calculate previous wheel velocities
    const float v_left_prev = vel_cmd_prev.x - vel_cmd_prev.th * _wheel_separation * 0.5f;
    const float v_right_prev = vel_cmd_prev.x + vel_cmd_prev.th * _wheel_separation * 0.5f;

    // decide if wheel speed is increasing (acceleration) or decreasing (deceleration)
    const bool left_acc = f_left * v_left_prev >= 0.0f;
    const bool right_acc = f_right * v_right_prev >= 0.0f;

    // saturation magnitude
    const float f_left_mag = left_acc ? _f_wheel_max : -_f_wheel_min;
    const float f_right_mag = right_acc ? _f_wheel_max : -_f_wheel_min;

    // limit
    clip_both(f_left, -f_left_mag, f_left_mag, f_right, -f_right_mag, f_right_mag);

    // convert back to to x/th accelerations
    acc.x = 1.0f / _mass * (f_left + f_right);
    acc.th = 0.5f * _wheel_separation / _inertia * (f_right - f_left);

    // convert back to vel
    vel_cmd.x = vel_cmd_prev.x + acc.x * dt;
    vel_cmd.th = vel_cmd_prev.th + acc.th * dt;
}

void VelocityLimiter::clip_both(
        float& v1, const float v1_min, const float v1_max, float& v2, const float v2_min, const float v2_max) const {
    const float v1_scale = clip(v1, v1_min, v1_max);
    v2 *= v1_scale;
    const float v2_scale = clip(v2, v2_min, v2_max);
    v1 *= v2_scale;
}

void VelocityLimiter::clip_both(float& v1, float& v2, const float min, const float max) const {
    clip_both(v1, min, max, v2, min, max);
}

float VelocityLimiter::clip(float& val, const float min, const float max) const {
    if (val > max) {
        assert(max > 0.0f);
        const float scale = max / val;
        val = max;
        return scale;
    }

    if (val < min) {
        assert(min < 0.0f);
        const float scale = min / val;
        val = min;
        return scale;
    }

    return 1.0f;
}

}  // namespace diff_drive_controller
