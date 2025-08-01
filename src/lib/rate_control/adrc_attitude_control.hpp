/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.  
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file adrc_attitude_control.hpp
 *
 * ADRC (Active Disturbance Rejection Control) attitude control for multicopter.
 * Based on StarryPilot ADRC implementation.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

extern "C" {
#include "adrc.h"
}

struct DelayBlock {
	uint16_t size;
	uint16_t head;
	float *data;
};

class ADRCAttitudeControl
{
public:
	ADRCAttitudeControl() = default;
	~ADRCAttitudeControl();

	/**
	 * Initialize ADRC attitude controller
	 * @param dt sampling time in seconds
	 */
	void init(float dt);

	/**
	 * Reset ADRC attitude controller
	 */
	void reset(float dt);

	/**
	 * Set ADRC attitude control parameters
	 * @param td_control_r2 TD control gain
	 * @param td_control_h2f TD control bandwidth factor
	 * @param td_r0 TD tracking gain
	 * @param nlsef_r1 NLSEF gain
	 * @param nlsef_h1f NLSEF bandwidth factor  
	 * @param nlsef_c NLSEF damping factor
	 * @param nlsef_ki NLSEF integral gain
	 * @param leso_w LESO bandwidth
	 * @param gamma disturbance compensation gain
	 * @param b0 control effectiveness parameter
	 */
	void setADRCAttitudeGains(float td_control_r2, float td_control_h2f, float td_r0,
				  float nlsef_r1, float nlsef_h1f, float nlsef_c, float nlsef_ki,
				  float leso_w, float gamma, float b0);

	/**
	 * Update ADRC attitude observer (should run at high frequency, e.g., 1kHz)
	 * @param angular_velocity current angular velocity [rad/s]
	 * @param base_throttle current base throttle [0,1]
	 */
	void observerUpdate(const matrix::Vector3f &angular_velocity, float base_throttle);

	/**
	 * Run ADRC attitude control loop
	 * @param attitude_error attitude error in Euler angles [rad]
	 * @param angular_velocity current angular velocity [rad/s]
	 * @param base_throttle current base throttle [0,1]
	 * @return rate setpoint for roll and pitch, direct torque for yaw [rad/s or normalized]
	 */
	matrix::Vector3f update(const matrix::Vector3f &attitude_error, const matrix::Vector3f &angular_velocity, float base_throttle);

	/**
	 * Apply disturbance compensation
	 * @param control_input control input before compensation
	 * @return compensated control output
	 */
	matrix::Vector2f disturbanceCompensation(const matrix::Vector2f &control_input);

private:
	// ADRC components for roll and pitch (yaw uses traditional PID)
	TD_Controller_Def _roll_td_controller;
	TD_Controller_Def _pitch_td_controller;
	ADRC_TD_Def _roll_td;
	ADRC_TD_Def _pitch_td;
	ADRC_NLSEF_Def _roll_nlsef;
	ADRC_NLSEF_Def _pitch_nlsef;
	ADRC_LESO_Def _roll_leso;
	ADRC_LESO_Def _pitch_leso;

	// Delay blocks for disturbance compensation
	DelayBlock _roll_leso_delay;
	DelayBlock _pitch_leso_delay;

	// Integral states
	float _integral_roll{0.0f};
	float _integral_pitch{0.0f};

	// Physical parameters (adapted for PX4 default quadrotor)
	static constexpr float _sin_45 = 0.70711f;
	static constexpr float _cR = 400.0f;          // Reduced for typical PX4 quadrotor
	static constexpr float _cT = 8.54858e-6f;     // Typical thrust coefficient
	static constexpr float _L = 0.225f;           // Typical arm length for PX4 quadrotor
	static constexpr float _d = 50.0f;            // Reduced drag coefficient
	static constexpr float _Ixx_yy = 0.0347563f;  // PX4 default quadrotor inertia
	static constexpr float _b_const = 8.0f * _cR * _cT * _L * _sin_45;

	// ADRC parameters (very conservative defaults for stability)
	float _td_control_r2{8.0f};       // Reduced from 15.0 for much smoother tracking
	float _td_control_h2f{5.0f};      // Reduced from 10.0 for more stability
	float _td_r0{200.0f};             // Reduced from 500.0 for less aggressive derivative
	float _nlsef_r1{20.0f};           // Reduced from 50.0 for gentler control
	float _nlsef_h1f{10.0f};          // Reduced from 25.0 for stability
	float _nlsef_c{0.05f};            // Increased from 0.02 for smoother nonlinearity
	float _nlsef_ki{0.01f};           // Reduced from 0.02 to prevent integral windup
	float _leso_w{30.0f};             // Reduced from 60.0 for less aggressive observer
	float _gamma{0.15f};              // Reduced from 0.30 for gentler disturbance compensation
	float _b0{100.0f};                // Reduced from 200.0 for smaller control gains

	float _dt{0.004f};
	bool _initialized{false};

	// Helper functions
	bool createDelayBlock(DelayBlock *block, uint16_t size);
	void flushDelayBlock(DelayBlock *block);
	void pushDelayBlock(DelayBlock *block, float val);
	float popDelayBlock(DelayBlock *block);
};