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
 * @file adrc_attitude_control.cpp
 *
 * ADRC (Active Disturbance Rejection Control) attitude control implementation.
 */

#include "adrc_attitude_control.hpp"
#include <mathlib/math/Limits.hpp>
#include <stdlib.h>

using namespace matrix;

ADRCAttitudeControl::~ADRCAttitudeControl()
{
	if (_roll_leso_delay.data) {
		free(_roll_leso_delay.data);
	}
	if (_pitch_leso_delay.data) {
		free(_pitch_leso_delay.data);
	}
}

void ADRCAttitudeControl::setADRCAttitudeGains(float td_control_r2, float td_control_h2f, float td_r0,
		float nlsef_r1, float nlsef_h1f, float nlsef_c, float nlsef_ki,
		float leso_w, float gamma, float b0)
{
	_td_control_r2 = td_control_r2;
	_td_control_h2f = td_control_h2f;
	_td_r0 = td_r0;
	_nlsef_r1 = nlsef_r1;
	_nlsef_h1f = nlsef_h1f;
	_nlsef_c = nlsef_c;
	_nlsef_ki = nlsef_ki;
	_leso_w = leso_w;
	_gamma = gamma;
	_b0 = b0;
}

void ADRCAttitudeControl::init(float dt)
{
	_dt = dt;

	// Initialize TD controllers for generating rate setpoints
	adrc_td_control_init(&_roll_td_controller, _dt, _td_control_r2, _td_control_h2f * _dt);
	adrc_td_control_init(&_pitch_td_controller, _dt, _td_control_r2, _td_control_h2f * _dt);

	// Initialize TD for derivative calculation
	adrc_td_init(&_roll_td, _dt, _td_r0, _dt);
	adrc_td_init(&_pitch_td, _dt, _td_r0, _dt);

	// Initialize LESO (Linear Extended State Observer) at high frequency (1kHz)
	adrc_leso_init(&_roll_leso, 0.001f, _leso_w, _b0);
	adrc_leso_init(&_pitch_leso, 0.001f, _leso_w, _b0);

	// Initialize NLSEF (Nonlinear State Error Feedback)
	adrc_nlsef_init(&_roll_nlsef, _dt, _nlsef_r1, _nlsef_h1f * _dt, _nlsef_c);
	adrc_nlsef_init(&_pitch_nlsef, _dt, _nlsef_r1, _nlsef_h1f * _dt, _nlsef_c);

	// Create delay blocks for control signal delay compensation (3 samples delay)
	createDelayBlock(&_roll_leso_delay, 3);
	createDelayBlock(&_pitch_leso_delay, 3);

	// Reset integral states
	_integral_roll = 0.0f;
	_integral_pitch = 0.0f;

	_initialized = true;
}

void ADRCAttitudeControl::reset(float dt)
{
	_dt = dt;

	// Re-initialize all components
	adrc_td_control_init(&_roll_td_controller, _dt, _td_control_r2, _td_control_h2f * _dt);
	adrc_td_control_init(&_pitch_td_controller, _dt, _td_control_r2, _td_control_h2f * _dt);

	adrc_td_init(&_roll_td, _dt, _td_r0, _dt);
	adrc_td_init(&_pitch_td, _dt, _td_r0, _dt);

	adrc_leso_init(&_roll_leso, 0.001f, _leso_w, _b0);
	adrc_leso_init(&_pitch_leso, 0.001f, _leso_w, _b0);

	adrc_nlsef_init(&_roll_nlsef, _dt, _nlsef_r1, _nlsef_h1f * _dt, _nlsef_c);
	adrc_nlsef_init(&_pitch_nlsef, _dt, _nlsef_r1, _nlsef_h1f * _dt, _nlsef_c);

	// Flush delay blocks
	flushDelayBlock(&_roll_leso_delay);
	flushDelayBlock(&_pitch_leso_delay);

	// Reset integral states
	_integral_roll = 0.0f;
	_integral_pitch = 0.0f;
}

void ADRCAttitudeControl::observerUpdate(const Vector3f &angular_velocity, float base_throttle)
{
	if (!_initialized) {
		return;
	}

	// Update b0 parameter based on base throttle (dynamic parameter adaptation)
	// Constrain base throttle to reasonable range to avoid numerical issues
	float bt = math::constrain(base_throttle, 0.1f, 0.8f);
	float b0 = _b_const * (_cR * bt + _d) / _Ixx_yy;
	
	// Constrain b0 to prevent numerical instability
	b0 = math::constrain(b0, 50.0f, 500.0f);
	
	_roll_leso.b0 = b0;
	_pitch_leso.b0 = b0;

	// Update observers with current angular velocity
	adrc_leso(&_roll_leso, angular_velocity(0));
	adrc_leso(&_pitch_leso, angular_velocity(1));
}

Vector3f ADRCAttitudeControl::update(const Vector3f &attitude_error, const Vector3f &angular_velocity, float base_throttle)
{
	if (!_initialized) {
		init(0.004f); // Default 250Hz attitude control loop
	}

	Vector3f output;

	// TD control generates target angular velocity setpoints for roll and pitch
	// This is the main output of the ADRC attitude controller
	float roll_rate_sp = adrc_td_control(&_roll_td_controller, attitude_error(0));
	float pitch_rate_sp = adrc_td_control(&_pitch_td_controller, attitude_error(1));

	// Apply more conservative rate limits to prevent aggressive maneuvers
	const float max_rate = 0.5f; // Maximum 0.5 rad/s rate setpoint
	roll_rate_sp = math::constrain(roll_rate_sp, -max_rate, max_rate);
	pitch_rate_sp = math::constrain(pitch_rate_sp, -max_rate, max_rate);

	// Calculate rate tracking errors for observer update
	float roll_rate_error = roll_rate_sp - angular_velocity(0);
	float pitch_rate_error = pitch_rate_sp - angular_velocity(1);

	// Use TD to extract derivative of rate error
	adrc_td(&_roll_td, roll_rate_error);
	adrc_td(&_pitch_td, pitch_rate_error);

	// NLSEF control law for internal loop compensation (not directly used as output)
	float u0_roll = adrc_nlsef(&_roll_nlsef, roll_rate_error, _roll_td.v2) / _roll_leso.b0;
	float u0_pitch = adrc_nlsef(&_pitch_nlsef, pitch_rate_error, _pitch_td.v2) / _pitch_leso.b0;

	// Integral action for steady-state error elimination
	if (fabsf(_integral_roll) < 0.1f) {
		_integral_roll += roll_rate_error * _nlsef_ki * _dt;
	}
	if (fabsf(_integral_pitch) < 0.1f) {
		_integral_pitch += pitch_rate_error * _nlsef_ki * _dt;
	}

	u0_roll += _integral_roll;
	u0_pitch += _integral_pitch;

	// Constrain control outputs
	u0_roll = math::constrain(u0_roll, -0.3f, 0.3f);
	u0_pitch = math::constrain(u0_pitch, -0.3f, 0.3f);

	// Apply disturbance compensation (for internal observer state update)
	Vector2f u0(u0_roll, u0_pitch);
	disturbanceCompensation(u0);  // Updates internal LESO states

	// Output rate setpoints, not direct torque commands
	// The rate controller will use these setpoints
	output(0) = roll_rate_sp;   // Roll rate setpoint [rad/s]
	output(1) = pitch_rate_sp;  // Pitch rate setpoint [rad/s]
	output(2) = 0.0f;           // Yaw handled by traditional controller

	return output;
}

Vector2f ADRCAttitudeControl::disturbanceCompensation(const Vector2f &control_input)
{
	Vector2f output;

	// Disturbance compensation using LESO estimates
	output(0) = control_input(0) - _gamma * _roll_leso.z2 / _roll_leso.b0;
	output(1) = control_input(1) - _gamma * _pitch_leso.z2 / _pitch_leso.b0;

	// Constrain compensated output - more conservative limits
	output(0) = math::constrain(output(0), -0.3f, 0.3f);
	output(1) = math::constrain(output(1), -0.3f, 0.3f);

	// Apply control signal delay compensation
	pushDelayBlock(&_roll_leso_delay, output(0));
	_roll_leso.u = popDelayBlock(&_roll_leso_delay);

	pushDelayBlock(&_pitch_leso_delay, output(1));
	_pitch_leso.u = popDelayBlock(&_pitch_leso_delay);

	return output;
}

bool ADRCAttitudeControl::createDelayBlock(DelayBlock *block, uint16_t size)
{
	block->data = (float *)malloc(size * sizeof(float));
	if (block->data == nullptr) {
		return false;
	}
	block->size = size;
	block->head = 0;
	for (int i = 0; i < size; i++) {
		block->data[i] = 0.0f;
	}
	return true;
}

void ADRCAttitudeControl::flushDelayBlock(DelayBlock *block)
{
	if (block == nullptr) {
		return;
	}
	block->head = 0;
	for (int i = 0; i < block->size; i++) {
		block->data[i] = 0.0f;
	}
}

void ADRCAttitudeControl::pushDelayBlock(DelayBlock *block, float val)
{
	block->head = (block->head + 1) % block->size;
	block->data[block->head] = val;
}

float ADRCAttitudeControl::popDelayBlock(DelayBlock *block)
{
	uint16_t tail = (block->head + 1) % block->size;
	return block->data[tail];
}