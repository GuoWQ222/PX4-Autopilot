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
 * @file adrc_control.cpp
 *
 * ADRC (Active Disturbance Rejection Control) angular rate control implementation.
 */

#include "adrc_control.hpp"
#include <mathlib/math/Limits.hpp>

using namespace matrix;

void ADRCControl::setADRCGains(const Vector3f &td_h0, const Vector3f &td_r0,
	const Vector3f &leso_w, const Vector3f &leso_b0,
	const Vector3f &nlsef_r1, const Vector3f &nlsef_h1, 
	const Vector3f &nlsef_c, const Vector3f &nlsef_ki)
{
	_td_h0 = td_h0;
	_td_r0 = td_r0;
	_leso_w = leso_w;
	_leso_b0 = leso_b0;
	_nlsef_r1 = nlsef_r1;
	_nlsef_h1 = nlsef_h1;
	_nlsef_c = nlsef_c;
	_nlsef_ki = nlsef_ki;
}

void ADRCControl::init(float dt)
{
	_dt = dt;

	for (int i = 0; i < 3; i++) {
		// Initialize Linear Extended State Observer (LESO) - StarryPilot style
		adrc_leso_init(&_leso[i], _dt, _leso_w(i), _leso_b0(i));

		// Initialize Nonlinear State Error Feedback (NLSEF)
		adrc_nlsef_init(&_nlsef[i], _dt, _nlsef_r1(i), _nlsef_h1(i), _nlsef_c(i));

		// Initialize Tracking Differentiator (TD)
		adrc_td_init(&_td[i], _dt, _td_r0(i), _td_h0(i));
	}

	// Reset integral states
	_integral.zero();

	_initialized = true;
}

Vector3f ADRCControl::update(const Vector3f &rate, const Vector3f &rate_sp, const float dt)
{
	if (!_initialized) {
		init(dt);
	}

	Vector3f torque_output;

	for (int i = 0; i < 3; i++) {
		// Apply input saturation to rate setpoint to prevent aggressive commands
		float rate_sp_limited = math::constrain(rate_sp(i), -3.0f, 3.0f);
		
		// Update tracking differentiator with limited setpoint
		adrc_td(&_td[i], rate_sp_limited);

		// Update Linear Extended State Observer with actual rate
		adrc_leso(&_leso[i], rate(i));

		// Calculate tracking error using TD output and LESO estimate
		float e1 = _td[i].v1 - _leso[i].z1;  // position error
		float e2 = _td[i].v2 - _leso[i].z2;  // velocity error

		// Apply error saturation to prevent excessive control effort
		e1 = math::constrain(e1, -1.0f, 1.0f);
		e2 = math::constrain(e2, -5.0f, 5.0f);

		// Calculate control output using NLSEF
		float u0 = adrc_nlsef(&_nlsef[i], e1, e2);

		// Add integral action for steady-state error elimination (with anti-windup)
		float rate_error = rate_sp_limited - rate(i);
		rate_error = math::constrain(rate_error, -0.5f, 0.5f); // Limit error for integration
		
		if (fabsf(_integral(i)) < 0.05f) { // Reduced integral limit
			_integral(i) += rate_error * _nlsef_ki(i) * _dt;
		}
		u0 += _integral(i);

		// Apply strict u0 saturation before disturbance compensation
		u0 = math::constrain(u0, -0.5f, 0.5f);

		// Total control effort (compensate disturbance with limited gain)
		float disturbance_compensation = math::constrain(_leso[i].z2 / _leso_b0(i), -0.3f, 0.3f);
		float u = u0 - disturbance_compensation;

		// Update LESO input for next iteration
		_leso[i].u = u;

		// Final output saturation with more conservative limits
		torque_output(i) = math::constrain(u, -0.8f, 0.8f);
	}

	return torque_output;
}

void ADRCControl::reset()
{
	for (int i = 0; i < 3; i++) {
		// Reset LESO states
		_leso[i].z1 = 0.0f;
		_leso[i].z2 = 0.0f;
		_leso[i].u = 0.0f;

		// Reset TD states
		_td[i].v1 = 0.0f;
		_td[i].v2 = 0.0f;
	}

	// Reset integral states
	_integral.zero();
}

void ADRCControl::getADRCStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	// Fill status structure with ADRC-specific information (using LESO)
	rate_ctrl_status.rollspeed_integ = _leso[0].z2;  // Use z2 as disturbance estimate
	rate_ctrl_status.pitchspeed_integ = _leso[1].z2;
	rate_ctrl_status.yawspeed_integ = _leso[2].z2;

	// Store observer states for roll
	rate_ctrl_status.additional_status[0] = _leso[0].z1;
	// Store observer states for pitch
	rate_ctrl_status.additional_status[1] = _leso[1].z1;
	// Store observer states for yaw
	rate_ctrl_status.additional_status[2] = _leso[2].z1;
}
