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
 * @file adrc_control.hpp
 *
 * ADRC (Active Disturbance Rejection Control) 3 axis angular rate control.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <uORB/topics/rate_ctrl_status.h>

extern "C" {
#include "adrc.h"
}

class ADRCControl
{
public:
	ADRCControl() = default;
	~ADRCControl() = default;

	/**
	 * Set the ADRC control parameters (StarryPilot style)
	 * @param td_h0 TD time constant for each axis
	 * @param td_r0 TD gain for each axis
	 * @param leso_w LESO bandwidth for each axis
	 * @param leso_b0 control gain for LESO
	 * @param nlsef_r1 NLSEF gain for each axis
	 * @param nlsef_h1 NLSEF bandwidth for each axis
	 * @param nlsef_c NLSEF nonlinear factor for each axis
	 * @param nlsef_ki NLSEF integral gain for each axis
	 */
	void setADRCGains(const matrix::Vector3f &td_h0, const matrix::Vector3f &td_r0,
		const matrix::Vector3f &leso_w, const matrix::Vector3f &leso_b0,
		const matrix::Vector3f &nlsef_r1, const matrix::Vector3f &nlsef_h1, 
		const matrix::Vector3f &nlsef_c, const matrix::Vector3f &nlsef_ki);

	/**
	 * Initialize ADRC controllers for all axes
	 * @param dt sampling time in seconds
	 */
	void init(float dt);

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt time step
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp, const float dt);

	/**
	 * Reset ADRC controllers
	 */
	void reset();

	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getADRCStatus(rate_ctrl_status_s &rate_ctrl_status);

private:
	// ADRC controllers for each axis (roll, pitch, yaw)
	ADRC_LESO_Def _leso[3];  // Use LESO like StarryPilot
	ADRC_NLSEF_Def _nlsef[3];
	ADRC_TD_Def _td[3];
	
	// Integral states for steady-state error elimination
	matrix::Vector3f _integral;

	// TD parameters (more conservative)
	matrix::Vector3f _td_h0{0.008f, 0.008f, 0.008f};      // Increased from 0.004f for smoother tracking
	matrix::Vector3f _td_r0{300.0f, 300.0f, 300.0f};      // Reduced from 1000.0f for less aggressive derivative

	// LESO parameters (much more conservative)
	matrix::Vector3f _leso_w{50.0f, 50.0f, 50.0f};        // Reduced from 120.0f for stability
	matrix::Vector3f _leso_b0{150.0f, 150.0f, 150.0f};    // Reduced from 400.0f for smaller control gains

	// NLSEF parameters (much more conservative)
	matrix::Vector3f _nlsef_r1{30.0f, 30.0f, 30.0f};      // Reduced from 100.0f for gentler control
	matrix::Vector3f _nlsef_h1{15.0f, 15.0f, 15.0f};      // Reduced from 50.0f for stability
	matrix::Vector3f _nlsef_c{0.05f, 0.05f, 0.05f};       // Increased from 0.01f for smoother nonlinearity
	matrix::Vector3f _nlsef_ki{0.01f, 0.01f, 0.01f};      // Reduced from 0.05f to prevent integral windup

	float _dt{0.0025f};
	bool _initialized{false};
};
