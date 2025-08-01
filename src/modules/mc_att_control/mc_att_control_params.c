/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_P, 4.0f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 4.0f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_P, 2.8f);

/**
 * Yaw weight
 *
 * A fraction [0,1] deprioritizing yaw compared to roll and pitch in non-linear attitude control.
 * Deprioritizing yaw is necessary because multicopters have much less control authority
 * in yaw compared to the other axes and it makes sense because yaw is not critical for
 * stable hovering or 3D navigation.
 *
 * For yaw control tuning use MC_YAW_P. This ratio has no impact on the yaw gain.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_WEIGHT, 0.4f);

/**
 * Max roll rate
 *
 * Limit for roll rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_MAX, 220.0f);

/**
 * Max pitch rate
 *
 * Limit for pitch rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_MAX, 220.0f);

/**
 * Max yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_MAX, 200.0f);

/**
 * Manual tilt input filter time constant
 *
 * Setting this parameter to 0 disables the filter
 *
 * @unit s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_MAN_TILT_TAU, 0.0f);

/**
 * Enable ADRC attitude control
 *
 * Enable Active Disturbance Rejection Control (ADRC) for attitude control.
 * 0: Disabled (use traditional PID)
 * 1: ADRC enabled for roll and pitch only
 * 2: Full ADRC for all axes
 *
 * @min 0
 * @max 2
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_INT32(MC_ADRC_ATT_EN, 2);

/**
 * ADRC TD Control R2 parameter
 *
 * Tracking differentiator control gain for ADRC attitude control.
 * Higher values provide faster tracking but may introduce oscillations.
 *
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_TD_R2, 15.0f);

/**
 * ADRC TD Control H2 factor
 *
 * Tracking differentiator bandwidth factor for ADRC attitude control.
 * Multiplied by sample time to get actual bandwidth.
 *
 * @min 1.0
 * @max 50.0
 * @decimal 1
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_TD_H2F, 10.0f);

/**
 * ADRC TD R0 parameter
 *
 * Tracking differentiator gain for derivative calculation in ADRC.
 *
 * @min 100.0
 * @max 5000.0
 * @decimal 1
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_TD_R0, 500.0f);

/**
 * ADRC NLSEF R1 parameter
 *
 * Nonlinear state error feedback gain for ADRC attitude control.
 *
 * @min 10.0
 * @max 500.0
 * @decimal 1
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_NLSEF_R1, 50.0f);

/**
 * ADRC NLSEF H1 factor
 *
 * Nonlinear state error feedback bandwidth factor for ADRC attitude control.
 * Multiplied by sample time to get actual bandwidth.
 *
 * @min 10.0
 * @max 200.0
 * @decimal 1
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_NLSEF_H1, 25.0f);

/**
 * ADRC NLSEF C parameter
 *
 * Nonlinear state error feedback damping coefficient for ADRC attitude control.
 *
 * @min 0.001
 * @max 0.1
 * @decimal 3
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_NLSEF_C, 0.02f);

/**
 * ADRC NLSEF integral gain
 *
 * Integral gain for ADRC attitude control to eliminate steady-state errors.
 *
 * @min 0.0
 * @max 0.2
 * @decimal 3
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_NLSEF_KI, 0.02f);

/**
 * ADRC LESO bandwidth
 *
 * Linear Extended State Observer bandwidth for ADRC attitude control.
 * Higher values provide faster disturbance estimation but may be more sensitive to noise.
 *
 * @min 50.0
 * @max 500.0
 * @decimal 1
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_LESO_W, 60.0f);

/**
 * ADRC disturbance compensation gain
 *
 * Gamma parameter for disturbance compensation in ADRC attitude control.
 * Controls how much of the estimated disturbance is compensated.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_GAMMA, 0.30f);

/**
 * ADRC control effectiveness parameter
 *
 * B0 parameter representing the control effectiveness in ADRC attitude control.
 * Should be tuned based on vehicle characteristics.
 *
 * @min 100.0
 * @max 1000.0
 * @decimal 1
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_B0, 200.0f);

/**
 * ADRC observer update frequency
 *
 * Frequency at which the ADRC observer is updated. Higher frequencies improve
 * disturbance estimation accuracy but increase computational load.
 *
 * @unit Hz
 * @min 100
 * @max 2000
 * @group ADRC Attitude Control
 */
PARAM_DEFINE_INT32(MC_ADRC_OBS_FREQ, 250);
