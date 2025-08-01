/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file mc_rate_control_params.c
 *
 * Parameters for multicopter rate controller
 */

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_P, 0.15f);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_I, 0.2f);

/**
 * Roll rate integrator limit
 *
 * Roll rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large roll moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RR_INT_LIM, 0.30f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_FF, 0.0f);

/**
 * Roll rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_ROLLRATE_K * (MC_ROLLRATE_P * error
 * 			     + MC_ROLLRATE_I * error_integral
 * 			     + MC_ROLLRATE_D * error_derivative)
 * Set MC_ROLLRATE_P=1 to implement a PID in the ideal form.
 * Set MC_ROLLRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_K, 1.0f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_P, 0.15f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_I, 0.2f);

/**
 * Pitch rate integrator limit
 *
 * Pitch rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large pitch moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PR_INT_LIM, 0.30f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_FF, 0.0f);

/**
 * Pitch rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_PITCHRATE_K * (MC_PITCHRATE_P * error
 * 			     + MC_PITCHRATE_I * error_integral
 * 			     + MC_PITCHRATE_D * error_derivative)
 * Set MC_PITCHRATE_P=1 to implement a PID in the ideal form.
 * Set MC_PITCHRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_K, 1.0f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.2f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * Yaw rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large yaw moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YR_INT_LIM, 0.30f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_FF, 0.0f);

/**
 * Yaw rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_YAWRATE_K * (MC_YAWRATE_P * error
 * 			     + MC_YAWRATE_I * error_integral
 * 			     + MC_YAWRATE_D * error_derivative)
 * Set MC_YAWRATE_P=1 to implement a PID in the ideal form.
 * Set MC_YAWRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_K, 1.0f);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);

/**
 * Low pass filter cutoff frequency for yaw torque setpoint
 *
 * Reduces vibrations by lowering high frequency torque caused by rotor acceleration.
 * 0 disables the filter
 *
 * @min 0
 * @max 10
 * @unit Hz
 * @decimal 3
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_TQ_CUTOFF, 2.f);

/**
 * Enable ADRC (Active Disturbance Rejection Control) for rate control
 *
 * Enable ADRC rate control instead of traditional PID control.
 * When enabled, ADRC parameters are used instead of PID parameters.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_ADRC_ENABLE, 0);

/**
 * ADRC roll axis TD time constant (h0)
 *
 * Time constant for Tracking Differentiator on roll axis.
 * Controls the smoothness of the reference tracking.
 *
 * @min 0.001
 * @max 0.1
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_TD_H0_R, 0.004f);

/**
 * ADRC pitch axis TD time constant (h0)
 *
 * Time constant for Tracking Differentiator on pitch axis.
 * Controls the smoothness of the reference tracking.
 *
 * @min 0.001
 * @max 0.1
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_TD_H0_P, 0.004f);

/**
 * ADRC yaw axis TD time constant (h0)
 *
 * Time constant for Tracking Differentiator on yaw axis.
 * Controls the smoothness of the reference tracking.
 *
 * @min 0.001
 * @max 0.1
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_TD_H0_Y, 0.004f);

/**
 * ADRC roll axis TD gain (r0)
 *
 * Gain for Tracking Differentiator on roll axis.
 * Higher values provide faster tracking but may cause oscillations.
 *
 * @min 100.0
 * @max 5000.0
 * @decimal 1
 * @increment 100.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_TD_R0_R, 1000.0f);

/**
 * ADRC pitch axis TD gain (r0)
 *
 * Gain for Tracking Differentiator on pitch axis.
 * Higher values provide faster tracking but may cause oscillations.
 *
 * @min 100.0
 * @max 5000.0
 * @decimal 1
 * @increment 100.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_TD_R0_P, 1000.0f);

/**
 * ADRC yaw axis TD gain (r0)
 *
 * Gain for Tracking Differentiator on yaw axis.
 * Higher values provide faster tracking but may cause oscillations.
 *
 * @min 100.0
 * @max 5000.0
 * @decimal 1
 * @increment 100.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_TD_R0_Y, 1000.0f);

/**
 * ADRC roll axis ESO beta1 gain
 *
 * Observer gain beta1 for Extended State Observer on roll axis.
 * Controls the observer convergence speed.
 *
 * @min 10.0
 * @max 200.0
 * @decimal 1
 * @increment 5.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ESO_B1_R, 70.0f);

/**
 * ADRC pitch axis ESO beta1 gain
 *
 * Observer gain beta1 for Extended State Observer on pitch axis.
 * Controls the observer convergence speed.
 *
 * @min 10.0
 * @max 200.0
 * @decimal 1
 * @increment 5.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ESO_B1_P, 70.0f);

/**
 * ADRC yaw axis ESO beta1 gain
 *
 * Observer gain beta1 for Extended State Observer on yaw axis.
 * Controls the observer convergence speed.
 *
 * @min 10.0
 * @max 200.0
 * @decimal 1
 * @increment 5.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ESO_B1_Y, 70.0f);

/**
 * ADRC roll axis ESO beta2 gain
 *
 * Observer gain beta2 for Extended State Observer on roll axis.
 * Controls the disturbance estimation capability.
 *
 * @min 500.0
 * @max 10000.0
 * @decimal 1
 * @increment 100.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ESO_B2_R, 2500.0f);

/**
 * ADRC pitch axis ESO beta2 gain
 *
 * Observer gain beta2 for Extended State Observer on pitch axis.
 * Controls the disturbance estimation capability.
 *
 * @min 500.0
 * @max 10000.0
 * @decimal 1
 * @increment 100.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ESO_B2_P, 2500.0f);

/**
 * ADRC yaw axis ESO beta2 gain
 *
 * Observer gain beta2 for Extended State Observer on yaw axis.
 * Controls the disturbance estimation capability.
 *
 * @min 500.0
 * @max 10000.0
 * @decimal 1
 * @increment 100.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ESO_B2_Y, 2500.0f);

/**
 * ADRC roll axis ESO b0 control gain
 *
 * Control gain b0 for Extended State Observer on roll axis.
 * Represents the control effectiveness.
 *
 * @min 50.0
 * @max 1000.0
 * @decimal 1
 * @increment 10.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ESO_B0_R, 400.0f);

/**
 * ADRC pitch axis ESO b0 control gain
 *
 * Control gain b0 for Extended State Observer on pitch axis.
 * Represents the control effectiveness.
 *
 * @min 50.0
 * @max 1000.0
 * @decimal 1
 * @increment 10.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ESO_B0_P, 400.0f);

/**
 * ADRC yaw axis ESO b0 control gain
 *
 * Control gain b0 for Extended State Observer on yaw axis.
 * Represents the control effectiveness.
 *
 * @min 50.0
 * @max 1000.0
 * @decimal 1
 * @increment 10.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ESO_B0_Y, 400.0f);

/**
 * ADRC roll axis NLSEF r1 gain
 *
 * Nonlinear State Error Feedback gain r1 for roll axis.
 * Controls the position error feedback strength.
 *
 * @min 10.0
 * @max 500.0
 * @decimal 1
 * @increment 10.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_SEF_R1_R, 100.0f);

/**
 * ADRC pitch axis NLSEF r1 gain
 *
 * Nonlinear State Error Feedback gain r1 for pitch axis.
 * Controls the position error feedback strength.
 *
 * @min 10.0
 * @max 500.0
 * @decimal 1
 * @increment 10.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_SEF_R1_P, 100.0f);

/**
 * ADRC yaw axis NLSEF r1 gain
 *
 * Nonlinear State Error Feedback gain r1 for yaw axis.
 * Controls the position error feedback strength.
 *
 * @min 10.0
 * @max 500.0
 * @decimal 1
 * @increment 10.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_SEF_R1_Y, 100.0f);

/**
 * ADRC roll axis NLSEF h1 gain
 *
 * Nonlinear State Error Feedback gain h1 for roll axis.
 * Controls the velocity error feedback strength.
 *
 * @min 5.0
 * @max 200.0
 * @decimal 1
 * @increment 5.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_SEF_H1_R, 50.0f);

/**
 * ADRC pitch axis NLSEF h1 gain
 *
 * Nonlinear State Error Feedback gain h1 for pitch axis.
 * Controls the velocity error feedback strength.
 *
 * @min 5.0
 * @max 200.0
 * @decimal 1
 * @increment 5.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_SEF_H1_P, 50.0f);

/**
 * ADRC yaw axis NLSEF h1 gain
 *
 * Nonlinear State Error Feedback gain h1 for yaw axis.
 * Controls the velocity error feedback strength.
 *
 * @min 5.0
 * @max 200.0
 * @decimal 1
 * @increment 5.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_SEF_H1_Y, 50.0f);

/**
 * ADRC roll axis NLSEF c gain
 *
 * Nonlinear State Error Feedback nonlinear factor c for roll axis.
 * Controls the nonlinearity strength.
 *
 * @min 0.001
 * @max 0.1
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_SEF_C_R, 0.01f);

/**
 * ADRC pitch axis NLSEF c gain
 *
 * Nonlinear State Error Feedback nonlinear factor c for pitch axis.
 * Controls the nonlinearity strength.
 *
 * @min 0.001
 * @max 0.1
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_SEF_C_P, 0.01f);

/**
 * ADRC yaw axis NLSEF c gain
 *
 * Nonlinear State Error Feedback nonlinear factor c for yaw axis.
 * Controls the nonlinearity strength.
 *
 * @min 0.001
 * @max 0.1
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_SEF_C_Y, 0.01f);

/**
 * ADRC roll axis ESO nonlinear factor (alpha)
 *
 * Nonlinear factor for Extended State Observer on roll axis.
 * Controls the nonlinearity strength of the observer.
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ALPHA_R, 0.25f);

/**
 * ADRC pitch axis ESO nonlinear factor (alpha)
 *
 * Nonlinear factor for Extended State Observer on pitch axis.
 * Controls the nonlinearity strength of the observer.
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ALPHA_P, 0.25f);

/**
 * ADRC yaw axis ESO nonlinear factor (alpha)
 *
 * Nonlinear factor for Extended State Observer on yaw axis.
 * Controls the nonlinearity strength of the observer.
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_ALPHA_Y, 0.25f);

/**
 * ADRC roll axis ESO linear interval (delta)
 *
 * Linear interval for Extended State Observer on roll axis.
 * Defines the linear region around zero.
 *
 * @min 0.001
 * @max 0.5
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_DELTA_R, 0.1f);

/**
 * ADRC pitch axis ESO linear interval (delta)
 *
 * Linear interval for Extended State Observer on pitch axis.
 * Defines the linear region around zero.
 *
 * @min 0.001
 * @max 0.5
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_DELTA_P, 0.1f);

/**
 * ADRC yaw axis ESO linear interval (delta)
 *
 * Linear interval for Extended State Observer on yaw axis.
 * Defines the linear region around zero.
 *
 * @min 0.001
 * @max 0.5
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ADRC_DELTA_Y, 0.1f);
