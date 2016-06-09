/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file mini_commander.cpp
 *
 * Lean mini version of the big brother commander.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_armed.h>

#include "mini_commander.h"

MiniCommander::MiniCommander() :
	_task_is_running(false),
	_task_should_exit(false),
	_battery_status_sub(-1),
	_offboard_control_mode_sub(-1),
	_vehicle_global_position_sub(-1),
	_vehicle_local_position_sub(-1),
	_vehicle_land_detected_sub(-1),
	_home_position_pub(nullptr),
	_vehicle_control_mode_pub(nullptr),
	_vehicle_status_pub(nullptr),
	_actuator_armed_pub(nullptr),
	_offboard_control_mode{},
	_vehicle_global_position{},
	_vehicle_local_position{},
	_vehicle_land_detected{},
	_home_position{},
	_home_position_set(false),
	_failsafe_sm(),
	_failsafe_sm_inputs{},
	_arming_sm(),
	_arming_sm_inputs{}
{
	/* Set sane defaults for input fields. */
	_failsafe_sm_inputs.landed = true;
	_failsafe_sm_inputs.offboard_ok = false;
	_failsafe_sm_inputs.gps_ok = false;

	_arming_sm_inputs.landed = true;
	_arming_sm_inputs.home_position_set = false;
}

MiniCommander::~MiniCommander()
{}

void
MiniCommander::task_main()
{
	_task_is_running = true;

	while (!_task_should_exit) {

		_check_topics();

		/* Continuously try to set home position unless armed. */
		if (!_arming_sm.is_armed()) {
			_set_home_position();
			_arming_sm_inputs.home_position_set = _home_position_set;
		}

		_failsafe_sm.input(_failsafe_sm_inputs);
		_failsafe_sm.spin();

		_arming_sm.input(_arming_sm_inputs);
		_arming_sm.spin();

		_publish_topics();
		usleep(_approx_interval_us);
	}

	_task_is_running = false;
}

void
MiniCommander::print_status()
{
	PX4_INFO("current navigation state: %s", _failsafe_sm.get_nav_state_str());
	PX4_INFO("in failsafe: %s", _failsafe_sm.is_in_failsafe() ? "yes" : "no");
	PX4_INFO("home positon set: %s", _home_position_set ? "yes" : "no");
	PX4_INFO("armed: %s", _arming_sm.is_armed() ? "yes" : "no");
	PX4_INFO("ready to arm: %s", _arming_sm.is_ready_to_arm() ? "yes" : "no");
}

void
MiniCommander::arm()
{
	_arming_sm.request_arm();
}

void
MiniCommander::_check_topics()
{
	_check_battery_status();
	_check_offboard_control_mode();
	_check_vehicle_global_position();
	_check_vehicle_local_position();
	_check_vehicle_land_detected();
}

void
MiniCommander::_publish_topics()
{
	/* Note that the home position is published in _set_home_position() */
	_publish_vehicle_control_mode();
	_publish_vehicle_status();
	_publish_actuator_armed();
}

void
MiniCommander::_check_battery_status()
{
	if (_battery_status_sub == -1) {
		_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	}

	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		battery_status_s battery_status;
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &battery_status);

		if (battery_status.warning == battery_status_s::BATTERY_WARNING_LOW) {
			// TODO: maybe do something
		} else if (battery_status.warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
			// TODO: maybe do something
		}
	}
}

void
MiniCommander::_check_offboard_control_mode()
{
	if (_offboard_control_mode_sub == -1) {
		_offboard_control_mode_sub = orb_subscribe(ORB_ID(offboard_control_mode));
	}

	bool updated;
	orb_check(_offboard_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(offboard_control_mode), _offboard_control_mode_sub, &_offboard_control_mode);
	}

	if (_offboard_control_mode.timestamp != 0 &&
	    hrt_elapsed_time(&_offboard_control_mode.timestamp) < _offboard_timeout_us) {
		_failsafe_sm_inputs.offboard_ok = true;

	} else {
		_failsafe_sm_inputs.offboard_ok = false;
	}
}

void
MiniCommander::_check_vehicle_global_position()
{
	if (_vehicle_global_position_sub == -1) {
		_vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	}

	bool updated;
	orb_check(_vehicle_global_position_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_global_position), _vehicle_global_position_sub, &_vehicle_global_position);
		/* Just update the topic. */
	}

	/* TODO: improve these GPS ok/lost tests with estimator information. */
	if (_vehicle_global_position.timestamp != 0 &&
	    hrt_elapsed_time(&_vehicle_global_position.timestamp) < _vehicle_global_position_timeout_us) {
		_failsafe_sm_inputs.gps_ok = true;

	} else {
		_failsafe_sm_inputs.gps_ok = false;
	}
}

void
MiniCommander::_check_vehicle_local_position()
{
	if (_vehicle_local_position_sub == -1) {
		_vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	}

	bool updated;
	orb_check(_vehicle_local_position_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &_vehicle_local_position);
		/* Just update the topic. */
	}
}

void
MiniCommander::_check_vehicle_land_detected()
{
	if (_vehicle_land_detected_sub == -1) {
		_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	}

	bool updated;
	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);

		/* Don't check a timeout for this topic, just accept whatever we get. */
		if (_vehicle_land_detected.landed) {
			_failsafe_sm_inputs.landed = true;
			_arming_sm.input(_arming_sm_inputs);

		} else {
			_failsafe_sm_inputs.landed = false;
			_arming_sm.input(_arming_sm_inputs);
		}
	}
}

void MiniCommander::_set_home_position()
{
	/* If there is no global position yet, give up. */
	if (_vehicle_global_position.timestamp == 0) {
		return;
	}

	/* If there is no local position yet, give up as well. */
	if (_vehicle_local_position.timestamp == 0 ||
	    !_vehicle_local_position.xy_valid ||
	    !_vehicle_local_position.z_valid) {
		return;
	}

	/* We need to be landed, to set home. */
	if (!_vehicle_land_detected.landed) {
		return;
	}

	/* Ensure that the GPS accuracy is good enough for intializing home */
	if (_vehicle_global_position.eph > _eph_threshold ||
	    _vehicle_global_position.epv > _epv_threshold) {
		return;
	}

	_home_position.timestamp = hrt_absolute_time();
	_home_position.lat = _vehicle_global_position.lat;
	_home_position.lon = _vehicle_global_position.lon;
	_home_position.alt = _vehicle_global_position.alt;

	_home_position.x = _vehicle_local_position.x;
	_home_position.y = _vehicle_local_position.y;
	_home_position.z = _vehicle_local_position.z;

	// TODO: check if this var is correct. */
	_home_position.yaw = _vehicle_local_position.yaw;

	if (!_home_position_set) {
		PX4_INFO("Home position set");
		_home_position_set = true;
	}

	_publish_home_position();
}

void MiniCommander::_publish_home_position()
{
	/* Check if it's ready to be published. */
	if (!_home_position_set) {
		return;
	}

	/* Announce new home position. */
	if (_home_position_pub != nullptr) {
		orb_publish(ORB_ID(home_position), _home_position_pub, &_home_position);

	} else {
		_home_position_pub = orb_advertise(ORB_ID(home_position), &_home_position);
	}
}

void MiniCommander::_publish_vehicle_control_mode()
{
	vehicle_control_mode_s vehicle_control_mode;

	vehicle_control_mode.timestamp = hrt_absolute_time();
	vehicle_control_mode.flag_armed = _arming_sm.is_armed();
	vehicle_control_mode.flag_external_manual_override_ok = false; // not applicable
	vehicle_control_mode.flag_system_hil_enabled = false; // not applicable

	/* All these are common for all cases. */
	vehicle_control_mode.flag_control_manual_enabled = false;
	vehicle_control_mode.flag_control_rattitude_enabled = false;
	vehicle_control_mode.flag_control_force_enabled = false; // not used in commander
	vehicle_control_mode.flag_control_termination_enabled = false; // not used in commander

	switch (_failsafe_sm.get_nav_state()) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:

		/* We use MANUAL as disabled, so everything off. */
		vehicle_control_mode.flag_control_auto_enabled = false;
		vehicle_control_mode.flag_control_offboard_enabled = false;
		vehicle_control_mode.flag_control_rates_enabled = false;
		vehicle_control_mode.flag_control_attitude_enabled = false;
		vehicle_control_mode.flag_control_acceleration_enabled = false;
		vehicle_control_mode.flag_control_velocity_enabled = false;
		vehicle_control_mode.flag_control_position_enabled = false;
		vehicle_control_mode.flag_control_altitude_enabled = false;
		vehicle_control_mode.flag_control_climb_rate_enabled = false;

		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:

		/*
		 * The control flags depend on what is ignored according to the offboard control mode topic
		 * Inner loop flags (e.g. attitude) also depend on outer loop ignore flags (e.g. position)
		 *
		 * This section is mostly copied from commander.cpp.
		 */

		vehicle_control_mode.flag_control_rates_enabled =
			!_offboard_control_mode.ignore_bodyrate ||
			!_offboard_control_mode.ignore_attitude ||
			!_offboard_control_mode.ignore_position ||
			!_offboard_control_mode.ignore_velocity ||
			!_offboard_control_mode.ignore_acceleration_force;

		vehicle_control_mode.flag_control_attitude_enabled =
			!_offboard_control_mode.ignore_attitude ||
			!_offboard_control_mode.ignore_position ||
			!_offboard_control_mode.ignore_velocity ||
			!_offboard_control_mode.ignore_acceleration_force;

		vehicle_control_mode.flag_control_acceleration_enabled =
			!_offboard_control_mode.ignore_acceleration_force;

		vehicle_control_mode.flag_control_velocity_enabled =
			(!_offboard_control_mode.ignore_velocity ||
			 !_offboard_control_mode.ignore_position) &&
			!vehicle_control_mode.flag_control_acceleration_enabled;

		vehicle_control_mode.flag_control_climb_rate_enabled =
			(!_offboard_control_mode.ignore_velocity ||
			 !_offboard_control_mode.ignore_position) &&
			!vehicle_control_mode.flag_control_acceleration_enabled;

		vehicle_control_mode.flag_control_position_enabled =
			!_offboard_control_mode.ignore_position &&
			!vehicle_control_mode.flag_control_acceleration_enabled;

		vehicle_control_mode.flag_control_altitude_enabled =
			(!_offboard_control_mode.ignore_velocity ||
			 !_offboard_control_mode.ignore_position) &&
			!vehicle_control_mode.flag_control_acceleration_enabled;

		vehicle_control_mode.flag_control_offboard_enabled = true;
		vehicle_control_mode.flag_control_auto_enabled = false;

		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:

	/* Fall through */
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:

		vehicle_control_mode.flag_control_auto_enabled = true;
		vehicle_control_mode.flag_control_offboard_enabled = false;
		vehicle_control_mode.flag_control_rates_enabled = true;
		vehicle_control_mode.flag_control_attitude_enabled = true;
		vehicle_control_mode.flag_control_altitude_enabled = true;
		vehicle_control_mode.flag_control_climb_rate_enabled = true;
		vehicle_control_mode.flag_control_position_enabled = true;
		vehicle_control_mode.flag_control_velocity_enabled = true;
		vehicle_control_mode.flag_control_acceleration_enabled = false;

		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:

		vehicle_control_mode.flag_control_auto_enabled = true;
		vehicle_control_mode.flag_control_offboard_enabled = false;
		vehicle_control_mode.flag_control_rates_enabled = true;
		vehicle_control_mode.flag_control_attitude_enabled = true;
		vehicle_control_mode.flag_control_position_enabled = false;
		vehicle_control_mode.flag_control_velocity_enabled = false;
		vehicle_control_mode.flag_control_acceleration_enabled = false;
		vehicle_control_mode.flag_control_altitude_enabled = false;
		vehicle_control_mode.flag_control_climb_rate_enabled = true;

		break;

	default:
		PX4_ERR("unknown mode");
		break;
	}

	if (_vehicle_control_mode_pub != nullptr) {

		orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &vehicle_control_mode);

	} else {
		_vehicle_control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &vehicle_control_mode);
	}
}

void MiniCommander::_publish_vehicle_status()
{
	vehicle_status_s vehicle_status;

	vehicle_status.timestamp = hrt_absolute_time();
	vehicle_status.nav_state = _failsafe_sm.get_nav_state();
	vehicle_status.arming_state = _arming_sm.is_armed() ?
				      vehicle_status_s::ARMING_STATE_ARMED :
				      vehicle_status_s::ARMING_STATE_STANDBY;
	vehicle_status.hil_state = vehicle_status_s::HIL_STATE_OFF;
	vehicle_status.failsafe = _failsafe_sm.is_in_failsafe();

	// TODO: Some stuff is just hardcoded here.
	vehicle_status.system_type = 2; // quadrotor
	vehicle_status.component_id = 1; // as the default
	vehicle_status.is_rotary_wing = true;
	vehicle_status.is_vtol = false; // not a hybrid
	vehicle_status.vtol_fw_permanent_stab = false; // not applicable
	vehicle_status.in_transition_mode = false; // not applicable
	vehicle_status.in_transition_mode = false; // not applicable
	vehicle_status.rc_signal_lost = false; // not applicable
	vehicle_status.rc_input_mode = 1; // RC input disabled
	vehicle_status.data_link_lost = false; // not applicable
	vehicle_status.data_link_lost_counter = 0; // not applicable
	vehicle_status.engine_failure = false; // not applicable
	vehicle_status.engine_failure_cmd = false; // not applicable
	vehicle_status.mission_failure = false; // not applicable
	vehicle_status.onboard_control_sensors_present = 0; // unused
	vehicle_status.onboard_control_sensors_enabled = 0; // unused
	vehicle_status.onboard_control_sensors_health = 0; // unused

	if (_vehicle_status_pub != nullptr) {

		orb_publish(ORB_ID(vehicle_status), _vehicle_status_pub, &vehicle_status);

	} else {
		_vehicle_status_pub = orb_advertise(ORB_ID(vehicle_status), &vehicle_status);
	}
}

void MiniCommander::_publish_actuator_armed()
{
	actuator_armed_s actuator_armed;
	actuator_armed.timestamp = hrt_absolute_time();
	actuator_armed.armed = _arming_sm.is_armed();
	actuator_armed.prearmed = true;
	actuator_armed.ready_to_arm = _arming_sm.is_ready_to_arm();
	actuator_armed.lockdown = false;
	actuator_armed.force_failsafe = false;
	actuator_armed.in_esc_calibration_mode = false;

	if (_actuator_armed_pub != nullptr) {

		orb_publish(ORB_ID(actuator_armed), _actuator_armed_pub, &actuator_armed);

	} else {
		_actuator_armed_pub = orb_advertise(ORB_ID(actuator_armed), &actuator_armed);
	}
}
