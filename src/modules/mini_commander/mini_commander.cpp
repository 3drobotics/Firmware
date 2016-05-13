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
	_arming_sm()
{
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
		}

		_failsafe_sm.spin();
		_arming_sm.spin();

		_publish_topics();
		usleep(_approx_interval_us);
	}

	_task_is_running = false;
}

void
MiniCommander::print_status()
{
	PX4_INFO("current navigation state: %d", _failsafe_sm.get_nav_state());
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
	    hrt_elapsed_time(&_offboard_control_mode.timestamp) < _offboard_timeout) {
		_failsafe_sm.offboard_ok();

	} else {
		_failsafe_sm.offboard_lost();
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
			_failsafe_sm.landed();

		} else {
			_failsafe_sm.in_air();
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

	_home_position_set = true;
	// TODO: add an armed state machine for this.
	//_failsafe_sm.home_position_set();

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
	// TODO: actually do this
}

void MiniCommander::_publish_vehicle_status()
{
	// TODO: actually do this
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
