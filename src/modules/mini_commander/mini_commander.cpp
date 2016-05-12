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
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>

#include "mini_commander.h"

MiniCommander::MiniCommander() :
	_task_is_running(false),
	_task_should_exit(false),
	_battery_status_sub(-1),
	_offboard_control_mode_sub(-1),
	_vehicle_global_position_sub(-1),
	_vehicle_attitude_sub(-1),
	_vehicle_land_detected_sub(-1),
	_fsm()
{}

MiniCommander::~MiniCommander()
{}

void
MiniCommander::task_main()
{
	_task_is_running = true;
	while (!_task_should_exit) {

		_check_topics();

		_fsm.spin();

		_publish_topics();
		usleep(_approx_interval_us);
	}
	_task_is_running = false;
}

void
MiniCommander::print_status()
{
	PX4_INFO("current navigation state: %d", _fsm.get_nav_state());
}

void
MiniCommander::_check_topics()
{
	_check_battery_status();
	_check_offboard_control_mode();
	_check_vehicle_global_position();
	_check_vehicle_attitude();
	_check_vehicle_land_detected();
}

void
MiniCommander::_publish_topics()
{
	_publish_home_position();
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
		offboard_control_mode_s offboard_control_mode;
		orb_copy(ORB_ID(offboard_control_mode), _offboard_control_mode_sub, &offboard_control_mode);

		// TODO: do something useful
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
		vehicle_global_position_s vehicle_global_position;
		orb_copy(ORB_ID(vehicle_global_position), _vehicle_global_position_sub, &vehicle_global_position);

		// TODO: do something useful
	}
}

void
MiniCommander::_check_vehicle_attitude()
{
	if (_vehicle_attitude_sub == -1) {
		_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	}

	bool updated;
	orb_check(_vehicle_attitude_sub, &updated);

	if (updated) {
		vehicle_attitude_s vehicle_attitude;
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &vehicle_attitude);

		// TODO: do something useful
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
		vehicle_land_detected_s vehicle_land_detected;
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &vehicle_land_detected);

		// TODO: do something useful
	}
}

void MiniCommander::_publish_home_position()
{
	// TODO: actually do this
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
	// TODO: actually do this
}
