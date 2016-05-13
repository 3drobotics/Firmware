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
 * @file mini_commander.h
 *
 * Lean mini version of the big brother commander.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <px4_posix.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/home_position.h>

#include "mini_commander_fsm.h"



class MiniCommander
{
public:
	MiniCommander();
	~MiniCommander();

	bool is_running() { return _task_is_running; }

	/* Print some info. */
	void print_status();

	void task_main();
private:
	void _check_topics();
	void _check_battery_status();
	void _check_offboard_control_mode();
	void _check_vehicle_global_position();
	void _check_vehicle_local_position();
	void _check_vehicle_land_detected();

	/*
	 * Try to set the home position.
	 */
	void _set_home_position();

	void _publish_topics();
	void _publish_home_position();
	void _publish_vehicle_control_mode();
	void _publish_vehicle_status();
	void _publish_actuator_armed();


	bool _task_is_running;
	bool _task_should_exit;
	static constexpr unsigned _approx_interval_us = 100000;

	static constexpr unsigned _offboard_timeout = 500000;

	/* TODO: make these a parameter. */
	static constexpr float _eph_threshold = 5.0f;
	static constexpr float _epv_threshold = 10.0f;

	int _battery_status_sub;
	int _offboard_control_mode_sub;
	int _vehicle_global_position_sub;
	int _vehicle_local_position_sub;
	int _vehicle_land_detected_sub;

	orb_advert_t _home_position_pub;
	orb_advert_t _vehicle_control_mode_pub;
	orb_advert_t _vehicle_status_pub;
	orb_advert_t _actuator_armed_pub;

	offboard_control_mode_s _offboard_control_mode;
	vehicle_global_position_s _vehicle_global_position;
	vehicle_local_position_s _vehicle_local_position;
	vehicle_land_detected_s _vehicle_land_detected;

	home_position_s _home_position;
	bool _home_position_set;

	actuator_armed_s _actuator_armed;

	MiniCommanderFsm _fsm;
};
