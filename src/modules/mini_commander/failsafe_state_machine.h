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
 * @file failsafe_state_machine.h
 *
 * Hierarchical state machine for the mini commander
 * handling failsafe when offboard control is lost.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <px4_defines.h>
#include <systemlib/fsm.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/vehicle_status.h>

/*
 * For unittests, the the call to hrt_absolute_time()
 * goes to a mocked function.
 */
#ifdef MOCK_TIME
#define HRT_ABSOLUTE_TIME hrt_absolute_time_mock
extern uint64_t hrt_absolute_time_mock();
#else
#define HRT_ABSOLUTE_TIME hrt_absolute_time
#endif


class FailsafeStateMachine
{
public:
	FailsafeStateMachine();

	~FailsafeStateMachine();

	struct InputFields {
		bool landed;
		bool offboard_ok;
		bool gps_ok;
	};

	/*
	 * Provide inputs to the state machine.
	 */
	void input(const InputFields &fields)
	{
		state->input(fields);
	}

	/*
	 * Check for timeouts happening inside the state machine.
	 */
	void spin()
	{
		state->spin();
	}

	/*
	 * Translate the internal state to a NAVIGATION_STATE.
	 */
	uint8_t get_nav_state()
	{
		return state->get_nav_state();
	}

	/*
	 * Get the string to a navigation state.
	 */
	const char *get_nav_state_str()
	{
		switch (state->get_nav_state()) {
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			return "MANUAL";

		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
			return "OFFBOARD";

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
			return "AUTO_LOITER";

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
			return "AUTO_RTL";

		case vehicle_status_s::NAVIGATION_STATE_DESCEND:
			return "AUTO_RTL";

		default:
			return "Unknown";
		}
	}

	bool is_in_failsafe()
	{
		return state->is_in_failsafe();
	}

private:
	static void unhandled_event()
	{
	}

	class State : public fsm::GenericState<FailsafeStateMachine, State>
	{
	public:
		using GenericState::GenericState;

		virtual uint8_t get_nav_state() = 0;
		virtual bool is_in_failsafe() = 0;

		virtual void input(const InputFields &fields) = 0;
	};
	fsm::StateRef<State> state;

	class Disabled : public State
	{
	public:
		using State::State;

		uint8_t get_nav_state()
		{
			return vehicle_status_s::NAVIGATION_STATE_MANUAL;
		}

		bool is_in_failsafe()
		{
			return false;
		}

		void input(const InputFields &fields)
		{
			if (fields.offboard_ok) {

				/* As soon as offboard commands are appearing it's
				 * fine to switch to Offboard. */
				change<Offboard>();
			}
		}
	};

	class Offboard : public State
	{
	public:
		using State::State;

		uint8_t get_nav_state()
		{
			return vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
		}

		bool is_in_failsafe()
		{
			return false;
		}

		void input(const InputFields &fields)
		{
			if (!fields.offboard_ok && fields.landed) {

				/* If we are not in-air, there is no point going into
				 * failsafe, just go straight to disabled. */
				change<Disabled>();

			} else if (!fields.offboard_ok && !fields.landed) {

				/* If we are in-air and lose offboard, we need to enter
				 * failsafe and do something. */
				if (fields.gps_ok) {
					change<Wait>();
				} else {
					change<Descend>();
				}
			}
		}
	};

	class Wait : public State
	{
	public:
		using State::State;

		uint8_t get_nav_state()
		{
			return vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		}

		bool is_in_failsafe()
		{
			return true;
		}

		void input(const InputFields &fields)
		{
			if (fields.offboard_ok) {
				/* Offboard has highest priority because it can take
				 * appropriate action on GPS and landed flags. */
				change<Offboard>();

			} else if (!fields.gps_ok) {
				/* We can't wait without GPS, so we'll have to descend. */
				change<Descend>();

			} else if (fields.landed) {
				/* This is unlikely but if it happens, let's trust the
				 * land_detector. */
				change<Disabled>();
			}
		}

		void entry()
		{
			/* When entering Wait, let's start a timer. */
			_time_wait_started_us = HRT_ABSOLUTE_TIME();
		}

		void exit()
		{
			/* Clean up timer. */
			_time_wait_started_us = 0;
		}

		void spin()
		{
			/* If timer is done, proceed to RTL. */
			if ((HRT_ABSOLUTE_TIME() - _time_wait_started_us) > _timeout) {
				change<Rtl>();
			}
		}

	private:
		const uint64_t _timeout = 5000000; // Wait for 5s before going to RTL.
		uint64_t _time_wait_started_us = 0;
	};

	class Rtl : public State
	{
	public:
		using State::State;

		uint8_t get_nav_state()
		{
			return vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		}

		bool is_in_failsafe()
		{
			return true;
		}

		void input(const InputFields &fields)
		{
			if (fields.offboard_ok) {
				/* Offboard has highest priority because it can take
				 * appropriate action on GPS and landed flags. */
				change<Offboard>();

			} else if (!fields.gps_ok) {
				/* Can't do RTL without GPS, so we need to just descend. */
				change<Descend>();

			} else if (fields.landed) {
				/* Once RTL is done, go back to disabled. */
				change<Disabled>();
			}
		}
	};

	class Descend : public State
	{
	public:
		using State::State;

		uint8_t get_nav_state()
		{
			return vehicle_status_s::NAVIGATION_STATE_DESCEND;
		}

		bool is_in_failsafe()
		{
			return true;
		}

		void input(const InputFields &fields)
		{
			if (fields.offboard_ok) {
				/* Offboard has highest priority because it can take
				 * appropriate action on GPS and landed flags. */
				change<Offboard>();

			} else if (fields.gps_ok) {
				/* GPS is back, let's wait. */
				change<Wait>();

			} else if (fields.landed) {
				/* We're done descending, go back to disabled. */
				change<Disabled>();
			}
		}
	};
};
