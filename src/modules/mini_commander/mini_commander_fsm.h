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
 * @file mini_commander_fsm.h
 *
 * Hierarchical state machine for the mini commander.
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


class MiniCommanderFsm
{
public:
	MiniCommanderFsm();

	~MiniCommanderFsm();

public:
	void offboard_ok()
	{
		state->offboard_ok();
	}

	void offboard_lost()
	{
		state->offboard_lost();
	}

	void gps_ok()
	{
		/* Update internal state. */
		_is_gps_ok = true;

		state->gps_ok();
	}

	void gps_lost()
	{
		/* Update internal state. */
		_is_gps_ok = false;

		state->gps_lost();
	}

	void landed()
	{
		/* Update internal state. */
		_is_landed = true;

		state->landed();
	}

	void in_air()
	{
		/* Update internal state. */
		_is_landed = false;

		state->in_air();
	}

	/*
	 * Check for timeouts happening inside the state machine.
	 */
	void spin()
	{
		state->spin();
	}

	uint8_t get_nav_state()
	{
		return state->get_nav_state();
	}

private:
	static void unhandled_event()
	{
		PX4_INFO("unhandled event");
	}

	class State : public fsm::GenericState<MiniCommanderFsm, State>
	{
	public:
		using GenericState::GenericState;

		virtual uint8_t get_nav_state() = 0;

		virtual void offboard_ok() { unhandled_event(); }
		virtual void offboard_lost() { unhandled_event(); }
		virtual void gps_ok() { unhandled_event(); }
		virtual void gps_lost() { unhandled_event(); }
		virtual void landed() { unhandled_event(); }
		virtual void in_air() { unhandled_event(); }
	};
	fsm::StateRef<State> state;

	class FailsafeState : public fsm::GenericState<MiniCommanderFsm, FailsafeState>
	{
	public:
		using GenericState::GenericState;

		virtual uint8_t get_nav_state() = 0;

		virtual void gps_ok() { unhandled_event(); }
		virtual void gps_lost() { unhandled_event(); }
	};

	class Disabled : public State
	{
	public:
		using State::State;

		uint8_t get_nav_state()
		{
			return vehicle_status_s::NAVIGATION_STATE_MANUAL;
		}

		void entry()
		{
			PX4_INFO("Not doing anything.");
		}

		void exit()
		{
			PX4_INFO("Oh something is happening.");
		}

		void offboard_ok()
		{
			change<Offboard>();
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

		void entry()
		{
			PX4_INFO("Controlled offboard.");
		}

		void exit()
		{
			PX4_INFO("No more offboard.");
		}

		void offboard_lost()
		{
			if (_machine._is_landed) {
				/* If we are not in-air, there is no point going into
				 * failsafe, just go straight to disabled. */

				change<Disabled>();

			} else {
				change<Failsafe>();
			}
		}
	};

	class Failsafe : public State
	{
	public:
		using State::State;

		uint8_t get_nav_state()
		{
			return failsafe_state->get_nav_state();
		}

		void entry()
		{
			PX4_INFO("In failsafe!");

			/* When entering failsafe, look at GPS ok flag to decide
			 * where to go. */
			if (_machine._is_gps_ok) {
				/* With GPS, we can just wait for a bit. */
				FailsafeState::init<FailsafeWait>(_machine, failsafe_state);

			} else {
				/* Without GPS, we have to descend. */
				FailsafeState::init<FailsafeDescend>(_machine, failsafe_state);
			}
		}

		void exit()
		{
			PX4_INFO("Out of failsafe.");
		}

		void spin()
		{
			failsafe_state->spin();
		}

		void offboard_ok()
		{
			change<Offboard>();
		}

		void landed()
		{
			change<Disabled>();
		}

		void gps_ok()
		{
			failsafe_state->gps_ok();
		}

		void gps_lost()
		{
			failsafe_state->gps_lost();
		}

	private:
		fsm::StateRef<FailsafeState> failsafe_state;
	};

	class FailsafeWait : public FailsafeState
	{
	public:
		using FailsafeState::FailsafeState;

		uint8_t get_nav_state()
		{
			return vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		}

		void entry()
		{
			_time_wait_started_us = HRT_ABSOLUTE_TIME();
		}

		void exit()
		{
			PX4_INFO("Done waiting.");
			_time_wait_started_us = 0;
		}

		void spin()
		{
			if ((HRT_ABSOLUTE_TIME() - _time_wait_started_us) > _timeout) {
				change<FailsafeRtl>();
			}
		}

		void gps_lost()
		{
			change<FailsafeDescend>();
		}

	private:
		const uint64_t _timeout = 5000000; // Wait for 5s before going to RTL.
		uint64_t _time_wait_started_us = 0;
	};

	class FailsafeRtl : public FailsafeState
	{
	public:
		using FailsafeState::FailsafeState;

		uint8_t get_nav_state()
		{
			return vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		}

		void entry()
		{
			PX4_INFO("Return to land!");
		}
		void exit()
		{
			PX4_INFO("Aborting return to land.");
		}

		void gps_lost()
		{
			change<FailsafeDescend>();
		}
	};

	class FailsafeDescend : public FailsafeState
	{
	public:
		using FailsafeState::FailsafeState;

		uint8_t get_nav_state()
		{
			return vehicle_status_s::NAVIGATION_STATE_DESCEND;
		}

		void entry()
		{
			PX4_INFO("Just go down.");
		}
		void exit()
		{
			PX4_INFO("No more going down?");
		}

		void gps_ok()
		{
			change<FailsafeWait>();
		}
	};

	/*
	 * The state machine needs to keep track internally of the flags about
	 * GPS and landed because it needs to evaluate forks when entering the
	 * failsafe state from offboard.
	 *
	 * This is a hacky approach but it prevents the state diagram from
	 * blowing up too much because we don't need hierarchical states for
	 * all possible combinations.
	 */
	bool _is_landed;
	bool _is_gps_ok;
};
