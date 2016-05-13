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
 * @file arming_state_machine.h
 *
 * Simple state machine for the mini commander
 * handling if the vehicle is armed.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <px4_defines.h>
#include <systemlib/fsm.h>
#include <drivers/drv_hrt.h>

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


class ArmingStateMachine
{
public:
	ArmingStateMachine();

	~ArmingStateMachine();

public:
	void landed()
	{
		state->landed();
	}

	void in_air()
	{
		state->in_air();
	}

	void home_position_set()
	{
		state->home_position_set();
	}

	void arming_requested()
	{
		state->arming_requested();
	}

	/*
	 * Check for timeouts happening inside the state machine.
	 */
	void spin()
	{
		state->spin();
	}

	/*
	 * Getter to the outside world.
	 *
	 * return true if armed
	 */
	bool is_armed()
	{
		return state->is_armed();
	}

	/*
	 * Getter about the "ready to arm" state.
	 *
	 * return true if ready to armed
	 */
	bool is_ready_to_arm()
	{
		return state->is_ready_to_arm();
	}


private:
	static void unhandled_event()
	{
	}

	class State : public fsm::GenericState<ArmingStateMachine, State>
	{
	public:
		using GenericState::GenericState;

		virtual bool is_armed() = 0;
		virtual bool is_ready_to_arm() = 0;

		virtual void landed() { unhandled_event(); }
		virtual void in_air() { unhandled_event(); }
		virtual void home_position_set() { unhandled_event(); }
		virtual void arming_requested() { unhandled_event(); }
	};
	fsm::StateRef<State> state;

	class ArmedInAir : public State
	{
	public:
		using State::State;

		void landed()
		{
			PX4_INFO("we get the landed signal");
			change<DisarmedNotReady>();
		}

		bool is_armed()
		{
			return true;
		}

		bool is_ready_to_arm()
		{
			return true;
		}
	};

	class ArmedOnGround : public State
	{
	public:
		using State::State;

		void entry()
		{
			_time_on_ground_started_us = HRT_ABSOLUTE_TIME();
		}

		void exit()
		{
			_time_on_ground_started_us = 0;
		}

		void spin()
		{
			if ((HRT_ABSOLUTE_TIME() - _time_on_ground_started_us) > _timeout) {
				change<DisarmedNotReady>();
			}
		}

		bool is_armed()
		{
			return true;
		}

		bool is_ready_to_arm()
		{
			return true;
		}

		void in_air()
		{
			change<ArmedInAir>();
		}

	private:
		const uint64_t _timeout = 5000000; // Wait for 5s before disarming again.
		uint64_t _time_on_ground_started_us = 0;
	};

	class DisarmedNotReady : public State
	{
	public:
		using State::State;

		bool is_armed()
		{
			return false;
		}

		bool is_ready_to_arm()
		{
			return false;
		}

		void home_position_set()
		{
			change<DisarmedReady>();
		}
	};

	class DisarmedReady : public State
	{
	public:
		using State::State;

		bool is_armed()
		{
			return false;
		}

		bool is_ready_to_arm()
		{
			return true;
		}

		void arming_requested()
		{
			change<ArmedOnGround>();
		}
	};
};
