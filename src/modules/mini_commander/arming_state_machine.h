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
	struct InputFields {
		bool landed;
		bool home_position_set;
	};

	/*
	 * Provide inputs to the state machine.
	 */
	void input(const InputFields &fields)
	{
		state->input(fields);
	}

	/*
	 * One time event to try to arm.
	 * */
	void request_arm()
	{
		state->request_arm();
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
	 * return true if ready to be armed
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

		virtual void input(const InputFields &fields) = 0;
		virtual void request_arm() { unhandled_event(); }
	};
	fsm::StateRef<State> state;

	class ArmedInAir : public State
	{
	public:
		using State::State;

		void input(const InputFields &fields)
		{
			if (fields.landed) {
				/* Once landed, we disarm immediately. */
				change<DisarmedNotReady>();
			}
		}

		bool is_armed()
		{
			return true;
		}

		bool is_ready_to_arm()
		{
			return false;
		}
	};

	class ArmedOnGround : public State
	{
	public:
		using State::State;

		void input(const InputFields &fields)
		{
			if (!fields.landed) {
				/* Once in-air, go to in-air starte. */
				change<ArmedInAir>();
			}
		}

		void entry()
		{
			/* Start the timer. */
			_time_on_ground_started_us = HRT_ABSOLUTE_TIME();
		}

		void exit()
		{
			/* Reset the timer. */
			_time_on_ground_started_us = 0;
		}

		void spin()
		{
			/* After a while not taking off, go back to disarmed. */
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
			return false;
		}

	private:
		const uint64_t _timeout = 5000000; // Wait for 5s before disarming again.
		uint64_t _time_on_ground_started_us = 0;
	};

	class DisarmedNotReady : public State
	{
	public:
		using State::State;

		void input(const InputFields &fields)
		{
			if (fields.home_position_set) {
				/* If we have a home set, we are ready be armed. */
				change<DisarmedReady>();
			}
		}

		bool is_armed()
		{
			return false;
		}

		bool is_ready_to_arm()
		{
			return false;
		}
	};

	class DisarmedReady : public State
	{
	public:
		using State::State;

		void input(const InputFields &fields)
		{
		}

		bool is_armed()
		{
			return false;
		}

		bool is_ready_to_arm()
		{
			return true;
		}

		void request_arm()
		{
			/* If arming is requested, we are ready. */
			change<ArmedOnGround>();
		}
	};
};
