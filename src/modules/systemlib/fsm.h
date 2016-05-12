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
 * @file fsm.h
 *
 * Simple finite state machine borrowed from:
 * http://codereview.stackexchange.com/questions/40686/state-pattern-c-template
 * (licensed as public domain).
 *
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <memory>

namespace fsm {

template <class State>
using StateRef = std::unique_ptr<State>;

template <typename StateMachine, class State>
class GenericState
{
public:
	GenericState(StateMachine &machine, StateRef<State> &state) :
		_machine(machine),
		_state(state)
	{
	}

	virtual ~GenericState()
	{
	}

	template <class ConcreteState>
	static void init(StateMachine &machine, StateRef<State> &state)
	{
		state = StateRef<State>(new ConcreteState(machine, state));
		state->entry();
	}

	/*
	 * Run the current state once
	 *
	 * This can be used to implement e.g. a timeout mechanism: Every time
	 * this function is called, it can check if it has reached the timeout
	 * and react to it.
	 */
	virtual void spin()
	{
	}

protected:
	template <class ConcreteState>
	void change()
	{
		exit();
		init<ConcreteState>(_machine, _state);
	}

	StateMachine &_machine;

private:
	virtual void entry()
	{
	}

	virtual void exit()
	{
	}

	StateRef<State> &_state;
};

} // namespace fsm
