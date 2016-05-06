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
 * @file mini_commander_main.cpp
 *
 * Lean mini version of the big brother commander, main function to start/stop
 * the module.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include "mini_commander.h"


/*
 * Main hook for mini_commander.
 */
extern "C" __EXPORT int mini_commander_main(int argc, char *argv[]);

namespace mini_commander {

/*
 * Print the correct usage.
 */
void usage(const char *reason);

/*
 * Helper function to start the main task.
 */
static void task_main_trampoline();

static int _task = -1;
static MiniCommander *_instance = nullptr;

void usage(const char *reason)
{
	if (reason && *reason > 0) {
		PX4_WARN("%s", reason);
	}

	PX4_INFO("usage: mini_commander {start|stop|status}");
}

void task_main_trampoline()
{
	mini_commander::_instance->task_main();
}

} // namespace mini_commander


int mini_commander_main(int argc, char *argv[])
{
	if (argc < 2) {
		mini_commander::usage("missing command");
		return -1;
	}

	if (!strcmp(argv[1], "start")) {

		if (mini_commander::_instance) {
			PX4_WARN("already running");
			/* this is not an error */
			return 0;
		}

		mini_commander::_instance = new MiniCommander();

		if (!mini_commander::_instance) {
			PX4_ERR("alloc failed");
			return -1;
		}

		/* start the task */
		mini_commander::_task = px4_task_spawn_cmd("mini_commander",
							   SCHED_DEFAULT,
							   SCHED_PRIORITY_MAX - 5,
							   1500,
							   (px4_main_t)&mini_commander::task_main_trampoline,
							   nullptr);

		if (mini_commander::_task < 0) {
			PX4_ERR("task start failed");
			return -errno;
		}
		return 0;
	}

	/* commands needing the app to run below */
	if (!mini_commander::_instance || !mini_commander::_instance->is_running()) {
		PX4_WARN("mini_commander not started");
		return -1;
	}

	if (!strcmp(argv[1], "stop")) {

		unsigned i = 0;
		if (mini_commander::_task != -1) {
			do {
				/* wait 20ms */
				usleep(20000);

				/* if we have given up, kill it */
				if (++i > 50) {
					px4_task_delete(mini_commander::_task);
					break;
				}
			} while (mini_commander::_instance->is_running());
			mini_commander::_task = -1;

			PX4_INFO("terminated.");
		}

		delete mini_commander::_instance;
		mini_commander::_instance = nullptr;
		return 0;
	}


	if (!strcmp(argv[1], "status")) {
		mini_commander::_instance->print_status();
		return 0;
	}

	mini_commander::usage("unrecognized command");
	return -1;
}

