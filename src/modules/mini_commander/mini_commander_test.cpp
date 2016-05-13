#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <modules/uORB/topics/vehicle_status.h>
#include <modules/mini_commander/failsafe_state_machine.h>

#include "gtest/gtest.h"


/*
 * In order to run the tests quicker, we mock the time function.
 */
static uint64_t absolute_time_us = 1234567890;

uint64_t hrt_absolute_time_mock()
{
	return absolute_time_us;
}

/*
 * During waiting, we need to continuously spin the state machine to check
 * timeouts.
 */
void spin_for(uint64_t time_us, uint64_t interval_us, FailsafeStateMachine &fsm)
{
	for (uint64_t i = 0; i < time_us; i += interval_us) {
		fsm.spin();
		absolute_time_us += interval_us;
	}
}

/*
 * Try to go as exhaustive as possible through all the states and transitions.
 */
TEST(MiniCommanderTest, FailsafeStateMachine)
{
	FailsafeStateMachine fsm;
	/* We should start in MANUAL which corresponds to Disabled. */
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* Once offboard is ok, we can go into offboard. */
	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* GPS lost in offboard shouldn't change anything because that's
	 * supposed to be handled in the offboard controller. */
	fsm.gps_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* We took off in offboard. */
	fsm.in_air();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* However, once offboard is lost (and without GPS), we have to descend. */
	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Offboard is good again, do whatever offboard wants. */
	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Oh, offboard lost again, once again to descend. */
	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Yes GPS is still lost, no change. */
	fsm.gps_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Ok offboard back once again. */
	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Now GPS is recovered in offboard */
	fsm.gps_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Offboard lost should now wait, and not descend. */
	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* We lost GPS anyway, let's descend */
	fsm.gps_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* GPS back, wait again. */
	fsm.gps_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* Let's let the timer expire and go into RTL. */
	spin_for(60000000, 10000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	/* GPS lost again, to descend. */
	fsm.gps_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Oh, GPS is back, let's wait once again. */
	fsm.gps_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* Timeout to RTL again */
	spin_for(60000000, 10000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	/* We land successfully, go back to Disabled. */
	fsm.landed();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* In air doesn't confuse use, we're still disabled. */
	fsm.in_air();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);
}

/*
 * Specific tests for the timeout behaviour of the state machine.
 */
TEST(MiniCommanderTest, FailsafeStateMachineWaitTimeout)
{
	FailsafeStateMachine fsm;

	/* Start in disabled, as usual. */
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* We happen to have GPS today. */
	fsm.gps_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* Go to offboard, all normal. */
	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Taking off. */
	fsm.in_air();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Once offboard is lost, we should end up in loiter. */
	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* After 2 seconds of spinning at 10 Hz, we should still be in loiter. */
	spin_for(2000000, 100000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* However, after 6s ( > 5s), we should end up in RTL. */
	spin_for(4000000, 100000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	/* Try if we can go back to offboard. */
	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Try one more time to check if the timer resets. */
	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* After 2 seconds of spinning at 10 Hz, we should still be in loiter. */
	spin_for(2000000, 100000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* However, after 6s ( > 5s), we should end up in RTL. */
	spin_for(4000000, 100000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
}

