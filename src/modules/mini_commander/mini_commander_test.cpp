#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <modules/uORB/topics/vehicle_status.h>
#include <modules/mini_commander/failsafe_state_machine.h>
#include <modules/mini_commander/arming_state_machine.h>

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
template <class StateMachine>
void spin_for(uint64_t time_us, uint64_t interval_us, StateMachine &sm)
{
	for (uint64_t i = 0; i < time_us; i += interval_us) {
		sm.spin();
		absolute_time_us += interval_us;
	}
}

/*
 * Try to go as exhaustive as possible through all the states and transitions.
 */
TEST(MiniCommanderTest, FailsafeStateMachine)
{
	FailsafeStateMachine sm;
	FailsafeStateMachine::InputFields inputs;

	inputs.landed = true;
	inputs.offboard_ok = false;
	inputs.gps_ok = false;

	/* We should start in MANUAL which corresponds to Disabled. */
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* Once offboard is ok, we can go into offboard. */
	inputs.offboard_ok = true;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* GPS lost in offboard shouldn't change anything because that's
	 * supposed to be handled in the offboard controller. */
	inputs.gps_ok = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* We took off in offboard. */
	inputs.landed = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* However, once offboard is lost (and without GPS), we have to descend. */
	inputs.offboard_ok = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Offboard is good again, do whatever offboard wants. */
	inputs.offboard_ok = true;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Oh, offboard lost again, once again to descend. */
	inputs.offboard_ok = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Yes GPS is still lost, no change. */
	inputs.gps_ok = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Ok offboard back once again. */
	inputs.offboard_ok = true;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Now GPS is recovered in offboard */
	inputs.gps_ok = true;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Offboard lost should now wait, and not descend. */
	inputs.offboard_ok = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* We lost GPS anyway, let's descend */
	inputs.gps_ok = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* GPS back, wait again. */
	inputs.gps_ok = true;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* Let's let the timer expire and go into RTL. */
	spin_for(60000000, 10000, sm);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	/* GPS lost again, to descend. */
	inputs.gps_ok = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Oh, GPS is back, let's wait once again. */
	inputs.gps_ok = true;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* Timeout to RTL again */
	spin_for(60000000, 10000, sm);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	/* We land successfully, go back to Disabled. */
	inputs.landed = true;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* In air doesn't confuse use, we're still disabled. */
	inputs.landed = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);
}

/*
 * Specific tests for the timeout behaviour of the state machine.
 */
TEST(MiniCommanderTest, FailsafeStateMachineWaitTimeout)
{
	FailsafeStateMachine sm;
	FailsafeStateMachine::InputFields inputs;

	inputs.landed = true;
	inputs.offboard_ok = false;
	inputs.gps_ok = false;

	/* Start in disabled, as usual. */
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* We happen to have GPS today. */
	inputs.gps_ok = true;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* Go to offboard, all normal. */
	inputs.offboard_ok = true;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Taking off. */
	inputs.landed = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Once offboard is lost, we should end up in loiter. */
	inputs.offboard_ok = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* After 2 seconds of spinning at 10 Hz, we should still be in loiter. */
	spin_for(2000000, 100000, sm);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* However, after 6s ( > 5s), we should end up in RTL. */
	spin_for(4000000, 100000, sm);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	/* Try if we can go back to offboard. */
	inputs.offboard_ok = true;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Try one more time to check if the timer resets. */
	inputs.offboard_ok = false;
	sm.input(inputs);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* After 2 seconds of spinning at 10 Hz, we should still be in loiter. */
	spin_for(2000000, 100000, sm);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* However, after 6s ( > 5s), we should end up in RTL. */
	spin_for(4000000, 100000, sm);
	ASSERT(sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
}

/*
 * Tests for the arming state machine.
 */
TEST(MiniCommanderTest, ArmingStateMachine)
{
	ArmingStateMachine sm;
	ArmingStateMachine::InputFields inputs;

	inputs.landed = false;
	inputs.home_position_set = false;

	/* We must be disarmed on startup! */
	ASSERT_FALSE(sm.is_armed());

	/* Don't allow arming without home position. */
	sm.request_arm();
	ASSERT_FALSE(sm.is_armed());

	/* Now a home position is supplied. */
	inputs.home_position_set = true;
	sm.input(inputs);
	ASSERT_FALSE(sm.is_armed());

	/* Request again. */
	sm.request_arm();
	ASSERT_TRUE(sm.is_armed());

	/* Being landed after arming doesn't mean we disarm straightaway. */
	inputs.landed = true;
	sm.input(inputs);
	ASSERT_TRUE(sm.is_armed());

	/* After a timeout of 1s we're still armed. */
	spin_for(1000000, 100000, sm);
	ASSERT_TRUE(sm.is_armed());

	/* But after 6s, we're disarmed again. */
	spin_for(5000000, 100000, sm);
	ASSERT_FALSE(sm.is_armed());

	/* Let's try to arm again, again we need home first. */
	sm.request_arm();
	ASSERT_FALSE(sm.is_armed());

	/* Ok, give it home. */
	inputs.home_position_set = true;
	sm.input(inputs);
	ASSERT_FALSE(sm.is_armed());
	sm.request_arm();
	ASSERT_TRUE(sm.is_armed());

	/* Now let's take off. */
	inputs.landed = false;
	sm.input(inputs);
	ASSERT_TRUE(sm.is_armed());

	/* Throw random request at it. */
	sm.request_arm();
	ASSERT_TRUE(sm.is_armed());

	inputs.home_position_set = true;
	sm.input(inputs);
	ASSERT_TRUE(sm.is_armed());

	inputs.landed = false;
	sm.input(inputs);
	ASSERT_TRUE(sm.is_armed());

	/* Land again, we should disarm. */
	inputs.landed = true;
	sm.input(inputs);
	ASSERT_FALSE(sm.is_armed());

	/* In air once again doesn't bother us. */
	inputs.landed = false;
	sm.input(inputs);
	ASSERT_FALSE(sm.is_armed());
}
