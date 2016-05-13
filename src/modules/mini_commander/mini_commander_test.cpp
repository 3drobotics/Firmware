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
	FailsafeStateMachine failsafe_sm;
	/* We should start in MANUAL which corresponds to Disabled. */
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* Once offboard is ok, we can go into offboard. */
	failsafe_sm.offboard_ok();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* GPS lost in offboard shouldn't change anything because that's
	 * supposed to be handled in the offboard controller. */
	failsafe_sm.gps_lost();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* We took off in offboard. */
	failsafe_sm.in_air();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* However, once offboard is lost (and without GPS), we have to descend. */
	failsafe_sm.offboard_lost();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Offboard is good again, do whatever offboard wants. */
	failsafe_sm.offboard_ok();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Oh, offboard lost again, once again to descend. */
	failsafe_sm.offboard_lost();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Yes GPS is still lost, no change. */
	failsafe_sm.gps_lost();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Ok offboard back once again. */
	failsafe_sm.offboard_ok();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Now GPS is recovered in offboard */
	failsafe_sm.gps_ok();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Offboard lost should now wait, and not descend. */
	failsafe_sm.offboard_lost();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* We lost GPS anyway, let's descend */
	failsafe_sm.gps_lost();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* GPS back, wait again. */
	failsafe_sm.gps_ok();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* Let's let the timer expire and go into RTL. */
	spin_for(60000000, 10000, failsafe_sm);
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	/* GPS lost again, to descend. */
	failsafe_sm.gps_lost();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	/* Oh, GPS is back, let's wait once again. */
	failsafe_sm.gps_ok();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* Timeout to RTL again */
	spin_for(60000000, 10000, failsafe_sm);
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	/* We land successfully, go back to Disabled. */
	failsafe_sm.landed();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* In air doesn't confuse use, we're still disabled. */
	failsafe_sm.in_air();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);
}

/*
 * Specific tests for the timeout behaviour of the state machine.
 */
TEST(MiniCommanderTest, FailsafeStateMachineWaitTimeout)
{
	FailsafeStateMachine failsafe_sm;

	/* Start in disabled, as usual. */
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* We happen to have GPS today. */
	failsafe_sm.gps_ok();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	/* Go to offboard, all normal. */
	failsafe_sm.offboard_ok();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Taking off. */
	failsafe_sm.in_air();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Once offboard is lost, we should end up in loiter. */
	failsafe_sm.offboard_lost();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* After 2 seconds of spinning at 10 Hz, we should still be in loiter. */
	spin_for(2000000, 100000, failsafe_sm);
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* However, after 6s ( > 5s), we should end up in RTL. */
	spin_for(4000000, 100000, failsafe_sm);
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	/* Try if we can go back to offboard. */
	failsafe_sm.offboard_ok();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	/* Try one more time to check if the timer resets. */
	failsafe_sm.offboard_lost();
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* After 2 seconds of spinning at 10 Hz, we should still be in loiter. */
	spin_for(2000000, 100000, failsafe_sm);
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	/* However, after 6s ( > 5s), we should end up in RTL. */
	spin_for(4000000, 100000, failsafe_sm);
	ASSERT(failsafe_sm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
}

/*
 * Tests for the arming state machine.
 */
TEST(MiniCommanderTest, ArmingStateMachine)
{
	ArmingStateMachine arming_sm;

	/* We must be disarmed on startup! */
	ASSERT_FALSE(arming_sm.is_armed());

	/* Don't allow arming without home position. */
	arming_sm.arming_requested();
	ASSERT_FALSE(arming_sm.is_armed());

	/* Now a home position is supplied. */
	arming_sm.home_position_set();
	ASSERT_FALSE(arming_sm.is_armed());

	/* Request again. */
	arming_sm.arming_requested();
	ASSERT_TRUE(arming_sm.is_armed());

	/* Being landed after arming doesn't mean we disarm straightaway. */
	arming_sm.landed();
	ASSERT_TRUE(arming_sm.is_armed());

	/* After a timeout of 1s we're still armed. */
	spin_for(1000000, 100000, arming_sm);
	ASSERT_TRUE(arming_sm.is_armed());

	/* But after 6s, we're disarmed again. */
	spin_for(5000000, 100000, arming_sm);
	ASSERT_FALSE(arming_sm.is_armed());

	/* Let's try to arm again, again we need home first. */
	arming_sm.arming_requested();
	ASSERT_FALSE(arming_sm.is_armed());

	/* Ok, give it home. */
	arming_sm.home_position_set();
	ASSERT_FALSE(arming_sm.is_armed());
	arming_sm.arming_requested();
	ASSERT_TRUE(arming_sm.is_armed());

	/* Now let's take off. */
	arming_sm.in_air();
	ASSERT_TRUE(arming_sm.is_armed());

	/* Throw random request at it. */
	arming_sm.arming_requested();
	ASSERT_TRUE(arming_sm.is_armed());

	arming_sm.home_position_set();
	ASSERT_TRUE(arming_sm.is_armed());

	arming_sm.in_air();
	ASSERT_TRUE(arming_sm.is_armed());

	/* Land again, we should disarm. */
	arming_sm.landed();
	ASSERT_FALSE(arming_sm.is_armed());

	/* In air once again doesn't bother us. */
	arming_sm.in_air();
	ASSERT_FALSE(arming_sm.is_armed());
}
