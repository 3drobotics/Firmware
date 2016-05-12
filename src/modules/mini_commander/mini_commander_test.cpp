#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <modules/uORB/topics/vehicle_status.h>
#include <modules/mini_commander/mini_commander_fsm.h>

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
void spin_for(uint64_t time_us, uint64_t interval_us, MiniCommanderFsm &fsm)
{
	for (uint64_t i = 0; i < time_us; i += interval_us) {
		fsm.spin();
		absolute_time_us += interval_us;
	}
}

/*
 * Try to go as exhaustive as possible through all the states and transitions.
 */
TEST(MiniCommanderTest, Fsm)
{
	MiniCommanderFsm fsm;
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	// GPS lost in offboard shouldn't change anything.
	fsm.gps_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	fsm.gps_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	// Even though we have lost GPS before, we end up in loiter until the GPS lost event is triggered again.
	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	fsm.gps_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);

	fsm.landed();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	// GPS lost in landed shouldn't change anything.
	fsm.gps_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	// Let's go to RTL and check that GPS out goes to descend.
	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);
	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
	spin_for(60000000, 10000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
	fsm.gps_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_DESCEND);
	fsm.landed();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	// Let's go to RTL again and check that it ends when landed.
	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);
	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
	spin_for(60000000, 10000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
	fsm.landed();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);
}

/*
 * Specific tests for the timeout behaviour of the state machine.
 */
TEST(MiniCommanderTest, FsmWaitTimeout)
{
	MiniCommanderFsm fsm;

	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_MANUAL);

	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	// Once offboard is lost, we should end up in loiter.
	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	// After 2 seconds of spinning at 10 Hz, we should still be in loiter.
	spin_for(2000000, 100000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	// However, after 6s ( > 5s), we should end up in RTL.
	spin_for(4000000, 100000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	// Try if we can go back to offboard.
	fsm.offboard_ok();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	// Try one more time to check if the timer resets.
	fsm.offboard_lost();
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	// After 2 seconds of spinning at 10 Hz, we should still be in loiter.
	spin_for(2000000, 100000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	// However, after 6s ( > 5s), we should end up in RTL.
	spin_for(4000000, 100000, fsm);
	ASSERT(fsm.get_nav_state() == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
}

