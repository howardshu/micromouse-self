/*
 * controller.c
 */

#include "main.h"
#include "controller.h"
#include "pid.h"

/*
 * We recommend you implement this function so that move(1) will move your rat 1 cell forward.
 */
void move(int8_t n) {
	/*
	 * For assignment 3.1: Don't worry about implementing this yet
	 * For assignment 3.2: This function should set the distance and angle goals appropriately for PID (hint: using the setGoal functions in pid.c)
	 * and wait until the error becomes sufficiently small and persistent before exiting. This function should NOT exit before then.
	 *
	 * HINT: Use a while loop to wait until the rat has moved the desired distance
	 *
	 * You should also call resetPID before exiting this function so your rat is ready for the next instruction.
	 */

	/** TODO */
	setPIDGoalA(0);
	setPIDGoalD(n); // TODO
	while (!PIDdone());
	resetPID();
}

/*
 * We recommend you implement this function so that turn(1) turns your rat 90 degrees in your positive rotation
 * direction and turn(-1) turns the other way.
 */
void turn(int8_t n) {
	/*
	 * For assignment 3.1: Don't worry about implementing this yet
	 * For assignment 3.2: This function should set the distance and angle goals appropriately for PID (hint: using the setGoal functions in pid.c)
	 * and wait until the error becomes sufficiently small and persistent before exiting. This function should NOT exit before then.
	 *
	 * HINT: Use a while loop to wait until the turn is complete
	 *
	 * You should also call resetPID before exiting this function so your rat is ready for the next instruction.
	 */
	/** TODO */
	resetPID();
	const int turn90DegreeEncodingCount = 445;
	setPIDGoalA(n * turn90DegreeEncodingCount);
	setPIDGoalD(0);

	/** TODO: goes straight after making the turn for a while, then returns PIDdone */
	while (!PIDdone());
	resetPID();
}
