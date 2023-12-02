/*
 * pid.c
 */

#include "main.h"
#include "motors.h"
#include "encoders.h"

// Parameters
const float kPw = 0.1;
const float kDw = 0.05;
const float kPx = 1;
const float kDx = 0.5;

const int ANGLE_CORRECTION_MAX = 0.8;

// Global Vars
int angleError;
int oldAngleError;
float distanceError;
float oldDistanceError;

int goalAngle;
int goalDistance;
int zeroErrorCount;

void resetPID() {
    /*
     * For assignment 3.1: This function does not need to do anything
     * For assignment 3.2: This function should reset all the variables you define in this file to help with PID to their default
     *  values. You should also reset your motors and encoder counts (if you tell your rat to turn 90 degrees, there will be a big
     * difference in encoder counts after it turns. If you follow that by telling your rat to drive straight without first
     * resetting the encoder counts, your rat is going to see a huge angle error and be very unhappy).
     *
     * You should additionally set your distance and error goal values (and your oldDistanceError and oldAngleError) to zero.
     */
	angleError = 0;
	oldAngleError = 0;
	distanceError = 0;
	oldDistanceError = 0;
	goalAngle = 0;
	goalDistance = 0;
	zeroErrorCount = 0;

	// Reset motors and encoders
	resetMotors();
	resetEncoders();
}


/*
 * This function should return PWM_MAX if pwm > PWM_MAX, -PWM_MAX if pwm < -PWM_MAX, and pwm otherwise.
 */
float limitAngleCorrection(float angle) {
//	if (angle > ANGLE_CORRECTION_MAX) {
//		return ANGLE_CORRECTION_MAX;
//	} else if (angle < -ANGLE_CORRECTION_MAX) {
//		return -ANGLE_CORRECTION_MAX;
//	}
	return angle;
}

void updatePID() {
    /*
     * This function will do the heavy lifting PID logic. It should do the following: read the encoder counts to determine an error,
     * use that error along with some PD constants you determine in order to determine how to set the motor speeds, and then actually
     * set the motor speeds.
     *
     * For assignment 3.1: implement this function to get your rat to drive forwards indefinitely in a straight line. Refer to pseudocode
     * example document on the google drive for some pointers
     *
     * TIPS (assignment 3.1): Create kPw and kDw variables, and use a variable to store the previous error for use in computing your
     * derivative term. You may get better performance by having your kDw term average the previous handful of error values instead of the
     * immediately previous one, or using a stored error from 10-15 cycles ago (stored in an array?). This is because systick calls so frequently
     * that the error change may be very small and hard to operate on.
     *
     * For assignment 3.2: implement this function so it calculates distanceError as the difference between your goal distance and the average of
     * your left and right encoder counts. Calculate angleError as the difference between your goal angle and the difference between your left and
     * right encoder counts. Refer to pseudocode example document on the google drive for some pointers.
     */
	// Difference in left and right encoder counts
	const int encoderCountDifference = getRightEncoderCounts() - getLeftEncoderCounts();

	// how much more the right motor spins compared to left motor
//	angleError = encoderCountDifference; // part 1
	angleError = encoderCountDifference - goalAngle;
	// positive if right motor spins more, negative if left motor spins more
	float angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);
//	float angleCorrection = kPw * angleError;
	oldAngleError = angleError;

//	distanceError = 0.5; // part 1
	distanceError = (0.5 * encoderCountDifference) - goalDistance;
	float distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError);
	oldDistanceError = distanceError;

	// Set motor accordingly
	setMotorLPWM(distanceCorrection + limitAngleCorrection(angleCorrection));
//	setMotorLPWM(1);
	setMotorRPWM(distanceCorrection - limitAngleCorrection(angleCorrection));
//	setMotorRPWM(-1);


	// If the error is close to 0, increment count
	float errorThreshold = 10;
	if ((angleError < 0 && angleError > -errorThreshold) ||
			(angleError > 0 && angleError < errorThreshold)) {
		zeroErrorCount++;
	}
}

void setPIDGoalD(int16_t distance) {
    /*
     * For assignment 3.1: this function does not need to do anything.
     * For assignment 3.2: this function should set a variable that stores the goal distance.
     */
	goalDistance = distance;
}

void setPIDGoalA(int16_t angle) {
    /*
     * For assignment 3.1: this function does not need to do anything
     * For assignment 3.2: This function should set a variable that stores the goal angle.
     */
	goalAngle = angle;
}

int8_t PIDdone(void) {  // There is no bool type in C. True/False values are represented as 1 or 0.
    /*
     * For assignment 3.1: this function does not need to do anything (your rat should just drive straight indefinitely)
     * For assignment 3.2: this function should return true if the rat has achieved the set goal. One way to do this by having updatePID() set some variable when
     * the error is zero (realistically, have it set the variable when the error is close to zero, not just exactly zero). You will have better results if you make
     * PIDdone() return true only if the error has been sufficiently close to zero for a certain number, say, 50, of SysTick calls in a row.
     */
	return zeroErrorCount >= 50;
//	return 0;
}
