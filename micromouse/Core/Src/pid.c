/*
 * pid.c
 */

#include "main.h"
#include "motors.h"
#include "encoders.h"

// Parameters
const float kPw = 0.01;
const float kDw = 0;
const float kPx = 0.001;
const float kDx = 0;
const float MAX_ACCEL = 0.005;
const float ANGLE_CORRECTION_MAX = 0.3;
const float DISTANCE_CORRECTION_MAX = 0.5;
const float ERR_THRESHOLD = 25;

// Global Vars
int angleError;
int oldAngleError;
int distanceError;
int oldDistanceError;

int goalAngle;
int goalDistance;
int zeroErrorCount;
float oldLPWM;
float oldRPWM;

float absol(float input) {
	return input < 0 ? -input : input;
}

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

	oldLPWM = 0;
	oldRPWM = 0;

	// Reset motors and encoders
	resetMotors();
	resetEncoders();
}


/*
 * This function should return PWM_MAX if pwm > PWM_MAX, -PWM_MAX if pwm < -PWM_MAX, and pwm otherwise.
 */
float limitCorrection(const float pwm, const float limit) {
	if (pwm > limit) {
		return limit;
	}
	if (pwm < -limit) {
		return -limit;
	}
	return pwm;
}

float limitAccel(const float pwm, float* oldPWM) {
	// If new pwm value is too far from old pwm value
	float newPWM;
	if (pwm > *oldPWM) {
		float maxPWM = *oldPWM + MAX_ACCEL;
		newPWM = pwm > maxPWM ? maxPWM : pwm;
	} else {
		float minPWM = *oldPWM - MAX_ACCEL;
		newPWM = pwm < minPWM ? minPWM : pwm;
	}
	*oldPWM = newPWM;
	return newPWM;
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
	const int leftCount = getLeftEncoderCounts(), rightCount = getRightEncoderCounts();
	const int encoderCountDifference = rightCount - leftCount;
	const int encoderCountAverage = (rightCount + leftCount) / 2;

	// how much more the right motor spins compared to left motor
//	angleError = encoderCountDifference; // part 1
	angleError = encoderCountDifference - goalAngle;
	// positive if right motor spins more, negative if left motor spins more
	const float angleCorrection = limitCorrection(kPw * angleError + kDw * (angleError - oldAngleError), ANGLE_CORRECTION_MAX);
//	float angleCorrection = kPw * angleError;
	oldAngleError = angleError;

//	distanceError = 0.5; // part 1
	distanceError = goalDistance - encoderCountAverage;
	const float distanceCorrection = limitCorrection(kPx * distanceError + kDx * (distanceError - oldDistanceError), DISTANCE_CORRECTION_MAX);
	oldDistanceError = distanceError;

	// Set motor accordingly
	setMotorLPWM(limitAccel(distanceCorrection + angleCorrection, &oldLPWM));
//	setMotorLPWM(1);
	setMotorRPWM(limitAccel(distanceCorrection - angleCorrection, &oldRPWM));
//	setMotorRPWM(-1);


	// If the error is close to 0, increment count
	if (absol(angleError) < ERR_THRESHOLD && absol(distanceError) < ERR_THRESHOLD) {
		zeroErrorCount++;
	} else {
		zeroErrorCount = 0;
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
