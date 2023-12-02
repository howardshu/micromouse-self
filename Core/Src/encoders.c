/*
 * encoders.c
 */

#include "main.h"
#include "encoders.h"

/*
 * Implement this function so it returns the right encoder value
 * Right encoder is TIM2
 * Multiply by -1 to return positive values when moving forwards
 */
int16_t getRightEncoderCounts() {
	return -1 * TIM2->CNT;
}

/*
 * Implement this function so it returns the left encoder value
 * Left encoder is TIM1
 * Multiply by -1 to return positive values when moving forwards
 */
int16_t getLeftEncoderCounts() {
	return -1 * TIM1->CNT;
}

/*
 * This function has already been implemented for you. Enjoy! :)
 */
void resetEncoders() {
	TIM1->CNT = (int16_t) 0;
	TIM2->CNT = (int16_t) 0;
}
