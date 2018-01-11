/*
 * PicomotorActuation.h
 *
 * Created: 11/26/2017 9:16:27 AM
 *  Author: Thibaud
 */ 


#ifndef PICOMOTORACTUATION_H_
#define PICOMOTORACTUATION_H_

#include "Drivers.h"
#include "Memory.h"
#include "BoardManagement.h"
#include <math.h>

#ifndef OK
#define OK 0
#endif

// PARAMETERS
uint16_t PosFineStepBoundaries[5] = {0,29,57,85,113}; // Only low bound of intervals
uint16_t NegFineStepBoundaries[2] = {139,170}; // Only low bound of intervals

// ERRORS ENUM
enum picomotor_algorithm {
	PICOMOTOR_ESTIMATION_INIT_CODE = 201,
	PICOMOTOR_ESTIMATION_ENCODERSTATEMONITOR_CODE,
	PICOMOTOR_ESTIMATION_NUMTICKSCALC_CODE,
	PICOMOTOR_ESTIMATION_MOVEINTERVALS_CODE,
	PICOMOTOR_ESTIMATION_INITIALIZEPICO_CODE,
	PICOMOTOR_ESTIMATION_CALIBRATEPICO_CODE,
	PICOMOTOR_ESTIMATION_SETPICOLOCATION_CODE,
	
	CURRENT_STATE_OOB, 
	PREVIOUS_STATE_OOB,
	INCORRECT_STATE_CHANGE,
	STATE_MONITOR_CRITICAL,
	FINE_STEPS_TOO_SMALL,
	FINE_STEPS_TOO_LARGE,
	COARSE_INTERVALS_ZERO,
	MAX_TICKS_COUNT,
	MOVE_INTERVALS_WRONG_DIR,
	LOCATE_PICOMOTOR_CRITICAL
	};
	
// FUNCTIONS
int PICOMOTOR_ESTIMATION_INIT(uint32_t max_ticks)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | PICOMOTOR_ESTIMATION_INIT_CODE;
	
	// Set size of encoder gaps
	REGISTER[memory_ENCODER0_INTERVAL_SIZE] = 212; //nm
	REGISTER[memory_ENCODER1_INTERVAL_SIZE] = 212; //nm
	REGISTER[memory_ENCODER2_INTERVAL_SIZE] = 212; //nm
	
	// Set maximum of ticks count within an interval
	REGISTER[memory_PICO_MAX_TICKS_COUNT] = max_ticks;
	
	// Turn on picomotor voltage
	int error = ActivatePICOV();
	if(error) return error;
	
	return OK;
}
int EncoderStateMonitor(uint8_t current_state, uint8_t prev_state, uint8_t* update)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | PICOMOTOR_ESTIMATION_ENCODERSTATEMONITOR_CODE;
	
	// current_state = state00 or state10 or state11 or state01
	// prev_state = state00 or state10 or state11 or state01
	// OUTPUT = update = -1 (current state is one step down) or 0 (no change of state) or 1 (current state is one step up)
	
	///TODO: Delete the trick
	//if((current_state == state00 || current_state == state11) && (prev_state == state00 || prev_state == state11)) {*update = 0; return OK;}
	
	//Check the state values
	if(!(current_state==state00) && !(current_state==state01) && !(current_state==state10) && !(current_state==state11)) return CURRENT_STATE_OOB;
	if(!(prev_state==state00) && !(prev_state==state01) && !(prev_state==state10) && !(prev_state==state11)) return PREVIOUS_STATE_OOB;
	
	//Check for incorrect state change (abs difference in state = 2)
	if (abs(current_state-prev_state)==2) return INCORRECT_STATE_CHANGE;
	
	//Return state change
	if(current_state==prev_state) {*update = 0; return OK;}
	if(((current_state-prev_state)==1) || ((current_state-prev_state)==-3)) {*update = 1; return OK;}
	if(((current_state-prev_state)==-1) || ((current_state-prev_state)==3)) {*update = -1; return OK;}
	
	return STATE_MONITOR_CRITICAL;
}
int NumTicksCalc(int index, int32_t currentLocation, int32_t desiredLocation, int32_t* CoarseIntervals, int32_t* FineSteps)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | PICOMOTOR_ESTIMATION_NUMTICKSCALC_CODE;
	
	// desiredLocation in nm (0 assumed to be the calibrated 0 at an interval limit)
	// CorseIntervals = number of encoder intervals to move the picomotor (signed int)
	// FineSteps = number of ticks to move the picomotor after the Coarse adjustment (signed int)
	
	uint32_t IntervalSize = REGISTER[memory_ENCODER0_INTERVAL_SIZE + index];
	
	// Find which encoder interval is the desiredLocation
	int32_t desiredInterval = (int32_t)desiredLocation/IntervalSize - (desiredLocation<0);
	
	// Find where within this previous interval is the desiredLocation
	int32_t desiredRemainder = desiredLocation % IntervalSize + IntervalSize*(desiredLocation<0);
	
	// Find current encoder interval
	int32_t currentInterval = (int32_t)currentLocation/IntervalSize;

	// Calculate the CoarseIntervals from the currentInterval and the desiredRemainder
	// Calculate FineSteps from desiredRemainder
	if (desiredRemainder < NegFineStepBoundaries[0])
	{
		// Approach the desiredInterval from the left
		*CoarseIntervals = desiredInterval - currentInterval - 1;
		
		// Size of PosFineStepBoundaries
		uint8_t sizeP = sizeof(PosFineStepBoundaries)/sizeof(PosFineStepBoundaries[0]);
		
		// Compare the remainder to boundaries
		for (uint8_t II=sizeP-1; II >= 0; II--)
		{
			if(desiredRemainder >= PosFineStepBoundaries[II])
			{
				*FineSteps = II + 1;
				break;
			}
		}
		
		// Check the FineSteps is correct
		if(*FineSteps<=0) return FINE_STEPS_TOO_SMALL;
		if(*FineSteps>sizeP) return FINE_STEPS_TOO_LARGE;
	}
	else
	{
		// Approach the desiredInterval from the right
		*CoarseIntervals = desiredInterval - currentInterval + 1;
		
		// Size of NegFineStepBoundaries
		uint8_t sizeN = sizeof(NegFineStepBoundaries)/sizeof(NegFineStepBoundaries[0]);
		
		// Compare the remainder to boundaries
		for (uint8_t II=sizeN-1; II >= 0; II--)
		{
			if(desiredRemainder >= NegFineStepBoundaries[II])
			{
				*FineSteps = II - sizeN;
				break;
			}
		}
		
		// Check the FineSteps is correct
		if(*FineSteps<-sizeN) return FINE_STEPS_TOO_SMALL;
		if(*FineSteps>=0) return FINE_STEPS_TOO_LARGE;
	}
	if(CoarseIntervals==0) return COARSE_INTERVALS_ZERO;
	
	return OK;	
	
}
int MoveIntervals(int index, int32_t CoarseIntervals, int32_t* MovedIntervals, int32_t* MovedTicks)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | PICOMOTOR_ESTIMATION_MOVEINTERVALS_CODE;
	
	// index = 0 or 1 or 2 (index of picomotor)
	// CoarseIntervals = number of encoder intervals to move the picomotor (signed int)
	// MovedIntervals = OUTPUT actual number of intervals moved by the function. Should be equal to CoarseIntervals if everything went fine
	// MovedTicks = OUTPUT actual number of ticks moved by the function. Just for info.
	
	int error = 0;
	
	// Get the direction
	int8_t dir;
	if(CoarseIntervals > 0) dir = 1;
	else if(CoarseIntervals < 0) dir = -1;
	else return OK; //No need to move
	
	// Get current state of encoders
	uint8_t prev_state, current_state;
	error = GetEncoderState(index, &current_state);
	if(error) return error;
	
	// Loop
	*MovedIntervals = 0;
	*MovedTicks = 0;
	uint8_t update = 0;
	for (uint32_t II=0; II < abs(CoarseIntervals); II++)
	{
		// Update previous state for next interval
		prev_state = current_state; 
		// Count the number of ticks
		uint32_t ticks_count=0;
		
		// Actuate while a new state is not reached
		do
		{
			// Move the picomotor one tick
			MovePicomotor(index,dir);
			*MovedTicks+=dir;
			
			// Count the step
			ticks_count++;
			if(ticks_count>REGISTER[memory_PICO_MAX_TICKS_COUNT]) return MAX_TICKS_COUNT;
			
			// Update current state
			error = GetEncoderState(index, &current_state);
			if(error) return error;
			
			// Update the the state monitor
			error = EncoderStateMonitor(current_state, prev_state, &update);
			if(error) return error;
			
			// Check the update (good direction)
			if(update==-dir) return MOVE_INTERVALS_WRONG_DIR; 
			
		} while (!update);
		
		*MovedIntervals+=dir;
	}
	
	return OK;	
}
int InitializePicomotor(int index, bool limit)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | PICOMOTOR_ESTIMATION_INITIALIZEPICO_CODE;
	
	// index = 0 or 1 or 2 (index of picomotor)
	// limit = 0 (initialize to closest lower encoder inteval switch) or 1 (initialize to soft switch)
	
	//TODO: Initialize picomotor
	
	return OK;
}
int CalibratePicomotor(int index, int16_t intervals, float* mean, float* std)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | PICOMOTOR_ESTIMATION_CALIBRATEPICO_CODE;
	
	// index = 0 or 1 or 2 (index of picomotor)
	// intervals = number of intervals to use for calibration (the more the better/the longer)
	// mean = OUTPUT mean of an actuation tick (this value converges quickly to its true value. No need for a large "intervals" input)
	// std = OUTPUT standard deviation of an actuation tick (This value converges slowly. Need a large "intervals" input)
	
	int error;
	
	// The calculation of the mean and standard deviation of one actuation is calculated from aggregating the results of "sqrt(intervals)" intervals
	int8_t dir; 
	if(intervals>=0) dir = 1;
	if(intervals<0) dir = -1;
	
	// To calculate the mean and std, we need the sum and the sum of squares
	uint32_t sum_ticks = 0;
	uint32_t sum_ticks_squared = 0;
	
	// Get data
	int32_t MovedIntervals, MovedTicks;
	for(int16_t II=0; II<abs(intervals); II++)
	{
		// Move the desired amount of intervals
		error = MoveIntervals(index, dir, &MovedIntervals, &MovedTicks);
		if(error) return error;
		
		sum_ticks += MovedTicks;
		sum_ticks_squared += pow(MovedTicks,2);
	}
	
	// From sum_ticks and sum_ticks_squared, get the mean and std
	*mean = (float)REGISTER[memory_ENCODER0_INTERVAL_SIZE + index]/sum_ticks*intervals;	
	*std = sqrt(sum_ticks_squared/abs(sum_ticks)-(sum_ticks/intervals))*(*mean);
		
	return OK;
}
int SetPicomotorLocation(int index, int32_t currentLocation, int32_t desiredLocation)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | PICOMOTOR_ESTIMATION_SETPICOLOCATION_CODE;
	
	// INPUT index = 0 or 1 or 2 (index of picomotor)
	// INPUT currentLocation in nm (0 assumed to be the calibrated 0 at an interval limit)
	// INPUT desiredLocation in nm (0 assumed to be the calibrated 0 at an interval limit)
	
	int error = 0;
	
	// Get the number of Coarse intervals to go and the fine steps
	int32_t CoarseIntervals, FineSteps;
	error = NumTicksCalc(index, currentLocation,desiredLocation,&CoarseIntervals,&FineSteps);
	if(error) return error;
	
	// Move the coarse intervals
	int32_t MovedIntervals, MovedTicks;
	error = MoveIntervals(index, CoarseIntervals, &MovedIntervals, &MovedTicks);
	if(error) return error;
	
	// Fine adjustment
	if(FineSteps > 0)
	{
		// Move into the interval
		error = MoveIntervals(index, 1, &MovedIntervals, &MovedTicks);
		if(error) return error;
		
		// Fine steps
		error = MovePicomotor(index, FineSteps-1);
		if(error) return error;
	}
	else if(FineSteps < 0)
	{
		// Move into the interval
		error = MoveIntervals(index, -1, &MovedIntervals, &MovedTicks);
		if(error) return error;
		
		// Fine steps
		error = MovePicomotor(index, FineSteps+1);
		if(error) return error;
	}
	else return LOCATE_PICOMOTOR_CRITICAL; //Should never happen, FineSteps is always non-zero
	
	return OK;
}

#endif /* PICOMOTORACTUATION_H_ */