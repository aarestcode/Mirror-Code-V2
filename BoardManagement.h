/*
 * BoardManagement.h
 *
 * Created: 11/26/2017 9:26:04 AM
 *  Author: Thibaud
 */ 


#ifndef BOARDMANAGEMENT_H_
#define BOARDMANAGEMENT_H_

#include "Drivers.h"
#include "Memory.h"

#ifndef OK
#define OK 0
#endif

typedef enum{
	MODE_NOMINAL,
	MODE_ELECTRODE,
	MODE_PICOMOTOR,
	SAFE_MODE,
} Mode_t;

Mode_t Mode;
Mode_t PreviousMode;

enum board_management {
	BOARD_MANAGEMENT_SET_MODE = 241,
	BOARD_MANAGEMENT_CHECK_HEALTH,
	
	SET_FROM_SAFE_MODE,
	
};

int SetMode(Mode_t _Mode){
	
	// If same as current mode, return
	if (_Mode == Mode) return OK;
	
	// Cannot change mode if currently in Safe Mode
	if (Mode == SAFE_MODE) return SET_FROM_SAFE_MODE;
	
	// If set to safe mode, add a count to the register
	if (_Mode == SAFE_MODE) REGISTER[memory_SAFE_MODE_COUNT]++;

	if (Mode == MODE_NOMINAL){
		if (_Mode == MODE_PICOMOTOR){
			int error = ActivatePICOV();
			if (error) return error;
		}
		if (_Mode == MODE_ELECTRODE){
			int error = ActivateHV();
			if (error) return error;
		}
	}
	if (Mode == MODE_PICOMOTOR){
		int error = DeactivatePICOV();
		if (error) return error;
		
		if (_Mode == MODE_ELECTRODE){
			int error = ActivateHV();
			if (error) return error;
		}
	}
	if (Mode == MODE_ELECTRODE){
		int error = DeactivateHV();
		if (error) return error;
		
		if (_Mode == MODE_PICOMOTOR){
			int error = ActivatePICOV();
			if (error) return error;
		}
	}
	
	PreviousMode = Mode;
	Mode = _Mode;
	REGISTER[memory_MODE] = Mode;
	return OK;
}
int CheckHealth(void){
	
	// Check temperature sensors	
	if (Mode == SAFE_MODE){
		// If nominal temperature, go back to nominal mode and reset safe mode counter
		if (0){
			REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER] = 0;
			Mode = MODE_NOMINAL;
		}
		
		return OK;
	}
	
	else { // Mode != SAFE_MODE
		// If temperature out of range, increment safe mode counter
		if (0) {
			REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER]++;
		
			// If safe mode counter overflows, trigger safe mode
			if (REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER] > REGISTER[memory_SAFE_MODE_TRIGGER_OVERFLOW]) {
				int error = SetMode(SAFE_MODE);
				if (error) return error;
			}
		}
	}
	
	if (Mode == MODE_PICOMOTOR) {
		// Check feedback voltage
		
		// If out of range, increment safe mode counter
		if (0){
			REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER]++;
			
			// If safe mode counter overflows, trigger safe mode
			if (REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER] > REGISTER[memory_SAFE_MODE_TRIGGER_OVERFLOW]){
				int error = SetMode(SAFE_MODE);
				if (error) return error;
			}
		}
	}
	
	if (Mode == MODE_ELECTRODE) {
		// Check feedback voltage
		
		// If out of range, increment safe mode counter
		if (0){
			REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER]++;
			
			// If safe mode counter overflows, trigger safe mode
			if (REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER] > REGISTER[memory_SAFE_MODE_TRIGGER_OVERFLOW]){
				int error = SetMode(SAFE_MODE);
				if (error) return error;
			}
		}
	}

	return OK;
	
}


#endif /* BOARDMANAGEMENT_H_ */