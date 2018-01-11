/*
 * CodeProgramming.h
 *
 * Created: 11/26/2017 9:22:25 AM
 *  Author: Thibaud
 */ 


#ifndef CODEPROGRAMMING_H_
#define CODEPROGRAMMING_H_

#include "Drivers.h"
#include "Memory.h"
#include <math.h>

#ifndef OK
#define OK 0
#endif

enum eeprom_prog {
	EEPROM_PROG_SAVE = 231
};
int SaveApplicationFromCameraToEEPROM(int port, CodeID_t CodeID, uint8_t Npages){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | EEPROM_PROG_SAVE;

	uint8_t buffer[SPM_PAGESIZE];
	int status;
	
	// Save size of code
	status = WriteCodeInfoinEEPROM(CodeID, Npages);
	if(status) return status;
	
	for (int II = 0; II < Npages; II++){
		SendFeedback(port,245,0); // Trigger the camera to send a page
		
		status = LoadData(port, buffer, SPM_PAGESIZE);
		if(status) return status;
		
		status = WritePageInEEPROM(CodeID, II, buffer);
		if(status) return status;
	}
	
	return OK;
}





#endif /* CODEPROGRAMMING_H_ */