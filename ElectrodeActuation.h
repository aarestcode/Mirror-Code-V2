/*
 * ElectrodeActuation.h
 *
 * Created: 11/26/2017 9:21:08 AM
 *  Author: Thibaud
 */ 


#ifndef ELECTRODEACTUATION_H_
#define ELECTRODEACTUATION_H_

#include "Drivers.h"
#include "Memory.h"
#include "BoardManagement.h"
#include <math.h>

#ifndef OK
#define OK 0
#endif

#define N_electrodes 41 //Number of electrodes

enum electrode_algorithm {
	ELECTRODE_ACTUATION_INIT_CODE = 221,
	ELECTRODE_ACTUATION_ACTUATE_CODE
};

int ActuateElectrode(int channel){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | ELECTRODE_ACTUATION_ACTUATE_CODE;
	
	unsigned int memory_address = memory_ELECTRODE1 + channel;
	if ( REGISTER[memory_address] == 0 ) return OK;
	
	int status;
	
	// 1. Check voltage
	uint16_t limit = (uint16_t)REGISTER[memory_ELECTRODE_LIMIT_V];
	uint16_t bias = (uint16_t)REGISTER[memory_HV_BIAS];
	uint16_t voltage = REGISTER[memory_address] & 0xffff;
	if(voltage > bias+limit) {
		voltage = bias+limit;
		REGISTER[memory_address] = (REGISTER[memory_address] & 0xffff0000) | voltage;
	}
	if(voltage < bias-limit) {
		voltage = bias-limit;
		REGISTER[memory_address] = (REGISTER[memory_address] & 0xffff0000) | voltage;
	}
	
	// 2. Set desired voltage
	if (voltage != REGISTER[memory_HV]){
		status = SetVoltage(voltage);  // Set DAC value
		if(status) {
			REGISTER[memory_ELECTRODE_ERROR] = (REGISTER[memory_ELECTRODE_ERROR] << 8) | status;
			return status;
		}
		_delay_ms(REGISTER[memory_HV_TIMER]);
	}
	
	// 3. Turn channel on
	status = ChannelOn(channel);  // Start charging channel
	if(status) {
		REGISTER[memory_ELECTRODE_ERROR] = (REGISTER[memory_ELECTRODE_ERROR] << 8) | status;
		return status;
	}
	
	// 4. Charge electrode
	_delay_ms((REGISTER[memory_address] >> 24) & 0xff);
	
	// 5. Turn channel off
	status = ChannelOff(channel);
	if(status) {
		REGISTER[memory_ELECTRODE_ERROR] = (REGISTER[memory_ELECTRODE_ERROR] << 8) | status;
		return status;
	}
	
	// 6. Update timer in electrode data
	REGISTER[memory_address] = ((REGISTER[memory_address] & 0xff0000) << 8) | (REGISTER[memory_address] & 0xffffff);
	
	return OK;
}
int StartElectrodeActuation(void)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | ELECTRODE_ACTUATION_INIT_CODE;
	
	// Set times
	REGISTER[memory_HV_TIMER] = 15; // Time for HV to stabilize [ms]
	
	// Set maximum voltage
	REGISTER[memory_ELECTRODE_LIMIT_V] = 8088; // Limit (plus/minus) from bias
	
	// Set step increment voltage
	REGISTER[memory_HV_STEP] = 337; // 10V steps
	
	// Set bias voltage
	REGISTER[memory_HV_BIAS] = 8191; // 8191 = +240V Bias
		
	// Set Mode	
	int error = SetMode(MODE_PICOMOTOR);
	if(error) return error;
	
	long volt;
	
	// Initialize voltages for all electrodes (+ delays of 10ms)
	int N_increments = floor((double)(0x3fff - REGISTER[memory_HV_BIAS])/(double)REGISTER[memory_HV_STEP]);
	for(int II=0; II<N_increments; II++){
		
		volt = 0x3fff - II*REGISTER[memory_HV_STEP];
		
		error = SetBias(volt);
		if(error) return error;
		
		error = SetVoltage(volt);
		if(error) return error;
		
		_delay_ms(REGISTER[memory_HV_TIMER]);
		
		for (int ch=0; ch < N_electrodes; ch++){
			REGISTER[memory_ELECTRODE1+ch] = ((long)10<<24) | ((long)10<<16) | (volt & 0xffff);
			error = ActuateElectrode(ch); // Should go very quickly since the voltage is already set
		}
		if(error) return error;
	}
	
	error = SetBias(REGISTER[memory_HV_BIAS]);
	if(error) return error;
	
	error = SetVoltage(REGISTER[memory_HV_BIAS]);
	if(error) return error;
	
	_delay_ms(REGISTER[memory_HV_TIMER]);
	
	for (int ch=0; ch < N_electrodes; ch++){
		REGISTER[memory_ELECTRODE1+ch] = ((long)10<<24) | ((long)10<<16) | (REGISTER[memory_HV_BIAS] & 0xffff);
		error = ActuateElectrode(ch); // Should go very quickly since the voltage is already set
	}
	if(error) return error;
		
	return OK;
}

#endif /* ELECTRODEACTUATION_H_ */