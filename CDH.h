/*
 * CDH.h
 *
 * Created: 6/16/2016 4:13:29 PM
 *  Author: Thibaud
 */ 


#ifndef CDH_H_
#define CDH_H_

#include "Memory.h"
#include "Interfaces.h"
#include "Drivers.h"
#include "ElectrodeActuation.h"
#include "PicomotorActuation.h"
#include "CodeProgramming.h"
#include "BoardManagement.h"

int ParseCommand(int port)
{	
	//--------------------------------------------------
    //                 MESSAGE CHECK
	//--------------------------------------------------
	uint8_t command =0;
	int32_t data = 0;
	uint16_t checksum;
	if (CheckMessage(port, &command, &data, &checksum)){
		if ( port == 1 ) return SendFeedback(port,253,((uint32_t)REGISTER[memory_UART0_INDEX] << 24) | ((uint32_t)command << 16) | checksum);
		if ( port == 2 ) return SendFeedback(port,253,((uint32_t)REGISTER[memory_UART1_INDEX] << 24) | ((uint32_t)command << 16) | checksum);
	}	
	
	//--------------------------------------------------
    //                       PRIVATE
	//--------------------------------------------------
	// command = 0
	if (command	== 0){
		int error = SendFeedback(port,0,0xAA12e57); //Send back AAReST written in Hex
		if(error) return error;
	}

	/*--------------------------------------------------
                       REGISTER WRITE
	--------------------------------------------------*/
	// command = 1 to memoryCOUNT
	else if (command < memoryCOUNT)
	{	
		REGISTER[command] = data;
		int error = SendFeedback(port,command,0);
		if(error) return error;	
	}

	/*--------------------------------------------------
                       REGISTER READ
	--------------------------------------------------*/
	// command = 150
	else if(command == 150){
		int error = SendFeedback(port,data,REGISTER[data]);
		if(error) return error;
	}
	
	/*--------------------------------------------------
                          ACTIONS
	--------------------------------------------------*/
	// command = 151-239
	
	// RE-INITIALIZE UART0
	else if (command==151){
		int status = UART0_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE UART1
	else if (command==152){
		int status = UART1_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE SPI
	else if (command==153){
		int status = SPI_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE I2C
	else if (command==154){
		int status = I2C_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// TEST I2C
	else if (command==155){
		int status = I2C_WRITE(data, (uint8_t [0]){}, 0);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE ADC
	else if (command==156){
		int status = ADC_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE COMMUNICATIONS
	else if (command==157){
		int status = COMMUNICATION_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE POWER
	else if (command==160){
		int status = POWER_INIT();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// ACTIVATE ELECTRODE HV
	else if (command==161){
		int status = ActivateHV();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// DEACTIVATE ELECTRODE HV
	else if (command==162){
		int status = DeactivateHV();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// ACTIVATE PICOMOTOR HV
	else if (command==163){
		int status = ActivatePICOV();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// DEACTIVATE PICOMOTOR HV
	else if (command==164){
		int status = DeactivatePICOV();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// CHANGE VARIABLE HV
	else if (command==165){
		int status = SetVoltage(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// CHANGE BIAS HV
	else if (command==166){
		int status = SetBias(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// ENABLE SUPPLY VOLTAGE
	else if (command==167){
		int status = EnableSV(data,true);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// DISABLE SUPPLY VOLTAGE
	else if (command==168){
		int status = EnableSV(data,false);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// ENABLE CURRENT LIMITER
	else if (command==169){
		int status = EnableCL(data,true);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// DISABLE CURRENT LIMITER
	else if (command==170){
		int status = EnableCL(data,false);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// MEASURE FB VOLTAGE
	else if (command==171){
		uint16_t val;
		int status = MeasureV(data,&val);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// CURRENT LIMITER FAULT
	else if (command==172){
		int status = IsCLFault(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE SEPERATION DEVICE
	else if (command==175){
		int status = SEP_DEV_INIT();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RELEASE SEPERATION DEVICE
	else if (command==176){
		int status = ReleaseMirror(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// SEPERATION DEVICE OFF
	else if (command==177){
		int status = IsMirrorConstrained();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE PICOMOTORS DRIVER
	else if (command==179){
		int status = PICOMOTORS_INIT();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE PICOMOTORS ESTIMATION ALGORITHM
	else if (command==180){
		int status = PICOMOTOR_ESTIMATION_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// LEFT PICOMOTOR
	else if(command==181){ // MOVE BY TICKS
		int status = MovePicomotor(0,data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==182){ // MOVE BY INTERVALS
		int32_t MovedIntervals, MovedTicks;
		int status = MoveIntervals(0, data, &MovedIntervals, &MovedTicks);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==183){ // MOVE BY NM (THROUGH ALGORITHM)
		int status = SetPicomotorLocation(0, REGISTER[memory_PICO0_LOCATION], data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==184){ // INITIALIZE
		int status = InitializePicomotor(0, data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==185){ // CALIBRATE
		float mean, std;
		int status = CalibratePicomotor(0, data, &mean, &std);
		if(!status){
			REGISTER[memory_PICO0_MEAN] =  (int32_t)(mean*1000000);
			REGISTER[memory_PICO0_STD] =  (int32_t)(  std*1000000);
		}
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==186){ // MEASURE ENCODER STATE
		uint8_t state;
		int status = GetEncoderState(data, &state);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RIGHT PICOMOTOR
	else if(command==191){ // MOVE BY TICKS
		int status = MovePicomotor(1,data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==192){ // MOVE BY INTERVALS
		int32_t MovedIntervals, MovedTicks;
		int status = MoveIntervals(1, data, &MovedIntervals, &MovedTicks);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==193){ // MOVE BY NM (THROUGH ALGORITHM)
		int status = SetPicomotorLocation(1, REGISTER[memory_PICO1_LOCATION], data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==194){ // INITIALIZE
		int status = InitializePicomotor(1, data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==195){ // CALIBRATE
		float mean, std;
		int status = CalibratePicomotor(1, data, &mean, &std);
		if(!status){
			REGISTER[memory_PICO1_MEAN] =  (int32_t)(mean*1000000);
			REGISTER[memory_PICO1_STD] =  (int32_t)(std*1000000);
		}
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==196){ // MEASURE ENCODER STATE
		uint8_t state;
		int status = GetEncoderState(1, &state);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// BOTTOM PICOMOTOR
	else if(command==201){ // MOVE BY TICKS
		int status = MovePicomotor(2,data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==202){ // MOVE BY INTERVALS
		int32_t MovedIntervals, MovedTicks;
		int status = MoveIntervals(2, data, &MovedIntervals, &MovedTicks);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==203){ // MOVE BY NM (THROUGH ALGORITHM)
		int status = SetPicomotorLocation(2, REGISTER[memory_PICO2_LOCATION], data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==204){ // INITIALIZE
		int status = InitializePicomotor(2, data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==205){ // CALIBRATE
		float mean, std;
		int status = CalibratePicomotor(2, data, &mean, &std);
		if(!status) {
			REGISTER[memory_PICO2_MEAN] = (int32_t)(mean*1000000);
			REGISTER[memory_PICO2_STD] =  (int32_t)(std*1000000);
		}
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==206){ // MEASURE ENCODER STATE
		uint8_t state;
		int status = GetEncoderState(2, &state);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE MUX
	else if (command==210){
		int status = MULTIPLEXER_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// TURN CHANNEL ON
	else if (command==211){ 
		int status = ChannelOn(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	// TURN CHANNEL OFF
	else if (command==212){ 
		int status = ChannelOff(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE ELECTRODE ALGORITHM
	else if (command==213){
		int status = StartElectrodeActuation();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// ACTUATE ELECTRODE
	else if (command==214){
		int status = ActuateElectrode(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE THERMO-SENSORS
	else if (command==220){
		int status = TEMP_SENSORS_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// MEASURE TEMP FROM PCT2075
	else if(command==221){
		int16_t temp;
		int status = GetTemperaturePCT2075(data, &temp);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// MEASURE TEMP FROM TMP006
	else if(command==222){
		int16_t temp;
		int status = GetTemperatureTMP006(data, &temp);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE WATCHDOG TIMER
	else if (command==230){
		int status = WATCHDOG_INIT();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// DISABLE WATCHDOG TIMER
	else if (command==231){
		DisableWatchdogTimer();
		int error = SendFeedback(port,command,0);
		if(error) return error;
	}
	
	/*--------------------------------------------------
                       SPECIAL COMMANDS
	--------------------------------------------------*/
	// command = 240-255

	// SAVE MEMORY
	else if(command==240){
		int status = SaveRegister(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}

	// LOAD MEMORY
	else if(command==241){
		int status = LoadRegister(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// WRITE CODE TO EEPROM
	else if(command==245){
		uint8_t Npages = data & 0xffff;
		CodeID_t CodeID = data >> 16;
		int status =  SaveApplicationFromCameraToEEPROM(port, CodeID, Npages);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// GET SIZE OF CODE IN EEPROM
	else if(command==246){
		uint8_t Npages;
		int status = GetSizeofCode(data, &Npages);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// READ DWORD OF CODE IN EEPROM
	else if(command==247){
		uint32_t dword;
		int status = ReadDWordFromEEPROM(data>>16, data & 0xffff, &dword);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	//PING
	else if(command==255){
		int error = SendFeedback(port,command,0);
		if(error) return error;
		REGISTER[memory_BOOT_SAFE] = 0;
		SaveRegisterValue(0, memory_BOOT_SAFE);
	}
	
	// WRONG COMMAND
	else{
		int error = SendFeedback(port,254,command);
		if(error) return error;	
	}
	
	return OK;
}

#endif /* CDH_H_ */