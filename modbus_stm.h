#ifndef __modbus_stm_h
#define __modbus_stm_h

#include "main.h"

// DEFINE YOUR SLAVE ADDERESS HERE IN HEXADECIMAL
#define SLAVE_ADD 0x01

// PUMP STATES IN TRANSLATION TO THE VALUE TO BE WRITTEN TO CHANGE STATE

#define PUMP_OFF 		1	//PUMP IN OFF STATE
#define PUMP_ON_SP_CN 	2	//PUMP ON AND IN SPEED CONTROL STATAE
#define PUMP_ON_PC_CN	3	//PUMP ON AND IN PROCESS CONTROL STATE
#define PUMP_ERROR_MODE 4	//PUMP IN ERROR MODE,GOTO PUMP_OFF STATE AND THEN TO ANY OTHER STATE
#define PUMP_ON_SS_CN	7	//PUMP ON AND IN SPEED SAFETY CONTROL STATE
#define PUMP_ON_PS_CN	8	//PUMP ON AND IN PROCESS CONTROL STATE

//TRANSLATION OF LOCAL STATUS RETURN VALUES
#define SUCCESS 		1	//SUCCESSFULLY WRITTEN AND ACKNOWLEDGED DATA
#define SOFT_ERROR		0	//ERROR IN WRITING DATA
#define HARD_ERROR		2	//CHECK ERROR MAP FOR ERROR CODE AND COMPARE IN DOCUMENTATION

//GLOBAL VARIABLES
uint8_t rec_data[255];		//ARRAY OF RECIEVED DATA FROM SLAVE
uint8_t hold_reg_map[56];	//HOLDING REGISTERS ARRAY
uint8_t inp_reg_map[66];	//INPUT REGISTERS ARRAY
int8_t write_reg_map[249];	//ARRAY TO WRITE TO MULTIPLE REGISTERS
uint8_t error_map[3];		//ERROR MAPPED TO ERRORS RETURNED BY SLAVE, CHECK ARRAY FOR EXACT ERROR
int status ;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

//MODBUS FUNCTIONS

int READ_HOLDING_REG(uint8_t Add_HI,uint8_t Add_LO,uint8_t Num_reg_HI,uint8_t Num_reg_LO);

int READ_INPUT_REG(uint8_t Add_HI,uint8_t Add_LO,uint8_t Num_reg_HI,uint8_t Num_reg_LO);

int WRITE_SINGLE_REG(uint8_t Add_HI,uint8_t Add_LO,uint16_t value);

int WRITE_MULTI_REG(uint8_t Add_HI, uint8_t Add_LO, uint8_t Num_of_reg_HI, uint8_t Num_of_reg_LO);

uint8_t CHANGE_STATE(uint8_t next_state);

int GET_STATE();

void Send_Data(uint8_t * buff,uint8_t size);

void Receive_data(uint8_t size);
#endif
