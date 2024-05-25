/*
 * defs.h
 *
 *  Created on: May 24, 2024
 *      Author: thebu
 */

#ifndef INC_DEFS_H_
#define INC_DEFS_H_


#include "main.h"
#include "cmsis_os.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core

#ifndef INC_ADDRESS_LIST_H
#define INC_ADDRESS_LIST_H
#include "addressList.h"
#endif

#include "SPIRIT_Config.h" // API code for the expansion board
#include "spsgrf.h" //init code for the wireless module

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>




//////  USER OPTIONS  /////
#define LOGON_USERNAME "sandy"
#define LOGON_USER_ADDRESS 0xB0

#define SHOW_OTHER_HEARTBEATS 0
#define SHOW_MY_HEARTBEATS 0
#define SHOW_MY_ANNOUNCEMENTS 0
#define SHOW_MY_ACKS 0
#define SHOW_PAYLOAD_INFO 0
#define SHOW_BAD_PACKETS 0
#define SHOW_OTHER_ACKS 0

#define CARDIAC_ARREST 0 //send huge heartbeats to see if everyone null terminated properly

#define DEFAULT_TEXT_COLOR "\x1B[37m" 	//white text
#define PRIVATE_TEXT_COLOR "\x1B[36m" 	//cyan text




//////  NETWORK STANDARD DEFINES  /////

#define PACKET_ANNOUNCEMENT 1
#define PACKET_ANNOUNCEMENT_RESP 2
#define PACKET_HEARTBEAT 3
#define PACKET_MESSAGE 4

#define USER_DEAD_TIME 110
#define HEARTBEAT_TIME 30

#define MAX_RX_LOAD 96

////////  OTHER DEFINES  //////////////
#define MAX_USERS 256

#define TX_Q_SIZE 16
#define RX_Q_SIZE 16

#define CLEAR_SCREEN "\033[2J"

#define MASSIVE_MESSAGE \
		"jjjjjjjjjj" \
		"aaaaaaaaaa" \
		"ffffffffff" \
		"aaaaaaaaaa" \
		"gggggggggg" \
		"rrrrrrrrrr" \
		"tttttttttt" \
		"yuuuuuuuuu" \
		"iiiiiiiiii" \
		"iiiiiiiiii" \
		"iiiiiiiiii" \
		"iiiiiiiiii" \
		"iiiiiiiiii" \
		"iiiiiiiiii" \
		"iiiiiiiiii" \
		"iiiiiiiiii" \
		"iiiiiiiiii" \
		"iiiiiiiiii" \
		"!!!!!!!!!?"


/////  STRUCTS  /////////////////////

typedef struct {
	bool valid;
	uint8_t type;
	char* user;
	char* message;
	uint8_t source;
	uint8_t dest;
}Payload;

typedef struct {
	bool valid;
	char raw[MAX_RX_LOAD];
	uint8_t source;
	uint8_t dest;
	long t;
}ReceivedPayload;

typedef struct{
	char username[21]; //including null character
//	unsigned int address;
	long timeLastSeen;
} User;




////////  EXTERN GLOBALS ////////////////

extern SemaphoreHandle_t Flag_Spirit;

/////// FUNCTION DEFINITIONS /////////////

char* getName(uint16_t add);

void RTOS_ISR_setPriority(uint32_t IRQn);

void myHAL_UART_printf(const char* format, ...);
void myHAL_UART_reset();

void SpiritGotoReadyState(void);
void SpiritChangeAddress(uint8_t address);








#endif /* INC_DEFS_H_ */
