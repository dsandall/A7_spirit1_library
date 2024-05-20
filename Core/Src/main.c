/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "queue.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SPIRIT_Config.h" // API code for the expansion board
#include "spsgrf.h" //init code for the wireless module
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>


/* FREERTOS Tasks, Semaphores, and Variables  ------------------------------------*/
void Task_TX(void *argument);
void Task_printUsers(void *argument);
void Task_RX(void *argument);

TaskHandle_t Task_TXHandler, Task_RXHandler, Task_HBHandler, Task_HandleUpdatesHandler;
SemaphoreHandle_t FLAG_SPIRIT;

void RTOS_ISR_setPriority(uint32_t IRQn){
	HAL_NVIC_SetPriorityGrouping(0);
	uint32_t lowPriority = NVIC_EncodePriority(0, 10, 0);
	NVIC_SetPriority(IRQn, lowPriority);
}


void SystemClock_Config(void);
void MX_FREERTOS_Init(void);







//////  USER OPTIONS  /////

#define SHOW_OTHER_HEARTBEATS 0
#define SHOW_MY_HEARTBEATS 0

#define LOGON_USERNAME "sugma"
#define LOGON_USER_ADDRESS 0xB0

#define DEFAULT_TEXT_COLOR "\x1B[37m" //white text
#define PRIVATE_TEXT_COLOR "\x1B[36m" //cyan text

#define CARDIAC_ARREST 0






//////  NETWORK STANDARD DEFINES  /////

#define PACKET_ANNOUNCEMENT 1
#define PACKET_ANNOUNCEMENT_RESP 2
#define PACKET_HEARTBEAT 3
#define PACKET_MESSAGE 4

#define MAX_USERS 256

#define MAX_ADDRESS_INDEX 0xC9

#define USER_DEAD_TIME 110

#define HEARTBEAT_TIME 5



//////  STRUCTS AND GLOBALS  /////


typedef struct {
	bool valid;
	uint8_t type;
	char* user;
	char* message;
	uint8_t source;
	uint8_t dest;
}Payload;

#define MAX_RX_LOAD 512 //todo: what should this be?
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

User usersOnline[MAX_USERS];

uint8_t currentUser = LOGON_USER_ADDRESS;

const char* names[MAX_ADDRESS_INDEX + 1] = {
    [J_MANESH]    = "J_MANESH",
    [N_MASTEN]    = "N_MASTEN",
    [M_PROVINCE]  = "M_PROVINCE",
    [J_KRAMMER]   = "J_KRAMMER",
    [N_DELAPENA]  = "N_DELAPENA",
    [J_GALICINAO] = "J_GALICINAO",
    [J_PARK]      = "J_PARK",
    [S_MARTIN]    = "S_MARTIN",
    [A_GROTE]     = "A_GROTE",
    [M_NOON]      = "M_NOON",
    [J_SHAFFER]   = "J_SHAFFER",
    [D_PETERS]    = "D_PETERS",
    [S_SELTZER]   = "S_SELTZER",
    [W_COLBURN]   = "W_COLBURN",
    [L_CAPUTI]    = "L_CAPUTI",
    [D_ROLAND]    = "D_ROLAND",
    [A_DOSANJH]   = "A_DOSANJH",
    [A_RAJESH]    = "A_RAJESH",
    [M_FESLER]    = "M_FESLER",
    [B_KENNEDY]   = "B_KENNEDY",
    [D_CURIEL]    = "D_CURIEL",
    [L_PEDROZA]   = "L_PEDROZA",
    [M_HERRERA]   = "M_HERRERA",
    [H_EVANS]     = "H_EVANS",
    [L_MCCARTHY]  = "L_MCCARTHY",
    [R_LEONTINI]  = "R_LEONTINI",
    [M_WONG]      = "M_WONG",
    [J_RAMIREZ]   = "J_RAMIREZ",
    [D_CALDERA]   = "D_CALDERA",
    [D_SANDALL]   = "D_SANDALL",
    [C_BAE]       = "C_BAE",
    [D_ROBERDS]   = "D_ROBERDS",
    [P_MULPURU]   = "P_MULPURU",
    [T_GREEN]     = "T_GREEN",
	[DEEZ_NUTZ]   = "DEEZ_NUTZ"
};



TickType_t startTime;

uint8_t skinSuit;

char myUsername[21] = LOGON_USERNAME;


///// FUNCTION DEFINIITIONS //////

void myHAL_UART_printf(const char* format, ...);
void myHAL_UART_clear();
void SpiritGotoReadyState(void);
void confirm_TX();
void get_RX();
#define CREATE_PAYLOAD_HEARTBEAT() createPayload(PAYLOAD_HEARTBEAT, myUsername, NULL, 0xFF); //todo
void createPayload(int type, char* username, char* message, uint8_t dest);








void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  SpiritIrqs xIrqStatus;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (GPIO_Pin != SPIRIT1_GPIO3_Pin){return;}

  SpiritIrqGetStatus(&xIrqStatus);

  if (xIrqStatus.IRQ_RX_FIFO_ALMOST_FULL){
	  //todo:
	  //save part of the message
	  //don't pass the flag on
	  //increment the message pointer to it isn't overwritten

  }

  if (xIrqStatus.IRQ_TX_FIFO_ALMOST_EMPTY){
	  //todo:
  }

  if (xIrqStatus.IRQ_TX_DATA_SENT)
  {
	  confirm_TX();
	  xSemaphoreGiveFromISR(FLAG_SPIRIT, &xHigherPriorityTaskWoken);
  }

  if (xIrqStatus.IRQ_RX_DATA_READY)
  {
	  get_RX();
	  xSemaphoreGiveFromISR(FLAG_SPIRIT, &xHigherPriorityTaskWoken);
  }

  if (xIrqStatus.IRQ_RX_DATA_DISC)
  {
//	myHAL_UART_printf("  | || || |_\r\n");
  }

  if (xIrqStatus.IRQ_RX_TIMEOUT){
	myHAL_UART_printf("timeout\r\n");
  }

  SpiritIrqClearStatus();
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



//TX//////////////
#define TX_Q_SIZE 16
Payload TXq[TX_Q_SIZE];
uint8_t currentReadTX;
uint8_t currentWriteTX;

#define RX_Q_SIZE 16
ReceivedPayload RXq[RX_Q_SIZE];
uint8_t currentReadRX;
uint8_t currentWriteRX;

void confirm_TX(){


	switch (TXq[currentReadTX].type){
		case PACKET_HEARTBEAT:
			if(SHOW_MY_HEARTBEATS){
				myHAL_UART_printf("my heart still beats 4 u \r\n");
			}
			break;

		case PACKET_MESSAGE:
			myHAL_UART_printf("message sent: %s \r\n", TXq[currentReadTX].message);
			break;

		case PACKET_ANNOUNCEMENT:
		    myHAL_UART_printf("going live!\r\n");
			break;

		case PACKET_ANNOUNCEMENT_RESP:
		    myHAL_UART_printf("I see you %s! \r\n", names[TXq[currentReadTX].dest]);
			break;

		default:
		    myHAL_UART_printf("umm... what the sigma? you sent a broken packet \r\n", TXq[currentReadTX].type, TXq[currentReadTX].user);
			break;


	}
	// myHAL_UART_printf("payload sent: type(%d) %s \r\n", TXq[currentReadTX].type, TXq[currentReadTX].user);
	currentReadTX = (++currentReadTX) %TX_Q_SIZE;
}





void Task_TX(void *argument){
	char loadString[100]; //NOT a string
	while (1)
	{


		  if(TXq[currentReadTX].valid){

			  if(skinSuit != 0){impersonate();}

			  SpiritGotoReadyState(); //interrupt any other thang going down
			  if(xSemaphoreTake(FLAG_SPIRIT, 10) == 1){

				SpiritPktStackSetDestinationAddress(TXq[currentReadTX].dest);

				uint8_t type = TXq[currentReadTX].type;
				uint16_t len = 1+strlen(myUsername)+1;

				loadString[0] = type;
				strcpy(&loadString[1], myUsername);
				if (type == 4){
					strcpy(&loadString[strlen(myUsername)+1+1], TXq[currentReadTX].message);
					len = 1+ strlen(myUsername)+1+1+strlen(TXq[currentReadTX].message);
				}

				TXq[currentReadTX].valid = 0;

				SPSGRF_StartTx(loadString, len);
			  }
		//			  vTaskDelay(1000);
		  }

		  vTaskDelay(50);


 }
}



//USERS//////////////

void printUsersOnline(){
	TickType_t currentTime = xTaskGetTickCount();
	myHAL_UART_printf("--- Users Online @t=%d:\r\n", (currentTime-startTime)/1000);
	myHAL_UART_printf("- 0x%02X(%s)(%s) You!\r\n", currentUser, names[currentUser], myUsername);

	for (int i = 0; i < MAX_USERS; i++){
		if (usersOnline[i].timeLastSeen != 0){
			myHAL_UART_printf("- 0x%02X(%d)(%s)(%s) seen %d s ago\r\n", i, i, names[i], usersOnline[i].username, (currentTime - usersOnline[i].timeLastSeen)/1000);
		}
	}
}


void reapUsers(){
	TickType_t currentTime = xTaskGetTickCount();
	for (int i = 0; i < MAX_USERS; i++){
		if ((usersOnline[i].timeLastSeen != 0)){
			if((currentTime-usersOnline[i].timeLastSeen)/1000 > USER_DEAD_TIME){
				myHAL_UART_printf("reaping user 0x%02X\r\n", i);
				usersOnline[i].timeLastSeen = 0;
//				usersOnline[i].username = 0;
			}
		}
	}
}

//RX//////////////
// This should: determine type of recieved packet, add node to onlinelist, send ACKS, print if message
ReceivedPayload currentRX;
void get_RX(){

	//put into queue
	int mine = currentWriteRX;
	currentWriteRX = (++currentWriteRX) %RX_Q_SIZE;


	//get payload info and sanitize payloads
	uint16_t rxLen = SPSGRF_GetRxData((uint8_t *) &RXq[mine].raw);
	RXq[mine].raw[rxLen+1] = '\0'; //ensure null termination for bad little nodes

	RXq[mine].dest = SpiritPktStackGetReceivedDestAddress();
	RXq[mine].source = SpiritPktStackGetReceivedSourceAddress();
	RXq[mine].valid = 1;
	RXq[mine].t = xTaskGetTickCount();

	//update with RX time
	usersOnline[RXq[mine].source].timeLastSeen = RXq[mine].t;

	//todo: add RX packet to queue, handle_rx LATER
//	handle_RX(&RXq[mine]);

}

void handle_RX(){
	//check if private or broadcast

	if(RXq[currentReadRX].valid){

		uint8_t mine = currentReadRX; 					//take currentReadRX
		currentReadRX = (++currentReadRX) %RX_Q_SIZE;	//increment currentReadRX
		ReceivedPayload* load = &RXq[mine];				//set load

		load->valid = 0; //invalidate the oad


		bool private = false;
		if (load->dest == SpiritPktStackGetBroadcastAddress()){
			private = true;
			HAL_UART_Transmit(&huart2, PRIVATE_TEXT_COLOR, 8, HAL_MAX_DELAY);//set text color
		}


		//set username if the username is to spec
		if(strlen(&load->raw[1])<=21){
			strcpy(&usersOnline[load->source].username, &load->raw[1]);
		}


		//if announcement, send ack
		//if ack, do nothing
		//if heartbeat, do nothing
		//if message, print message
		//and check for bad payloads

		if(load->raw[0] == PACKET_ANNOUNCEMENT){
			// send ack
			myHAL_UART_printf("Announcement: (%s)0x%02X has Joined\r\n", &usersOnline[load->source].username, load->source);

			createPayload(PACKET_ANNOUNCEMENT_RESP, myUsername, NULL, load->source);

		} else if (load->raw[0] == PACKET_MESSAGE) {
			//print message
			char* i = (char*)load->raw;
			while(*i != '\0'){i++;}
			i++;

			myHAL_UART_printf("Message from 0x%02X(%s): %s\r\n", load->source, names[load->source], i);

		} else if ((load->raw[0] == PACKET_ANNOUNCEMENT_RESP) | (load->raw[0] == PACKET_HEARTBEAT)){
			//do nothing
			if(load->raw[0] == PACKET_HEARTBEAT){
				if (SHOW_OTHER_HEARTBEATS) {myHAL_UART_printf("<3beat from 0x%02X\r\n", load->source);}
			} else {
				myHAL_UART_printf("ACK by 0x%02X\r\n", load->source);
			}

		} else{
			//todo:untested case
			myHAL_UART_printf("Bad Packet(%02X:%02X:%02X:%02X) from 0x%02X(%d)(%s)\r\n",
					load->raw[0], load->raw[1], load->raw[2], load->raw[3],
					load->source, load->source, names[load->source]);
			return;
		}

		if(private){ HAL_UART_Transmit(&huart2, DEFAULT_TEXT_COLOR, 8, HAL_MAX_DELAY); }	//set color to white



	}
}


void Task_RX(void *argument){
	while(1){
		if(xSemaphoreTake(FLAG_SPIRIT, 10) == 1){
		  SPSGRF_StartRx();
		}
	}
}

void Task_HandleUpdates(void *argument){
	while(1){
		handle_RX();
		vTaskDelay(10);
	}
}



void Task_BeatHeart(void *argument){
	vTaskDelay(HEARTBEAT_TIME * 1000);
	while(1){
		if(CARDIAC_ARREST){
			char* massivemesssage = "jjjjjjjjjj"
					"aaaaaaaaaa"
					"ffffffffff"
					"aaaaaaaaaa"
					"gggggggggg"
					"rrrrrrrrrr"
					"tttttttttt"
					"yuuuuuuuuu"
					"iiiiiiiiii"
					"iiiiiiiiii"
					"iiiiiiiiii"
					"iiiiiiiiii"
					"iiiiiiiiii"
					"iiiiiiiiii"
					"iiiiiiiiii"
					"iiiiiiiiii"
					"iiiiiiiiii"
					"iiiiiiiiii"
					"!!!!!!!!!?";
			createPayload(PACKET_MESSAGE, myUsername, massivemesssage, 0xFF);

		} else {
			createPayload(PACKET_HEARTBEAT, myUsername, NULL, 0xFF);
		}
		vTaskDelay(HEARTBEAT_TIME * 1000);
	}
}

void createPayload(int type, char* username, char* message, uint8_t dest){

	int myWriteTX = currentWriteTX;
	currentWriteTX = (++currentWriteTX) %TX_Q_SIZE;

	TXq[myWriteTX].type = type;
	TXq[myWriteTX].user = username;

	if(type == 4){
		TXq[myWriteTX].message = message;
	}

	TXq[myWriteTX].dest = dest;
	TXq[myWriteTX].valid = 1;
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Create the tasks */
	BaseType_t retVal = xTaskCreate(Task_TX, "Task_TX", configMINIMAL_STACK_SIZE,
		NULL, tskIDLE_PRIORITY + 4, &Task_TXHandler);
	if (retVal != 1) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(Task_RX, "Task_RX", configMINIMAL_STACK_SIZE,
		NULL, tskIDLE_PRIORITY + 3, &Task_RXHandler);
	if (retVal != 1) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(Task_BeatHeart, "Task_BeatHeart", configMINIMAL_STACK_SIZE,
		NULL, tskIDLE_PRIORITY + 2, &Task_HBHandler);
	if (retVal != 1) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(Task_HandleUpdates, "Task_HandleUpdates", configMINIMAL_STACK_SIZE,
		NULL, tskIDLE_PRIORITY + 2, &Task_HandleUpdatesHandler);
	if (retVal != 1) { while(1);}	// check if task creation failed

	// Create Binary Semaphore
	FLAG_SPIRIT = xSemaphoreCreateBinary();
	if (FLAG_SPIRIT == NULL) { while(1); }



	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();

	//	enable interrupts for USART2 receive
	USART2->CR1 |= USART_CR1_RXNEIE;					// enable RXNE interrupt on USART2
	USART2->ISR &= ~(USART_ISR_RXNE);					// clear interrupt flagwhile (message[i] != 0)

	NVIC->ISER[1] = (1 << (USART2_IRQn & 0x1F));		// enable USART2 ISR
	__enable_irq();

	HAL_UART_Transmit(&huart2, DEFAULT_TEXT_COLOR, 8, HAL_MAX_DELAY);//set color to white


	SPSGRF_Init();





	//  impersonate(LOGON_USER_ADDRESS);
	//  strcpy(myUsername, LOGON_USERNAME);


	//Queue initial Announcement Packet, get start time, set semaphore
	createPayload(PACKET_ANNOUNCEMENT, myUsername, NULL, 0xFF);
	startTime = xTaskGetTickCount();
	xSemaphoreGive(FLAG_SPIRIT);

	myHAL_UART_clear();
	myHAL_UART_printf("RTOS NET ONLINE\r\n");

	vTaskStartScheduler();
}


char userInput[100];
int userInputPos = 0;
void USART2_IRQHandler(void){
	char r;

	if (USART2->ISR & USART_ISR_RXNE){

		r = USART2->RDR; // copy received char

		if (r == 13){
			//enter
			userInput[userInputPos] = '\0';
			userInputPos = 0;
			HAL_UART_Transmit(&huart2, &r, 1, HAL_MAX_DELAY);

			handleCommand(userInput);


		} else if (r == 127){
			//backspace case
			userInputPos--;
			HAL_UART_Transmit(&huart2, &r, 1, HAL_MAX_DELAY);
		}
		else {
			//NOT enter
			userInput[userInputPos++] = r;
			HAL_UART_Transmit(&huart2, &r, 1, HAL_MAX_DELAY);

		}

		USART2->ISR &= ~(USART_ISR_RXNE); // clear the flag
	}
	else {
		USART2->ISR = 0; // clear the flag

	}
}

void impersonate(){
	//change username and address to a desired person's

	SpiritGotoReadyState();

	 uint8_t tempRegValue[3];

//	  /* Check the parameters */
//	  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(pxPktStackAddresses->xFilterOnMyAddress));
//	  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(pxPktStackAddresses->xFilterOnMulticastAddress));
//	  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(pxPktStackAddresses->xFilterOnBroadcastAddress));
//
//	  /* Reads the filtering options register */
//	  g_xStatus = SpiritSpiReadRegisters(PCKT_FLT_OPTIONS_BASE, 1, &tempRegValue[0]);
//
//
//	  /* Writes value on the register */
//	  g_xStatus = SpiritSpiWriteRegisters(PCKT_FLT_OPTIONS_BASE, 1, &tempRegValue[0]);

	  /* Fills array with the addresses passed in the structure */
	  tempRegValue[0] = BROADCAST_ADDRESS;
	  tempRegValue[1] = MULTICAST_ADDRESS;
	  tempRegValue[2] = skinSuit;

	  /* Writes them on the addresses registers */
	  g_xStatus = SpiritSpiWriteRegisters(PCKT_FLT_GOALS_BROADCAST_BASE, 3, tempRegValue);



	currentUser = skinSuit;
	skinSuit = 0; //clear skinsuit flag, avoids unnecessary calling of the above functions

	strcpy(&myUsername, names[currentUser]);

}

void handleCommand(char* input){
	//this is after the string has been entered and the user hits enter
	myHAL_UART_printf("                              entered: (%s) \r\n", userInput);


	if (userInput[0] == '/'){

		switch (userInput[1]) {

			case 'u': 	//list online users
				reapUsers();
				printUsersOnline();
				break;

			case 'b':	//broadcast message
				createPayload(PACKET_MESSAGE, myUsername, &userInput[3], 0xFF);
				break;

			case 'p':	//private message
		        char hex_str[3];
		        hex_str[0] = userInput[2];
		        hex_str[1] = userInput[3];
		        hex_str[2] = '\0';

		        uint8_t PInt = strtol(hex_str, NULL, 16);
				createPayload(PACKET_MESSAGE, myUsername, &userInput[5], PInt);
				break;

			case 'i':	//impersonate
				char hexStr[3];
				hexStr[0] = userInput[2]; hexStr[1] = userInput[3]; hexStr[2] = '\0';
				uint8_t hexInt = (uint8_t)strtol(hexStr, NULL, 16);
				skinSuit = hexInt; //save the address to be impersonated until next transmission (cannot change address in IRQ)
				break;

			default: 	//Bad command
				myHAL_UART_printf("                              Bad command: (%s) \r\n", userInput);
				break;
		}

	} else {
		//else just assume it's a broadcast message
		createPayload(PACKET_MESSAGE, myUsername, userInput, 0xFF);
	}

}






void myHAL_UART_printf(const char* format, ...) {
	va_list args;
	va_start(args, format);

	// Allocate temporary buffer for formatted string
	static char buffer[1024]; // Adjust buffer size as needed
	int formatted_length = vsnprintf(buffer, sizeof(buffer), format, args);

	// Check for potential buffer overflow (optional)
	if (formatted_length >= sizeof(buffer)) {
		// Handle buffer overflow (e.g., print error message)
		while(1);
	} else {
		// Print the formatted string
		HAL_UART_Transmit(&huart2, buffer, formatted_length, HAL_MAX_DELAY);
	}

	va_end(args);
}

void myHAL_UART_clear(){
	char clear[] = "\x1B[2J\x1B[0m\x1B[H"; // clear
	HAL_UART_Transmit(&huart2, clear, strlen(clear), 100);

}

void SpiritGotoReadyState(void) {
  static unsigned int i;
  /* Wait for the radio to enter the ready state */
  do {
    /* Go to the ready state */
    if (g_xStatus.MC_STATE == MC_STATE_LOCK) {
      SpiritCmdStrobeReady();
    } else {
      SpiritCmdStrobeSabort();
    }
    /* Delay for state transition */
    for (i = 0; i != 0xFF; i++)
      ;
    /* Update the global status register variable */
    SpiritRefreshStatus();
  } while (g_xStatus.MC_STATE != MC_STATE_READY);

  xSemaphoreGive(FLAG_SPIRIT);
}





























/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
