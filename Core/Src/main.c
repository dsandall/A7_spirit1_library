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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>



#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core


#define PACKET_ANNOUNCEMENT 1
#define PACKET_ANNOUNCEMENT_RESP 2
#define PACKET_HEARTBEAT 3
#define PACKET_MESSAGE 4


/* FREERTOS Tasks, Semaphores, and Variables  ------------------------------------*/
void Task_TX(void *argument);
void Task_printUsers(void *argument);
void Task_RX(void *argument);



TaskHandle_t Task_TXHandler, Task_printUsersHandler, Task_RXHandler, Task_HBHandler;
SemaphoreHandle_t FLAG_SPIRIT;

void RTOS_ISR_setPriority(uint32_t IRQn){
	HAL_NVIC_SetPriorityGrouping(0);
	uint32_t lowPriority = NVIC_EncodePriority(0, 10, 0);
	//DMA1_Channel3_IRQn
	NVIC_SetPriority(IRQn, lowPriority);
}


typedef struct User{
	char username[20];
	unsigned int address;
	long timeLastSeen;
} User;

char myUsername[32] = "Xx_L_xX";
TickType_t startTime;

struct User usersOnline[256];

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MAX_INDEX 0xC9

const char* names[MAX_INDEX + 1] = {
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

const char* getName(int hex) {
    if (hex <= MAX_INDEX && names[hex] != NULL) {
        return names[hex];
    } else {
        return NULL;  // Indicating the hex is not found
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




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  SpiritIrqs xIrqStatus;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (GPIO_Pin != SPIRIT1_GPIO3_Pin){return;}

  SpiritIrqGetStatus(&xIrqStatus);


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
	myHAL_UART_printf("  | || || |_\r\n");
  }

  if (xIrqStatus.IRQ_RX_TIMEOUT){
	myHAL_UART_printf("timeout\r\n");
  }

  SpiritIrqClearStatus();
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}




//TX//////////////
char TXpayload[] = "3kwikskoped";

void confirm_TX(){
    myHAL_UART_printf("payload sent: %s\r\n", TXpayload);
}


typedef struct {
	int valid;
	int type;
	char* user;
	char* message;
}Payload;

Payload TXq[66];
int currentReadTX;
int currentWriteTX;
void Task_TX(void *argument){
	char loadString[100]; //NOT a string
	  while (1)
	  {


		  if(TXq[currentReadTX].valid){

			  SpiritGotoReadyState(); //interrupt any other thang going down
			  if(xSemaphoreTake(FLAG_SPIRIT, 10) == 1){
//				TXpayload[0] = PACKET_HEARTBEAT; //set heartbeat type

				int type = TXq[currentReadTX].type;
				int len = 1+strlen(myUsername)+1;
				loadString[0] = type;
				strcpy(&loadString[1], myUsername);
				if (type == 4){
					strcpy(&loadString[strlen(myUsername)+1], TXq[currentReadTX].message);
					len = 1+ strlen(myUsername)+1+strlen(TXq[currentReadTX].message);
				}

				currentReadTX++;
				SPSGRF_StartTx(loadString, len);
			  }
			  vTaskDelay(1000);

		  }
		  vTaskDelay(50);

	  }
}


//USERS//////////////

void printUsersOnline(){
	TickType_t currentTime = xTaskGetTickCount();
	myHAL_UART_printf("--- Users Online @t=%d:\r\n", (currentTime-startTime)/1000);
	for (int i = 0; i < 256; i++){
		if (usersOnline[i].address != 0){
			myHAL_UART_printf("- 0x%x(%d)(%s) seen %d s ago\r\n", usersOnline[i].address, usersOnline[i].address, names[usersOnline[i].address], (currentTime - usersOnline[i].timeLastSeen)/1000);
		}
	}
}

#define USER_DEAD_TIME 110

void reapUsers(){
	TickType_t currentTime = xTaskGetTickCount();
	for (int i = 0; i < 256; i++){
		if ((usersOnline[i].address != 0)){
			if((currentTime-usersOnline[i].timeLastSeen)/1000 > USER_DEAD_TIME){
				myHAL_UART_printf("reaping user 0x%x\r\n", usersOnline[i].address);
				usersOnline[i].address = 0;
				usersOnline[i].timeLastSeen = 0;
//				usersOnline[i].username = 0;
			}
		}
	}
}

void Task_printUsers(void *argument){
	while (1){
	  reapUsers();
	  printUsersOnline();
	  vTaskDelay(3500);
	}
}

//RX//////////////
void get_RX(){
	uint8_t sadd = SpiritPktStackGetReceivedSourceAddress();
	uint8_t RXpayload[100];
	RXpayload[0] = 0;
	RXpayload[1] = 0;
	RXpayload[2] = 0;
	RXpayload[3] = 0;
	RXpayload[4] = 0;
	RXpayload[5] = 0;

	usersOnline[sadd].timeLastSeen = xTaskGetTickCount();
	usersOnline[sadd].address = sadd;

	int rxLen = SPSGRF_GetRxData(&RXpayload);
	RXpayload[rxLen+1] = '\0';

	HAL_UART_Transmit(&huart2, "Received: ", 10, HAL_MAX_DELAY);

	myHAL_UART_printf("%02X:%02X:%02X:%02X from 0x%x", RXpayload[0], RXpayload[1], RXpayload[2], RXpayload[3], sadd);

	HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);

}

void printPacket(uint8_t * packet){
	//byte 0 - heart beat
	//byte 1-21 - another
	//byte 22-272 - another one
}

void Task_RX(void *argument){
	while(1){
		if(xSemaphoreTake(FLAG_SPIRIT, 10) == 1){
		  SPSGRF_StartRx();
		}
	}
}
/* USER CODE END 0 */


void Task_BeatHeart(void *argument){
	while(1){
		currentWriteTX++;
		TXq[currentWriteTX].type = PACKET_HEARTBEAT;
		TXq[currentWriteTX].user = myUsername;
		TXq[currentWriteTX].valid = 1;
		vTaskDelay(5000);
	}
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */



  RTOS_ISR_setPriority(EXTI9_5_IRQn);


  /* Create the tasks */
  BaseType_t retVal = xTaskCreate(Task_TX, "Task_TX", configMINIMAL_STACK_SIZE,
  		NULL, tskIDLE_PRIORITY + 4, &Task_TXHandler);
  if (retVal != 1) { while(1);}	// check if task creation failed

  retVal = xTaskCreate(Task_printUsers, "Task_printUsers", configMINIMAL_STACK_SIZE,
  		NULL, tskIDLE_PRIORITY + 2, &Task_printUsersHandler);
  if (retVal != 1) { while(1);}	// check if task creation failed

  retVal = xTaskCreate(Task_RX, "Task_RX", configMINIMAL_STACK_SIZE,
  		NULL, tskIDLE_PRIORITY + 3, &Task_RXHandler);
  if (retVal != 1) { while(1);}	// check if task creation failed

  retVal = xTaskCreate(Task_BeatHeart, "Task_BeatHeart", configMINIMAL_STACK_SIZE,
  		NULL, tskIDLE_PRIORITY + 3, &Task_HBHandler);
  if (retVal != 1) { while(1);}	// check if task creation failed


  // Create Semaphores for task2 and task3
  FLAG_SPIRIT = xSemaphoreCreateBinary();
  if (FLAG_SPIRIT == NULL) { while(1); }



  //Initialization transmissisons
  TXq[0].type = PACKET_HEARTBEAT;
  TXq[0].user = myUsername;
  TXq[0].valid = 1;


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

	//	enable interrupts for USART2 receive
	USART2->CR1 |= USART_CR1_RXNEIE;					// enable RXNE interrupt on USART2
	USART2->ISR &= ~(USART_ISR_RXNE);					// clear interrupt flagwhile (message[i] != 0)

	NVIC->ISER[1] = (1 << (USART2_IRQn & 0x1F));		// enable USART2 ISR
	__enable_irq();

  /* USER CODE BEGIN 2 */
  startTime = xTaskGetTickCount();

  myHAL_UART_clear();
  myHAL_UART_printf("let's goooo");

  SPSGRF_Init();

  SpiritPktStackSetDestinationAddress(0xFF);

  xSemaphoreGive(FLAG_SPIRIT);

  vTaskStartScheduler();

  /* USER CODE END 2 */
}


char userInput[100];
int userInputPos = 0;
void USART2_IRQHandler(void){
	char r;

	if (USART2->ISR & USART_ISR_RXNE){

		r = USART2->RDR; // copy received char

		if (r != 13){
			//NOT enter
			userInput[userInputPos++] = r;

		} else {
			//enter
			userInput[userInputPos] = '\0';
			userInputPos = 0;
			myHAL_UART_printf("entered: (%s) \r\n", userInput);

		}

		HAL_UART_Transmit(&huart2, &r, 1, HAL_MAX_DELAY);

		USART2->ISR &= ~(USART_ISR_RXNE); // clear the flag
	}
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
