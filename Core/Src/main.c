#include "defs.h"

///// FUNCTION DEFINIITIONS //////
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

void Task_TX(void *argument);
void Task_RX(void *argument);
void Task_HandleUpdates(void *argument);
void Task_BeatHeart(void *argument);
TaskHandle_t Task_TXHandler, Task_RXHandler, Task_HBHandler, Task_HandleUpdatesHandler;

void confirm_TX();
void handle_RX();
void get_RX();
void createPayload(int type, char* username, char* message, uint8_t dest);
void handleCommand(char* input);
void impersonate();
void virus();
void reapUsers();

Payload TXq[TX_Q_SIZE];
uint8_t currentReadTX;
uint8_t currentWriteTX;

ReceivedPayload RXq[RX_Q_SIZE];
uint8_t currentReadRX;
uint8_t currentWriteRX;

User usersOnline[MAX_USERS];

uint8_t currentUser = LOGON_USER_ADDRESS;
char myUsername[21] = LOGON_USERNAME;

TickType_t startTime;

uint8_t spreadVirus;
uint8_t skinSuit;





void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	SpiritIrqs xIrqStatus;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (GPIO_Pin != SPIRIT1_GPIO3_Pin){return;}

	SpiritIrqGetStatus(&xIrqStatus);

	if (xIrqStatus.IRQ_TX_DATA_SENT)
	{
		confirm_TX();
		xSemaphoreGiveFromISR(Flag_Spirit, &xHigherPriorityTaskWoken);
	}

	if (xIrqStatus.IRQ_RX_DATA_READY)
	{
		get_RX();
		xSemaphoreGiveFromISR(Flag_Spirit, &xHigherPriorityTaskWoken);
	}

	if (xIrqStatus.IRQ_RX_TIMEOUT){
		myHAL_UART_printf("timeout\r\n");
	}

	SpiritIrqClearStatus();
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



char userInput[100];
uint8_t userInputPos = 0;
void USART2_IRQHandler(void){
	char r;

	if (USART2->ISR & USART_ISR_RXNE){

		r = USART2->RDR; // copy received char

		if (r == 13){
			//enter
			userInput[userInputPos] = '\0';
			userInputPos = 0;
			HAL_UART_Transmit(&huart2, (uint8_t*) &r, 1, HAL_MAX_DELAY);

			handleCommand(userInput);


		} else if (r == 127 || r == '\b'){
			//backspace case
			userInput[--userInputPos] = '\0';

			HAL_UART_Transmit(&huart2, (uint8_t*) "\x1B[D" , 3, HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*) ' ' , 1, HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*) "\x1B[D" , 3, HAL_MAX_DELAY);

		}
		else {
			//NOT enter
			userInput[userInputPos++] = r;
			HAL_UART_Transmit(&huart2, (uint8_t*) &r, 1, HAL_MAX_DELAY);

		}

		USART2->ISR &= ~(USART_ISR_RXNE); // clear the flag
	}
	else {
		USART2->ISR = 0; // clear the flag

	}
}




void Task_TX(void *argument){
	char loadString[100]; //NOT a string
	while (1)
	{

		  if(TXq[currentReadTX].valid){

			  SpiritGotoReadyState(); //interrupt any other thang going down
			  if(xSemaphoreTake(Flag_Spirit, 10) == 1){

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

				SPSGRF_StartTx((uint8_t*) loadString, len);
			  }

		  }

		  vTaskDelay(50);

	}
}



void Task_RX(void *argument){
	while(1){
		if(xSemaphoreTake(Flag_Spirit, 10) == 1){
		  SPSGRF_StartRx();
		}
	}
}



void Task_HandleUpdates(void *argument){
	while(1){

		handle_RX();

		reapUsers();

		if (spreadVirus){virus();}
		if (skinSuit != 0){impersonate();}

		vTaskDelay(10);
	}
}



void Task_BeatHeart(void *argument){
	vTaskDelay(HEARTBEAT_TIME * 1000);
	while(1){
		#if CARDIAC_ARREST
			createPayload(PACKET_MESSAGE, myUsername, MASSIVE_MESSAGE, BROADCAST_ADDRESS);
		#else
			createPayload(PACKET_HEARTBEAT, myUsername, NULL, BROADCAST_ADDRESS);
		#endif
		vTaskDelay(HEARTBEAT_TIME * 1000);
	}
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
	Flag_Spirit = xSemaphoreCreateBinary();
	if (Flag_Spirit == NULL) { while(1); }



	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();

	//	enable interrupts for USART2 receive
	USART2->CR1 |= USART_CR1_RXNEIE;				// enable RXNE interrupt on USART2
	USART2->ISR &= ~(USART_ISR_RXNE);				// clear interrupt flagwhile (message[i] != 0)

	NVIC->ISER[1] = (1 << (USART2_IRQn & 0x1F));	// enable USART2 ISR
	__enable_irq();

	HAL_UART_Transmit(&huart2, (uint8_t*) DEFAULT_TEXT_COLOR, 8, HAL_MAX_DELAY);//set color to white


	SPSGRF_Init();


	//Queue initial Announcement Packet, get start time, set semaphore
	createPayload(PACKET_ANNOUNCEMENT, myUsername, NULL, BROADCAST_ADDRESS);
	startTime = xTaskGetTickCount();
	xSemaphoreGive(Flag_Spirit);

	myHAL_UART_reset();
	myHAL_UART_printf(
			"~~~ RTOS NET ONLINE ~~~\r\n"
			"Available Commands:\r\n"
			"  List online users - /u \r\n"
			"  (default) Broadcast Message - /b \r\n"
			"  Private Message - /p{XX} \r\n"
			"  Impersonate - /i{XX} \r\n"
			"  Send Clear Screen - /n \r\n"
			"  Send Virus - /v \r\n"
			);


	vTaskStartScheduler();
}





void createPayload(int type, char* username, char* message, uint8_t dest){

	int myWriteTX = currentWriteTX;
	currentWriteTX = (currentWriteTX+1) %TX_Q_SIZE;

	TXq[myWriteTX].type = type;
	TXq[myWriteTX].user = username;

	if(type == 4){
		TXq[myWriteTX].message = message;
	}

	TXq[myWriteTX].dest = dest;
	TXq[myWriteTX].valid = 1;
}



void confirm_TX(){

	switch (TXq[currentReadTX].type){
		case PACKET_HEARTBEAT:
			#if SHOW_MY_HEARTBEATS
				myHAL_UART_printf("my heart still beats 4 u \r\n");
			#endif

			break;

		case PACKET_MESSAGE:
			if (TXq[currentReadTX].dest == BROADCAST_ADDRESS){
				myHAL_UART_printf("broadcast sent: %s \r\n",
					TXq[currentReadTX].message
				);
			} else {
				myHAL_UART_printf("private message sent to %02X: %s \r\n",
					TXq[currentReadTX].dest, TXq[currentReadTX].message
				);
			}
			break;

		case PACKET_ANNOUNCEMENT:
			#if SHOW_MY_ANNOUNCEMENTS
				myHAL_UART_printf("going live! (announcement)\r\n");
			#endif

			break;

		case PACKET_ANNOUNCEMENT_RESP:
			#if SHOW_MY_ACKS
				myHAL_UART_printf("I see you %s! (ack)\r\n",
						getName(TXq[currentReadTX].dest)
				);
			#endif

			break;

		default:
		    myHAL_UART_printf("umm... you sent a broken packet? \r\n",
				TXq[currentReadTX].type, TXq[currentReadTX].user
			);
			break;

	}

	#if SHOW_PAYLOAD_INFO
	myHAL_UART_printf("payload sent: type(%d) user:%s message:%s \r\n",
		TXq[currentReadTX].type, TXq[currentReadTX].user, TXq[currentReadTX].message
	);
	#endif

	currentReadTX = (currentReadTX+1) %TX_Q_SIZE;
}




void get_RX(){

	//put into queue
	int mine = currentWriteRX;
	currentWriteRX = (currentWriteRX+1) %RX_Q_SIZE;


	//get payload info and sanitize payloads
	uint16_t rxLen = SPSGRF_GetRxData((uint8_t *) &RXq[mine].raw);
	RXq[mine].raw[rxLen+1] = '\0'; //ensure null termination for bad little nodes

	RXq[mine].dest = SpiritPktStackGetReceivedDestAddress();
	RXq[mine].source = SpiritPktStackGetReceivedSourceAddress();
	RXq[mine].valid = 1;
	RXq[mine].t = xTaskGetTickCount();

	//update with RX time
	usersOnline[RXq[mine].source].timeLastSeen = RXq[mine].t;

}




void handle_RX(){
	//check if private or broadcast

	if(RXq[currentReadRX].valid){

		uint8_t mine = currentReadRX; 					//take currentReadRX
		currentReadRX = (currentReadRX+1) %RX_Q_SIZE;	//increment currentReadRX

		ReceivedPayload* load = &RXq[mine];				//set load

		load->valid = 0; //invalidate the oad


		bool private = false;
		if (load->dest == SpiritPktStackGetBroadcastAddress()){
			private = true;
			HAL_UART_Transmit(&huart2, (uint8_t*) PRIVATE_TEXT_COLOR, 8, HAL_MAX_DELAY);//set text color
		}


		//set username if the username is to spec
		if(strlen(&load->raw[1])<=21){
			strcpy((char *)&usersOnline[load->source].username, &load->raw[1]);
		}


		//if announcement, send ack
		//if ack, do nothing
		//if heartbeat, do nothing
		//if message, print message
		//and check for bad payloads

		if(load->raw[0] == PACKET_ANNOUNCEMENT){
			// send ack
			myHAL_UART_printf("Announcement: (%s)0x%02X has Joined\r\n",
				&usersOnline[load->source].username, load->source
			);

			createPayload(PACKET_ANNOUNCEMENT_RESP, myUsername, NULL, load->source);

		} else if (load->raw[0] == PACKET_MESSAGE) {
			//print message
			char* i = (char*)load->raw;
			while(*i != '\0'){i++;}
			i++;

			myHAL_UART_printf("Message from 0x%02X(%s): %s\r\n",
				load->source, getName(load->source), i
			);

		} else if ((load->raw[0] == PACKET_ANNOUNCEMENT_RESP) | (load->raw[0] == PACKET_HEARTBEAT)){
			//do nothing
			if(load->raw[0] == PACKET_HEARTBEAT){
				#if SHOW_OTHER_HEARTBEATS
					myHAL_UART_printf("<3beat from 0x%02X\r\n", load->source);
				#endif
			} else {
				myHAL_UART_printf("ACK by 0x%02X\r\n", load->source);
			}

		} else{
			myHAL_UART_printf("Bad Packet(%02X:%02X:%02X:%02X) from 0x%02X(%d)(%s)\r\n",
				load->raw[0], load->raw[1], load->raw[2], load->raw[3],
				load->source, load->source, getName(load->source)
			);
		}

		if(private){  //set color to white
			HAL_UART_Transmit(&huart2, (uint8_t*) DEFAULT_TEXT_COLOR, 8, HAL_MAX_DELAY);
		}


	}
}








//USER LIST/////////////

void printUsersOnline(){
	TickType_t currentTime = xTaskGetTickCount();
	myHAL_UART_printf("--- Users Online @t=%d:\r\n", (currentTime-startTime)/1000);
	myHAL_UART_printf("- 0x%02X %13s - %20s You!\r\n",
		currentUser, getName(currentUser), myUsername
	);

	for (int i = 0; i < MAX_USERS; i++){
		if (usersOnline[i].timeLastSeen != 0){
			myHAL_UART_printf("- 0x%02X %13s - %20s seen %3d s ago\r\n",
				i, getName(i), usersOnline[i].username,
				(currentTime - usersOnline[i].timeLastSeen)/1000
			);
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
				usersOnline[i].username[0] = '\0';

			}
		}
	}
}




/////// HANDLING USER INPUT /////////////////////////////////////////////////////////////

void handleCommand(char* input){
	//this is after the string has been entered and the user hits enter
	myHAL_UART_printf(">>> entered: (%s) \r\n", userInput);


	if (userInput[0] == '/'){

		switch (userInput[1]) {

			case 'u': 	//list online users
				printUsersOnline();
				break;

			case 'b':	//broadcast message
				createPayload(PACKET_MESSAGE, myUsername, &userInput[3], BROADCAST_ADDRESS);
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
				skinSuit = hexInt;
				//save the address until next transmission (cannot easily change address in IRQ)
				break;

			case 'n':	//Send clear screen to all other nodes
				createPayload(PACKET_MESSAGE, myUsername, CLEAR_SCREEN, BROADCAST_ADDRESS);
				break;

			case 'v': 	//send an announcement from 0xFF, triggering a broadcast ACK
				spreadVirus = 1;
				break;

			default: 	//Bad command
				myHAL_UART_printf(">>> bad command: (%s) \r\n", userInput);
				break;
		}

	} else {
		//else just assume it's a broadcast message
		createPayload(PACKET_MESSAGE, myUsername, userInput, 0xFF);
	}

}








void impersonate(){
	//change username and address to a desired person's
	SpiritChangeAddress(skinSuit);

	currentUser = skinSuit;

	if (usersOnline[skinSuit].timeLastSeen != 0){
		strcpy((char *)&myUsername, &usersOnline[skinSuit].username[0]);
	} else {
		strcpy((char *)&myUsername, getName(skinSuit));
	}

	skinSuit = 0; //clear skinSuit flag, avoids unnecessary calling of the above functions
}

void virus(){
	spreadVirus = 0;
	skinSuit = BROADCAST_ADDRESS;
	impersonate();
	createPayload(PACKET_ANNOUNCEMENT, myUsername, NULL, BROADCAST_ADDRESS);
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

