/*
 * defs.c
 *
 *  Created on: May 24, 2024
 *      Author: thebu
 */
#include "defs.h"

const char* names[BROADCAST_ADDRESS + 1] = {
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
	[DEEZ_NUTZ]   = "DEEZ_NUTZ",
	[BROADCAST_ADDRESS] = "BROADCAST"
};

char* noUser = "NO_NAME";
char* getName(uint16_t add){
	if (*names[add] != '\0'){
		return names[add];
	} else {
		return noUser;
	}
}




//////////// FREERTOS FUNCTIONS ////////////////////

///////// impersonate ///////////////
/* this function sets the interrupt priority of a
 *    given interrupt to a FreeRTOS acceptable value
*/
void RTOS_ISR_setPriority(uint32_t IRQn){
	HAL_NVIC_SetPriorityGrouping(0);
	uint32_t lowPriority = NVIC_EncodePriority(0, 10, 0);
	NVIC_SetPriority(IRQn, lowPriority);
}




//////////// UART FUNCTIONS /////////////////////////////

///////// myHAL_UART_printf ///////////////
/* this is printf, but for UART
*/
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


///////// myHAL_UART_reset ///////////////
/* this clears the screen, resets the background and text color,
 *     and sets the cursor to the top left for UART terminals.
*/
void myHAL_UART_reset(){
	char reset[] = "\x1B[2J\x1B[0m\x1B[H"; // clear
	HAL_UART_Transmit(&huart2, reset, strlen(reset), HAL_MAX_DELAY);

}



////////// SPIRIT FUNCTIONS /////////////////////////

SemaphoreHandle_t Flag_Spirit;

///////// SpiritGotoReadyState ///////////////
/* this forces the spirit into ready state
*/
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

  xSemaphoreGive(Flag_Spirit);
}

///////// SpiritChangeAddress ///////////////
/* this changes the address of the spirit. Must not be called from an interrupt.
*/
void SpiritChangeAddress(uint8_t address){
	SpiritGotoReadyState();

	uint8_t tempRegValue[3];

	/* Fills array with the addresses passed in the structure */
	tempRegValue[0] = BROADCAST_ADDRESS;
	tempRegValue[1] = MULTICAST_ADDRESS;
	tempRegValue[2] = address;

	/* Writes them on the addresses registers */
	g_xStatus = SpiritSpiWriteRegisters(PCKT_FLT_GOALS_BROADCAST_BASE, 3, tempRegValue);

}
