/**
 * @file slider.c
 * @author Karston Christensen
 * @date Feb 3rd, 2021
 * @brief Contains all the slider driver functions
 * @details
 * Contains functions to initialize the slider and read its position value.
 *
 */


//***********************************************************************************
// Include files
//***********************************************************************************

#include "slider.h"
#include "capsense.h"


/***************************************************************************//**
 * @brief
 *   Driver to initialize the CAPSENSE slider
 *
 * @details
 * 	 This routine is a low level driver.  It is entirely encapsulated from other drivers.
 * 	 It initializes the CAPSENSE
 * 	 module.
 *
 * @note
 *   This function is normally called once to initialize operation of the CAPSENSE
 *   module.
 *
 ******************************************************************************/
void slider_setup() {
	// Configure the slider
	// Call CAPSENSE library function
	CAPSENSE_Init();
}


/***************************************************************************//**
 * @brief
 *   Driver to obtain CAPSENSE slider value.
 *
 * @details
 * 	 This routine is a low level driver.
 *
 * @note
 *   This function is normally called every time the CAPSENSE slider value is desired.
 *
 * @param[in] position
 *   Pointer to the global variable that stores slider position value.
 *
 ******************************************************************************/
void slider_position(uint8_t *position) {
	uint8_t pos_cnt = 0;
	CAPSENSE_Sense();
	// set position to inactive state
	*position = INACTIVE;
	// Determine which pads of the slider are pressed
	if (CAPSENSE_getPressed(0)) {
		*position = HARD_LEFT;
		pos_cnt++;
	}
	if (CAPSENSE_getPressed(1)) {
		*position = SOFT_LEFT;
		pos_cnt++;
	}
	if (CAPSENSE_getPressed(2)) {
		*position = SOFT_RIGHT;
		pos_cnt++;
	}
	if (CAPSENSE_getPressed(3)) {
		*position = HARD_RIGHT;
		pos_cnt++;
	}
	if (pos_cnt > 1) {
		*position = INACTIVE;
	}
	return;
}

