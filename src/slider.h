#ifndef SRC_HEADER_FILES_SLIDER_H_
#define SRC_HEADER_FILES_SLIDER_H_

/**
 * @file slider.h
 * @author Karston Christensen
 * @date February 3, 2021
 * @brief Contains all the slider driver functions prototypes and defns for critical information.
 *
 */

#include <stdint.h>

// useful definitions for determining slider state

enum slider_pos {
	INACTIVE,
	HARD_LEFT,
	SOFT_LEFT,
	SOFT_RIGHT,
	HARD_RIGHT,
};

// Function prototypes
void slider_setup(void);
void slider_position(uint8_t *position);


#endif /* SRC_HEADER_FILES_SLIDER_H_ */
