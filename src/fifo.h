/*
 * fifo.h
 *
 *  Created on: Mar 25, 2021
 *      Author: Karston Christensen
 */

#ifndef SRC_FIFO_H_
#define SRC_FIFO_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

// define the queue data structure
/* Declare a fifo for button actions */
#define FIFO_LEN	10

struct fifo_struct {
	uint8_t fifo[FIFO_LEN];
	uint8_t rd;
	uint8_t wr;
};

// Init the fifo
void fifo_init(struct fifo_struct *fifo);
// Array FIFO function declarations
void fifo_push(struct fifo_struct *fifo, uint8_t val);
// Array FIFO function declarations
uint8_t fifo_pop(struct fifo_struct *fifo);
bool fifo_isempty(struct fifo_struct *fifo);

#endif /* SRC_FIFO_H_ */
