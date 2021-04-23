/*
 * fifo.c
 *
 *  Created on: Mar 25, 2021
 *      Author: Karston Christensen
 */

#include "fifo.h"
#include <stdio.h>
#include <stdlib.h>

void fifo_init(struct fifo_struct *fifo) {
	fifo->rd = 0;
	fifo->wr = 0;
}

// Array FIFO function declarations
void fifo_push(struct fifo_struct *fifo, uint8_t val) {
	fifo->fifo[fifo->wr] = val;
	fifo->wr += 1;
    if (fifo->wr == FIFO_LEN) {
    	fifo->wr = 0;
    }
}

// Array FIFO function declarations
uint8_t fifo_pop(struct fifo_struct *fifo) {
	uint8_t val = fifo->fifo[fifo->rd];
	fifo->rd += 1;
    if (fifo->rd == FIFO_LEN) {
    	fifo->rd = 0;
    }
    return val;
}

// Array FIFO
bool fifo_isempty(struct fifo_struct *fifo) {
	if (fifo->rd == fifo->wr) {
		return true;
	}
	else {
		return false;
	}
}
