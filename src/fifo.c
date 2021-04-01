/*
 * fifo.c
 *
 *  Created on: Mar 25, 2021
 *      Author: Karston Christensen
 */

#include "fifo.h"
#include <stdio.h>
#include <stdlib.h>

struct node_t* create_queue() {
    struct node_t* new_node = (struct node_t*)malloc(sizeof(struct node_t));
    new_node->btn = 0;
    new_node->next = NULL;
    return new_node;
}

struct node_t* create_new_node(uint8_t btn) {
    struct node_t* new_node = (struct node_t*)malloc(sizeof(struct node_t));
    new_node->btn = btn;
    new_node->next = NULL;
    return new_node;
}

uint8_t peek(struct node_t** head) {
    return (*head)->btn;
}

// pop from the front of the queue
void pop(struct node_t** head) {
    struct node_t* top = *head;
    (*head) = (*head)->next;
    free(top);
}

// push to the back of the queue
void push(struct node_t** head, uint8_t btn) {
	// get the head of the queue
    struct node_t* start = (*head);
    // Create a new Node
    struct node_t* new_node = create_new_node(btn);
    // Traverse to the end of the queue
	while (start->next != NULL) {
		start = start->next;
	}
	// Insert at the end of the queue
	start->next = new_node;
	new_node->next = NULL;
}

int is_empty(struct node_t** head) {
    return (*head) == NULL;
}

void empty_queue(struct node_t** head) {
    while (!is_empty(head)) {
        pop(head);
    }
}


// Array FIFO function declarations
void fifo_push(uint8_t *fifo, uint8_t *wr_ptr, uint8_t len, uint8_t val) {
	fifo[*wr_ptr] = val;
    *wr_ptr += 1;
    if (*wr_ptr == len) {
    	*wr_ptr = 0;
    }
}

// Array FIFO function declarations
uint8_t fifo_pop(uint8_t *fifo, uint8_t *rd_ptr, uint8_t len) {
	uint8_t val = fifo[*rd_ptr];
    *rd_ptr += 1;
    if (*rd_ptr == len) {
    	*rd_ptr = 0;
    }
    return val;
}

// Array FIFO
bool fifo_isempty(uint8_t *fifo, uint8_t *wr_ptr, uint8_t *rd_ptr) {
	if (*wr_ptr == *rd_ptr) {
		return true;
	}
	else {
		return false;
	}
}
