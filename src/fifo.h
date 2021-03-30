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
//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the Queue's Node information
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t {
    // Task information
    uint8_t btn;
    // Pointer to the next node in the queue
    struct node_t* next;
};

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Creates a queue.
///
/// @param[in] task The task information
/// @param[in] size The size of the task array
///
/// @return the head of the new queue
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t* create_queue(void);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Create a new node for the queue
///
/// @param task The task information
///
/// @return a newly allocated task
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t* create_new_node(uint8_t btn);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Returns the top node in the queue
///
/// @param head The head of the queue
///
/// @return the btn value at the top of the queue
//----------------------------------------------------------------------------------------------------------------------------------
uint8_t peek(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Removes the element at the top of the queue.
///
/// @param head The head of the queue.
//----------------------------------------------------------------------------------------------------------------------------------
void pop(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Push a new btn action into the queue
///
/// @param head The head of the queue
/// @param task The btn action to be put into the queue
//----------------------------------------------------------------------------------------------------------------------------------
void push(struct node_t** head, uint8_t btn);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Determines whether the specified head is empty.
///
/// @param head The head of the Queue
///
/// @return True if the specified head is empty, False otherwise.
//----------------------------------------------------------------------------------------------------------------------------------
int is_empty(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Remove all items from the queue
///
/// @param head The head of the queue
//----------------------------------------------------------------------------------------------------------------------------------
void empty_queue(struct node_t** head);


// Array FIFO function declarations
void fifo_push(uint8_t *fifo, uint8_t *wr_ptr, uint8_t len, uint8_t val);
// Array FIFO function declarations
uint8_t fifo_pop(uint8_t *fifo, uint8_t *rd_ptr, uint8_t len);
bool fifo_isempty(uint8_t *fifo, uint8_t *wr_ptr, uint8_t *rd_ptr);

#endif /* SRC_FIFO_H_ */
