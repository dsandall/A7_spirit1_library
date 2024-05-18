/*
 * queue.h
 *
 *  Created on: May 18, 2024
 *      Author: thebu
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_



#include <stdbool.h>

// Define the structure for a queue node
typedef struct QueueNode {
    int data;
    struct QueueNode* next;
} QueueNode;

// Define the structure for the queue
typedef struct Queue {
    QueueNode* front;
    QueueNode* rear;
} Queue;

// Function declarations
QueueNode* createNode(int data);
void initializeQueue(Queue* queue);
bool isEmpty(Queue* queue);
void enqueue(Queue* queue, int data);
int dequeue(Queue* queue);
int front(Queue* queue);
int rear(Queue* queue);



#endif /* INC_QUEUE_H_ */
