#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

// Function to create a new queue node
QueueNode* createNode(int data) {
    QueueNode* newNode = (QueueNode*)malloc(sizeof(QueueNode));
    if (!newNode) {
        perror("Failed to create node");
        exit(EXIT_FAILURE);
    }
    newNode->data = data;
    newNode->next = NULL;
    return newNode;
}

// Function to initialize the queue
void initializeQueue(Queue* queue) {
    queue->front = NULL;
    queue->rear = NULL;
}

// Function to check if the queue is empty
bool isEmpty(Queue* queue) {
    return queue->front == NULL;
}

// Function to enqueue an element to the queue
void enqueue(Queue* queue, int data) {
    QueueNode* newNode = createNode(data);
    if (queue->rear == NULL) {
        queue->front = queue->rear = newNode;
        return;
    }
    queue->rear->next = newNode;
    queue->rear = newNode;
}

// Function to dequeue an element from the queue
int dequeue(Queue* queue) {
    if (isEmpty(queue)) {
        printf("Queue is empty. Cannot dequeue.\n");
        return -1;
    }
    QueueNode* temp = queue->front;
    int data = temp->data;
    queue->front = queue->front->next;

    if (queue->front == NULL) {
        queue->rear = NULL;
    }
    free(temp);
    return data;
}

// Function to get the front element of the queue
int front(Queue* queue) {
    if (isEmpty(queue)) {
        printf("Queue is empty.\n");
        return -1;
    }
    return queue->front->data;
}

// Function to get the rear element of the queue
int rear(Queue* queue) {
    if (isEmpty(queue)) {
        printf("Queue is empty.\n");
        return -1;
    }
    return queue->rear->data;
}
