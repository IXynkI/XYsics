#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <MathCore.h>

//Generic Lists
typedef size_t DataType;

static const DataType TYPE_CHAR   = sizeof(char);
static const DataType TYPE_INT    = sizeof(int);
static const DataType TYPE_FLOAT  = sizeof(float);
static const DataType TYPE_DOUBLE = sizeof(double);


typedef struct ListNode
{
    void *data;
    struct ListNode *next;
    struct ListNode *prev;
} ListNode;

typedef struct List
{
    ListNode *head;
    DataType type;
} List;

List *createList(DataType size);
ListNode *createListNode(ListNode *prev);
void appendToList(List *list, void *data);
void *getDataAt(List *list, int index);
void replaceDataAt(List *list, int index, void* data);
void removeNodeAt(List *list, int index);
void removeNode(List *list, ListNode *nodeToRemove);
void freeList(List *list);

#endif