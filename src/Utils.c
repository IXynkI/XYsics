#include <Utils.h>
#include <MathCore.h>

List *createList(DataType size)
{
    List *list = (List *)malloc(size);
    if (list == NULL)
    {
        printf("Error while allocting memory for List");
        exit(1);
    }
    list->head = NULL;
    list->type = size; 
    return list;
}

ListNode *createListNode(ListNode *prev)
{
    ListNode *newNode = (ListNode *)malloc(sizeof(ListNode));
    if (newNode == NULL)
    {
        printf("Error while allocting memory for ListNode");
        exit(1);
    }
    newNode->prev = prev;
    newNode->data = NULL;
    newNode->next = NULL;
    return newNode;
}

void appendToList(List *list, void *data)
{
    if (list->head == NULL)
    {
        list->head = createListNode(NULL);
        list->head->data = malloc(list->type);
        if(!list->head->data) {free(list->head);}
        memcpy(list->head->data, data, list->type);
        return;
    }
    ListNode *current = list->head;
    while (current->next != NULL)
    {
        current = current->next;
    }
    current->next = createListNode(current);
    current->next->data = malloc(list->type);
    if(!current->next->data) {free(current->next);}
    memcpy(current->next->data, data, list->type);
}

void *getDataAt(List *list, int index)
{
    if (list == NULL)
    {
        printf("Errow while getting data from List. List isn't initialized\n");
        return NULL;
    }

    ListNode *current = list->head;
    int count = 0;

    while (current != NULL)
    {
        if (count == index)
        {
            return current->data;
        }
        count++;
        current = current->next;
    }

    printf("Error while getting data from List. Index %d is out of bounds\n", index);
    return NULL;
}

void replaceDataAt(List *list, int index, void *data)
{
    if (list == NULL)
    {
        printf("Error while replacing data in List. List isn't initialized\n");
        return;
    }

    ListNode *current = list->head;
    int count = 0;

    while (current != NULL)
    {
        if (count == index)
        {
            current->data = data;
            return;
        }
        count++;
        current = current->next;
    }

    printf("Error while replacing data in List. Index %d is out of bounds\n", index);
    return;
}

void removeNodeAt(List *list, int index)
{
    if (list == NULL)
    {
        printf("Error while deleting Node in List. List isn't initialized\n");
        return;
    }

    ListNode *current = list->head;
    int count = 0;

    while (current != NULL)
    {
        if (count == index)
        {
            if (current->prev != NULL)
            {
                current->prev->next = current->next;
            } else{
                list->head = current->next;
            }
            
            if(current->next != NULL)
            {
                current->next->prev = current->prev;
            }

            free(current->data);
            free(current);
        }
        count++;
        current = current->next;
    }

    printf("Error while deleting Node in List. Index %d is out of bounds\n", index);
    return;
}

void removeNode(List *list, ListNode *nodeToRemove) {
    if (nodeToRemove == NULL || list == NULL) {
        return;
    }

    if (nodeToRemove->prev != NULL) {
        nodeToRemove->prev->next = nodeToRemove->next;
    } else {
        list->head = nodeToRemove->next;
    }

    if (nodeToRemove->next != NULL) {
        nodeToRemove->next->prev = nodeToRemove->prev;
    }
    free(nodeToRemove->data);
    free(nodeToRemove);
}

void freeList(List *list)
{
    ListNode *current = list->head;
    while (current != NULL)
    {
        ListNode *next = current->next;
        free(current->data);
        free(current);
        current = next;
    }
    free(list);
}
