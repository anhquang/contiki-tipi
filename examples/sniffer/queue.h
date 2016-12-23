#include "contiki.h"
#include <stdbool.h>
/* queu implementation */
#define MAX 10


typedef struct {
uint8_t p[256];
uint16_t length;
} packet_t;
static packet_t queue[MAX]; 

packet_t peek();
bool isEmpty();
bool isFull();
int size();
void insert(packet_t data);
packet_t removeData();
