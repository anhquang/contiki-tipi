#include "queue.h"

static packet_t queue[MAX];
int front = 0;
int rear = -1;
int itemCount = 0;

packet_t peek() {
   return queue[front];
}

bool isEmpty() {
   return itemCount == 0;
}

bool isFull() {
   return itemCount == MAX;
}

int size() {
   return itemCount;
}  

void insert(packet_t data) {

   if(!isFull()) {
	
      if(rear == MAX-1) {
         rear = -1;            
      }       

      queue[++rear] = data;
      itemCount++;
   }
}

packet_t removeData() {
   packet_t data = queue[front++];
	
   if(front == MAX) {
      front = 0;
   }
	
   itemCount--;
   return data;  
}
