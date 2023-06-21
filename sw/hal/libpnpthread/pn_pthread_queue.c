/*
 * This file is part of the ParaNut project.
 * Copyright 2021-2022 Felix Wagner (<felix.wagner1@hs-augsburg.de>)
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 * Date: 30.01.2022
 */

#include "pn_pthread.h"
#include "pn_pthread_queue.h"
#include <stdlib.h>

/************************************************************
 * QUEUE for storing threads
 * **********************************************************/

/* QUEUE structure:
 * head                         tail
 * newest -> entry -> entry -> oldest 
*/
/*TODO mabe use a ciclic buffer of fixed length*/

/*allocates space for a node and fills it with thread information*/
thread_queue_node* createnode(const pn_pthread_attr_t *attr,
		      void *(*start_routine) (void *), void *arg){
  thread_queue_node *node = malloc(sizeof(thread_queue_node));
  node->data = malloc(sizeof(thread_info));

  if (!node) {
    return NULL;
  }
  node->state = NODE_STATE_MAIDEN;
  node->data->attr = attr;
  node->data->start_routine = start_routine;
  node->data->arg = arg;
  node->next = NULL;
  return node;
}

thread_queue* init_queue(){
  thread_queue *queue = malloc(sizeof(thread_queue));
  queue->head = NULL;
  queue->tail = NULL;
  return queue;
}

thread_queue* enqueue(thread_queue_node *newNode, thread_queue* queue){
  if(newNode == NULL){
    return NULL;
  }else if(queue == NULL){
    return NULL;
  }else if(queue->tail != NULL){
    queue->tail->next = newNode; 
    queue->tail = newNode;
  }else{
    queue->head = newNode;
    queue->tail = newNode;
  }
  return queue;
}

/*deque retrive does not free space allocated for the thread! this is because a thread is dequed once a core starts to execute it!
call queue_free to free the space of a flaoting queue entry*/
thread_queue_node* dequeue(thread_queue * queue){   
  PN_CMSK core = 1 << pn_coreid();

  if(queue != NULL){
    thread_queue_node * current = queue->head;
    if(current == NULL){
      return NULL;
    }

    // test head as it must be handled differently
    PN_CMSK affinity;
    pn_pthread_getaffinity_np(current->data->attr, sizeof(PN_CMSK), &affinity);

    if( affinity == 0 || (affinity & core) > 0){
          queue->head = current->next;
          // if head == tail -> queue is about to be empty
          if(current->next == NULL){
            queue->tail = NULL;
          }
          return current;  
    }

    thread_queue_node * last = current;
    current = current->next;

    // if head is tail -> queue is about to be empty
    while(1){
      if(current != NULL){
        PN_CMSK affinity;
        pn_pthread_getaffinity_np(current->data->attr, sizeof(PN_CMSK), &affinity);
        if(affinity == 0 || (affinity & core) > 0){
          last->next = current->next;
          if(queue->tail == current){
            queue->tail = last;
          }
          return current;  
        }else{
          last = current;
          current = current->next;
        }
      }else{
        return NULL;
      }
     
    }
  }
  return NULL;  
}        

thread_queue_node* dequeue_node(thread_queue * queue, thread_queue_node * requested){   
  PN_CMSK core = 1 << pn_coreid();

  if(queue != NULL){
    thread_queue_node * current = queue->head;
    if(current == NULL){
      return NULL;
    }

    // test head as it must be handled differently
    PN_CMSK affinity;
    pn_pthread_getaffinity_np(current->data->attr, sizeof(PN_CMSK), &affinity);
    if(requested == current){
      if( affinity == 0 || (affinity & core) > 0){
          queue->head = current->next;
          // if head == tail -> queue is about to be empty
          if(current->next == NULL){
            queue->tail = NULL;
          }
          return current;  
      }else{
        return NULL;
      }
    }
    

    thread_queue_node * last = current;
    current = current->next;

    // if head is tail -> queue is about to be empty
    while(1){
      if(current != NULL){
        if(requested == current){
          PN_CMSK affinity;
          pn_pthread_getaffinity_np(current->data->attr, sizeof(PN_CMSK), &affinity);
          if(affinity == 0 || (affinity & core) > 0){
            last->next = current->next;
            if(queue->tail == current){
              queue->tail = last;
            }
            return current; 
          } else {
            return NULL;
          }
        }
        last = current;
        current = current->next;
      }else{
        return NULL;
      }
     
    }
  }
  return NULL;  
}

void destroy_queue(thread_queue * queue){
  thread_queue_node * current = queue->head;
  thread_queue_node * tmp;
  /* from queue pointer to start*/
  while(current != NULL){
    tmp = current->next;
    free(current);
    current = tmp;
  }
  free(queue);
}

/*be careful not to execute this on a already freed node!*/
void queue_free_node(thread_queue_node* node){
  free(node);
  return;
}

void print_queue(thread_queue* queue){
  thread_queue_node * current = queue->head;
  thread_queue_node * tmp;
  /* from queue pointer to start*/
  while(current != NULL){
    tmp = current->next;
    printf("%p\n", current );
    current = tmp;
  }
}
