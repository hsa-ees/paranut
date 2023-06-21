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

#ifndef PN_PTHREAD_QUEUE_H
#define PN_PTHREAD_QUEUE_H	1
#include "pn_pthread_types.h"

/* DEFINES: ***********************************************************/
# define NODE_STATE_MAIDEN 0
# define NODE_STATE_ACTIVE 1
# define NODE_STATE_DONE 2

/* QUEUE DATATYPES: ***********************************************************/
typedef struct _thread_queue_node thread_queue_node;
typedef struct _thread_queue thread_queue;
typedef struct _thread_info thread_info;
typedef int nodestate;

struct _thread_info{
    const pn_pthread_attr_t *attr;    
    void *(*start_routine) (void *);
    void *arg;
};

struct _thread_queue_node{
   nodestate state;
   thread_info *data;
   void **thread_return;
   thread_queue_node *next;
   PN_CMSK coremask;
};

struct _thread_queue{
   thread_queue_node *head;
   thread_queue_node *tail;
};


/* QUEUE FUNCTIONS: ***********************************************************/
thread_queue_node* createnode(const pn_pthread_attr_t *attr,
		      void *(*start_routine) (void *), void *arg);

thread_queue* enqueue(thread_queue_node *newNode, thread_queue *queue);

thread_queue_node* dequeue(thread_queue *queue); 

thread_queue_node* dequeue_node(thread_queue * queue, thread_queue_node * requested);

void queue_free_node(thread_queue_node* node);

void destroy_queue(thread_queue *queue);

void print_queue(thread_queue* queue);

thread_queue* init_queue();
#endif