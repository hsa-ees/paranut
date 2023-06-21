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
#include "pn_pthread_attr.h"
#include <stdint.h>
#include <string.h>
#include <encoding.h>
#include <stdlib.h>
#include <time.h>

/* DEFINES:
----------------------------------------------------------------------------- */
#define BITMASK 0xFF
#define DATA_SIZE 3000
#define MAX_CORES 16 // must be adopted if the ParaNut is ever to exceed 16 mode 2 cores

#define _CLKS_PER_MSEC (read_csr(0xFC6)/1000)

/*Commonly Used Assembly Functions*********************************************/
extern PN_CMSK       read_PNCE_as(void);
/******************************************************************************/


/* GLOBAL VARIABLES:
 ------------------------------------------------------------------------------ */
/* QUEUEING:
-------------------------------------------------------------------------------
a pseudorandomly ordered queue provides means to store threading taskes to be 
executed. Enabled CoPUs are responsible for polling new threading tasks as the
CePU can not keep track of those. However new tasks can only be added by the 
CePU (Theoretically CoPUs could create ones as well, but POSIX does not expect 
the ability to create threads from within threads) */
/* corecount holds the number of queued threads that mention the thread. 
corecount[n] refers to the pn_coreid. This means, corecound [0] is always NULL.
This is not efficient, but readable */

int corecount[MAX_CORES] = {0}; 
_pn_spinlock corecount_mutex;

thread_queue *queue = NULL;
_pn_spinlock queue_mutex;

PN_CMSK available_cores = 0; /* stores cores enabled for functional multithreading*/


int increase_corecount(PN_CMSK coremask){
  pn_spinlock_lock(&corecount_mutex);
  coremask = coremask & available_cores; // ignore cores that are mentioned but not available
  for (int i = 0; i<MAX_CORES; i++){
    if (coremask & (1<<i)){
      corecount[i]++;
    }
  }
  pn_spinlock_unlock(&corecount_mutex);
  return 0;
}

int decrease_corecount(PN_CMSK coremask){
  pn_spinlock_lock(&corecount_mutex);
  coremask = coremask & available_cores; // ignore cores that are mentioned but not available
  for (int i = 0; i<MAX_CORES; i++){
    if (coremask & (1<<i)){
      corecount[i]--;
    }
  }
  pn_spinlock_unlock(&corecount_mutex);
  return 0;
}

PN_CMSK create_active_coremask(){
  PN_CMSK coremask = 0;
  pn_spinlock_lock(&corecount_mutex);
  for (int i = 1; i<MAX_CORES; i++){
    if(corecount[i] > 0){
      coremask = coremask | (1 << i);
    }
    
  }
  coremask = coremask | 1; // CePU must be set
  pn_spinlock_unlock(&corecount_mutex);
  return coremask;
}

void run_CoPU(){
  while(1){
    pn_spinlock_lock(&queue_mutex);  
    thread_queue_node *node = dequeue(queue);
    if(node != NULL){
      node->state = NODE_STATE_ACTIVE;
    } else {
      pn_spinlock_unlock(&queue_mutex);
      pn_halt();
    }
    pn_spinlock_unlock(&queue_mutex);
    if(node != NULL){
      /*TODO Thread return*/
      node->thread_return = node->data->start_routine(node->data->arg);
      node->state = NODE_STATE_DONE;
      PN_CMSK coremask;
      pn_pthread_getaffinity_np(node->data->attr, sizeof(PN_CMSK), &coremask);
      decrease_corecount(coremask);
      PN_CMSK activeCores = create_active_coremask();
      if(!(pn_coreid() & activeCores)){
         pn_halt();
      }
    }
  }
}

/* Only mode 2 CoPUs are capable of running a thread, CePU is in charge of managing */
int pn_pthread_create(pn_pthread_t *newthread, pn_pthread_attr_t *attr,
		      void *(*start_routine) (void *), void *arg)
{
   /* init global vars if not already happened*/
  if(available_cores == 0){
    available_cores = pn_m2cap();
    pn_spinlock_init(&queue_mutex);
    pn_spinlock_init(&corecount_mutex);
    queue = init_queue();
  };


  /* create node from thread data */
  thread_queue_node* node = createnode(attr, start_routine, arg);
  *newthread = node;

  pn_spinlock_lock(&queue_mutex);
  enqueue(node, queue);
  pn_spinlock_unlock(&queue_mutex);
  /* create core-bitmask for selective thread creation*/
  PN_CMSK coremask; 
  pn_pthread_getaffinity_np(attr, sizeof(PN_CMSK), &coremask);
  if (coremask == 0){
    coremask = available_cores;
  }
  increase_corecount(coremask);
  // increase_corecount(coremask);
  if(pn_run_threaded_m (create_active_coremask(), *run_CoPU, NULL) < 0){
    /* return -1 on error */
    return -1;
  };
  
  return 0;
}

int pn_pthread_join (pn_pthread_t node, void **thread_return)
{
  if(node == NULL){
    return -1;
  }
  /* check whether running in CePU */
  int cid = pn_coreid();
  if (cid == 0){
    /* wait for  */
    thread_queue_node *execnode = NULL;
    pn_spinlock_lock(&queue_mutex);
    nodestate state = node->state;
    
    if(state == NODE_STATE_MAIDEN){
      execnode = dequeue_node(queue, node);
      if(execnode != NULL){
        node->state = NODE_STATE_ACTIVE;
      }
    }
    pn_spinlock_unlock(&queue_mutex);

    if(execnode != NULL){
      node->thread_return = node->data->start_routine(node->data->arg);
      node->state = NODE_STATE_DONE;

      PN_CMSK coremask;
      pn_pthread_getaffinity_np(node->data->attr, sizeof(PN_CMSK), &coremask);
      decrease_corecount(coremask);
    } else {
      while (state != NODE_STATE_DONE) {
        pn_spinlock_lock(&queue_mutex);
        state = node->state;
        pn_spinlock_unlock(&queue_mutex);
        /*do i need to wait to give other cores a chance to lock the mutex?*/
      }
    }    

    pn_spinlock_lock(&queue_mutex);
    if(thread_return != NULL){
      *thread_return = node->thread_return;
    }
    queue_free_node(node);
    pn_spinlock_unlock(&queue_mutex);
  }
  return 0;
}

pn_pthread_t pn_pthread_self(){
  //TODO: return something sensefull
  return NULL;
}

void pn_pthread_exit(void *value){
  // TODO: make return value
  return;
}

int pn_pthread_mutex_init (pn_pthread_mutex_t *mutex,
		      const pn_pthread_mutexattr_t *mutexattr)
{
  pn_spinlock_init((_pn_spinlock*)mutex);
  return 0;
}

int pn_pthread_mutex_lock (pn_pthread_mutex_t *mutex)
{
  pn_spinlock_lock((_pn_spinlock*)mutex);
  return 0;
}

int pn_pthread_mutex_unlock (pn_pthread_mutex_t *mutex)
{
  pn_spinlock_unlock((_pn_spinlock*)mutex);
  return 0;
}

int pn_pthread_mutex_destroy (pn_pthread_mutex_t *mutex){
  pn_spinlock_destroy((_pn_spinlock*)mutex);
  return 0;
}

// CONDITIONALS

int	pn_pthread_cond_broadcast(pn_pthread_cond_t *condition){
  condition->signaling = condition->waiting;
  return 0;
}

int	pn_pthread_cond_destroy(pn_pthread_cond_t *condition){
  free(condition);
  return -1;
}

int	pn_pthread_cond_init(pn_pthread_cond_t *condition, const pn_pthread_condattr_t *conditionattr){
  condition->signaling = 0;
  condition->waiting = 0;
  return 0;
}

int	pn_pthread_cond_signal(pn_pthread_cond_t *condition){
  PN_CMSK signaledCore = 1;
  while(!(condition->waiting & signaledCore)){
    signaledCore = signaledCore << 1;
    if((signaledCore & (1 << MAX_CORES))){
      return -1;
    }
  }
  condition->signaling |= signaledCore;
  return 0;
}

int	pn_pthread_cond_wait(pn_pthread_cond_t *condition, pn_pthread_mutex_t *mutex){
  condition->waiting = (PN_CMSK)(condition->waiting | pn_coreid());
  while( !(condition->signaling & pn_coreid())){
    pn_pthread_mutex_unlock(mutex);
    pn_pthread_mutex_lock(mutex);
  }
  condition->signaling = condition->signaling ^ pn_coreid();
  condition->waiting = condition->waiting ^ pn_coreid();
  return 0;
}

int init_pthread(){
     /* init global vars if not already happened*/
  if(available_cores == 0){
    available_cores = pn_m2cap();
    pn_spinlock_init(&queue_mutex);
    pn_spinlock_init(&corecount_mutex);
    queue = init_queue();
  };
  return 0;

}

/* used to start linked mode using the pthread library.*/
/* must be ccalled from CePU, other cores may be running in threaded mode.
 The requested number of cores will be allocated for linked mode. If this number can not be allocated, returns an error.*/
int pn_pthread_start_linked(pn_pthread_t *newthread, pn_pthread_attr_t *attr,
		      void (*start_routine)(void *args, PN_CID cid), void *arg, PN_NUMC numcores)
{


  init_pthread();

  /* create core-bitmask for selective thread creation*/
  PN_CMSK  coremask = 0b11;
  for (int i = 2; i < numcores; i++) coremask |= (1 << i);
  // pn_pthread_getaffinity_np(attr, sizeof(PN_CMSK), &coremask);
  // if (coremask == 0){
  //   coremask = available_cores;
  // }
  // increase_corecount(coremask);
  if(pn_run_linked_m (coremask, start_routine, arg) < 0){
    /* return -1 on error */
    return -1;
  };
  
  return 0;
}

int pn_pthread_equal(pn_pthread_t t1, pn_pthread_t t2){
  if (t1 == t2){
    return 1;
  }else{
    return 0;
  }
}


int pn_pthread_once(pn_pthread_once_t *once_control, void (*init_routine)(void)){
  if(*once_control>PTHREAD_ONCE_INIT){
    return 0;
  }else{
    *once_control=PTHREAD_ONCE_INPROGRESS;
    init_routine();
    *once_control=PTHREAD_ONCE_DONE;
    return 0;
  }
}

int pn_pthread_attr_setdetachstate(pn_pthread_attr_t *attr, int state) {return -1;};
int pn_pthread_attr_setinheritsched(pn_pthread_attr_t *attr, int sched) {return -1;};
int pn_pthread_attr_setschedparam(pn_pthread_attr_t *attr, pn_sched_param *param) {return -1;};
int pn_pthread_attr_setschedpolicy(pn_pthread_attr_t *attr, int policy) {return -1;};
int pn_pthread_attr_setstacksize(pn_pthread_attr_t *attr , size_t size) {return -1;};

int pn_pthread_barrier_init(pn_pthread_barrier_t *restrict barrier,const pn_pthread_barrierattr_t *restrict attr, unsigned count) {return -1;};
int pn_pthread_barrier_destroy(pn_pthread_barrier_t *barrier) {return -1;};
int pn_pthread_barrier_wait(pn_pthread_barrier_t *barrier) {return -1;};

int pn_pthread_detach(pn_pthread_t thread) {return -1;};
int pn_pthread_getspecific(pn_pthread_key_t key) {return -1;};
int pn_pthread_key_create(pn_pthread_key_t *key , void (*desctructor)(void *)) {return -1;};

int pn_pthread_mutexattr_getprotocol(const pn_pthread_mutexattr_t *mutexattr, int *proto) {return -1;};
int pn_pthread_mutexattr_setprioceiling(pn_pthread_mutexattr_t *mutexattr, int ceiling) {return -1;};
int pn_pthread_mutexattr_setprotocol(pn_pthread_mutexattr_t *mutexattr, int proto) {return -1;};
int pn_pthread_mutexattr_setpshared(pn_pthread_mutexattr_t *mutexattr, int shared) {return -1;};

int pn_pthread_setcancelstate(int state, int *oldstate) {return -1;};
int pn_pthread_setcanceltype(int type, int *oldtype) {return -1;};
int pn_pthread_setspecific(pn_pthread_key_t key, const void *value) {return -1;};
int pn_pthread_testcancel() {return -1;};
