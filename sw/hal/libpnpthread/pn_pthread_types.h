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
#ifndef PN_PTHREADTYPES_H
#define PN_PTHREADTYPES_H
#include <stdio.h>


/* TODO: this is dependant on Word Size -> configurable! */
# define __SIZEOF_PN_PTHREAD_MUTEX_T 24
# define __SIZEOF_PN_PTHREAD_ATTR_T 64
# define __SIZEOF_PN_PTHREAD_MUTEX_T 24
# define __SIZEOF_PN_PTHREAD_MUTEXATTR_T 4
# define __SIZEOF_PN_PTHREAD_RWLOCK_T 32
# define __SIZEOF_PN_PTHREAD_BARRIER_T 20

/* FORWARD DECLARATIONS ***********************************************************/
struct _thread_queue_node;
struct _thread_queue;

struct _pn_sched_param;
struct _pn_pthread_attr_t;

struct _pn_pthread_cond_t;

/* TYPES ***************************************************************************/

typedef struct _pn_sched_param pn_sched_param;

/**
 * \typedef    pn_pthread_t
 * \brief      Used to describe a thread
 */
typedef struct _thread_queue_node* pn_pthread_t;

/**
 * \typedef    pn_pthread_mutex_t
 * \brief      Used to describe a mutex
 */
typedef struct __pn_spinlock pn_pthread_mutex_t;

typedef struct _pn_pthread_cond_t pn_pthread_cond_t;

typedef int pn_pthread_once_t;

struct _pn_pthread_cond_t
{
  PN_CMSK signaling;
  PN_CMSK waiting;
};

typedef int pn_pthread_condattr_t;


/* opaque attributes */
/**
 * \typedef    pn_pthread_attr_t
 * \brief      Used to provide threading attributes
 */
typedef union 
{
  char __size[__SIZEOF_PN_PTHREAD_ATTR_T];
  long int __align;
} pn_pthread_attr_t;

typedef union
{
  char __size[__SIZEOF_PN_PTHREAD_MUTEXATTR_T];
  int __align;
} pn_pthread_mutexattr_t;

typedef PN_CMSK cpu_set_t;

#endif
/* TODO: add mutex initialiyer constatnt*/