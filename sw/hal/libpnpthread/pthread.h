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

#ifndef PTHREAD_H
#define PTHREAD_H
#ifndef _SYS__PTHREADTYPES_H_
#define	_SYS__PTHREADTYPES_H_
#define _BITS_PTHREADTYPES_COMMON_H
// types
#include "pn_pthread.h"
/* TODO: this is dependant on Word Size -> configurable! */
# define __SIZEOF_PTHREAD_MUTEX_T __SIZEOF_PN_PTHREAD_MUTEX_T
# define __SIZEOF_PTHREAD_ATTR_T __SIZEOF_PN_PTHREAD_ATTR_T
# define __SIZEOF_PTHREAD_MUTEXATTR_T __SIZEOF_PN_PTHREAD_MUTEXATTR_T
# define __SIZEOF_PTHREAD_RWLOCK_T __SIZEOF_PN_PTHREAD_RWLOCK_T
# define __SIZEOF_PTHREAD_BARRIER_T __SIZEOF_PN_PTHREAD_BARRIER_T

typedef pn_pthread_t pthread_t;

typedef pn_pthread_mutex_t pthread_mutex_t;

typedef pn_pthread_attr_t pthread_attr_t;

typedef pn_pthread_mutexattr_t pthread_mutexattr_t;

typedef pn_pthread_once_t pthread_once_t;

// functions

int pthread_create(pthread_t *__restrict __newthread, pthread_attr_t *__restrict __attr,
          void *(*__start_routine) (void *), void *__restrict __arg);

int pthread_join (pthread_t __th, void **__thread_return);

pthread_t pthread_self();

void pthread_exit(void *value);

int pthread_mutex_init (pthread_mutex_t *mutex,
		      const pthread_mutexattr_t *mutexattr);

#define PTHREAD_MUTEX_INITIALIZER ((pthread_mutex_t) PN_PTHREAD_MUTEX_INITIALIZER)

int pthread_mutex_lock (pthread_mutex_t *mutex);

int pthread_mutex_unlock (pthread_mutex_t *mutex);

int pthread_mutex_destroy (pthread_mutex_t *mutex);

int pthread_attr_init (pthread_attr_t *__attr) __THROW __nonnull ((1));

int pthread_attr_destroy (pthread_attr_t *__attr) __THROW __nonnull ((1));

int pthread_attr_setaffinity_np (const pthread_attr_t *attr, size_t cpusetsize,
        cpu_set_t *cpuset);

int pthread_attr_getaffinity_np (const pthread_attr_t *attr, size_t cpusetsize,
        cpu_set_t *cpuset);

int pthread_attr_getstacksize(pthread_attr_t *attr, size_t *stacksize);

int pthread_equal(pthread_t t1, pthread_t t2);

int pthread_once(pthread_once_t *once_control, void (*init_routine)(void));

// cpuset
int CPU_ZERO(cpu_set_t *cpuset);

int CPU_SET(int id, cpu_set_t  *cpuset);

// NOT IMPLEMENTED
typedef int pthread_barrier_t;
typedef int pthread_barrierattr_t;
typedef int pthread_key_t;
typedef pn_sched_param sched_param;

int pthread_attr_setdetachstate(pthread_attr_t *attr, int state);
int pthread_attr_setinheritsched(pthread_attr_t *attr, int sched);
int pthread_attr_setschedparam(pthread_attr_t *attr, sched_param *param);
int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy);
int pthread_attr_setstacksize(pthread_attr_t *, size_t);

int pthread_barrier_init(pthread_barrier_t *restrict barrier,const pthread_barrierattr_t *restrict attr, unsigned count);
int pthread_barrier_destroy(pthread_barrier_t *barrier);
int pthread_barrier_wait(pthread_barrier_t *barrier);

int pthread_detach(pthread_t thread);
int pthread_getspecific(pthread_key_t key);
int pthread_key_create(pthread_key_t *, void (*)(void *));

int pthread_mutexattr_getprotocol(const pthread_mutexattr_t *, int *);
int pthread_mutexattr_setprioceiling(pthread_mutexattr_t *, int);
int pthread_mutexattr_setprotocol(pthread_mutexattr_t *, int);
int pthread_mutexattr_setpshared(pthread_mutexattr_t *, int);

int pthread_setcancelstate(int, int *);
int pthread_setcanceltype(int, int *);
int pthread_setspecific(pthread_key_t, const void *);
int pthread_testcancel(void);


#endif
#endif