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

#ifndef PN_PTHREAD_H
#define PN_PTHREAD_H

/**
 * \file
 * \brief      pthread API.
 */

/** 
 * \mainpage POSIX Threads Documentation
 * 
 * \section ParaNut pthread Library Documentation
 * This is a library intending to provide a POSIX-compatoble threading implementation.
 * It is intended to be used instead of libparanut and provide a layer of abstraction and additional functionalities, such as thread queueing.
 * It is does not yet provide all functions and types associated with pthreads. It is however already implementing all main features:
 * Thread creation, Thread joining, Mutexes, and some Threading attributes. 
 * However, the state of this library should be considered incomplete, untested and not fit for production.
 * note, that pn_pthread.h should be included before <stdio.h>, <stdlib.h> or any other library that subsequently includes pthread_types.
 * As this library redefines those, it must be present befor in order to prevent standard definitions to be loaded.
*/

#include "paranut.h"
#include "pn_pthread_types.h"
#include <stdio.h>

/**
 * \defgroup   pthread pthread
 *
 * \brief      Functions for accessing ParaNut parallelization
 *
 * This Module contains all currently supported Posix Thread compatible function calls. For further information and example usages 
 * consider the official POSIX documentation.
 * Additionally information about additional functionality, partial implementations and deviations from the standard are supplied. 
 */
 
/**
 * \addtogroup pthread
 * @{
 */

/**
 * \fn         int pn_pthread_create(pn_pthread_t *__restrict __newthread, pn_pthread_attr_t *__restrict __attr,
 *         void *(*__start_routine) (void *), void *__restrict __arg)
 * \brief      create a new thread
 * 
 * Cannot be called from CoPU.
 * 
 * \return     Non-zero value in case of error
 */
int pn_pthread_create(pn_pthread_t *__restrict __newthread, pn_pthread_attr_t *__restrict __attr,
          void *(*__start_routine) (void *), void *__restrict __arg);


/**
 * \fn         int pn_pthread_start_linked(pn_pthread_t *newthread, pn_pthread_attr_t *attr,
 *		      void *(*start_routine) (void *), void *arg, PN_NUMC numcores
 * \brief     start a specified function in linked mode.
 * 
 * Cannot be called from CoPU.
 * 
 * \return     Non-zero value in case of error
 */
int pn_pthread_start_linked(pn_pthread_t *newthread, pn_pthread_attr_t *attr,
		      void (*start_routine)(void *args, PN_CID cid), void *arg, PN_NUMC numcores);

/**
 * \fn         pn_pthread_join (pn_pthread_t __th, void **__thread_return)
 * \brief      suspends execution until specified thread is finished.
 * 
 * The pn_pthread_join() function suspends execution of the calling thread until the target thread terminates, 
 * unless the target thread has already terminated. On return from a successful pn_pthread_join() 
 * call with a non-NULL value_ptr argument, the value passed to pn_pthread_exit() by the terminating thread is made available through __thread_return.
 * It internally creates a queue which is executed in a fifo manner. On ParaNut pn_thread_create is used to enable SMT parallelization.
 * Additional threading options may be supplied by providing pn_pthread_attr_t (see \ref pn_pthread_attr_init)
 * 
 * \return     Non-zero value in case of error
 */
int pn_pthread_join (pn_pthread_t __th, void **__thread_return);

pn_pthread_t pn_pthread_self();

/**
 * \fn         pn_pthread_exit(void *value)
 * \brief      suspends execution until specified thread is finished.
 * 
 * he pn_pthread_exit() function terminates the calling thread and makes the value 
 * __thread_return available to any successful join with the terminating thread
 * 
 * \return     The pn_pthread_exit() function cannot return to its caller. 
 */
void pn_pthread_exit(void *value);


/**
 * \fn         pn_pthread_mutex_init (pn_pthread_mutex_t *mutex,
		      const pn_pthread_mutexattr_t *mutexattr)
 * \brief      initializes a mutex
 * 
 * The pn_pthread_mutex_init() function initialises the mutex referenced by mutex.
 * ATTETION: setting mutexattr does not have any effect as of yet.
 * \todo implement mutexattr 
 * 
 * \return     Non-zero value in case of error 
 */
int pn_pthread_mutex_init (pn_pthread_mutex_t *mutex,
		      const pn_pthread_mutexattr_t *mutexattr);

#define PN_PTHREAD_MUTEX_INITIALIZER {0}

/**
 * \fn         pn_pthread_mutex_lock (pn_pthread_mutex_t *mutex)
 * \brief      locks a mutex
 * 
 * The mutex object referenced by mutex is locked by calling pn_pthread_mutex_lock(). 
 * If the mutex is already locked, the calling thread blocks until the mutex becomes available. 
 * This operation returns with the mutex object referenced by mutex in the locked state with the calling thread as its owner. 
 * 
 * \return     Non-zero value in case of error 
 */
int pn_pthread_mutex_lock (pn_pthread_mutex_t *mutex);

/**
 * \fn         pn_pthread_mutex_unlock (pn_pthread_mutex_t *mutex)
 * \brief      unlocks a mutex
 * 
 * The pn_pthread_mutex_unlock() function releases the mutex object referenced by mutex.
 * 
 * \return     Non-zero value in case of error 
 */
int pn_pthread_mutex_unlock (pn_pthread_mutex_t *mutex);

/**
 * \fn         pn_pthread_mutex_unlock (pn_pthread_mutex_t *mutex)
 * \brief      unlocks a mutex
 * 
 * The pn_pthread_mutex_unlock() function releases the mutex object referenced by mutex.
 * 
 * \return     Non-zero value in case of error 
 */
int	pn_pthread_mutex_destroy (pn_pthread_mutex_t *mutex);

/**
	CONDITIONALS
*/
#define PN_PTHREAD_COND_INITIALIZER {1}


/**
 * \fn         pn_pthread_cond_broadcast(pn_pthread_cond_t *)
 * \brief      change condition state fo all waiting threads

 * \return     Non-zero value in case of error 
 */
int	pn_pthread_cond_broadcast(pn_pthread_cond_t *);

/**
 * \fn         pn_pthread_cond_destroy(pn_pthread_cond_t *)
 * \brief      release the condition variable
 * 
 * \return     Non-zero value in case of error 
 */
int	pn_pthread_cond_destroy(pn_pthread_cond_t *);

/**
 * \fn         pn_pthread_cond_init(pn_pthread_cond_t *, const pn_pthread_condattr_t *)
 * \brief      initialize a condition variable
 * 
 * Currently the condition attribute can not be used to make a configuration
 * 
 * \return     Non-zero value in case of error 
 */
int	pn_pthread_cond_init(pn_pthread_cond_t *, const pn_pthread_condattr_t *);

/**
 * \fn         pn_pthread_cond_signal(pn_pthread_cond_t *)
 * \brief      signal a single waiting thread
 * 
 * \return     Non-zero value in case of error 
 */
int	pn_pthread_cond_signal(pn_pthread_cond_t *);

/**
 * \fn         pn_pthread_cond_wait(pn_pthread_cond_t *, pn_pthread_mutex_t *)
 * \brief      wait for a condition signal
 * 
 * \return     Non-zero value in case of error 
 */
int	pn_pthread_cond_wait(pn_pthread_cond_t *, pn_pthread_mutex_t *);



/**
 * \fn         pn_pthread_attr_init (pn_pthread_attr_t *__attr)
 * \brief      initializes a thread attributes object
 * 
 * The function pn_pthread_attr_init() initialises a thread attributes object attr with the default value 
 * for all of the individual attributes used by a given implementation. 
 * 
 * \return     Non-zero value in case of error 
 */
int pn_pthread_attr_init (pn_pthread_attr_t *__attr) __THROW __nonnull ((1));

/**
 * \fn         pn_pthread_attr_destroy (pn_pthread_attr_t *__attr)
 * \brief      destroys a thread attributes object
 * 
 * The pn_pthread_attr_destroy() function is used to destroy a thread attributes object. 
 * An implementation may cause pn_pthread_attr_destroy() to set attr to an implementation-dependent invalid value. The behaviour of using the attribute after it has been destroyed is undefined. 
 * \return     Non-zero value in case of error 
 */
int pn_pthread_attr_destroy (pn_pthread_attr_t *__attr) __THROW __nonnull ((1));

int pn_pthread_attr_getstacksize(pn_pthread_attr_t *attr, size_t *stacksize);

/**
 * \fn         pn_pthread_attr_setpnaffinity (pn_pthread_attr_t *attr, PN_CMSK coremask)
 * \brief      sets pn affinity on thread attribues object
 * 
 * This function is specific to ParaNut. It is used to set a coremask which defines a list of ParaNut cores that may be used to execute the thread.
 * It accepts a bitmask in the form of PN_CMSK. Each set bit declares a core to be used by the thread.
 * E.g.: coremask = 6 -> 00000110 -> The created thread will be executed by core 2 or 4
 * Notice: bit 0 (CePU) must not be on (value of coremask must not be uneven), obviously only available mode 2 cores can be used.
 * If PN_CMSK is NULL, all CoPUs capable of Mode 2 are set to 1. 
 * In the queueing model, a core is halted, as soon as no waiting queue entries mention it in their affinity settings. 
 * 
 *  * \return     Non-zero value in case of error 
 */
int pn_pthread_setaffinity_np (pn_pthread_attr_t *attr, size_t cpusetsize,
			       const cpu_set_t *cpus);

/**
 * \fn         pn_pthread_attr_getpnaffinity (const pn_pthread_attr_t *attr)
 * \brief      gets pn affinity from thread attribues object
 * 
 * This function is specific to ParaNut. It returns the pnaffinity attribute stored in the thread attributes object. 
 * See PN_CMSK in libparanut documentation info on how to interpret.
 * 
 *  * \return     Non-zero value in case of error 
 */
int pn_pthread_getaffinity_np(const pn_pthread_attr_t *attr, size_t cpusetsize,
        cpu_set_t *cpuset);
/*TODO pn_pthread_exit*/

/**
 * \fn         pn_pthread_equal(pn_pthread_t t1, pn_pthread_t t2)
 * \brief      checks whether the threads are identical
 * 
 * This function takes two threads as an input, it returns 1 if both are equal, 0 if not.
 * 
 *  * \return     int 0 or 1 
 * */
int pn_pthread_equal(pn_pthread_t t1, pn_pthread_t t2);

/**
 * \fn         pn_pthread_once(pn_pthread_once_t *once_control, void (*init_routine)(void))
 * \brief      makes sure "init_routine" is only executed once
 * 
 * This function ensures, that the given function is only executed once. The first thread will execute the 
 * function indicated by init_routine. All other thread will skip the execution.
 * 
 *  * \return     non-
 * */
#define PTHREAD_ONCE_INIT 0
#define PTHREAD_ONCE_INPROGRESS	1
#define PTHREAD_ONCE_DONE 2
int pn_pthread_once(pn_pthread_once_t *once_control, void (*init_routine)(void));

// NOT IMPLEMENTED
typedef int pn_pthread_barrier_t;
typedef int pn_pthread_barrierattr_t;
typedef int pn_pthread_key_t;
typedef struct _pn_sched_param pn_sched_param;

struct _pn_sched_param
{
  int sched;
};

int pn_pthread_attr_setdetachstate(pn_pthread_attr_t *attr, int state);
int pn_pthread_attr_setinheritsched(pn_pthread_attr_t *attr, int sched);
int pn_pthread_attr_setschedparam(pn_pthread_attr_t *attr, pn_sched_param *param);
int pn_pthread_attr_setschedpolicy(pn_pthread_attr_t *attr, int policy);
int pn_pthread_attr_setstacksize(pn_pthread_attr_t *attr, size_t size);

int pn_pthread_barrier_init(pn_pthread_barrier_t *restrict barrier,const pn_pthread_barrierattr_t *restrict attr, unsigned count);
int pn_pthread_barrier_destroy(pn_pthread_barrier_t *barrier);
int pn_pthread_barrier_wait(pn_pthread_barrier_t *barrier);

int pn_pthread_detach(pn_pthread_t thread);
int pn_pthread_getspecific(pn_pthread_key_t key);
int pn_pthread_key_create(pn_pthread_key_t *key , void (*desctructor)(void *));

int pn_pthread_mutexattr_getprotocol(const pn_pthread_mutexattr_t *mutexattr, int *proto);
int pn_pthread_mutexattr_setprioceiling(pn_pthread_mutexattr_t *mutexattr, int ceiling);
int pn_pthread_mutexattr_setprotocol(pn_pthread_mutexattr_t *mutexattr, int proto);
int pn_pthread_mutexattr_setpshared(pn_pthread_mutexattr_t *mutexattr, int shared);

int pn_pthread_setcancelstate(int state, int *oldstate);
int pn_pthread_setcanceltype(int type, int *oldtype);
int pn_pthread_setspecific(pn_pthread_key_t key, const void *value);
int pn_pthread_testcancel();




#endif