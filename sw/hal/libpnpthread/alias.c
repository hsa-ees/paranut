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

#include <pthread.h>
#include <pn_pthread.h>
//  pn_pthread_base.c

int pthread_create(pthread_t *newthread, pthread_attr_t *attr,
          void *(*start_routine) (void *), void *arg){
  return pn_pthread_create((pn_pthread_t*)newthread, (pn_pthread_attr_t*)attr, start_routine, arg);
};

int pthread_join (pthread_t node, void **thread_return){
  return pn_pthread_join((pn_pthread_t)node, thread_return);
};

pthread_t pthread_self(){
  //TODO: return something sensefull
  return pn_pthread_self();
}

void pthread_exit(void *value){
  pn_pthread_exit(value);
};

int pthread_mutex_init (pthread_mutex_t *mutex,
		      const pthread_mutexattr_t *mutexattr)
{
  return pn_pthread_mutex_init ((pn_pthread_mutex_t *)mutex,
		       (pn_pthread_mutexattr_t *)mutexattr);
};

int pthread_mutex_lock (pthread_mutex_t *mutex)
{
  return pn_pthread_mutex_lock ((pn_pthread_mutex_t*)mutex);
}

int pthread_mutex_unlock (pthread_mutex_t *mutex)
{
  return pn_pthread_mutex_unlock ((pn_pthread_mutex_t*)mutex);
}

int pthread_mutex_destroy (pthread_mutex_t *mutex){
  return pn_pthread_mutex_destroy ((pn_pthread_mutex_t *)mutex);
}

//  pn_pthread_attr.c

int pthread_attr_init (pthread_attr_t *attr)
{
  return pn_pthread_attr_init ((pn_pthread_attr_t *)attr);
};

int pthread_attr_setaffinity_np(const pthread_attr_t *attr, size_t cpusetsize,
        cpu_set_t *cpuset){
  return pn_pthread_setaffinity_np((pn_pthread_attr_t *)attr, cpusetsize, cpuset);
}

int pthread_attr_getaffinity_np(const pthread_attr_t *attr, size_t cpusetsize,
        cpu_set_t *cpuset){
  return pn_pthread_getaffinity_np((pn_pthread_attr_t *)attr, cpusetsize, cpuset);
}

int pthread_attr_getstacksize(pthread_attr_t *attr, size_t *stacksize){
  return pn_pthread_attr_getstacksize((pn_pthread_attr_t *)attr, stacksize);
}

int pthread_attr_destroy (pthread_attr_t *attr){
  return pn_pthread_attr_destroy ((pn_pthread_attr_t *)attr);
}

int pthread_equal(pthread_t t1, pthread_t t2){
    return pn_pthread_equal((pn_pthread_t) t1, (pn_pthread_t) t2);
}

int pthread_once(pthread_once_t *once_control, void (*init_routine)(void)){
  return pn_pthread_once((pn_pthread_once_t *)once_control, init_routine);
}

// CPU set mechanism

int CPU_ZERO(cpu_set_t *cpuset){
  *cpuset = 0;
  
  return 0;
}

int CPU_SET(int id, cpu_set_t  *cpuset){
  *cpuset = *cpuset | (1<<id);
  return 0;
}

//NOT IMPLEMENTED
int pthread_attr_setdetachstate(pthread_attr_t *attr, int state) {
  return pn_pthread_attr_setdetachstate(attr, state);
};
int pthread_attr_setinheritsched(pthread_attr_t *attr, int sched) {
  return pn_pthread_attr_setinheritsched(attr, sched);
};
int pthread_attr_setschedparam(pthread_attr_t *attr, sched_param *param) {
  return pn_pthread_attr_setschedparam(attr, (pn_sched_param *)param);
};
int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy) {
  return pn_pthread_attr_setschedpolicy(attr, policy);
};
int pthread_attr_setstacksize(pthread_attr_t *attr , size_t size) {
  return pn_pthread_attr_setstacksize(attr , size);
};

int pthread_barrier_init(pthread_barrier_t *restrict barrier,const pthread_barrierattr_t *restrict attr, unsigned count) {
  return pn_pthread_barrier_init(barrier, attr, count);
};
int pthread_barrier_destroy(pthread_barrier_t *barrier) {
  return pn_pthread_barrier_destroy(barrier);
};
int pthread_barrier_wait(pthread_barrier_t *barrier) {
  return pn_pthread_barrier_wait(barrier);
};

int pthread_detach(pthread_t thread) {
  return pn_pthread_detach(thread);
};
int pthread_getspecific(pthread_key_t key) {
  return pn_pthread_getspecific(key);
};
int pthread_key_create(pthread_key_t *key , void (*desctructor)(void *)) {
  return pn_pthread_key_create(key , desctructor);
};

int pthread_mutexattr_getprotocol(const pthread_mutexattr_t *mutexattr, int *proto) {
  return pn_pthread_mutexattr_getprotocol(mutexattr, proto);
};
int pthread_mutexattr_setprioceiling(pthread_mutexattr_t *mutexattr, int ceiling) {
  return pn_pthread_mutexattr_setprioceiling(mutexattr, ceiling);
};
int pthread_mutexattr_setprotocol(pthread_mutexattr_t *mutexattr, int proto) {
  return pn_pthread_mutexattr_setprotocol(mutexattr, proto);
};
int pthread_mutexattr_setpshared(pthread_mutexattr_t *mutexattr, int shared) {
  return pn_pthread_mutexattr_setpshared(mutexattr, shared);
};

int pthread_setcancelstate(int state, int *oldstate) {
  return pn_pthread_setcancelstate(state, oldstate);
};
int pthread_setcanceltype(int type, int *oldtype) {
  return pn_pthread_setcanceltype(type, oldtype);
};
int pthread_setspecific(pthread_key_t key, const void *value) {
  return pn_pthread_setspecific(key, value);
};
int pthread_testcancel() {
  return pn_pthread_testcancel();
};