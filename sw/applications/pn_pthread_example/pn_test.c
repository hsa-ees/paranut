/*
 * This file is part of the ParaNut project
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
 * 
 * Date: 30.01.2022
 */

#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

// a simple pthread example 
// compile with -lpnpthreads

static pn_pthread_mutex_t printf_mutex;

void *thread(void *n)
{
    float i = 2;
    float res = 321.21;
    int cid = pn_coreid();
    pthread_mutex_lock(&printf_mutex);
    printf ("%2i. Hello World from %d !\n", (int)n, cid);
    pthread_mutex_unlock(&printf_mutex);
    return 0;
}

int main () {
    int n;
    pthread_t threads[10];
    pthread_attr_t tattr[10];

    cpu_set_t one, two, onetwo, onethree, all;
    CPU_ZERO(&one);
    CPU_SET(1, &one);

    CPU_ZERO(&two);
    CPU_SET(2, &two);

    CPU_ZERO(&all);
    CPU_SET(0, &all);
    CPU_SET(1, &all);
    CPU_SET(2, &all);
    CPU_SET(3, &all);

    CPU_ZERO(&onetwo);
    CPU_SET(1, &onetwo);
    CPU_SET(2, &onetwo);

    CPU_ZERO(&onethree);
    CPU_SET(1, &onethree);
    CPU_SET(3, &onethree);

    printf ("numcores %d; cap2: %d\n", pn_numcores(), pn_m2cap());
    pthread_mutex_init(&printf_mutex, NULL);

    pthread_attr_setaffinity_np(&tattr[0], sizeof(PN_CMSK), &one); // executed by any core
    pthread_attr_setaffinity_np(&tattr[1], sizeof(PN_CMSK), &two); // executed by any core
    pthread_attr_setaffinity_np(&tattr[2], sizeof(PN_CMSK), &onethree); // executed by core 2
    pthread_attr_setaffinity_np(&tattr[3], sizeof(PN_CMSK), &onethree); // executed by core 1
    pthread_attr_setaffinity_np(&tattr[4], sizeof(PN_CMSK), &onetwo); // executed by core 2
    pthread_attr_setaffinity_np(&tattr[5], sizeof(PN_CMSK), &onetwo); // executed by core 2
    pthread_attr_setaffinity_np(&tattr[6], sizeof(PN_CMSK), &one); // executed by 1 or 2
    pthread_attr_setaffinity_np(&tattr[7], sizeof(PN_CMSK), &one); // executed by 1 or 2
    pthread_attr_setaffinity_np(&tattr[8], sizeof(PN_CMSK), &two); // executed by 1 or 3 (1 on hw)
    pthread_attr_setaffinity_np(&tattr[9], sizeof(PN_CMSK), &two); // executed by 1 or 3 (1 on hw)

    for (n = 0; n < 10; n++)
        pthread_create(&threads[n], &tattr[n], *thread, (void *) n);
    for (n = 0; n < 10; n++)
        pthread_join(threads[n], NULL);  
    return 0;
}