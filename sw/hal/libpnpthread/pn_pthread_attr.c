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
#include <string.h>
#include <stdlib.h>
#include "pn_pthread_attr.h"

#define GUARDSIZE 2
/* asserts to check type sizes at compile time */
#define ASSERT_TYPE_SIZE(type, size) 					\
  _Static_assert (sizeof (type) == size,				\
		  "sizeof (" #type ") != " #size)

#define ASSERT_PN_PTHREAD_INTERNAL_SIZE(type, internal) 			\
  _Static_assert (sizeof ((type) { { 0 } }).__size >= sizeof (internal),\
		  "sizeof (" #type ".__size) < sizeof (" #internal ")")



/* functions */
int pn_pthread_attr_init (pn_pthread_attr_t *attr)
{
  struct pn_pthread_attr *iattr;

  ASSERT_TYPE_SIZE (pn_pthread_attr_t, __SIZEOF_PN_PTHREAD_ATTR_T);
  ASSERT_PN_PTHREAD_INTERNAL_SIZE (pn_pthread_attr_t, struct pn_pthread_attr);

  /* Many elements are initialized to zero so let us do it all at
     once.  This also takes care of clearing the bytes which are not
     internally used.  */
  memset (attr, '\0', __SIZEOF_PN_PTHREAD_ATTR_T);

  iattr = (struct pn_pthread_attr *) attr;


  /* Default guard size is defined by pagesize. This is a bare metall implementiation. there is no virtual memory, thus a guardsize is defined statically  */
  iattr->guardsize = GUARDSIZE;

  return 0;
}

int pn_pthread_setaffinity_np (pn_pthread_attr_t *attr, size_t cpusetsize,
			       const cpu_set_t *cpuset)
{
  // TODO: take cpusetsize more into consideration
  if(attr == NULL || cpuset==NULL){
    return -1;
  }

  if(cpusetsize != sizeof(*cpuset) || cpusetsize != sizeof(PN_CMSK)){
    return -1;
  }

  struct pn_pthread_attr *iattr;

  iattr = (struct pn_pthread_attr *) attr;

  memcpy ( &(iattr->coremask),cpuset, sizeof(cpu_set_t));

  return 0;
}

int pn_pthread_getaffinity_np(const pn_pthread_attr_t *attr, size_t cpusetsize,
        cpu_set_t *cpuset){
  // TODO: take cpusetsize more into consideration
  if(cpuset==NULL){
    return -1;
  }
  if(attr==NULL){
    *cpuset = 0;
    return 0;
  } 

  struct pn_pthread_attr *iattr;

  iattr = (struct pn_pthread_attr *) attr;
  memcpy (cpuset, &(iattr->coremask), sizeof(cpu_set_t));
  return 0;

}

int pn_pthread_attr_getstacksize(pn_pthread_attr_t *attr, size_t *stacksize){
  stacksize = 0;
  return 0;
}

int pn_pthread_attr_destroy (pn_pthread_attr_t *attr)
{
  struct pn_pthread_attr *iattr;

  iattr = (struct pn_pthread_attr *) attr;

  if (iattr->extension != NULL){
      free (iattr->extension);
  }

  return 0;
}