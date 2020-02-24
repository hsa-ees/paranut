/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
     Demo application showcasing the CoPU Linked Mode (Mode 1).
     This application requires the libparanut.  

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this 
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/
 
#include <stdio.h> 
#include <stdint.h>
#include <time.h>
#include <encoding.h>
#include <stdlib.h>

#include "libparanut.h"

// DEFINES:
// ------------------------------------------------------------------------------
#define ASIZE 10000

#ifndef PARALLEL
#define PARALLEL 1
#endif

#define _CLKS_PER_MSEC (read_csr(0xFC7)/1000)

// GLOBAL VARIABLES:
// ------------------------------------------------------------------------------
int *a, *b, *s;

// FUNCTIONS:
// ------------------------------------------------------------------------------
void init_arrays(){
	// Initialize arrays
	for (int n = 0; n < ASIZE; n++) {
		a[n] = n;
		b[n] = ASIZE-n;
		s[n] = 0;
        //printf("s[%d] = %d, a[%d] = %d,  b[%d] = %d\n", n, s[n], n, a[n], n, b[n] );
	}
}

int test_result(){
	printf("Testing results ...\n");
	for (int n = 0; n < ASIZE; n++){
		if( s[n] != ASIZE) {
			printf("ERROR at s[%d]: %d != %d\n", n, s[n], ASIZE);
			return -1;
		}
	}
	printf("Test successful!\n");
	
	return 0;
}


int main(){
	int n, id;
#if PARALLEL == 1
	uint32_t cpus = read_csr(0xFC0);
#else
	uint32_t cpus = PARALLEL;
#endif
	uint32_t start, end, seq_time, par_time;
	
	printf("\nWelcome to the ParaNut mode 1 demo\n----------------------------------\n\n");
	printf("Current system has %d CPUs\n", cpus);
	
	a = malloc(ASIZE*sizeof(int));
	b = malloc(ASIZE*sizeof(int));
	s = malloc(ASIZE*sizeof(int));
	
    if(a == NULL || b == NULL || s == NULL){
        printf("ERROR: Could not allocate enough memory: a = %p, b = %p, s= %p", a, b, s);
        return -1;
    }    
	
	//------------------------------------------------
	// Sequential version
	//------------------------------------------------
	init_arrays();
	printf("\nSequential add (s[%d] = a[%d] + b[%d]) ...\n", ASIZE, ASIZE, ASIZE);
	
	start = read_csr(mcycle); // Start of measurement
	for (n = 0; n < ASIZE; n += 1)
		s[n] = a[n] + b[n];
	end = read_csr(mcycle); // End of measurement 	
	seq_time = end - start;
	printf("Sequential add cycles: %d\n", seq_time);
	
	if(test_result() != 0)
		return -1;
	
	
	//------------------------------------------------
	// Vectorized version
	//------------------------------------------------
	init_arrays();
	printf("\n%d times vector add (s[%d] = a[%d] + b[%d]) ...\n", cpus, ASIZE, ASIZE, ASIZE);
	
	start = read_csr(mcycle); // Start of measurement	
	id = pn_begin_linked (cpus);
	for (n = id; n < ASIZE; n += cpus)
		s[n] = a[n] + b[n];
	pn_end_linked ();
	end = read_csr(mcycle); // End of measurement 
	par_time = end - start;
	printf("vector add cycles: %d\n", par_time);
	
	if(test_result() != 0)
		return -1;
	
		
	//------------------------------------------------
	// Display statistic
	//------------------------------------------------
	printf("\nParaNut mode 1 demo results:\n----------------------------------\n\n");
	printf("           |   clks   |  time/ms  | \n");
	printf("sequential | %8d | %9.2f | \n", seq_time, (float)seq_time / _CLKS_PER_MSEC); 
	printf("vectorized | %8d | %9.2f | \n", par_time, (float)par_time / _CLKS_PER_MSEC); 
	printf("diff       | %8d | %9.2f | \n", seq_time - par_time, (float)seq_time / _CLKS_PER_MSEC - (float)par_time / _CLKS_PER_MSEC); 
	printf("Speedup: %.2f\n", (float)seq_time/par_time);
		
	//------------------------------------------------
	// Vectorized version
	//------------------------------------------------
	/*
	init_arrays();
	printf("%d times inefficient vector add ...\n", cpus);
	
	start = read_csr(mcycle); // Start of measurement	
	id = pn_begin_linked (cpus);
	for (n = 0; n < ASIZE; n += cpus)
		// Note: n is always identical in all threads
		s[n + id] = a[n + id] + b[n + id];
	
	pn_end_linked ();
	end = read_csr(mcycle); // End of measurement 
	printf("Inefficient vector add cycles: %d\n", end - start);
	
	if(test_result() != 0)
		return -1;
	*/

    // Cleanup
	free(a);
	free(b);
	free(s);
    
	printf("\n\nEnd of ParaNut mode 1 demo\n----------------------------------\n\n");
}	
