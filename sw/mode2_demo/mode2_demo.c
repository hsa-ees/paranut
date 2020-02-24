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
#include <string.h>
#include <stdlib.h>

#include "libparanut.h"

// DEFINES:
// ------------------------------------------------------------------------------
#define BITMASK 0xFF
#define DATA_SIZE 3000 //(1920*1080)

#define _CLKS_PER_MSEC (read_csr(0xFC7)/1000)

// GLOBAL VARIABLES:
// ------------------------------------------------------------------------------
volatile uint32_t global_val = 0;
uint32_t histogram[BITMASK];
char** thread_results;
_pn_spinlock lock;

// TYPEDEFS:
// ------------------------------------------------------------------------------
typedef struct {
	uint32_t* data;
	size_t data_length;
	uint32_t* hist;
	uint32_t bitmask;
} histogram_t;


// FUNCTIONS:
// ------------------------------------------------------------------------------

int atomic_increment(uint32_t* adr, const uint32_t increment_val) {
	register uint32_t lr_val asm ("a4");
	
	do {
		asm volatile ("lr.w %[lr_val], (%[adr])\n"					 // Load reserved old value
					  "add %[lr_val], %[lr_val], %[increment_val]\n" // Increment
					  "sc.w %[lr_val], %[lr_val], (%[adr])\n"		 // Try to store
						: [lr_val] "=r" (lr_val) 
						: [adr] "r" (adr), [increment_val] "r" (increment_val)
						: "memory" );				
		
	} while (lr_val != 0); // Successfull?
		
	return 0;		  
}

void histogram_func(void *arg) {
	histogram_t* result = (histogram_t*) arg;
	int shifts = 0;
	
	// Determine number of right shifts for given bitmask
	while(!((result->bitmask >> shifts) & 1)) {
		shifts++;
	}
	if(shifts > 31) 
		return; // ERROR
	
	// Calculate histogram
	for (int i = 0; i < result->data_length; i++){
		uint32_t val = (result->data[i] & result->bitmask) >> shifts;
		result->hist[val]++;
	}
}

void* atomic_histogram_func(void *arg) {
	histogram_t* result = (histogram_t*) arg;
	int shifts = 0;
	
	// Determine number of right shifts for given bitmask
	while(!((result->bitmask >> shifts) & 1)) {
		shifts++;
	}
	if(shifts > 31) 
		return 0; // ERROR
	
	// Calculate histogram
	for (int i = 0; i < result->data_length; i++){
		uint32_t val = (result->data[i] & result->bitmask) >> shifts;
		atomic_increment(result->hist + val, 1);
	}
	
	return 0;
}

char* parallel_func(int cpuid) {
	int val = 0; 
    
    // Malloc is not thread safe!
    pn_spinlock_lock(&lock);
    char * s = (char*) malloc(100);
    pn_spinlock_unlock(&lock);
    if(s == NULL){
        return 0; 
    }
    

	// Do some work
	for (int i = 0; i < DATA_SIZE; i++){
		val += i;
		val *= 3;
		val /= 5;
		val <<= 2;
		val >>= 5;
	}
	// Add input to value
	val += cpuid;
	
	// Create result as string:
	snprintf(s, 100, "Result of CPU #%d running thread #%d: %d", 
			cpuid, cpuid, val);
	
	return s;	
}

int main(){
	uint32_t cpus = read_csr(0xFC0);
	uint32_t start, end, seq_time;
	uint32_t *histogram1, *histogram2, *data;
    int cpuid;
	int block_size = DATA_SIZE/cpus;
	histogram_t histogram[cpus]; 
	    
    // Init 
    pn_exception_init();
    pn_cache_init();
    pn_spinlock_init(&lock);
		
	printf("\nWelcome to the ParaNut mode 2 demo\n----------------------------------\n\n");
	printf("Current system has %d CPUs\n", cpus);
	
    thread_results = malloc(cpus*sizeof(char*));
    if(thread_results == NULL){
        printf("ERROR: Could not allocate enough memory\n");
        return -1;
    }    
    
	// ------------------------------------------------------------------------------
	// Test independent parallel execution (no A-Extension required)
	//			
	// ------------------------------------------------------------------------------
	printf("\nTest independent task on all CPUs\n----------------------------------\n\n");
	cpuid = pn_begin_threaded (cpus);
    thread_results[cpuid] = parallel_func (cpuid);
    pn_end_threaded ();
    	
	for (int i = 0; i < cpus; i++){
		printf("\nThread #%d:\n", i);
		printf("%s\n", (char*) thread_results[i]);
        free(thread_results[i]);
	}	
	
	// ------------------------------------------------------------------------------
	// Test A-Extension: 
	//			High contention on single address
	// ------------------------------------------------------------------------------
	printf("\nTest A-Extension: High contention on single address\n----------------------------------\n\n");
	// Get base time for simple add operation on CePU without synchronization
	// ------------------------------------------------------------------------------
	printf("Get base time for simple add operation on CePU without synchronization...\n");
	global_val = 0;
	start = read_csr(mcycle); // Start of measurement
  	
    for (int i = 0; i < DATA_SIZE; i++){
		global_val++;
	}
	
    end = read_csr(mcycle); // End of measurement
	seq_time = end - start;
	printf("Add:\t\t %d |\t %9.2f ms \n", seq_time, (float)seq_time / _CLKS_PER_MSEC);   
	
	// Check Value 
	if(global_val != DATA_SIZE)
		printf("ERROR: Global value = %d != %d \n", global_val, DATA_SIZE);
		
	
	// Synchronized add on all CPUs
	// ------------------------------------------------------------------------------
    printf("Synchronized add on all CPUS...\n");
	global_val = 0;
	start = read_csr(mcycle); // Start of measurement
    
    cpuid = pn_begin_threaded (cpus);
    for (int i = 0; i < DATA_SIZE/cpus; i++){
		pn_spinlock_lock(&lock);
        global_val++;
        pn_spinlock_unlock(&lock);
	}
    pn_end_threaded ();
	
	end = read_csr(mcycle); // End of measurement
	seq_time = end - start;
	printf("Sync add:\t\t %d |\t %9.2f ms \n", seq_time, (float)seq_time / _CLKS_PER_MSEC);   
	
	// Check Value 
	if(global_val != DATA_SIZE)
		printf("ERROR: Global value = %d != %d \n", global_val, DATA_SIZE);
		
    
    printf("Atomic add on all CPUS...\n");
	global_val = 0;
	start = read_csr(mcycle); // Start of measurement
    
    cpuid = pn_begin_threaded (cpus);
   	for (int i = 0; i < DATA_SIZE/cpus; i++){
        atomic_increment((uint32_t*)&global_val, 1);
	}
    pn_end_threaded ();
	
	end = read_csr(mcycle); // End of measurement
	seq_time = end - start;
	printf("Atomic add:\t\t %d |\t %9.2f ms \n", seq_time, (float)seq_time / _CLKS_PER_MSEC);   
	
	// Check Value 
	if(global_val != DATA_SIZE)
		printf("ERROR: Global value = %d != %d \n", global_val, DATA_SIZE);
		
	
        
	// ------------------------------------------------------------------------------
	// Test A-Extension: 
	//			Histogram test (contention spread accross different addresses)
	// ------------------------------------------------------------------------------
	printf("\nHistogram test (contention spread accross different addresses)\n----------------------------------\n\n");
	// Initialize data:
	// ------------------------------------------------------------------------------
	printf("Initializing data for Histogram test...\n");
	histogram1 = (uint32_t*) malloc(0xFF*sizeof(uint32_t));
	histogram2 = (uint32_t*) malloc(0xFF*sizeof(uint32_t));
	data = (uint32_t*) malloc(DATA_SIZE*sizeof(uint32_t));
	if(data == NULL || histogram1 == NULL || histogram2 == NULL){
		printf("ERROR: Could not allocate enough memory\n");
		return 1;
	}
	memset(histogram1, 0x00, 0xFF*sizeof(uint32_t));
		
	// Fill array with (pseudo) random data
	srand(read_csr(mcycle));
	for (int i = 0; i < DATA_SIZE; i+=4){ // i++){
		data[i] = 1;
		data[i+1] = 2;
		data[i+2] = 3;
		data[i+3] = 4;
		//data[i] = rand()%0xFF;
	}
			
	// Get base time for single threaded histogram function without synchronization
	// ------------------------------------------------------------------------------
	printf("\nGet base time for single threaded histogram function without synchronization...\n");
	histogram[0].data = data;
	histogram[0].hist = histogram1;	
	histogram[0].data_length = DATA_SIZE;
	histogram[0].bitmask = 0xFF;
	
	start = read_csr(mcycle); // Start of measurement
  	histogram_func((void*) &histogram[0]);
	end = read_csr(mcycle); // End of measurement
	seq_time = end - start;
	printf("Histogram:\t\t %d |\t %9.2f ms \n", seq_time, (float)seq_time / _CLKS_PER_MSEC);   
	
	//for (int i = 0; i < 30; i++)
	//	printf("[%3d]: %d\n", i, histogram1[i]);	
	
	// Synchronized histogram on all CPUs (repeat for all possible block sizes)
	// ------------------------------------------------------------------------------
	for(int cur_cpus = cpus; cur_cpus > 1; cur_cpus--) {	
		// Reset data
		memset(histogram2, 0x00, 0xFF*sizeof(uint32_t));
		
		// Determine block size
		block_size = DATA_SIZE / cur_cpus;
		
		// Run test on x CPUs
		printf("\nParallel histogram test (%d blocks of size: %d) %p...\n", cur_cpus, block_size, histogram2);
		start = read_csr(mcycle); // Start of measurement
		
        cpuid = pn_begin_threaded (cur_cpus);
        if (cpuid < 0) {
            printf("ERROR: pn_begin_threaded returned %d\n", cpuid);
            return cpuid;
        }   
        histogram[cpuid].data = data + (block_size*cpuid);
        histogram[cpuid].hist = histogram2;	
		histogram[cpuid].data_length = block_size;
		histogram[cpuid].bitmask = 0xFF;
		atomic_histogram_func((void *)&histogram[cpuid]);
		//printf("Thread %d started on CPU #%d\n", i, threads[i]);		
		pn_end_threaded ();
		
		end = read_csr(mcycle); // End of measurement
		seq_time = end - start;
		printf("Parallel histogram:\t %d |\t %9.2f ms \n", seq_time, (float)seq_time / _CLKS_PER_MSEC);   
		
		//for (int i = 0; i < 30; i++)
		//	printf("[%3d]: %d\n", i, histogram2[i]);	
		
		for (int i = 0; i < 0xFF; i++){
			if(histogram1[i] != histogram2[i])
				printf("ERROR: Values of histogram1 and histogram2 differ: [%3d]: %d != %d \n", 
						i, histogram1[i],histogram2[i]);	
		}
	}
	
	free(histogram1);
	free(histogram2);
	free(data);
		
	printf("\n\nEnd of ParaNut mode 2 demo\n----------------------------------\n\n");
}	
