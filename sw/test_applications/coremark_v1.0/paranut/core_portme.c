/* 
	File: core_portme.c
*/
/*
	Author : Shay Gal-On, EEMBC
	Legal : TODO!
*/ 
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include "coremark.h"
#if CALLGRIND_RUN
#include <valgrind/callgrind.h>
#endif

#include "encoding.h"

#if (MEM_METHOD==MEM_MALLOC)
#include <malloc.h>
/* Function: portable_malloc
	Provide malloc() functionality in a platform specific way.
*/
void *portable_malloc(size_t size) {
	return malloc(size);
}
/* Function: portable_free
	Provide free() functionality in a platform specific way.
*/
void portable_free(void *p) {
	free(p);
}
#else
void *portable_malloc(size_t size) {
	return NULL;
}
void portable_free(void *p) {
	p=NULL;
}
#endif

#if (SEED_METHOD==SEED_VOLATILE)
#if VALIDATION_RUN
	volatile ee_s32 seed1_volatile=0x3415;
	volatile ee_s32 seed2_volatile=0x3415;
	volatile ee_s32 seed3_volatile=0x66;
#endif
#if PERFORMANCE_RUN
	volatile ee_s32 seed1_volatile=0x0;
	volatile ee_s32 seed2_volatile=0x0;
	volatile ee_s32 seed3_volatile=0x66;
#endif
#if PROFILE_RUN
	volatile ee_s32 seed1_volatile=0x8;
	volatile ee_s32 seed2_volatile=0x8;
	volatile ee_s32 seed3_volatile=0x8;
#endif
	volatile ee_s32 seed4_volatile=ITERATIONS;
	volatile ee_s32 seed5_volatile=0;
#endif
/* Porting: Timing functions
	How to capture time and convert to seconds must be ported to whatever is supported by the platform.
	e.g. Read value from on board RTC, read value from cpu clock cycles performance counter etc. 
	Sample implementation for standard time.h and windows.h definitions included.
*/
/* Define: TIMER_RES_DIVIDER
	Divider to trade off timer resolution and total time that can be measured.

	Use lower values to increase resolution, but make sure that overflow does not occur.
	If there are issues with the return value overflowing, increase this value.
	*/
#if USE_CLOCK
	#define NSECS_PER_SEC CLOCKS_PER_SEC
	#define EE_TIMER_TICKER_RATE 1000
	#define CORETIMETYPE clock_t 
	#define GETMYTIME(_t) (*_t=clock())
	#define MYTIMEDIFF(fin,ini) ((fin)-(ini))
	#define TIMER_RES_DIVIDER 1
	#define SAMPLE_TIME_IMPLEMENTATION 1
#elif defined(_MSC_VER)
	#define NSECS_PER_SEC 10000000
	#define EE_TIMER_TICKER_RATE 1000
	#define CORETIMETYPE FILETIME
	#define GETMYTIME(_t) GetSystemTimeAsFileTime(_t)
	#define MYTIMEDIFF(fin,ini) (((*(__int64*)&fin)-(*(__int64*)&ini))/TIMER_RES_DIVIDER)
	/* setting to millisces resolution by default with MSDEV */
	#ifndef TIMER_RES_DIVIDER
	#define TIMER_RES_DIVIDER 1000
	#endif
	#define SAMPLE_TIME_IMPLEMENTATION 1
#elif HAS_TIME_H
	#define NSECS_PER_SEC 1000000000
	#define EE_TIMER_TICKER_RATE 1000
	#define CORETIMETYPE struct timespec 
	#define GETMYTIME(_t) clock_gettime(CLOCK_REALTIME,_t)
	#define MYTIMEDIFF(fin,ini) ((fin.tv_sec-ini.tv_sec)*(NSECS_PER_SEC/TIMER_RES_DIVIDER)+(fin.tv_nsec-ini.tv_nsec)/TIMER_RES_DIVIDER)
	/* setting to 1/1000 of a second resolution by default with linux */
	#ifndef TIMER_RES_DIVIDER
	#define TIMER_RES_DIVIDER 1000000
	#endif
	#define SAMPLE_TIME_IMPLEMENTATION 1
#elif MY_TIME
	#define NSECS_PER_SEC 1000000000
	#define EE_TIMER_TICKER_RATE 1000
	#define CORETIMETYPE uint64_t
	#define GETMYTIME(_t) (*_t = ((uint64_t)read_csr(mcycleh) << 32) | read_csr(mcycle))
	#define MYTIMEDIFF(fin,ini) ((fin)-(ini))
	#ifndef TIMER_RES_DIVIDER
	#define TIMER_RES_DIVIDER (NSECS_PER_SEC/_CLKS_PER_SEC) // 25 MHz Clock by default, set accordingly
	#endif
	#define SAMPLE_TIME_IMPLEMENTATION 1
#else
	#define SAMPLE_TIME_IMPLEMENTATION 0
#endif
#define EE_TICKS_PER_SEC (NSECS_PER_SEC / TIMER_RES_DIVIDER)

#if SAMPLE_TIME_IMPLEMENTATION
/** Define Host specific (POSIX), or target specific global time variables. */
static CORETIMETYPE start_time_val, stop_time_val;


/* Function: start_time
	This function will be called right before starting the timed portion of the benchmark.

	Implementation may be capturing a system timer (as implemented in the example code) 
	or zeroing some system parameters - e.g. setting the cpu clocks cycles to 0.
*/
void start_time(void) {
	GETMYTIME(&start_time_val );      

#if CALLGRIND_RUN
	CALLGRIND_START_INSTRUMENTATION
#endif
#if MICA
    asm volatile("int3");/*1 */
#endif
}
/* Function: stop_time
	This function will be called right after ending the timed portion of the benchmark.

	Implementation may be capturing a system timer (as implemented in the example code) 
	or other system parameters - e.g. reading the current value of cpu cycles counter.
*/
void stop_time(void) {
#if CALLGRIND_RUN
	 CALLGRIND_STOP_INSTRUMENTATION 
#endif
#if MICA
    asm volatile("int3");/*1 */
#endif
	GETMYTIME(&stop_time_val );      
}
/* Function: get_time
	Return an abstract "ticks" number that signifies time on the system.
	
	Actual value returned may be cpu cycles, milliseconds or any other value,
	as long as it can be converted to seconds by <time_in_secs>.
	This methodology is taken to accomodate any hardware or simulated platform.
	The sample implementation returns millisecs by default, 
	and the resolution is controlled by <TIMER_RES_DIVIDER>
*/
CORE_TICKS get_time(void) {
	CORE_TICKS elapsed=(CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
	return elapsed;
}
/* Function: time_in_secs
	Convert the value returned by get_time to seconds.

	The <secs_ret> type is used to accomodate systems with no support for floating point.
	Default implementation implemented by the EE_TICKS_PER_SEC macro above.
*/
secs_ret time_in_secs(CORE_TICKS ticks) {
	secs_ret retval=((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
	return retval;
}
#else 
#error "Please implement timing functionality in core_portme.c"
#endif /* SAMPLE_TIME_IMPLEMENTATION */

ee_u32 default_num_contexts=MULTITHREAD;

/* Function: portable_init
	Target specific initialization code 
	Test for some common mistakes.
*/
void portable_init(core_portable *p, int *argc, char *argv[])
{
#if PRINT_ARGS
	int i;
	for (i=0; i<*argc; i++) {
		ee_printf("Arg[%d]=%s\n",i,argv[i]);
	}
#endif
	if (sizeof(ee_ptr_int) != sizeof(ee_u8 *)) {
		ee_printf("ERROR! Please define ee_ptr_int to a type that holds a pointer!\n");
	}
	if (sizeof(ee_u32) != 4) {
		ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
	}
#if (MAIN_HAS_NOARGC && (SEED_METHOD==SEED_ARG))
	ee_printf("ERROR! Main has no argc, but SEED_METHOD defined to SEED_ARG!\n");
#endif
	
#if (MULTITHREAD>1) && (SEED_METHOD==SEED_ARG)
	{
		int nargs=*argc,i;
		if ((nargs>1) && (*argv[1]=='M')) {
			default_num_contexts=parseval(argv[1]+1);
			if (default_num_contexts>MULTITHREAD)
				default_num_contexts=MULTITHREAD;
			/* Shift args since first arg is directed to the portable part and not to coremark main */
			--nargs;
			for (i=1; i<nargs; i++)
				argv[i]=argv[i+1];
			*argc=nargs;
		}
	}
#endif /* sample of potential platform specific init via command line, reset the number of contexts being used if first argument is M<n>*/
	p->portable_id=1;
}
/* Function: portable_fini
	Target specific final code 
*/
void portable_fini(core_portable *p)
{
	p->portable_id=0;
}

#if (MULTITHREAD>1)

/* Function: core_start_parallel
	Start benchmarking in a parallel context.
	
	Three implementations are provided, one using pthreads, one using fork and shared mem, and one using fork and sockets.
	Other implementations using MCAPI or other standards can easily be devised.
*/
/* Function: core_stop_parallel
	Stop a parallel context execution of coremark, and gather the results.
	
	Three implementations are provided, one using pthreads, one using fork and shared mem, and one using fork and sockets.
	Other implementations using MCAPI or other standards can easily be devised.
*/
#if USE_PTHREAD
ee_u8 core_start_parallel(core_results *res) {
	return (ee_u8)pthread_create(&(res->port.thread),NULL,iterate,(void *)res);
}
ee_u8 core_stop_parallel(core_results *res) {
	void *retval;
	return (ee_u8)pthread_join(res->port.thread,&retval);
}
#elif USE_FORK
static int key_id=0;
ee_u8 core_start_parallel(core_results *res) {
	key_t key=4321+key_id;
	key_id++;
	res->port.pid=fork();
	res->port.shmid=shmget(key, 8, IPC_CREAT | 0666);
	if (res->port.shmid<0) {
		ee_printf("ERROR in shmget!\n");
	}
	if (res->port.pid==0) {
		iterate(res);
		res->port.shm=shmat(res->port.shmid, NULL, 0);
		/* copy the validation values to the shared memory area  and quit*/
		if (res->port.shm == (char *) -1) {
			ee_printf("ERROR in child shmat!\n");
		} else {
			memcpy(res->port.shm,&(res->crc),8);
			shmdt(res->port.shm);
		}
		exit(0);
	}
	return 1;
}
ee_u8 core_stop_parallel(core_results *res) {
	int status;
	pid_t wpid = waitpid(res->port.pid,&status,WUNTRACED);
	if (wpid != res->port.pid) {
		ee_printf("ERROR waiting for child.\n");
		if (errno == ECHILD) ee_printf("errno=No such child %d\n",res->port.pid);
		if (errno == EINTR) ee_printf("errno=Interrupted\n");
		return 0;
	}
	/* after process is done, get the values from the shared memory area */
	res->port.shm=shmat(res->port.shmid, NULL, 0);
	if (res->port.shm == (char *) -1) {
		ee_printf("ERROR in parent shmat!\n");
		return 0;
	} 
	memcpy(&(res->crc),res->port.shm,8);
	shmdt(res->port.shm);
	return 1;
}
#elif USE_SOCKET
static int key_id=0;
ee_u8 core_start_parallel(core_results *res) {
	int bound, buffer_length=8;
	res->port.sa.sin_family = AF_INET;
	res->port.sa.sin_addr.s_addr = htonl(0x7F000001);
	res->port.sa.sin_port = htons(7654+key_id);
	key_id++;
	res->port.pid=fork();
	if (res->port.pid==0) { /* benchmark child */
		iterate(res);
		res->port.sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (-1 == res->port.sock) /* if socket failed to initialize, exit */   {
			ee_printf("Error Creating Socket");
		} else {
			int bytes_sent = sendto(res->port.sock, &(res->crc), buffer_length, 0,(struct sockaddr*)&(res->port.sa), sizeof (struct sockaddr_in));
			if (bytes_sent < 0)
				ee_printf("Error sending packet: %s\n", strerror(errno));
			close(res->port.sock); /* close the socket */
		}
		exit(0);
	} 
	/* parent process, open the socket */
	res->port.sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	bound = bind(res->port.sock,(struct sockaddr*)&(res->port.sa), sizeof(struct sockaddr));
	if (bound < 0)
		ee_printf("bind(): %s\n",strerror(errno));
	return 1;
}
ee_u8 core_stop_parallel(core_results *res) {
	int status;
	int fromlen=sizeof(struct sockaddr);
	int recsize = recvfrom(res->port.sock, &(res->crc), 8, 0, (struct sockaddr*)&(res->port.sa), &fromlen);
	if (recsize < 0) {
		ee_printf("Error in receive: %s\n", strerror(errno));
		return 0;
	}
	pid_t wpid = waitpid(res->port.pid,&status,WUNTRACED);
	if (wpid != res->port.pid) {
		ee_printf("ERROR waiting for child.\n");
		if (errno == ECHILD) ee_printf("errno=No such child %d\n",res->port.pid);
		if (errno == EINTR) ee_printf("errno=Interrupted\n");
		return 0;
	}
	return 1;
}
#else /* no standard multicore implementation */

/* Array for thread data */
int _thread_datas[2*MULTITHREAD]; 

void pn_thread_entry(int cid)
{
  int * thread_data_ptr = _thread_datas;
  void*(*start_routine) (void*);
  
  // multi-threaded programs override this function.
  // for the case of single-threaded programs, only let core 0 proceed.
  if (cid != 0) {
    if (thread_data_ptr[cid*2] != 0) {
      start_routine = (void * (*)(void *)) thread_data_ptr[cid*2];
      //printf("%d: %p\n", cid, start_routine);
      void* ret = start_routine((void*)thread_data_ptr[cid*2+1]);
      
      // Write return value (potential race condition)
      thread_data_ptr[cid*2+1] = (int)ret;
      // Reset jump adr (potential race condition)
      thread_data_ptr[cid*2] = 0;
      
      // Make sure the values are written to the Cache
      asm volatile ("fence");
    }
    asm volatile (".word 0x0000000B");
  }
}

int pnthread_join(unsigned int hartID, void **retval){
  unsigned int pnce; 
  int * thread_data_ptr = _thread_datas;

  // Don't need to wait for CePU 
  //printf("pnthread_join: Waiting on CPU%d\r\n", hartID);
  if( hartID != 0 ){	
    do {
      // read pnce and check if CoPU with specified hartID has reached halt instruction
      pnce =	rdpnce();
      pnce &= (1 << hartID);
    } while (pnce != 0);
  }

  // if retval is not null write return value
  if (retval != 0) {
    *retval = (void*)thread_data_ptr[hartID*2+1];
    //printf("pnthread_join: CPU %d, %p, 0x%08x\n", hartID, *retval, *(int*)*retval);
  }
  //printf("pnthread_join: CPU%d is halted\r\n", hartID);

  return 0;
}

int pnthread_create(void *hartID, void*(*start_routine) (void*), void *arg){
  unsigned int  pnm2cp, pnce, cpus = rdpncpus();
  unsigned int * ithread = hartID;
  int * thread_data_ptr = _thread_datas;
  int i;

  // read mode 2 capable CoPUs and enabled register
  pnm2cp = rdpnm2cp();
  pnce = rdpnce();
  //printf("pnthread_create: pnm2cp: 0x%x, pnce: 0x%x\r\n", pnm2cp, pnce);

  // find a not currently running mode 2 capable CoPU 
  *ithread = 0;
  for (i = 1; i < cpus; i++){
    pnce >>= 1;
    pnm2cp >>= 1;
    if((pnce & 0x1) == 0 && (pnm2cp & 0x1) == 1){
      pnce = 1 << i;
      // write back thread/hartID 
      *ithread = i; 
      break;
    }
  }

  // could not find any CoPU -> CePU must execute the thread and return afterwards
  if (*ithread == 0) {
    //printf("pnthread_create: Starting CePU: Jump address 0x%08x\r\n", start_routine);
    void* ret = start_routine(arg);
    //printf("pnthread_create: CePU return = %p, val = 0x%08x\n", ret);
    thread_data_ptr[1] = (int)ret;

  }else{
    //printf("pnthread_create: Starting CPU%d, Jump address 0x%08x, Arg address 0x%08x\r\n", *ithread, start_routine, arg);
    // write address of start routine and arg to memory
    thread_data_ptr[i*2] = (uint32_t)start_routine;
    thread_data_ptr[i*2+1] = (uint32_t)arg;
    asm volatile ("fence");

    // enable CoPU 
    setpnce(pnce);
    //printf("pnthread_create: After starting: pnce: 0x%x\r\n", rdpnce());
  }
  return 0;
}

ee_u8 core_start_parallel(core_results *res) {
	return (ee_u8)pnthread_create(&(res->port.hartID),iterate,(void *)res);
}
ee_u8 core_stop_parallel(core_results *res) {
	return pnthread_join(res->port.hartID, NULL);
}
//#error "Please implement multicore functionality in core_portme.c to use multiple contexts." 
#endif /* multithread implementations */
#endif
