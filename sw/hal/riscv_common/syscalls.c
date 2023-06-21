/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2019-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Anna Pfuetzner <anna.pfuetzner@hs-augsburg.de>
					      Nico Borgsmüller <nico.borgsmueller@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This file contains the ParaNut system call implementations for
    full newlib functionality

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


/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/times.h>
#include "encoding.h"


/* Defines */
#define FLUSH_TOHOST() ({asm volatile (			\
			"lui x6, %hi(pn_tohost)\n" 			\
			"addi x6, x6, %lo(pn_tohost)\n"		\
			".word (0x300B | ((6) << 15))\n"); 	\
		})

#define FLUSH_FROMHOST() ({asm volatile (		\
			"lui x6, %hi(pn_fromhost)\n" 			\
			"addi x6, x6, %lo(pn_fromhost)\n"		\
			".word (0x300B | ((6) << 15))\n"); 	\
		})


/* Variables */
extern int errno;

register char * stack_ptr asm("sp");

extern uint8_t pn_tohost;
extern uint8_t pn_fromhost;


/* Functions */
int __attribute__((weak)) main(int argc, char** argv)
{
  // Single-threaded programs override this function.
  printf("Implement main()!\n");
  return -1;
}


void _init(int cid, int nc)
{
  // Only CePU should ever get here.
  int ret = main(0, 0);
  exit(ret);
}

int _getpid(void)
{
	return 1;
}

int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}

void _exit (int status)
{
	kill(status, -1);
	// If we are being debugged this triggers the debugger to get control
	// before we hang/halt the execution
	asm volatile("ebreak");
	while (1) {asm (".word 0x0000000B;");}		/* Make sure we hang here */
}

int _read (int file, char *ptr, int len)
{
	int DataIdx;
	int c = -1;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		while(pn_fromhost == 0){
			FLUSH_FROMHOST();
		}
		c = pn_fromhost;
		if(c == EOF || c == '\n'){
			ptr = '\0';
			break;
		}
		*ptr++ = (char) c;

		// clear pn_fromhost after each character
		pn_fromhost = 0;
		FLUSH_FROMHOST();
	}

	// clear pn_fromhost at the end
	pn_fromhost = 0;
	FLUSH_FROMHOST();

	return DataIdx;
}

int _putc( int ch)
{
	// Wait till pn_tohost is empty
	while(pn_tohost != 0){
		FLUSH_TOHOST();
	}

	pn_tohost = ch & 0xFF;
	FLUSH_TOHOST();

	return ch;
}

int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		_putc(*ptr++);
	}

	return DataIdx;
}

caddr_t _sbrk(int incr)
{
	extern char _tdata_begin, _tdata_end, _tbss_end, __heap_end, __heap_start;
	static char *heap_brk;
	char *prev_heap_brk;

	if(heap_brk == 0)
		heap_brk = &__heap_start;


	prev_heap_brk = heap_brk;
	if (heap_brk + incr > &__heap_end)
	{
		_write (1, "Heap and stack collision\n", 25);
      	abort ();
	}

	heap_brk += incr;

	return (caddr_t) prev_heap_brk;
}

int _close(int file)
{
	return -1;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 0;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}

int _open(char *path, int flags, ...)
{
	/* Pretend like we always fail */
	return -1;
}

int _wait(int *status)
{
	errno = ECHILD;
	return -1;
}

int _unlink(char *name)
{
	errno = ENOENT;
	return -1;
}

int _gettimeofday(struct timeval *tv, struct timezone *tz){
	uint64_t cycles;
	time_t seconds, useconds;
	uint32_t clks_per_sec = read_pncsr(CSR_PNCLOCKINFO);

	cycles = read_csr(mcycle);
	cycles |= (uint64_t)read_csr(mcycleh) << 32;

	seconds = (time_t)(cycles / clks_per_sec);
  useconds = (time_t)(cycles / (clks_per_sec/1000000));

	if (tv != NULL){
		tv->tv_sec = seconds;
		tv->tv_usec = useconds;
		return 0;
	}

	return -1;
}

int _times(struct tms *buf)
{
	uint64_t cycles;
	time_t seconds;
	uint32_t clks_per_sec = read_pncsr(CSR_PNCLOCKINFO);

	cycles = read_csr(mcycle);
	cycles |= (uint64_t)read_csr(mcycleh) << 32;

	seconds = (time_t)(cycles / clks_per_sec);

	buf->tms_stime = seconds;
	buf->tms_utime = seconds;
	buf->tms_cstime = seconds;
	buf->tms_cutime = seconds;

	return (time_t)cycles;
}

int _stat(char *file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _link(char *old, char *new)
{
	errno = EMLINK;
	return -1;
}

int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

int _execve(char *name, char **argv, char **env)
{
	errno = ENOMEM;
	return -1;
}

int _feof(int *file)
{
	return 0;
}

int usleep(unsigned long useconds)
{}

