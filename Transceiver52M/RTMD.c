#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>

#ifdef __hpux
#include <sys/types.h>
#else
#include <stdint.h>
#endif

#define RTMD_THREAD_SAFE
//#define NO_CYCLE

#ifndef WITH_RTMD
#define WITH_RTMD
#endif
#include "RTMD.h"

#define  secs       _unsec._tval.secs
#define  usecs      _unsec._tval.usecs
#define  val        _unval._sln.val
#define  lineNumber _unval._sln.lineNumber
#define  tick       _unsec.tick
#define  val64      _unval.val64

#ifndef NO_CYCLE
#include "cycle.h"
#define EXTRA_SPACE (1024)
#else
#define EXTRA_SPACE 0
#endif

static RTMD_Node_t* mem = 0;

#if _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600
static void* node_alloc(size_t size)
{
    void* mem = NULL;
    int res = posix_memalign(&mem, sizeof(RTMD_Node_t), size*sizeof(RTMD_Node_t));
    if (res)
	perror("posix_memalign");
    return mem;
}
#else
//#warning Compiling without posix_memalign
#define node_alloc(x)  malloc((x)*sizeof(RTMD_Node_t))
#endif

/* Need to be volatile for proper work from signal handlers */
static volatile unsigned    ptr = 0;
static unsigned    count = 0;

int RTMD_IsFull(void) {
	return ptr==count;
}

void RTMD_InitStorage(unsigned max)
{
	int i;
	if (mem != NULL)
		return;

	mem = node_alloc((max + 2*EXTRA_SPACE));
	count = max;
	ptr = 0;

#ifdef NO_CYCLE
	for (i = 0; i < 32; i++)
		RTMD_VAL("#init", i);
#else
	for (i = 0; i < 1024; i++)
		RTMD_InitCycleTime("#cycle");
	for (i = 0; i < 64; i++)
		RTMD_VAL("#cyctst", i);
#endif
}

void RTMD_FlushStorage(const char* filename)
{
	FILE* f;
#ifndef NO_CYCLE
	int i;
	for (i = 0; i < 1024; i++)
		RTMD_InitCycleTime("#cycleend");
#endif

	f = fopen(filename, "w+b");
	if (f != NULL)
	{
		fwrite(mem, ptr * sizeof(RTMD_Node_t), 1, f);
		fclose(f);
	}
	else
	{
		fprintf(stderr, "RTMD: Can't flush RTMD to %s file!\n", filename);
	}

	free(mem);
	ptr = 0;
	count = 0;
	mem = NULL;
}

#ifdef NO_CYCLE
void RTMD_SetInt(const char* name, int clineNumber, int cval)
{
	if (mem == NULL)
		return;

	struct timeval tv;
	int err = gettimeofday(&tv, NULL);
	if (err)
	{
/*		fprintf(stderr, "RTMD: Can't get time!\n"); */
		return;
	}
	RTMD_SetTime(name, clineNumber, cval, &tv);
}
#endif

void RTMD_SetTime(const char* name, int clineNumber, int cval, const struct timeval *tv)
{
	if (mem == NULL)
		return;

	if (ptr < count)
	{
		unsigned slot;
#ifdef __hpux
		slot = ptr++;
#else
		slot =  __sync_fetch_and_add(&ptr, 1);
#endif
		/*
		 * You can use ' slot = ptr++; ' instead the instruction above,
		 * but this doesn't guarantee proper work in a multi-thread application
		 */

		if (slot < count)
		{
			mem[slot].secs = tv->tv_sec;
			mem[slot].usecs = tv->tv_usec;
			mem[slot].lineNumber = clineNumber;
			mem[slot].val = cval;

			strncpy(mem[slot].name, name, RTMD_MAX_NAME - 1);
			mem[slot].cycleFlag = RTMD_FLAG_GTOFDAY;
		}
	}
	else
	{
/* 		fprintf(stderr, "RTMD: Storage is full!\n");  */
	}

}



#ifndef NO_CYCLE
void RTMD_InitCycleTime(const char* name)
{
	struct timeval tv;
	int err = gettimeofday(&tv, NULL);
	if (err)
	{
/*		fprintf(stderr, "RTMD: Can't get time!\n"); */
		return;
	}

	if (mem == NULL)
		return;

	if (ptr < count + EXTRA_SPACE)
	{
		unsigned slot;
#ifdef __hpux
		slot = ptr++;
#else
		slot =  __sync_fetch_and_add(&ptr, 1);
#endif
		/*
		 * You can use ' slot = ptr++; ' instead the instruction above,
		 * but this doesn't guarantee proper work in a multi-thread application
		 */

		if (slot < count + EXTRA_SPACE)
		{
			mem[slot].secs = tv.tv_sec;
			mem[slot].usecs = tv.tv_usec;
			mem[slot].val64 = getticks();

			strncpy(mem[slot].name, name, RTMD_MAX_NAME - 1);
			mem[slot].cycleFlag = RTMD_FLAG_CYCLEINI;
		}
	}
	else
	{
/* 		fprintf(stderr, "RTMD: Storage is full!\n");  */
	}

}

void RTMD_SetInt(const char*  name, int clineNumber, int cval)
{
//	if (mem == NULL)
//		return;

	if (ptr < count)
	{
		unsigned slot;
#ifdef __hpux
		slot = ptr++;
#else
# ifdef RTMD_THREAD_SAFE
		slot =  __sync_fetch_and_add(&ptr, 1);
# else
		slot = ptr++;
# endif
#endif
		/*
		 * You can use ' slot = ptr++; ' instead the instruction above,
		 * but this doesn't guarantee proper work in a multi-thread application
		 */

		if (slot < count)
		{
			mem[slot].tick = getticks();
			mem[slot].lineNumber = clineNumber;
			mem[slot].val = cval;

#ifdef RTMD_SAFE_ALIGN
			strncpy(mem[slot].name, name, RTMD_MAX_NAME - 1);
#else
			//May not work on PA-RISC due to memory align
			*((int64_t * )mem[slot].name) = *((const int64_t * )name);
			*((int64_t * )&mem[slot].name[8]) = *((const int64_t * )&name[8]);
#endif
			mem[slot].cycleFlag = RTMD_FLAG_CYCLE;
		}
	}
	else
	{
/* 		fprintf(stderr, "RTMD: Storage is full!\n");  */
	}
}
#endif
