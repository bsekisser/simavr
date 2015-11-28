#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "sim_profiler.h"

#define CONFIG_RDTSC
//#define CONFIG_GETTIME

#ifdef CONFIG_RDTSC
	static inline uint64_t
		_sim_profiler_get_dtime(void)
	{
	   	uint32_t hi, lo;
	   	
		__asm__ __volatile__ ("xorl %%eax,%%edx\n" : : : "%eax", "%edx");
		__asm__ __volatile__ ("xorl %%edx,%%eax\n" : : : "%edx", "%eax");
		__asm__ __volatile__ ("xorl %%eax,%%edx\n" : : : "%eax", "%edx");
		__asm__ __volatile__ ("xorl %%edx,%%eax\n" : : : "%edx", "%eax");
		__asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
	
		uint64_t dtime_out = ((uint64_t)hi << 32) | (uint64_t)lo;
		return dtime_out;
	}
#else
	#ifdef CONFIG_GETTIME
		#include <time.h>
		static inline uint64_t
			_sim_profiler_get_dtime(void)
		{
			struct timespec t;

			clock_gettime(CLOCK_MONOTONIC, &t);

			uint64_t dtime_out = (((uint64_t)t.tv_sec) * 1000ULL * 1000ULL * 1000ULL) + ((uint64_t)t.tv_nsec);
			return dtime_out;
		}
	#else
		#include <sys/time.h>
		static inline uint64_t
			_sim_profiler_get_dtime(void)
		{
			struct timeval	t;
			uint64_t	dsec;
			uint64_t	dusec;

			gettimeofday(&t, (struct timezone *)NULL);
			dsec = t.tv_sec * 1000ULL * 1000ULL;
			dusec = ((uint64_t)t.tv_usec);

			uint64_t dtime_out = dsec + dusec;
			return dtime_out;
		}
	#endif
#endif

static inline uint64_t
_sim_profiler_get_elapsed_dtime(
	uint64_t start_time)
{
	uint64_t elapsed_dtime = _sim_profiler_get_dtime() - start_time;
	
	return elapsed_dtime;
}

typedef struct _sim_profiler_profile_t * _sim_profiler_profile_p;
typedef struct _sim_profiler_profile_t {
	const char *			name;
	void *				func;
	uint64_t			count;
	uint64_t			elapsed;
	uint64_t			start_time;
	double				average;
	double				percentage;
	uint8_t				iseq_flush;
	
	_sim_profiler_profile_p		sorted_next;
}_sim_profiler_profile_t;

typedef struct _sim_profiler_data_t * _sim_profiler_data_p;
typedef struct _sim_profiler_data_t {
	_sim_profiler_profile_t		inst[256];
	_sim_profiler_profile_t		inst_total;
	
	uint64_t 			iseq[65536];
	uint8_t				iseq_cache;
}_sim_profiler_data_t;

static _sim_profiler_data_t _sim_profiler_data;

void
sim_profiler_profile_iseq_start(uint8_t xop)
{
	_sim_profiler_profile_p profile = &_sim_profiler_data.inst[xop];
	profile->start_time = _sim_profiler_get_dtime();
}

void
sim_profiler_profile_iseq_stop(uint8_t xop)
{
	_sim_profiler_profile_p profile = &_sim_profiler_data.inst[xop];
	profile->elapsed = _sim_profiler_get_elapsed_dtime(profile->start_time);
	profile->count++;
	
	if (_sim_profiler_data.iseq_cache)
		_sim_profiler_data.iseq[(_sim_profiler_data.iseq_cache << 8) + xop]++;
	_sim_profiler_data.iseq_cache = profile->iseq_flush ? xop : 0;
}

#define KHz(hz) ((hz)*1000ULL)
#define MHz(hz) KHz(KHz(hz))

static uint64_t
_sim_profiler_calibrate_get_dtime_loop(void)
{
   	uint64_t start_time, elapsed_time = 0;

	int i;
	for(i = 1; i <= 1024; i++) {
		start_time = _sim_profiler_get_dtime();
		elapsed_time += _sim_profiler_get_elapsed_dtime(start_time);
	}
		
	uint64_t est_elapsed_per_loop = elapsed_time / i;

	printf("%s: estimate elapsed cycles per loop %016llu\n",
		__FUNCTION__, est_elapsed_per_loop);

	return est_elapsed_per_loop;
	
}

static uint64_t
_sim_profiler_calibrate_get_dtime_sleep(void)
{
   	uint64_t start_time = _sim_profiler_get_dtime();
	
	sleep(1);
		
	uint64_t elapsed_time = _sim_profiler_get_elapsed_dtime(start_time);

	printf("%s: estimate elapsed cycles during sleep(1) %016llu\n", __FUNCTION__, elapsed_time);

	return elapsed_time;
}

static uint64_t
_sim_profiler_dtime_calibrate(void)
{
	uint64_t cycle_time = _sim_profiler_calibrate_get_dtime_loop();
	uint64_t elapsed_time = 0;

	int i;
	for(i = 1; i <= 3; i++) {
		elapsed_time += _sim_profiler_calibrate_get_dtime_sleep() - cycle_time;
	}

	uint64_t ecdt = elapsed_time / i;
	double emhz = ecdt / MHz(1);
	printf("%s: elapsed time: %016llu  ecdt: %016llu  estMHz: %010.4f\n",
		__FUNCTION__, elapsed_time, ecdt, emhz);

	return(ecdt);
}

#define CLEAR_PROFILE_DATA(x) \
	x->average =0; \
	x->count = 0; \
	x->elapsed = 0; \
	x->func = 0; \
	x->percentage =0; \

void
sim_profiler_init(const char *op_names[256])
{
	if (!_sim_profiler_active)
		return;

	_sim_profiler_dtime_calibrate();

	for(int i = 0; i < 256 ; i++) {
		_sim_profiler_profile_p profile = &_sim_profiler_data.inst[i];
		CLEAR_PROFILE_DATA(profile);
		
		const char *name = op_names[i];
		profile->name = name;

		int skip_if = name == 0
			|| (0 == strncmp(name, "call", 4))
			|| (0 == strncmp(name, "cpse", 4))
			|| (0 == strncmp(name, "brxc", 4))
			|| (0 == strncmp(name, "brxs", 4))
			|| (0 == strncmp(name, "icall", 5))
			|| (0 == strncmp(name, "lcall", 5))
			|| (0 == strncmp(name, "ljmp", 4))
			|| (0 == strncmp(name, "rcall", 5))
			|| (0 == strncmp(name, "rjmp", 4))
			|| (0 == strncmp(name, "sbic", 4))
			|| (0 == strncmp(name, "sbis", 4))
			|| (0 == strncmp(name, "sbrc", 4))
			|| (0 == strncmp(name, "sbrs", 4));

		profile->iseq_flush = !skip_if;
		for(int j = 0; j < 256; j++)
			_sim_profiler_data.iseq[(i << 8) + j] = 0;
	}
}


#define AVERAGE(x) ((double)_sim_profiler_data.x.elapsed / \
	(double)_sim_profiler_data.x.count)

#define PERCENTAGE(x, y) (((double)_sim_profiler_data.x.elapsed / \
	(double)_sim_profiler_data.y.elapsed) * 100.0)

#define MAXixy(x, y, ix, iy) ((x > y) ? ix : iy)
#define MINixy(x, y, ix, iy) ((x < y) ? ix : iy)

#define ENQUEUE(_head, _elem, _elem_next, _sort_by) \
	do { \
		_elem->_elem_next = 0; \
		if (0 ==_head) { \
			_head = _elem; \
		} else if (_head->_sort_by > _elem->_sort_by) {\
			typeof(_elem) _test_elem = _head; \
			typeof(_elem) _prev_test_elem = 0; \
			while(_test_elem && (_test_elem->_sort_by > _elem->_sort_by)) { \
				_prev_test_elem = _test_elem; \
				_test_elem = _test_elem->_elem_next; \
			} \
			_elem->_elem_next = _test_elem; \
			_prev_test_elem->_elem_next = _elem; \
		} else { \
			_elem->_elem_next = _head; \
			_head = _elem; \
		} \
	} while(0);

extern void
sim_profiler_generate_report(void)
{
	if (!_sim_profiler_active)
		return;

	_sim_profiler_data_p core_profile = &_sim_profiler_data;
	_sim_profiler_profile_p opd;
	
	printf("\n\n>> core profile report\n");
	printf(">> raw profile list\n");

	const char * name_count_elapsed_average_status_line = 
		"name: %30s -- count: %012llu, elapsed: %016llu, avg: %012.4f\n";
	
	_sim_profiler_profile_p sorted_head = 0;
	for(int i = 0; i < 256; i++) {
		opd = &core_profile->inst[i];

		if(opd->name && opd->count) {
			core_profile->inst_total.count += opd->count;
			core_profile->inst_total.elapsed += opd->elapsed;
			
			opd->average = (double)opd->elapsed/(double)opd->count;
			printf(name_count_elapsed_average_status_line, 
				opd->name, opd->count, opd->elapsed, opd->average);

		/* sort next for count */
			ENQUEUE(sorted_head, opd, sorted_next, count);
		}
	}

	printf("\n\n>> instructions not called\n");
	
	for(int i = 0; i < 256; i++) {
		opd = &core_profile->inst[i];

		if(opd->name) {
			if(!opd->count)
				printf(name_count_elapsed_average_status_line, 
					opd->name, opd->count, opd->elapsed, opd->average);
			else
				opd->percentage = PERCENTAGE(inst[i], inst_total);
		}
	}

	printf("\n\n>> sorted by count\n");
	
	const char * name_count_elapsed_average_percentage_status_line = 
		"name: %30s -- count: %012llu, elapsed: %016llu, avg: %012.4f, pctg: %%%08.4f\n";

	for (opd = sorted_head; 0 != opd; opd = opd->sorted_next)
		printf(name_count_elapsed_average_percentage_status_line, 
			opd->name, opd->count, opd->elapsed, opd->average, opd->percentage);

	sorted_head = 0;
	for(int i = 0; i < 256; i++) {
		opd = &core_profile->inst[i];

		if(opd->name && opd->count) {
		/* sort next for average */
			ENQUEUE(sorted_head, opd, sorted_next, average);
		}
	}

	printf("\n\n>> sorted by average\n");
	
	for (opd = sorted_head; 0 != opd; opd = opd->sorted_next)
		printf(name_count_elapsed_average_percentage_status_line, 
			opd->name, opd->count, opd->elapsed, opd->average, opd->percentage);

	sorted_head = 0;
	for(int i = 0; i < 256; i++) {
		opd = &core_profile->inst[i];

		if(opd->name && opd->count) {
		/* sort next for average */
			ENQUEUE(sorted_head, opd, sorted_next, percentage);
		}
	}

	printf("\n\n>> sorted by percentage impact\n");

	for (opd = sorted_head; 0 != opd; opd = opd->sorted_next)
		printf(name_count_elapsed_average_percentage_status_line, 
			opd->name, opd->count, opd->elapsed, opd->average, opd->percentage);

	printf("\n\n>> top instruction sequence combinations\n");

	for(int x = 0; x < 256; x++) {
		if(0 == core_profile->inst[x].iseq_flush) {
			int ix = x << 8;
			for(int  y = 0; y < 256; y++) {
				core_profile->iseq[ix++] = 0;
			}
		}
	}	

	for(int  i = 0; i < 50; i++) {
		uint16_t maxiy = 0;
		for(uint32_t  ix = 0; ix < 65536; ix++) {
			maxiy = MAXixy(core_profile->iseq[ix], core_profile->iseq[maxiy], ix, maxiy);
		}
		
		_sim_profiler_profile_t *from = &core_profile->inst[maxiy >> 8];
		_sim_profiler_profile_t *tooo = &core_profile->inst[maxiy & 0xff];
		
		printf("from: %30s  --  to: %30s count: 0x%016llu %04x\n", 
			from->name, tooo->name, core_profile->iseq[maxiy], maxiy);

		core_profile->iseq[maxiy] = 0;
	}

}

