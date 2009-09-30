/*
 * High resolution timer test software
 *
 * Copyright (C) 2005,2006 Thomas Gleixner <tglx@linutronix.de>
 * (Enhanced by the Xenomai crew)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version
 * 2 as published by the Free Software Foundation;
 *
 */

#define VERSION_STRING "V 0.11xn"

#define INGO_TRACE 0
#define TGLX_TRACE 0
#ifndef IPIPE_TRACE
#define IPIPE_TRACE 0
#endif

#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <limits.h>

#include <linux/unistd.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/* Ugly, but .... */
#define gettid() syscall(__NR_gettid)
#define sigev_notify_thread_id _sigev_un._tid

extern int clock_nanosleep (clockid_t __clock_id, int __flags,
			    __const struct timespec *__req,
			    struct timespec *__rem);

#define USEC_PER_SEC	1000000
#define NSEC_PER_SEC	1000000000

#define MODE_CYCLIC		0
#define MODE_CLOCK_NANOSLEEP	1
#define MODE_SYS_ITIMER		2
#define MODE_SYS_NANOSLEEP	3
#define MODE_SYS_OFFSET		2

#define TIMER_RELTIME 	0

/* Must be power of 2 ! */
#define VALBUF_SIZE	16384

/* Struct to transfer parameters to the thread */
struct thread_param {
	int prio;
	int mode;
	int timermode;
	int signal;
	int clock;
	unsigned long max_cycles;
	struct thread_stat *stats;
	int bufmsk;
	unsigned long interval;
};

/* Struct for statistics */
struct thread_stat {
	unsigned long cycles;
	unsigned long cyclesread;
	long min;
	long max;
	long act;
	double avg;
	long *values;
	pthread_t thread;
	int threadstarted;
	int tid;
	int traced;
};

static int test_shutdown;
static int tracelimit = 100000;
static struct timespec start;

static inline void tsnorm(struct timespec *ts)
{
	while (ts->tv_nsec >= NSEC_PER_SEC) {
		ts->tv_nsec -= NSEC_PER_SEC;
		ts->tv_sec++;
	}
}

static inline long calcdiff(struct timespec t1, struct timespec t2)
{
	long diff;
	diff = USEC_PER_SEC * ((int) t1.tv_sec - (int) t2.tv_sec);
	diff += ((int) t1.tv_nsec - (int) t2.tv_nsec) / 1000;
	return diff;
}

/* 
 * timer thread
 * 
 * Modes:
 * - clock_nanosleep based
 * - cyclic timer based
 * 
 * Clock:
 * - CLOCK_MONOTONIC
 * - CLOCK_REALTIME
 * - CLOCK_MONOTONIC_HR
 * - CLOCK_REALTIME_HR
 * 
 */
void *timerthread(void *param)
{
	struct thread_param *par = param;
	struct sched_param schedp;
	sigset_t sigset;
	struct timespec now, next, interval;
	struct thread_stat *stat = par->stats;
	int policy = par->prio ? SCHED_FIFO : SCHED_OTHER;
	int err;
#ifdef __UNSUPPORTED
	struct itimerval itimer;
	struct sigevent sigev;
	timer_t timer;
	struct itimerspec tspec;
#endif
#if (INGO_TRACE + TGLX_TRACE)
	int stopped = 0;
#endif

	interval.tv_sec = par->interval / USEC_PER_SEC;
	interval.tv_nsec = (par->interval % USEC_PER_SEC) * 1000;

#if INGO_TRACE
	system("echo 1 > /proc/sys/kernel/trace_all_cpus");
	system("echo 1 > /proc/sys/kernel/trace_enabled");
	system("echo 1 > /proc/sys/kernel/trace_freerunning");
	system("echo 0 > /proc/sys/kernel/trace_print_at_crash");
	system("echo 1 > /proc/sys/kernel/trace_user_triggered");
	system("echo 0 > /proc/sys/kernel/trace_user_trigger_irq");
	system("echo 0 > /proc/sys/kernel/trace_verbose");
	system("echo 0 > /proc/sys/kernel/preempt_thresh");
	system("echo 0 > /proc/sys/kernel/wakeup_timing");
	system("echo 0 > /proc/sys/kernel/preempt_max_latency");
#endif

	stat->tid = gettid();

	sigemptyset(&sigset);
	sigaddset(&sigset, par->signal);
	sigprocmask(SIG_BLOCK, &sigset, NULL);
	
#ifdef __UNSUPPORTED
	if (par->mode == MODE_CYCLIC) {
		sigev.sigev_notify = SIGEV_THREAD_ID | SIGEV_SIGNAL;
		sigev.sigev_signo = par->signal;
		sigev.sigev_notify_thread_id = stat->tid;
		timer_create(par->clock, &sigev, &timer);
		tspec.it_interval = interval;
	}
#endif

	memset(&schedp, 0, sizeof(schedp));
	schedp.sched_priority = par->prio;
	err = pthread_setschedparam(pthread_self(), policy, &schedp);
#ifdef __XENO__
        if (err) {
            fprintf(stderr, "pthread_setschedparam: %s\n"
                    "(modprobe xeno_posix?)\n", strerror(err));
            test_shutdown = 1;
            return (void *) 1;
        }
#endif

	/* Get current time */
	next = start;
	next.tv_sec++;
	
#ifdef __UNSUPPORTED
	if (par->mode == MODE_CYCLIC) {
		if (par->timermode == TIMER_ABSTIME)
			tspec.it_value = next;
		else {
			tspec.it_value.tv_nsec = 0;
			tspec.it_value.tv_sec = 1;
		}
		timer_settime(timer, par->timermode, &tspec, NULL);
	}
	
	if (par->mode == MODE_SYS_ITIMER) {
		itimer.it_value.tv_sec = 1;
		itimer.it_value.tv_usec = 0;
		itimer.it_interval.tv_sec = interval.tv_sec;
		itimer.it_interval.tv_usec = interval.tv_nsec / 1000;
		setitimer (ITIMER_REAL,  &itimer, NULL);
	}
#endif

	stat->threadstarted++;

#if (INGO_TRACE + TGLX_TRACE)
	gettimeofday(0,(struct timezone *)1);
#endif

	while (!test_shutdown) {

		long diff;
#ifdef __UNSUPPORTED
		int sigs;
#endif

		/* Wait for next period */
		switch (par->mode) {
#ifdef __UNSUPPORTED
		case MODE_CYCLIC:
		case MODE_SYS_ITIMER:
			if (sigwait(&sigset, &sigs) < 0)
				goto out;
			break;
#endif
			
		case MODE_CLOCK_NANOSLEEP:
			if (par->timermode == TIMER_ABSTIME)
				clock_nanosleep(par->clock, TIMER_ABSTIME, &next, NULL);
			else {
				clock_gettime(par->clock, &now);
				clock_nanosleep(par->clock, TIMER_RELTIME, &interval, NULL);
				next.tv_sec = now.tv_sec + interval.tv_sec;
				next.tv_nsec = now.tv_nsec + interval.tv_nsec;
				tsnorm(&next);
			}
			break;
			
#ifdef __UNSUPPORTED
		case MODE_SYS_NANOSLEEP:
			clock_gettime(par->clock, &now);
			nanosleep(&interval, NULL);
			next.tv_sec = now.tv_sec + interval.tv_sec;
			next.tv_nsec = now.tv_nsec + interval.tv_nsec;
			tsnorm(&next);
			break;
#endif
		}
		clock_gettime(par->clock, &now);

		diff = calcdiff(now, next);
		if (diff < stat->min)
			stat->min = diff;
		if (diff > stat->max) {
			stat->max = diff;
#if IPIPE_TRACE
			if (stat->traced)
				xntrace_user_freeze(diff, 0);
#endif
		}
		stat->avg += (double) diff;

#if (INGO_TRACE + TGLX_TRACE)
		if (!stopped && (diff > tracelimit)) {
			stopped++;
			gettimeofday(0,0);
			test_shutdown++;
		}
#endif
		stat->act = diff;
		stat->cycles++;
		
		if (par->bufmsk)
			stat->values[stat->cycles & par->bufmsk] = diff;

		next.tv_sec += interval.tv_sec;
		next.tv_nsec += interval.tv_nsec;
		tsnorm(&next);
		
		if (par->max_cycles && par->max_cycles == stat->cycles)
			break;
	}

#ifdef __UNSUPPORTED
out:		
	if (par->mode == MODE_CYCLIC)
		timer_delete(timer);

	if (par->mode == MODE_SYS_ITIMER) {
		itimer.it_value.tv_sec = 0;
		itimer.it_value.tv_usec = 0;
		itimer.it_interval.tv_sec = 0;
		itimer.it_interval.tv_usec = 0;
		setitimer (ITIMER_REAL,  &itimer, NULL);
	}
#endif

	/* switch to normal */
	schedp.sched_priority = 0;
	pthread_setschedparam(pthread_self(), SCHED_OTHER, &schedp);

	stat->threadstarted = -1;

	return NULL;
}


/* Print usage information */
static void display_help(void)
{
	printf("cyclictest %s\n", VERSION_STRING);
	printf("Usage:\n"
	       "cyclictest <options>\n\n"
	       "-b USEC  --breaktrace=USEC send break trace command when latency > USEC\n"
	       "-c CLOCK --clock=CLOCK     select clock\n"
	       "                           0 = CLOCK_MONOTONIC (default)\n"
	       "                           1 = CLOCK_REALTIME\n"
	       "-d DIST  --distance=DIST   distance of thread intervals in us default=500\n"
	       "-i INTV  --interval=INTV   base interval of thread in us default=1000\n"
	       "-l LOOPS --loops=LOOPS     number of loops: default=0(endless)\n"
	       "-n       --nanosleep       use clock_nanosleep\n"
	       "-p PRIO  --prio=PRIO       priority of highest prio thread\n"
	       "-q       --quiet	   print only a summary on exit\n"
	       "-r       --relative        use relative timer instead of absolute\n"
#ifdef __UNSUPPORTED
	       "-s       --system          use sys_nanosleep and sys_setitimer\n"
#endif
	       "-t NUM   --threads=NUM     number of threads: default=1\n"
	       "-v       --verbose         output values on stdout for statistics\n"
	       "                           format: n:c:v n=tasknum c=count v=value in us\n");
	exit(0);
}

static int use_nanosleep = MODE_CLOCK_NANOSLEEP; /* make this default for now */
static int timermode  = TIMER_ABSTIME;
static int use_system;
static int priority = 99;
static int num_threads = 1;
static int max_cycles;
static int clocksel = 0;
static int verbose;
static int quiet;
static int interval = 1000;
static int distance = 500;

static int clocksources[] = {
	CLOCK_MONOTONIC,
	CLOCK_REALTIME,
};

/* Process commandline options */
static void process_options (int argc, char *argv[])
{
	int error = 0;
	for (;;) {
		int option_index = 0;
		/** Options for getopt */
		static struct option long_options[] = {
			{"breaktrace", required_argument, NULL, 'b'},
			{"clock", required_argument, NULL, 'c'},
			{"distance", required_argument, NULL, 'd'},
			{"interval", required_argument, NULL, 'i'},
			{"loops", required_argument, NULL, 'l'},
			{"nanosleep", no_argument, NULL, 'n'},
			{"priority", required_argument, NULL, 'p'},
			{"quiet", no_argument, NULL, 'q'},
			{"relative", no_argument, NULL, 'r'},
#ifdef __UNSUPPORTED
			{"system", no_argument, NULL, 's'},
#endif
			{"threads", required_argument, NULL, 't'},
			{"verbose", no_argument, NULL, 'v'},
			{"help", no_argument, NULL, '?'},
			{NULL, 0, NULL, 0}
		};
		int c = getopt_long (argc, argv, "b:c:d:i:l:np:qrt:v",
			long_options, &option_index);
		if (c == -1)
			break;
		switch (c) {
		case 'b': tracelimit = atoi(optarg); break;
		case 'c': clocksel = atoi(optarg); break;
		case 'd': distance = atoi(optarg); break;	
		case 'i': interval = atoi(optarg); break;	
		case 'l': max_cycles = atoi(optarg); break;
		case 'n': use_nanosleep = MODE_CLOCK_NANOSLEEP; break;
		case 'p': priority = atoi(optarg); break;
		case 'q': quiet = 1; break;
		case 'r': timermode = TIMER_RELTIME; break;	
#ifdef __UNSUPPORTED
		case 's': use_system = MODE_SYS_OFFSET; break;
#endif
		case 't': num_threads = atoi(optarg); break;
		case 'v': verbose = 1; break;
		case '?': error = 1; break;
		}				
	}

	if (clocksel < 0 || clocksel > ARRAY_SIZE(clocksources))
		error = 1;

	if (priority < 0 || priority > 99)
		error = 1;

	if (num_threads < 1)
		error = 1;

	if (error)
		display_help ();
}

static void sighand(int sig)
{
	test_shutdown = 1;
}

static void print_stat(struct thread_param *par, int index, int verbose)
{
	struct thread_stat *stat = par->stats;
	
	if (!verbose) {
		if (!quiet)
			printf("T:%2d (%5d) P:%2d I:%8ld C:%8lu "
			       "Min:%8ld Act:%8ld Avg:%8ld Max:%8ld\n",
			       index, stat->tid, par->prio, par->interval,
			       stat->cycles, stat->min, stat->act,
			       stat->cycles ?
			       (long)(stat->avg/stat->cycles) : 0, stat->max);
	} else {
		while (stat->cycles != stat->cyclesread) {
			long diff = stat->values[stat->cyclesread & par->bufmsk];
			printf("%8d:%8lu:%8ld\n", index, stat->cyclesread, diff);
			stat->cyclesread++;
		}
	}
}

int main(int argc, char **argv)
{
	sigset_t sigset;
	int signum = SIGALRM;
	int mode;
	struct thread_param *par;
	struct thread_stat *stat;
	pthread_attr_t thattr;
	int i, ret = -1;

#ifndef __XENO__
	if (geteuid()) {
		printf("need to run as root!\n");
		exit(-1);
	}
#endif

	mlockall(MCL_CURRENT | MCL_FUTURE);

	process_options(argc, argv);

	mode = use_nanosleep + use_system;
	
       	sigemptyset(&sigset);
       	sigaddset(&sigset, signum);
   	sigprocmask (SIG_BLOCK, &sigset, NULL);
	
	signal(SIGINT, sighand);
	signal(SIGTERM, sighand);
	
	par = calloc(num_threads, sizeof(struct thread_param));
	if (!par)
		goto out;
	stat = calloc(num_threads, sizeof(struct thread_stat));
	if (!stat)
		goto outpar;

	clock_gettime(clocksources[clocksel], &start);

	for (i = 0; i < num_threads; i++) {
		if (verbose) {
			stat[i].values = calloc(VALBUF_SIZE, sizeof(long));
			if (!stat[i].values)
				goto outall;
			par[i].bufmsk = VALBUF_SIZE - 1;
		}
		
		par[i].prio = priority;
		if (priority)
			priority--;
		par[i].clock = clocksources[clocksel];
		par[i].mode = mode;
		par[i].timermode = timermode;
		par[i].signal = signum;
		par[i].interval = interval;
		interval += distance;
		par[i].max_cycles = max_cycles;
		par[i].stats = &stat[i];
		stat[i].min = 1000000;
		stat[i].max = -1000000;
		stat[i].avg = 0.0;
		pthread_attr_init(&thattr);
		pthread_attr_setstacksize(&thattr, 131072);
		pthread_create(&stat[i].thread, &thattr, timerthread, &par[i]);
		stat[i].threadstarted = 1;
		stat[i].traced = (i == 0 && IPIPE_TRACE > 0);
	}
	
	while (!test_shutdown) {
		char lavg[256];
		int fd, len, allstopped;

		if (!verbose && !quiet) {
			fd = open("/proc/loadavg", O_RDONLY, 0666);
			len = read(fd, &lavg, 255);
			close(fd);
			lavg[len-1] = 0x0;
			printf("%s          \n\n", lavg);
		}

		allstopped = max_cycles ? 1 : 0;

		for (i = 0; i < num_threads; i++) {
			print_stat(&par[i], i, verbose);
			if (stat[i].cycles < max_cycles)
				allstopped = 0;
		}
		usleep(10000);
		if (test_shutdown || allstopped == num_threads)
			break;
		if (!verbose && !quiet)
			printf("\033[%dA", num_threads + 2);
	}
	if (quiet) {
		quiet = 0; /* Now we want to output the statistics */
		for (i = 0; i < num_threads; i++) {
			print_stat(&par[i], i, verbose);
		}
	}
	ret = 0;
 outall:
	test_shutdown = 1;
	for (i = 0; i < num_threads; i++) {
		if (stat[i].threadstarted > 0)
			pthread_kill(stat[i].thread, SIGTERM);
		if (stat[i].threadstarted)
			pthread_join(stat[i].thread, NULL);
		if (stat[i].values)
			free(stat[i].values);
	}
	free(stat);
 outpar:
	free(par);
 out:
	exit(ret);
}
