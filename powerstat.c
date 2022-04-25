/*
 * Copyright (C) 2011-2021 Canonical
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Author: Colin Ian King <colin.king@canonical.com>
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <inttypes.h>

#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <limits.h>
#include <dirent.h>
#include <ctype.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include <getopt.h>
#include <sched.h>

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <fcntl.h>

#include <linux/connector.h>
#include <linux/netlink.h>
#include <linux/cn_proc.h>

#define MIN_RUN_DURATION	(5*60)	/* We recommend a run of 5 minutes */
#define MIN_RUN_DURATION_RAPL	(60)	/* RAPL, 60 seconds is enough */

#define SAMPLE_DELAY		(10.0)	/* Delay between samples in seconds */
#define SAMPLE_DELAY_RAPL	(1.0)	/* Delay between samples for RAPL mode */

#define START_DELAY		(3*60)	/* Delay to wait before sampling */
#define START_DELAY_RAPL	(0.0)	/* Delay to wait before sampling, RAPL */

#define MIN_SAMPLE_DELAY	(0.5)	/* Minimum sample delay */
#define ROLLING_AVERAGE_SECS	(120)	/* 2 minute rolling average for power calculation */
#define STANDARD_AVERAGE_SECS	(120)
#define MAX_MEASUREMENTS 	(ROLLING_AVERAGE_SECS + 10)
#define MAX_PIDS		(32769)	/* Hash Max PIDs */
#define	RATE_ZERO_LIMIT		(0.001)	/* Less than this we call the power rate zero */
#define IDLE_THRESHOLD		(98)	/* Less than this and we assume the device is not idle */

#define MAX_POWER_DOMAINS	(16)	/* Maximum number of power domains allowed */
#define MAX_THERMAL_ZONES	(16)	/* Maximum number of thermal zones allowed */

/* Histogram specific constants */
#define MAX_DIVISIONS		(10)
#define HISTOGRAM_WIDTH		(40)

#define GOT_TGID		(0x01)
#define GOT_PPID		(0x02)
#define GOT_ALL			(GOT_TGID | GOT_PPID)

#define I915_ENERGY_UJ		"/sys/kernel/debug/dri/0/i915_energy_uJ"

#define SIZEOF_ARRAY(a)		(sizeof(a) / sizeof(a[0]))

#define MAX(x, y)		(x) > (y) ? (x) : (y)
#define MIN(x, y)		(x) > (y) ? (y) : (x)

#define FLOAT_TINY		(0.0000001)
#define FLOAT_CMP(a, b)		(fabs((a) - (b)) < FLOAT_TINY)

/* Statistics gathered from /proc/stat and process activity */
typedef enum {
	CPU_USER = 0,
	CPU_NICE,
	CPU_SYS,
	CPU_IDLE,
	CPU_IOWAIT,
	CPU_TOTAL,
	CPU_IRQ	,
	CPU_SOFTIRQ,
	CPU_INTR,
	CPU_CTXT,
	CPU_PROCS_RUN,
	CPU_PROCS_BLK,
	CPU_FREQ,
	PROC_FORK,
	PROC_EXEC,
	PROC_EXIT,
	POWER_TOTAL,
	POWER_GPU,
	POWER_DOMAIN_0,
	THERMAL_ZONE_0 = POWER_DOMAIN_0 + MAX_POWER_DOMAINS,
	MAX_VALUES = THERMAL_ZONE_0 + MAX_THERMAL_ZONES,
} stat_type;

/*
 *  C-State information, 1 for each unique C-state
 */
typedef struct cpu_state {
	struct cpu_state *hash_next;	/* next in hash table */
	struct cpu_state *list_next;	/* linked list of C-states */
	struct cpu_info  *cpu_info_list; /* linked list of CPUs that have this state */
	char 		*name;		/* C-State name, e.g. C1E-IVB */
	char		*name_short;	/* short name, e.g. C1E */
	uint64_t	latency;	/* latency */
	uint64_t	usage_total;	/* total usage count */
	double		resident;	/* percentage time resident in this state */
} cpu_state_t;

/*
 * Per CPU C-State info, total of these are
 * N-CPUs x N-CPU states
 */
typedef struct cpu_info {
	struct cpu_info	*hash_next;	/* Next in hash table */
	struct cpu_info *list_next;	/* Next in list of cpu_infos list */
	char		*state;		/* C-State sysfs name, e.g 'state2' */
	cpu_state_t 	*cpu_state;	/* C-State info */
	double		prev_tod;	/* Previous Time-of-Day */
	double		tod;		/* Time of Day */
	double		tod_diff;	/* Difference between current and previous tod */
	uint64_t	prev_time;	/* Previous #microseconds in this C-state */
	uint64_t	time;		/* Current #microseconds in this C-state */
	uint64_t	time_diff;	/* Difference in microseconds in this C-state */
	uint64_t	prev_usage;	/* Previous usage count */
	uint64_t	usage;		/* Current usage count */
	uint64_t	usage_diff;	/* Difference in usage count */
	uint32_t	cpu_id;		/* CPU ID */
} cpu_info_t;


typedef struct {
	double		threshold;
	double		scale;
	char 		*suffix;
} cpu_freq_scale_t;

/* Different freq scaling rates */
static const cpu_freq_scale_t cpu_freq_scale[] = {
	{ 1e1,  1e0,  "Hz" },
	{ 1e4,  1e3,  "KHz" },
	{ 1e7,  1e6,  "MHz" },
	{ 1e10, 1e9,  "GHz" },
	{ 1e13, 1e12, "THz" },
	{ 1e16, 1e15, "PHz" },
	{ -1.0, -1.0,  NULL }
};

#define MAX_STATES	(67)
#define MAX_CPUS	(1031)

static cpu_state_t	*cpu_states_list;	/* List of all CPU-states */
static cpu_state_t	*cpu_states[MAX_STATES];/* Hash of all CPU-states */
static cpu_info_t	*cpu_info[MAX_CPUS];	/* Hash of all CPU infos */
static const char *cpu_path = "/sys/devices/system/cpu";

/* Arg opt flags */
#define OPTS_SHOW_PROC_ACTIVITY	(0x0001)	/* dump out process activity */
#define OPTS_REDO_NETLINK_BUSY	(0x0002)	/* tasks fork/exec/exit */
#define OPTS_REDO_WHEN_NOT_IDLE	(0x0004)	/* when idle below idle_threshold */
#define OPTS_ZERO_RATE_ALLOW	(0x0008)	/* force allow zero rates */
#define OPTS_ROOT_PRIV		(0x0010)	/* has root privilege */
#define OPTS_STANDARD_AVERAGE	(0x0020)	/* calc standard average */
#define OPTS_RAPL		(0x0040)	/* use Intel RAPL */
#define OPTS_START_DELAY	(0x0080)	/* -d option used */
#define OPTS_SAMPLE_DELAY	(0x0100)	/* sample delay has been specified */
#define OPTS_DOMAIN_STATS	(0x0200)	/* Extra wide power domain stats */
#define OPTS_HISTOGRAM		(0x0400)	/* Histogram */
#define OPTS_CSTATES		(0x0800)	/* C-STATES dump */
#define OPTS_CPU_FREQ		(0x1000)	/* Average CPU frequency */
#define OPTS_NO_STATS_HEADINGS	(0x2000)	/* No stats headings */
#define OPTS_THERMAL_ZONE	(0x4000)	/* Thermal zones */
#define OPTS_GPU		(0x8000)	/* graphics */

#define OPTS_USE_NETLINK	(OPTS_SHOW_PROC_ACTIVITY | \
				 OPTS_REDO_NETLINK_BUSY |  \
				 OPTS_ROOT_PRIV)

#define SYS_CLASS_POWER_SUPPLY		"/sys/class/power_supply"
#define PROC_ACPI_BATTERY		"/proc/acpi/battery"

#define SYS_FIELD_VOLTAGE		"POWER_SUPPLY_VOLTAGE_NOW="
#define SYS_FIELD_WATTS_RATE		"POWER_SUPPLY_POWER_NOW="
#define SYS_FIELD_WATTS_LEFT		"POWER_SUPPLY_ENERGY_NOW="
#define SYS_FIELD_AMPS_RATE		"POWER_SUPPLY_CURRENT_NOW="
#define SYS_FIELD_AMPS_LEFT		"POWER_SUPPLY_CHARGE_NOW="
#define SYS_FIELD_STATUS_DISCHARGING  	"POWER_SUPPLY_STATUS=Discharging"

#if defined(__x86_64__) || defined(__x86_64) || \
    defined(__i386__) || defined(__i386)
#define POWERSTAT_X86
#endif

/* Measurement entry */
typedef struct {
	double	value;			/* Measurement value */
	time_t	when;			/* When it was measured */
} measurement_t;

/* Statistics entry */
typedef struct {
	double	value[MAX_VALUES];	/* /proc/stats values */
	bool	inaccurate[MAX_VALUES];	/* True if not accurate reading */
} stats_t;

/* /proc info cache */
typedef struct {
	pid_t	pid;			/* Process ID */
	char	*cmdline;		/* /proc/pid/cmdline text */
} proc_info_t;

/* Log item link list */
typedef struct log_item_t {
	struct log_item_t *next;	/* Next log item */
	char *text;			/* Log text */
} log_item_t;

/* Log list header */
typedef struct {
	log_item_t *head;		/* List head */
	log_item_t *tail;		/* List tail */
} log_t;

/* RAPL domain info */
typedef struct rapl_info {
	struct rapl_info *next;		/* Next RAPL domain */
	char *name;			/* RAPL name */
	char *domain_name;		/* RAPL domain name */
	double max_energy_uj;		/* Energy in micro Joules */
	double last_energy_uj;		/* Last energy reading in micro Joules */
	double t_last;			/* Time of last reading */
	bool is_package;		/* Is it a package? */
} rapl_info_t;

/* Thermal zone info */
typedef struct tz_info {
	struct tz_info *next;		/* Next TZ */
	char *name;			/* Thermal Zone pathname */
	char *type;			/* Thermal Zone type */
} tz_info_t;

#if defined(POWERSTAT_X86)
static rapl_info_t *rapl_list = NULL;		/* List of RAPL domains */
#endif
static tz_info_t *tz_list = NULL;		/* List of thermal zones */
static proc_info_t *proc_info[MAX_PIDS];	/* Proc hash table */
static uint32_t max_readings;			/* number of samples to gather */
static double sample_delay = SAMPLE_DELAY;	/* time between each sample in secs */
static int32_t start_delay = START_DELAY;	/* seconds before we start displaying stats */
static double idle_threshold = IDLE_THRESHOLD;	/* lower than this and the CPU is busy */
static log_t infolog;				/* log */
static uint32_t opts;				/* opt arg opt flags */
static volatile bool stop_recv;			/* sighandler stop flag */
static bool power_calc_from_capacity = false;	/* true of power is calculated via capacity change */
static const char *app_name = "powerstat";	/* name of application */
static const char *(*get_domain)(const int i) = NULL;
static uint8_t power_domains = 0;		/* Number of RAPL domains */
static uint8_t thermal_zones = 0;		/* Number of thermal zones */

static const char *tz_get_type(const int n);
static int tz_get_temperature(stats_t *stats);

/*
 *  Attempt to catch a range of signals so
 *  we can clean
 */
static const int signals[] = {
	/* POSIX.1-1990 */
#ifdef SIGHUP
	SIGHUP,
#endif
#ifdef SIGINT
	SIGINT,
#endif
#ifdef SIGQUIT
	SIGQUIT,
#endif
#ifdef SIGFPE
	SIGFPE,
#endif
#ifdef SIGTERM
	SIGTERM,
#endif
#ifdef SIGUSR1
	SIGUSR1,
#endif
#ifdef SIGUSR2
	SIGUSR2,
	/* POSIX.1-2001 */
#endif
#ifdef SIGXCPU
	SIGXCPU,
#endif
#ifdef SIGXFSZ
	SIGXFSZ,
#endif
	/* Linux various */
#ifdef SIGIOT
	SIGIOT,
#endif
#ifdef SIGSTKFLT
	SIGSTKFLT,
#endif
#ifdef SIGPWR
	SIGPWR,
#endif
#ifdef SIGINFO
	SIGINFO,
#endif
#ifdef SIGVTALRM
	SIGVTALRM,
#endif
};

/*
 *  strlcpy()
 *	BSD strlcpy
 */
static size_t strlcpy(char *dst, const char *src, size_t len)
{
	char *d = dst;
	const char *s = src;
	size_t n = len;

	if (n) {
		while (--n) {
			char c = *s++;

			*d++ = c;
			if (c == '\0')
				break;
		}
	}

	if (!n) {
		if (len)
			*d = '\0';
		while (*s)
			s++;
	}
	return (s - src - 1);
}

/*
 *   set_prioity
 *	set high priority to try and get netlink activity
 *	before short lived processes die
 */
static void set_priority(void)
{
	int max;
	struct sched_param param;
	int sched;

#if defined(SCHED_DEADLINE)
	sched = SCHED_DEADLINE;
#elif defined(SCHED_SCHED_FIFO)
	sched = SCHED_FIFO;
#elif defined(SCHED_RR)
	sched = SCHED_FIFO;
#else
	sched = SCHED_OTHER;	/* Oh well */
#endif
	if ((max = sched_get_priority_max(sched)) < 0)
		return;

	(void)memset(&param, 0, sizeof(param));
	param.sched_priority = max;
	(void)sched_setscheduler(getpid(), sched, &param);
}

/*
 *  file_get()
 *	read a line from a /sys file
 */
static char *file_get(const char *const file)
{
	FILE *fp;
	char buffer[4096];

	if ((fp = fopen(file, "r")) == NULL)
		return NULL;

	if (fgets(buffer, sizeof(buffer), fp) == NULL) {
		(void)fclose(fp);
		return NULL;
	}

	(void)fclose(fp);

	return strdup(buffer);
}

/*
 *  file_get_uint64()
 *	read a line from a /sys file
 */
static int file_get_uint64(const char *const file, uint64_t *val)
{
	FILE *fp;

	*val = 0;
	if ((fp = fopen(file, "r")) == NULL)
		return -1;

	if (fscanf(fp, "%" SCNu64, val) != 1) {
		*val = 0;
		(void)fclose(fp);
		return -1;
	}

	(void)fclose(fp);
	return 0;
}

/*
 *  cpu_freq_format()
 *	scale cpu freq into a human readable form
 */
static const char *cpu_freq_format(const double freq)
{
	static char buffer[40];
	char *suffix = "EHz";
	const double f = freq * 1000000.0; /* MHz to Hz */
	double scale = 1e18;
	size_t i;

	for (i = 0; cpu_freq_scale[i].suffix; i++) {
		if (f < cpu_freq_scale[i].threshold) {
			suffix = cpu_freq_scale[i].suffix;
			scale = cpu_freq_scale[i].scale;
			break;
		}
	}

	(void)snprintf(buffer, sizeof(buffer), "%5.2f %-3s",
		f / scale, suffix);

	return buffer;
}

/*
 *  get_parent_pid()
 *	get parent pid and set is_thread to true if process
 *	not forked but a newly created thread
 */
static pid_t get_parent_pid(const pid_t pid, bool *is_thread)
{
	FILE *fp;
	char path[PATH_MAX];
	char buffer[4096];
	pid_t tgid = 0, ppid = 0;
	unsigned int got = 0;

	*is_thread = false;
	(void)snprintf(path, sizeof(path), "/proc/%u/status", pid);
	if ((fp = fopen(path, "r")) == NULL)
		return 0;

	while (((got & GOT_ALL) != GOT_ALL) &&
	       (fgets(buffer, sizeof(buffer), fp) != NULL)) {
		if (!strncmp(buffer, "Tgid:", 5)) {
			if (sscanf(buffer + 5, "%10u", &tgid) == 1) {
				got |= GOT_TGID;
			} else {
				tgid = 0;
			}
		}
		if (!strncmp(buffer, "PPid:", 5)) {
			if (sscanf(buffer + 5, "%10u", &ppid) == 1) {
				got |= GOT_PPID;
			} else {
				ppid = 0;
			}
		}
	}
	(void)fclose(fp);

	if ((got & GOT_ALL) == GOT_ALL) {
		/*  TGID and PID are not the same if it is a thread */
		if (tgid != pid) {
			/* In this case, the parent is the TGID */
			ppid = tgid;
			*is_thread = true;
		}
	} else {
		ppid = 0;
	}

	return ppid;
}

/*
 *  tty_height()
 *      try and find height of tty
 */
static int tty_height(void)
{
#ifdef TIOCGWINSZ
	int fd = 0;
        struct winsize ws;

	(void)memset(&ws, 0, sizeof(ws));

        /* if tty and we can get a sane width, return it */
        if (isatty(fd) &&
            (ioctl(fd, TIOCGWINSZ, &ws) != -1) &&
            (0 < ws.ws_row) &&
            (ws.ws_row == (size_t)ws.ws_row))
                return ws.ws_row;
#endif
	return 25;	/* else standard tty 80x25 */
}


/*
 *  timeval_to_double
 *	timeval to a double (in seconds)
 */
static inline double timeval_to_double(const struct timeval *const tv)
{
	return (double)tv->tv_sec + ((double)tv->tv_usec / 1000000.0);
}

/*
 *  double_to_timeval
 *	seconds in double to timeval
 */
static inline void double_to_timeval(const double val, struct timeval *tv)
{
	tv->tv_sec = val;
	tv->tv_usec = (val - (time_t)val) * 1000000.0;
}

/*
 *  gettime_to_double()
 *	get time as a double
 */
static double gettime_to_double(void)
{
	struct timeval tv;

        if (gettimeofday(&tv, NULL) < 0) {
                (void)fprintf(stderr, "gettimeofday failed: errno=%d (%s).\n",
                        errno, strerror(errno));
		return -1.0;
        }
        return timeval_to_double(&tv);
}

/*
 *  get_time()
 *	Gather current time in buffer
 */
static void get_time(char *const buffer, const size_t buflen)
{
	struct tm tm;
	time_t now;

	now = time(NULL);
	if (now == ((time_t) -1)) {
		/* Unknown time! */
		(void)snprintf(buffer, buflen, "--:--:-- ");
		return;
	}
	(void)localtime_r(&now, &tm);

	(void)snprintf(buffer, buflen, "%2.2d:%2.2d:%2.2d ",
		tm.tm_hour, tm.tm_min, tm.tm_sec);
}

/*
 *  log_init()
 *	Initialise log head
 */
static inline void log_init(void)
{
	infolog.head = NULL;
	infolog.tail = NULL;
}

static int log_printf(const char *const fmt, ...) __attribute__((format(printf, 1, 2)));

/*
 *  log_printf()
 *	append log messages in log list
 */
static int log_printf(const char *const fmt, ...)
{
	char buffer[4096];
	char tmbuffer[10];
	va_list ap;
	log_item_t *log_item;
	size_t len;

	va_start(ap, fmt);
	get_time(tmbuffer, sizeof(tmbuffer));
	(void)vsnprintf(buffer, sizeof(buffer), fmt, ap);
	va_end(ap);

	if ((log_item = calloc(1, sizeof(*log_item))) == NULL) {
		(void)fprintf(stderr, "Out of memory allocating log item.\n");
		return -1;
	}
	len = strlen(buffer) + strlen(tmbuffer) + 1;
	if ((log_item->text = calloc(1, len)) == NULL) {
		free(log_item);
		(void)fprintf(stderr, "Out of memory allocating log item text.\n");
		return -1;
	}
	(void)snprintf(log_item->text, len, "%s%s", tmbuffer, buffer);

	if (infolog.head == NULL)
		infolog.head = log_item;
	else
		infolog.tail->next = log_item;

	infolog.tail = log_item;

	return 0;
}

/*
 *  log_dump()
 *	dump out any saved log messages
 */
static void log_dump(void)
{
	log_item_t *log_item;

	if (infolog.head != NULL)
		(void)printf("\nLog of fork()/exec()/exit() calls:\n");

	for (log_item = infolog.head; log_item; log_item = log_item->next)
		(void)printf("%s", log_item->text);
}

/*
 *  log_free()
 *	free log messages
 */
static void log_free(void)
{
	log_item_t *log_item = infolog.head;

	while (log_item) {
		log_item_t *log_next = log_item->next;
		free(log_item->text);
		free(log_item);

		log_item = log_next;
	}
	infolog.head = NULL;
	infolog.tail = NULL;
}

/*
 *  handle_sig()
 *	catch signals and flag a stop
 */
static void handle_sig(int dummy)
{
	(void)dummy;
	stop_recv = true;
}

/*
 *  netlink_connect()
 *	connect to netlink socket
 */
static int netlink_connect(void)
{
	int sock;
	struct sockaddr_nl addr;

    	if ((sock = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_CONNECTOR)) < 0) {
		if (errno == EPROTONOSUPPORT)
			return -EPROTONOSUPPORT;
		(void)fprintf(stderr, "socket failed: errno=%d (%s).\n",
			errno, strerror(errno));
		return -1;
	}

	(void)memset(&addr, 0, sizeof(addr));
	addr.nl_pid = getpid();
	addr.nl_family = AF_NETLINK;
	addr.nl_groups = CN_IDX_PROC;

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		(void)fprintf(stderr, "Bind failed: errno=%d (%s).\n",
			errno, strerror(errno));
		(void)close(sock);
		return -1;
	}

	return sock;
}

/*
 *  netlink_listen()
 *	proc connector listen
 */
static int netlink_listen(const int sock)
{
	struct iovec iov[3];
	struct nlmsghdr nlmsghdr;
	struct cn_msg cn_msg;
	enum proc_cn_mcast_op op;

	(void)memset(&nlmsghdr, 0, sizeof(nlmsghdr));
	nlmsghdr.nlmsg_len = NLMSG_LENGTH(sizeof(cn_msg) + sizeof(op));
	nlmsghdr.nlmsg_pid = getpid();
	nlmsghdr.nlmsg_type = NLMSG_DONE;
	iov[0].iov_base = &nlmsghdr;
	iov[0].iov_len = sizeof(nlmsghdr);

	(void)memset(&cn_msg, 0, sizeof(cn_msg));
	cn_msg.id.idx = CN_IDX_PROC;
        cn_msg.id.val = CN_VAL_PROC;
        cn_msg.len = sizeof(enum proc_cn_mcast_op);
	iov[1].iov_base = &cn_msg;
	iov[1].iov_len = sizeof(cn_msg);

        op = PROC_CN_MCAST_LISTEN;
	iov[2].iov_base = &op;
	iov[2].iov_len = sizeof(op);

	return writev(sock, iov, 3);
}

/*
 *  stats_set()
 *	set stats
 */
static void stats_set(
	stats_t *const stats,
	const double value,
	const bool inaccurate)
{
	int i;

	for (i = 0; i < MAX_VALUES; i++) {
		stats->value[i] = value;
		stats->inaccurate[i] = inaccurate;
	}
}

/*
 *  stats_set()
 *	clear stats
 */
static inline void stats_clear(stats_t *const stats)
{
	stats_set(stats, 0.0, false);
}

/*
 *  stats_clear_all()
 *	zero stats data
 */
static void stats_clear_all(stats_t *const stats, const long int n)
{
	int i;

	for (i = 0; i < n; i++)
		stats_clear(&stats[i]);
}

static void stats_cpu_freq_read(stats_t *const stats)
{
	struct dirent **cpu_list = NULL;
	int i, n_cpus, n = 0;
	double total_freq = 0;

	n_cpus = scandir("/sys/devices/system/cpu", &cpu_list, NULL, alphasort);
	for (i = 0; i < n_cpus; i++) {
		char *name = cpu_list[i]->d_name;

		if (!strncmp(name, "cpu", 3) && isdigit(name[3])) {
			char path[PATH_MAX];
			uint64_t freq;

			(void)snprintf(path, sizeof(path),
				"/sys/devices/system/cpu/%s/cpufreq/scaling_cur_freq",
				name);
			if (file_get_uint64(path, &freq) == 0) {
				total_freq += (double)freq / 1000.0;	/* In MHz */
				n++;
			}
		}
		free(cpu_list[i]);
	}
	if (n_cpus > -1)
		free(cpu_list);

	if (n)
		stats->value[CPU_FREQ] = total_freq / n;
	else
		stats->value[CPU_FREQ] = 0;
}

/*
 *  stats_read()
 *	gather pertinent /proc/stat data
 */
static int stats_read(stats_t *const stats)
{
	FILE *fp;
	char buf[4096];
	int i, j;

	static const stat_type indices[] = {
		CPU_USER, CPU_NICE, CPU_SYS, CPU_IDLE,
		CPU_IOWAIT, CPU_IRQ, CPU_SOFTIRQ, CPU_CTXT,
		CPU_INTR, CPU_PROCS_RUN, CPU_PROCS_BLK, -1
	};

	for (i = 0; (j = indices[i]) != -1; i++) {
		stats->value[j] = 0.0;
		stats->inaccurate[j] = true;
	}

	if ((fp = fopen("/proc/stat", "r")) == NULL) {
		(void)fprintf(stderr, "Cannot read /proc/stat, errno=%d (%s).\n",
			errno, strerror(errno));
		return -1;
	}

	while (fgets(buf, sizeof(buf), fp) != NULL) {
		if (strncmp(buf, "cpu ", 4) == 0)
			if (sscanf(buf, "%*s %15lf %15lf %15lf %15lf %15lf %15lf %15lf",
			    &(stats->value[CPU_USER]),
			    &(stats->value[CPU_NICE]),
			    &(stats->value[CPU_SYS]),
			    &(stats->value[CPU_IDLE]),
			    &(stats->value[CPU_IOWAIT]),
			    &(stats->value[CPU_IRQ]),
			    &(stats->value[CPU_SOFTIRQ])) == 7) {
				stats->inaccurate[CPU_USER] = false;
				stats->inaccurate[CPU_NICE] = false;
				stats->inaccurate[CPU_SYS] = false;
				stats->inaccurate[CPU_IDLE] = false;
				stats->inaccurate[CPU_IOWAIT] = false;
				stats->inaccurate[CPU_IRQ] = false;
				stats->inaccurate[CPU_SOFTIRQ] = false;
			}
		if (strncmp(buf, "ctxt ", 5) == 0)
			if (sscanf(buf, "%*s %15lf", &(stats->value[CPU_CTXT])) == 1)
				stats->inaccurate[CPU_CTXT] = false;
		if (strncmp(buf, "intr ", 5) == 0)
			if (sscanf(buf, "%*s %15lf", &(stats->value[CPU_INTR])) == 1)
				stats->inaccurate[CPU_INTR] = false;
		if (strncmp(buf, "procs_running ", 14) == 0)
			if (sscanf(buf, "%*s %15lf", &(stats->value[CPU_PROCS_RUN])) == 1)
				stats->inaccurate[CPU_PROCS_RUN] = false;
		if (strncmp(buf, "procs_blocked ", 14) == 0)
			if (sscanf(buf, "%*s %15lf", &(stats->value[CPU_PROCS_BLK])) == 1)
				stats->inaccurate[CPU_PROCS_BLK] = false;
	}
	(void)fclose(fp);

	if (opts & OPTS_CPU_FREQ)
		stats_cpu_freq_read(stats);

	return 0;
}

/*
 *  stats_sane()
 *	check if stats are accurate and calculate a
 *	sane -ve delta
 */
static double stats_sane(
	const stats_t *const s1,
	const stats_t *const s2,
	const int index)
{
	double ret;

	/* Discard inaccurate or empty stats */
	if (s1->inaccurate[index] || s2->inaccurate[index])
		return 0.0;

	/*
	 *  On Nexus 4 we occasionally get idle time going backwards so
	 *  work around this by ensuring we don't get -ve deltas.
	 */
	ret = s2->value[index] - s1->value[index];
	return ret < 0.0 ? 0.0 : ret;
}

#define INACCURATE(s1, s2, index)	\
	(s1->inaccurate[index] | s2->inaccurate[index])


/*
 *  stats_gather()
 *	gather up delta between last stats and current to get
 * 	some form of per sample accounting calculated.
 */
static bool stats_gather(
	const stats_t *const s1,
	const stats_t *const s2,
	stats_t *const res)
{
	double total;
	int i, j;
	bool inaccurate = false;

	static const int indices[] = {
		CPU_USER, CPU_NICE, CPU_SYS, CPU_IDLE,
		CPU_IOWAIT, -1
	};

	res->value[CPU_USER]    = stats_sane(s1, s2, CPU_USER);
	res->value[CPU_NICE]    = stats_sane(s1, s2, CPU_NICE);
	res->value[CPU_SYS]     = stats_sane(s1, s2, CPU_SYS);
	res->value[CPU_IDLE]    = stats_sane(s1, s2, CPU_IDLE);
	res->value[CPU_IOWAIT]  = stats_sane(s1, s2, CPU_IOWAIT);
	res->value[CPU_IRQ]     = stats_sane(s1, s2, CPU_IRQ);
	res->value[CPU_SOFTIRQ] = stats_sane(s1, s2, CPU_SOFTIRQ);
	res->value[CPU_CTXT]	= stats_sane(s1, s2, CPU_CTXT);
	res->value[CPU_INTR]	= stats_sane(s1, s2, CPU_INTR);

	for (i = 0; (j = indices[i]) != -1; i++)
		inaccurate |= (s1->inaccurate[j] | s2->inaccurate[j]);

	total = res->value[CPU_USER] + res->value[CPU_NICE] +
		res->value[CPU_SYS] + res->value[CPU_IDLE] +
		res->value[CPU_IOWAIT];

	/*
	 * This should not happen, but we need to avoid division
	 * by zero or weird results if the data is deemed valid
	 */
	if (!inaccurate && total <= 0.0)
		return false;
	res->value[CPU_TOTAL] = 100.0 * (total - res->value[CPU_IDLE]) / total;

	for (i = 0; (j = indices[i]) != -1; i++) {
		res->value[j] = (INACCURATE(s1, s2, j) || (total <= 0.0)) ?
			NAN : (100.0 * res->value[j]) / total;
	}
	res->value[CPU_CTXT] = (INACCURATE(s1, s2, CPU_CTXT) || (sample_delay <= 0.0)) ?
			NAN : res->value[CPU_CTXT] / sample_delay;
	res->value[CPU_INTR] = (INACCURATE(s1, s2, CPU_INTR) || (sample_delay <= 0.0)) ?
			NAN : res->value[CPU_INTR] / sample_delay;
	res->value[CPU_PROCS_RUN] = s2->inaccurate[CPU_PROCS_RUN] ? NAN : s2->value[CPU_PROCS_RUN];
	res->value[CPU_PROCS_BLK] = s2->inaccurate[CPU_PROCS_BLK] ? NAN : s2->value[CPU_PROCS_BLK];
	res->value[CPU_FREQ] = s2->value[CPU_FREQ];

	return true;
}

/*
 *  stats_headings()
 *	dump heading columns
 */
static void stats_headings(void)
{
	uint8_t i;

	if (opts & OPTS_USE_NETLINK)
		(void)printf("  Time    User  Nice   Sys  Idle    IO  Run Ctxt/s  IRQ/s Fork Exec Exit  Watts");
	else
		(void)printf("  Time    User  Nice   Sys  Idle    IO  Run Ctxt/s  IRQ/s  Watts");

	if (opts & (OPTS_DOMAIN_STATS |
		    OPTS_THERMAL_ZONE |
		    OPTS_CPU_FREQ |
		    OPTS_GPU))
		(void)putchar(' ');
		   
	if (opts & OPTS_DOMAIN_STATS) {
		for (i = 0; i < power_domains; i++)
			(void)printf(" %6.6s",
				get_domain ? get_domain(i) : "unknown");
	}
	if (opts & OPTS_THERMAL_ZONE) {
		for (i = 0; i < thermal_zones; i++)
			(void)printf(" %6.6s", tz_get_type(i));
	}
	if (opts & OPTS_CPU_FREQ)
		(void)printf(" %9.9s", "CPU Freq");
	if (opts & OPTS_GPU)
		(void)printf(" %6.6s", "GPU W");
	(void)printf("\n");
}

/*
 *  stats_ruler()
 *	pretty print ruler between rows
 */
static void stats_ruler(void)
{
	uint8_t i;

	if (opts & OPTS_USE_NETLINK)
		(void)printf("-------- ----- ----- ----- ----- ----- ---- ------ ------ ---- ---- ---- ------ ");
	else
		(void)printf("-------- ----- ----- ----- ----- ----- ---- ------ ------ ------ ");

	if (opts & OPTS_DOMAIN_STATS) {
		for (i = 0; i < power_domains; i++)
			(void)printf(" ------");
	}
	if (opts & OPTS_THERMAL_ZONE) {
		for (i = 0; i < thermal_zones; i++)
			(void)printf(" ------");
	}
	if (opts & OPTS_CPU_FREQ)
		(void)printf(" ---------");
	if (opts & OPTS_GPU)
		(void)printf(" ------");
	(void)printf("\n");
}

/*
 *  row_increment()
 *	bump row, reset if hit tty height
 */
static void row_increment(int *const row)
{
	if (!(opts & OPTS_NO_STATS_HEADINGS)) {
		int tty_rows = tty_height();

		(*row)++;
		if ((tty_rows > 2) && (*row >= tty_rows)) {
			stats_headings();
			*row = 2;
		}
	}
}

/*
 *  stats_print()
 *	print out statistics with accuracy depending if it's a summary or not
 */
static void stats_print(
	const char *const prefix,
	const bool summary,
	const stats_t *const s)
{
	char buf[10];
	uint8_t i;

	if (summary) {
		if (s->inaccurate[POWER_TOTAL])
			(void)snprintf(buf, sizeof(buf), " -N/A- ");
		else
			(void)snprintf(buf, sizeof(buf), "%6.2f ",
				s->value[POWER_TOTAL]);
	} else {
		(void)snprintf(buf, sizeof(buf), "%6.2f%s",
			s->value[POWER_TOTAL],
			s->inaccurate[POWER_TOTAL] ? "E" : " ");
	}

	if (opts & OPTS_USE_NETLINK) {
		char *fmt = summary ?
			"%8.8s %5.1f %5.1f %5.1f %5.1f %5.1f "
			"%4.1f %6.1f %6.1f %4.1f %4.1f %4.1f %s" :
			"%8.8s %5.1f %5.1f %5.1f %5.1f %5.1f "
			"%4.0f %6.0f %6.0f %4.0f %4.0f %4.0f %s";
		(void)printf(fmt,
			prefix,
			s->value[CPU_USER], s->value[CPU_NICE],
			s->value[CPU_SYS], s->value[CPU_IDLE],
			s->value[CPU_IOWAIT], s->value[CPU_PROCS_RUN],
			s->value[CPU_CTXT], s->value[CPU_INTR],
			s->value[PROC_FORK], s->value[PROC_EXEC],
			s->value[PROC_EXIT], buf);
	} else {
		char *fmt = summary ?
			"%8.8s %5.1f %5.1f %5.1f %5.1f %5.1f "
			"%4.1f %6.1f %6.1f %s" :
			"%8.8s %5.1f %5.1f %5.1f %5.1f %5.1f "
			"%4.0f %6.0f %6.0f %s";
		(void)printf(fmt,
			prefix,
			s->value[CPU_USER], s->value[CPU_NICE],
			s->value[CPU_SYS], s->value[CPU_IDLE],
			s->value[CPU_IOWAIT], s->value[CPU_PROCS_RUN],
			s->value[CPU_CTXT], s->value[CPU_INTR],
			buf);
	}
	if (opts & OPTS_DOMAIN_STATS) {
		for (i = 0; i < power_domains; i++)
			(void)printf(" %6.2f", s->value[POWER_DOMAIN_0 + i]);
	}
	if (opts & OPTS_THERMAL_ZONE) {
		for (i = 0; i < thermal_zones; i++)
			(void)printf(" %6.2f", s->value[THERMAL_ZONE_0 + i]);
	}
	if (opts & OPTS_CPU_FREQ)
		(void)printf(" %s", cpu_freq_format(s->value[CPU_FREQ]));
	if (opts & OPTS_GPU) {
		if (s->inaccurate[POWER_GPU])
			(void)printf("  -N/A-");
		else
			(void)printf(" %6.2f", s->value[POWER_GPU]);
	}
	(void)printf("\n");
}

/*
 *  stats_average_stddev_min_max()
 *	calculate average, std deviation, min and max
 */
static void stats_average_stddev_min_max(
	const stats_t *const stats,
	const int num,
	stats_t *const average,
	stats_t *const stddev,
	stats_t *const min,
	stats_t *const max,
	stats_t *const geometric_mean)
{
	int i, j, valid;

	for (j = 0; j < MAX_VALUES; j++) {
		double total = 0.0;
		double mant = 1.0;
		int64_t expon = 0;

		max->value[j] = -DBL_MAX;
		min->value[j] = DBL_MAX;

		for (valid = 0, i = 0; i < num; i++) {
			if (!stats[i].inaccurate[j]) {
				int e;
				double f;

				if (stats[i].value[j] > max->value[j])
					max->value[j] = stats[i].value[j];
				if (stats[i].value[j] < min->value[j])
					min->value[j] = stats[i].value[j];
				total += stats[i].value[j];
				
				f = frexp(stats[i].value[j], &e);
				mant *= f;
				expon += e;
				valid++;
			}
		}
		if (valid) {
			double inverse_n = 1.0 / (double)valid;

			geometric_mean->value[j] = pow(mant, inverse_n) * 
				pow(2.0, (double)expon / (double)valid);
			average->value[j] = total / (double)valid;
			total = 0.0;
			for (i = 0; i < num; i++) {
				if (!stats[i].inaccurate[j]) {
					double diff = (double)stats[i].value[j]
						 - average->value[j];
					diff = diff * diff;
					total += diff;
				}
			}
			stddev->value[j] = total / (double)num;
			stddev->value[j] = sqrt(stddev->value[j]);
		} else {
			average->inaccurate[j] = true;
			max->inaccurate[j] = true;
			min->inaccurate[j] = true;
			stddev->inaccurate[j] = true;

			average->value[j] = 0.0;
			max->value[j] = 0.0;
			min->value[j] = 0.0;
			stddev->value[j] = 0.0;
			geometric_mean->value[j] = 0.0;
		}
	}
}

/*
 *  stats_histogram()
 *	plot a simple ASCII art histogram
 */
static void stats_histogram(
	stats_t *const stats,
	const int num,
	const int value,
	const char *short_title,
	const char *title,
	const char *label,
	const double scale)
{
	int i, valid, digits = 0, width;
	double min = DBL_MAX, max = -DBL_MAX, division, prev;
	unsigned int bucket[MAX_DIVISIONS], max_bucket = 0;
	char buf[64];

	(void)memset(bucket, 0, sizeof(bucket));

	for (valid = 0, i = 0; i < num; i++) {
		if (!stats[i].inaccurate[value]) {
			stats[i].value[value] /= scale;
			if (stats[i].value[value] > max)
				max = stats[i].value[value];
			if (stats[i].value[value] < min)
				min = stats[i].value[value];
			valid++;
		}
	}

	if (valid <= 1)
		return;

	if (FLOAT_CMP(max - min, 0.0)) {
		(void)printf("\nRange is zero, cannot produce histogram of %s\n", short_title);
		return;
	}
	division = ((max * 1.000001) - min) / (MAX_DIVISIONS);
	for (i = 0; i < num; i++) {
		if (!stats[i].inaccurate[value]) {
			int v = floor((stats[i].value[value] - min) / division);
			v = v > MAX_DIVISIONS - 1 ? MAX_DIVISIONS -1 : v;
			bucket[v]++;
			if (max_bucket < bucket[v])
				max_bucket = bucket[v];
		}
	}

	(void)putchar('\n');
	(void)printf(title, num);
	(void)snprintf(buf, sizeof(buf), "%.0f", max);
	digits = strlen(buf) + 4;
	digits = (digits < 5) ? 5 : digits;
	width = 3 + (digits * 2);
	(void)snprintf(buf, sizeof(buf), "%*s%s",
		(width - 13) / 2, "", label);
	(void)printf("%-*s Count\n", width, buf);
	(void)snprintf(buf, sizeof(buf), "%%%d.3f - %%%d.3f %%5u ",
		digits, digits);

	prev = min;
	for (i = 0; i < MAX_DIVISIONS; i++) {
		unsigned int j;

		(void)printf(buf, prev, prev + division - 0.001, bucket[i]);

		for (j = 0; j < HISTOGRAM_WIDTH * bucket[i] / max_bucket; j++)
			(void)putchar('#');
		(void)putchar('\n');
		prev += division;
	}
}


/*
 *  calc_standard_average()
 *	calculate a standard average based on first sample
 *	and the current sample
 */
static void calc_standard_average(
	const double total_capacity,
	double *const rate,
	bool *const inaccurate)
{
	static time_t time_start = 0;
	static double total_capacity_start = 0.0;
	static bool first = true;
	time_t time_now, dt;
	double dw;

	time_now = time(NULL);
	if (time_now == ((time_t) -1)) {
		*rate = 0.0;
		*inaccurate = true;
		return;
	}

	if (first) {
		time_start = time_now;
		total_capacity_start = total_capacity;
		first = false;
		*rate = 0.0;
		*inaccurate = true;
		return;
	}

	dt = time_now - time_start;
	dw = total_capacity_start - total_capacity;
	if (dt <= 0 || dw <= 0.0) {
		/* Something is wrong, can't be a good sample */
		*rate = 0.0;
		*inaccurate = true;
		return;
	}

	*rate = 3600.0 * dw / dt;
	/* Only after a fairly long duration can we be sure it is reasonable */
	*inaccurate = dt < STANDARD_AVERAGE_SECS;
}

/*
 *  calc_rolling_average()
 *	calculate power by using rolling average
 *
 *  Battery is less helpful, we need to figure the power rate by looking
 *  back in time, measuring capacity drop and figuring out the rate from
 *  this.  We keep track of the rate over a sliding window of
 *  ROLLING_AVERAGE_SECS seconds.
 */
static void calc_rolling_average(
	const double total_capacity,
	double *const rate,
	bool *const inaccurate)
{
	static int index = 0;
	time_t time_now, dt;
	static measurement_t measurements[MAX_MEASUREMENTS];
	int i, j;

	time_now = time(NULL);
	if (time_now == ((time_t) -1)) {
		*rate = 0.0;
		*inaccurate = true;
		return;
	}
	measurements[index].value = total_capacity;
	measurements[index].when  = time_now;
	index = (index + 1) % MAX_MEASUREMENTS;
	*rate = 0.0;

	/*
	 * Scan back in time for a sample that's > ROLLING_AVERAGE_SECS
	 * seconds away and calculate power consumption based on this
	 * value and interval
	 */
	for (j = index, i = 0; i < MAX_MEASUREMENTS; i++) {
		j--;
		if (j < 0)
			j += MAX_MEASUREMENTS;

		if (measurements[j].when) {
			double dw = measurements[j].value - total_capacity;
			dt = time_now - measurements[j].when;
			*rate = 3600.0 * dw / dt;

			if (time_now - measurements[j].when > ROLLING_AVERAGE_SECS) {
				*inaccurate = false;
				break;
			}
		}
	}

	/*
	 *  We either have found a good measurement, or an estimate at this point, but
	 *  is it valid?
	 */
	if (*rate < 0.0) {
		*rate = 0.0;
		*inaccurate = true;
	}
}


/*
 *  calc_from_capacity()
 *	calculate either using standard or rolling averages
 */
static void calc_from_capacity(
	const double total_capacity,
	double *const rate,
	bool *const inaccurate)
{
	power_calc_from_capacity = true;

	if (opts & OPTS_STANDARD_AVERAGE)
		calc_standard_average(total_capacity, rate, inaccurate);
	else
		calc_rolling_average(total_capacity, rate, inaccurate);
}

/*
 *  power_get_sys_fs()
 *	get power discharge rate from battery via /sys interface
 */
static int power_get_sys_fs(
	stats_t *stats,
	bool *const discharging)
{
	DIR *dir;
	struct dirent *dirent;
	double total_watts = 0.0, total_capacity = 0.0;

	stats->value[POWER_TOTAL] = 0.0;
	stats->inaccurate[POWER_TOTAL] = true;
	*discharging = false;

	if ((dir = opendir(SYS_CLASS_POWER_SUPPLY)) == NULL) {
		(void)fprintf(stderr, "Device does not have %s, "
			"cannot run the test.\n",
			SYS_CLASS_POWER_SUPPLY);
		return -1;
	}

	do {
		dirent = readdir(dir);
		if (dirent && strlen(dirent->d_name) > 2) {
			char path[PATH_MAX];
			char *data;
			int  val;
			FILE *fp;

			/* Check that type field matches the expected type */
			(void)snprintf(path, sizeof(path), "%s/%s/type",
				SYS_CLASS_POWER_SUPPLY, dirent->d_name);
			if ((data = file_get(path)) != NULL) {
				bool mismatch = (strstr(data, "Battery") == NULL);
				free(data);
				if (mismatch)
					continue;	/* type don't match, skip this entry */
			} else
				continue;		/* can't check type, skip this entry */

			(void)snprintf(path, sizeof(path), "%s/%s/uevent",
				SYS_CLASS_POWER_SUPPLY, dirent->d_name);
			if ((fp = fopen(path, "r")) == NULL) {
				(void)fprintf(stderr, "Battery %s present but under "
					"supported - no state present.",
					dirent->d_name);
				(void)closedir(dir);
				return -1;
			} else {
				char buffer[4096];
				double voltage = 0.0;
				double amps_rate = 0.0;
				double amps_left = 0.0;
				double watts_rate = 0.0;
				double watts_left = 0.0;

				while (fgets(buffer, sizeof(buffer) - 1, fp) != NULL) {
					if (strstr(buffer, SYS_FIELD_STATUS_DISCHARGING))
						*discharging = true;

					if (strstr(buffer, SYS_FIELD_AMPS_LEFT) &&
					    strlen(buffer) > sizeof(SYS_FIELD_AMPS_LEFT) - 1) {
						if (sscanf(buffer + sizeof(SYS_FIELD_AMPS_LEFT) - 1, "%12d", &val) == 1)
							amps_left = (double)val / 1000000.0;
					}

					if (strstr(buffer, SYS_FIELD_WATTS_LEFT) &&
					    strlen(buffer) > sizeof(SYS_FIELD_WATTS_LEFT) - 1) {
						if (sscanf(buffer + sizeof(SYS_FIELD_WATTS_LEFT) - 1, "%12d", &val) == 1)
							watts_left = (double)val / 1000000.0;
					}

					if (strstr(buffer, SYS_FIELD_AMPS_RATE) &&
					    strlen(buffer) > sizeof(SYS_FIELD_AMPS_RATE) - 1) {
						if (sscanf(buffer + sizeof(SYS_FIELD_AMPS_RATE) - 1, "%12d", &val) == 1)
							amps_rate = (double)val / 1000000.0;
					}

					if (strstr(buffer, SYS_FIELD_WATTS_RATE) &&
					    strlen(buffer) > sizeof(SYS_FIELD_WATTS_RATE) - 1) {
						if (sscanf(buffer + sizeof(SYS_FIELD_WATTS_RATE) - 1, "%12d", &val) == 1)
							watts_rate = (double)val / 1000000.0;
					}

					if (strstr(buffer, SYS_FIELD_VOLTAGE) &&
					    strlen(buffer) > sizeof(SYS_FIELD_VOLTAGE) - 1) {
						if (sscanf(buffer + sizeof(SYS_FIELD_VOLTAGE) - 1, "%12d", &val) == 1)
							voltage = (double)val / 1000000.0;
					}
				}
				total_watts    += watts_rate + voltage * amps_rate;
				total_capacity += watts_left + voltage * amps_left;
				(void)fclose(fp);
			}
		}
	} while (dirent);

	(void)closedir(dir);

	if (! *discharging) {
		if (opts & OPTS_ZERO_RATE_ALLOW) {
			*discharging = true;	/* Lie */
			return 0;
		}
		(void)printf("Device is not discharging, cannot measure power usage.\n");
		(void)printf("Perhaps re-run with -z (ignore zero power)"
#if defined(POWERSTAT_X86)
			     " or -R (RAPL)"
#endif
			     "\n");
		return -1;
	}

	/*
 	 *  If the battery is helpful it supplies the rate already, in which case
	 *  we know the results from the battery are as good as we can and we don't
	 *  have to figure out anything from capacity change over time.
	 */
	if (total_watts > RATE_ZERO_LIMIT) {
		stats->value[POWER_TOTAL] = total_watts;
		stats->inaccurate[POWER_TOTAL] = (total_watts < 0.0);
		return 0;
	}

	/*  Rate not known, so calculate it from historical data, sigh */
	calc_from_capacity(total_capacity, &stats->value[POWER_TOTAL], &stats->inaccurate[POWER_TOTAL]);
	return 0;
}

/*
 *  power_get_proc_acpi()
 *	get power discharge rate from battery via /proc/acpi interface
 */
static int power_get_proc_acpi(
	stats_t *stats,
	bool *const discharging)
{
	DIR *dir;
	FILE *file;
	struct dirent *dirent;
	char filename[PATH_MAX];
	double total_watts = 0.0, total_capacity = 0.0;

	stats->value[POWER_TOTAL] = 0.0;
	stats->inaccurate[POWER_TOTAL] = true;
	*discharging = false;

	if ((dir = opendir(PROC_ACPI_BATTERY)) == NULL) {
		(void)fprintf(stderr, "Device does not have %s, "
			"cannot run the test.\n",
			PROC_ACPI_BATTERY);
		return -1;
	}

	while ((dirent = readdir(dir))) {
		double voltage = 0.0;

		double amps_rate = 0.0;
		double amps_left = 0.0;

		double watts_rate = 0.0;
		double watts_left = 0.0;

		char buffer[4096];
		char *ptr;

		if (strlen(dirent->d_name) < 3)
			continue;

		(void)sprintf(filename, "/proc/acpi/battery/%s/state", dirent->d_name);
		if ((file = fopen(filename, "r")) == NULL)
			continue;

		(void)memset(buffer, 0, sizeof(buffer));
		while (fgets(buffer, sizeof(buffer), file) != NULL) {
			if (strstr(buffer, "present:") &&
			    strstr(buffer, "no"))
				break;

			if (strstr(buffer, "charging state:") &&
			    (strstr(buffer, "discharging") || strstr(buffer, "critical")))
				*discharging = true;

			ptr = strchr(buffer, ':');
			if (ptr) {
				ptr++;
				if (strstr(buffer, "present voltage"))
					voltage = strtoull(ptr, NULL, 10) / 1000.0;

				if (strstr(buffer, "present rate")) {
					if (strstr(ptr, "mW"))
						watts_rate = strtoull(ptr, NULL, 10) / 1000.0 ;
					if (strstr(ptr, "mA"))
						amps_rate = strtoull(ptr, NULL, 10) / 1000.0;
				}

				if (strstr(buffer, "remaining capacity")) {
					if (strstr(ptr, "mW"))
						watts_left = strtoull(ptr, NULL, 10) / 1000.0 ;
					if (strstr(ptr, "mA"))
						amps_left = strtoull(ptr, NULL, 10) / 1000.0;
				}
			}
		}
		(void)fclose(file);

		/*
		 * Some HP firmware is broken and has an undefined
		 * 'present voltage' field and instead returns this in
		 * the design_voltage field, so work around this.
		 */
		if (FLOAT_CMP(voltage, 0.0)) {
			(void)sprintf(filename, "/proc/acpi/battery/%s/info", dirent->d_name);
			if ((file = fopen(filename, "r")) != NULL) {
				while (fgets(buffer, sizeof(buffer), file) != NULL) {
					ptr = strchr(buffer, ':');
					if (ptr) {
						ptr++;
						if (strstr(buffer, "design voltage:")) {
							voltage = strtoull(ptr, NULL, 10) / 1000.0;
							break;
						}
					}
				}
				(void)fclose(file);
			}
		}

		total_watts    += watts_rate + voltage * amps_rate;
		total_capacity += watts_left + voltage * amps_left;
	}
	(void)closedir(dir);

	if (! *discharging) {
		if (opts & OPTS_ZERO_RATE_ALLOW) {
			*discharging = true;	/* Lie */
			return 0;
		}
		(void)printf("Device is indicating it is not discharging and hence "
		       "we cannot measure power usage.\n");
		return -1;
	}

	/*
 	 * If the battery is helpful it supplies the rate already, in which
	 * case we know the results from the battery are as good as we can
	 * and we don't have to figure out anything from capacity change over
	 * time.
	 */
	if (total_watts > RATE_ZERO_LIMIT) {
		stats->value[POWER_TOTAL] = total_watts;
		stats->inaccurate[POWER_TOTAL] = (total_watts < 0.0);
		return 0;
	}

	/*  Rate not known, so calculate it from historical data, sigh */
	calc_from_capacity(total_capacity, &stats->value[POWER_TOTAL], &stats->inaccurate[POWER_TOTAL]);
	return 0;
}

#if defined(POWERSTAT_X86)

/*
 *  rapl_free_list()
 *	free RAPL list
 */
static void rapl_free_list(void)
{
	rapl_info_t *rapl = rapl_list;

	while (rapl) {
		rapl_info_t *next = rapl->next;

		free(rapl->name);
		free(rapl->domain_name);
		free(rapl);
		rapl = next;
	}
}

/*
 *  rapl_get_domain()
 *	get RAPL domain name for nth domain
 */
static const char *rapl_get_domain(const int n)
{
	int i;

	rapl_info_t *rapl = rapl_list;

	for (i = 0; i < n && rapl; i++) {
		rapl = rapl->next;
	}
	if (rapl) {
		if (rapl->is_package) {
			static char buf[128];

			(void)snprintf(buf, sizeof(buf), "pkg-%s", rapl->domain_name + 8);
			return buf;
		}
		return rapl->domain_name;
	}
	return "unknown";
}

/*
 *  rapl_get_domains()
 */
static int rapl_get_domains(void)
{
	DIR *dir;
        struct dirent *entry;
	int n = 0;

	dir = opendir("/sys/class/powercap");
	if (dir == NULL) {
		(void)printf("Device does not have RAPL, cannot measure power usage.\n");
		return -1;
	}

	while ((entry = readdir(dir)) != NULL) {
		char path[PATH_MAX];
		FILE *fp;
		rapl_info_t *rapl;

		/* Ignore non Intel RAPL interfaces */
		if (strncmp(entry->d_name, "intel-rapl", 10))
			continue;

		if ((rapl = calloc(1, sizeof(*rapl))) == NULL) {
			(void)fprintf(stderr, "Cannot allocate RAPL information.\n");
			(void)closedir(dir);
			return -1;
		}
		if ((rapl->name = strdup(entry->d_name)) == NULL) {
			(void)fprintf(stderr, "Cannot allocate RAPL name information.\n");
			(void)closedir(dir);
			free(rapl);
			return -1;
		}
		(void)snprintf(path, sizeof(path),
			"/sys/class/powercap/%s/max_energy_range_uj",
			entry->d_name);

		rapl->max_energy_uj = 0.0;
		if ((fp = fopen(path, "r")) != NULL) {
			if (fscanf(fp, "%lf\n", &rapl->max_energy_uj) != 1)
				rapl->max_energy_uj = 0.0;
			(void)fclose(fp);
		}
		(void)snprintf(path, sizeof(path),
			"/sys/class/powercap/%s/name",
			entry->d_name);

		rapl->domain_name = NULL;
		if ((fp = fopen(path, "r")) != NULL) {
			char domain_name[128];

			if (fgets(domain_name, sizeof(domain_name), fp) != NULL) {
				domain_name[strcspn(domain_name, "\n")] = '\0';
				rapl->domain_name = strdup(domain_name);
			}
			(void)fclose(fp);
		}
		if (rapl->domain_name == NULL) {
			free(rapl->name);
			free(rapl);
			continue;
		}

		rapl->is_package = (strncmp(rapl->domain_name, "package-", 8) == 0);
		rapl->next = rapl_list;
		rapl_list = rapl;
		n++;
	}
	(void)closedir(dir);
	power_domains = MIN(n, MAX_POWER_DOMAINS);

	if (!n)
		(void)printf("Device does not have any RAPL domains, cannot power measure power usage.\n");
	return n;
}

/*
 *  power_get_rapl_domain_names()
 *	get RAPL domain names
 */
static char *power_get_rapl_domain_names(void)
{
	char *names = NULL;
	size_t len = 0, n = 0;
	rapl_info_t *rapl;

	for (rapl = rapl_list; rapl && (n < MAX_POWER_DOMAINS); rapl = rapl->next, n++) {
		char new_name[strlen(rapl->domain_name) + 3];
		char *tmp;
		size_t new_len;

		new_name[0] = '\0';
		(void)snprintf(new_name, sizeof(new_name), "%s%s",
				len == 0 ? "" : ", ", rapl->domain_name);
		new_len = strlen(new_name);
		tmp = realloc(names, new_len + len + 1);
		if (!tmp) {
			(void)fprintf(stderr, "Out of memory allocating RAPL domain names.\n");
			free(names);
			names = NULL;
			break;
		}
		names = tmp;
		(void)strlcpy(names + len, new_name, new_len + 1);
		len += new_len;
	}

	return names;
}

/*
 *  power_get_rapl()
 *	get power discharge rate from battery via the RAPL interface
 */
static int power_get_rapl(
	stats_t *stats,
	bool *const discharging)
{
	double t_now;
	static bool first = true;
	rapl_info_t *rapl;
	int n = 0;

	stats->inaccurate[POWER_TOTAL] = false;	/* Assume OK until found otherwise */
	stats->value[POWER_TOTAL] = 0.0;
	get_domain = rapl_get_domain;
	*discharging = false;

	t_now = gettime_to_double();

	for (rapl = rapl_list; rapl && (n < MAX_POWER_DOMAINS); rapl = rapl->next) {
		char path[PATH_MAX];
		FILE *fp;
		double ujoules;

		(void)snprintf(path, sizeof(path),
			"/sys/class/powercap/%s/energy_uj",
			rapl->name);

		if ((fp = fopen(path, "r")) == NULL)
			continue;

		if (fscanf(fp, "%lf\n", &ujoules) == 1) {
			double t_delta = t_now - rapl->t_last;
			double last_energy_uj = rapl->last_energy_uj;

			rapl->t_last = t_now;

			/* Wrapped around since last time? */
			if (ujoules - rapl->last_energy_uj < 0.0) {
				rapl->last_energy_uj = ujoules;
				ujoules += rapl->max_energy_uj;
			} else {
				rapl->last_energy_uj = ujoules;
			}

			if (first || (t_delta <= 0.0)) {
				stats->value[POWER_DOMAIN_0 + n] = 0.0;
				stats->inaccurate[POWER_DOMAIN_0 + n] = true;
			} else {
				stats->value[POWER_DOMAIN_0 + n] =
					(ujoules - last_energy_uj) / (t_delta * 1000000.0);
				stats->inaccurate[POWER_DOMAIN_0 + n] = false;
			}
			if (rapl->is_package && !stats->inaccurate[POWER_DOMAIN_0 + n])
				stats->value[POWER_TOTAL] += stats->value[POWER_DOMAIN_0 + n];
			n++;
			*discharging = true;
		}
		(void)fclose(fp);
	}

	if (first) {
		stats->inaccurate[POWER_TOTAL] = true;
		first = false;
	}

	if (!n) {
		if (opts & OPTS_ZERO_RATE_ALLOW) {
			*discharging = true;	/* Lie */
			return 0;
		}
		(void)printf("Device does not have any RAPL domains, cannot power measure power usage.\n");
		return -1;
	}
	return 0;
}
#endif

/*
 *  power_get_gpu_i915()
 *	get Intel i915 GPU power stats
 *
 *  interface was removed from Linux 5.5 i915 driver by the commit:
 *
 *  commit 6311d260a1794d8ccef0ac8b39405fea0e1d9474
 *  Author: Tvrtko Ursulin <tvrtko.ursulin@intel.com>
 *  Date:   Thu Feb 6 14:07:25 2020 +0000
 *  drm/i915/debugfs: Remove i915_energy_uJ
 *
 */
static int power_get_gpu_i915(
	const stats_t *const s1,
	stats_t *s2,
	stats_t *const res)
{
#if defined(POWERSTAT_X86)
	uint64_t val;
	int ret;

	if (file_get_uint64(I915_ENERGY_UJ, &val) < 0) {
		s2->value[POWER_GPU] = 0.0;
		res->value[POWER_GPU] = 0.0;
		res->inaccurate[POWER_GPU] = true;
		ret = -1;
	} else {
		s2->value[POWER_GPU] = ((double)val / 1000000.0) / sample_delay;
		s2->inaccurate[POWER_GPU] = false;
		res->inaccurate[POWER_GPU] = false;
		res->value[POWER_GPU] = stats_sane(s1, s2, POWER_GPU);
		ret = 0;
	}
	return ret;
#else
	(void)s1;
	(void)s2;
	(void)res;

	return -1;
#endif
}

/*
 *  power_get_gpu()
 *	get generic GPU power stats
 */
static void power_get_gpu(
	const stats_t *const s1,
	stats_t *s2,
	stats_t *const res)
{

	if (power_get_gpu_i915(s1, s2, res) < 0) {
		s2->value[POWER_GPU] = 0.0;
		res->value[POWER_GPU] = 0.0;
		res->inaccurate[POWER_GPU] = true;
	}
}

/*
 *  tz_free_list()
 *	free thermal zone list
 */
static void tz_free_list(void)
{
	tz_info_t *tz = tz_list;

	while (tz) {
		tz_info_t *next = tz->next;

		free(tz->name);
		free(tz->type);
		free(tz);
		tz = next;
	}
}

/*
 *  tz_get_type()
 *	find the Nth thermal zone type
 */
static const char *tz_get_type(const int n)
{
	int i;

	tz_info_t *tz = tz_list;

	for (i = 0; i < n && tz; i++) {
		tz = tz->next;
	}
	if (tz) {
		return tz->type;
	}
	return "unknown";
}

/*
 *  tz_get_zones()
 *	collect valid thermal_zones
 */
static int tz_get_zones(void)
{
	DIR *dir;
        struct dirent *entry;
	int n = 0;

	dir = opendir("/sys/class/thermal");
	if (dir == NULL) {
		(void)printf("Device does not have thermal zones.\n");
		return -1;
	}

	while ((entry = readdir(dir)) != NULL) {
		char path[PATH_MAX];
		FILE *fp;
		tz_info_t *tz;

		/* Ignore non TZ interfaces */
		if (strncmp(entry->d_name, "thermal_zone", 12))
			continue;

		if ((tz = calloc(1, sizeof(*tz))) == NULL) {
			(void)fprintf(stderr, "Cannot allocate thermal information.\n");
			(void)closedir(dir);
			return -1;
		}
		if ((tz->name = strdup(entry->d_name)) == NULL) {
			(void)fprintf(stderr, "Cannot allocate thermal zone name information.\n");
			(void)closedir(dir);
			free(tz);
			return -1;
		}
		(void)snprintf(path, sizeof(path),
			"/sys/class/thermal/%s/type",
			entry->d_name);

		tz->type = NULL;
		if ((fp = fopen(path, "r")) != NULL) {
			char type[128];

			if (fgets(type, sizeof(type), fp) != NULL) {
				type[strcspn(type, "\n")] = '\0';
				tz->type  = strdup(type);
			}
			(void)fclose(fp);
		}
		if (tz->type == NULL) {
			free(tz->name);
			free(tz);
			continue;
		}

		tz->next = tz_list;
		tz_list = tz;
		n++;
	}
	(void)closedir(dir);
	thermal_zones = MIN(n, MAX_THERMAL_ZONES);

	return n;
}

/*
 *  tz_get_temperature()
 *	get temperatures from thermal zones
 */
static int tz_get_temperature(stats_t *const stats)
{
	tz_info_t *tz;
	int n = 0;

	for (tz = tz_list; tz && (n < thermal_zones); tz = tz->next) {
		char path[PATH_MAX];
		uint64_t temp;

		(void)snprintf(path, sizeof(path),
			"/sys/class/thermal/%s/temp",
			tz->name);
		if (file_get_uint64(path, &temp) == 0) {
			double temp_c = (double)temp / 1000.0;
			/* Threshold out insane values */
			if ((temp_c < 0.0) || (temp_c > 250.0))
				temp_c = 0.0;
			stats->value[THERMAL_ZONE_0 + n] = temp_c;
			n++;
		}
	}
	return 0;
}

/*
 *  power_get()
 *	fetch power via which ever interface is available
 */
static int power_get(
	stats_t *stats,
	bool *const discharging)
{
	struct stat buf;
	int i;

	for (i = POWER_TOTAL; i < POWER_DOMAIN_0 + MAX_POWER_DOMAINS; i++) {
		stats->value[i] = 0.0;
		stats->inaccurate[i] = true;
	}

#if defined(POWERSTAT_X86)
	if (opts & OPTS_RAPL)
		return power_get_rapl(stats, discharging);
#endif

	if ((stat(SYS_CLASS_POWER_SUPPLY, &buf) != -1) &&
	    S_ISDIR(buf.st_mode))
		return power_get_sys_fs(stats, discharging);

	if ((stat(PROC_ACPI_BATTERY, &buf) != -1) &&
	    S_ISDIR(buf.st_mode))
		return power_get_proc_acpi(stats, discharging);

	(void)fprintf(stderr, "Device does not seem to have a battery, cannot measure power.\n");
	return -1;
}

/*
 *  hash_djb2a()
 *	Hash a string, from Dan Bernstein comp.lang.c (xor version)
 */
static uint32_t hash_djb2a(const char *str, const uint32_t id)
{
	register uint32_t hash = 5381 + id;
	register int c;

	while ((c = *str++)) {
		/* (hash * 33) ^ c */
		hash = ((hash << 5) + hash) ^ c;
	}
	return hash;
}

/*
 *  cpu_state_get()
 *	get CPU C state based on name. We try to cache
 * 	these in cpu_states to make lookup faster
 */
static cpu_state_t *cpu_state_get(const char *name)
{
	uint32_t h = hash_djb2a(name, 0) % MAX_STATES;
	cpu_state_t *s = cpu_states[h];
	size_t len;
	char *ptr;

	while (s) {
		if (!strcmp(s->name, name))
			return s;
		s = s->hash_next;
	}
	if ((s = calloc(1, sizeof(*s))) == NULL)
		return NULL;
	if ((s->name = strdup(name)) == NULL) {
		free(s);
		return NULL;
	}
	if ((ptr = index(name, '-')) == NULL)
		len = strlen(name);
	else
		len = ptr - name;
	if ((s->name_short = calloc(1, len + 1)) == NULL) {
		free(s->name);
		free(s);
		return NULL;
	}
	(void)strlcpy(s->name_short, name, len + 1);
	s->name_short[len] = '\0';
	s->hash_next = cpu_states[h];
	cpu_states[h] = s;
	s->list_next = cpu_states_list;
	cpu_states_list = s;

	return s;
}

/*
 *  cpu_info_get()
 *	get per CPU active state information
 */
static cpu_info_t *cpu_info_get(const char *state, const uint32_t cpu_id)
{
	uint32_t h = hash_djb2a(state, cpu_id) % MAX_CPUS;
	cpu_info_t *ci = cpu_info[h];
	FILE *fp;
	char path[PATH_MAX];
	char buffer[64];

	while (ci) {
		if ((ci->cpu_id == cpu_id) &&
		    !strcmp(ci->state, state))
			return ci;
		ci = ci->hash_next;
	}

	if ((ci = calloc(1, sizeof(*ci))) == NULL)
		return NULL;
	ci->cpu_id = cpu_id;
	if ((ci->state = strdup(state)) == NULL) {
		free(ci);
		return NULL;
	}

	(void)snprintf(path, sizeof(path), "%s/cpu%" PRIu32 "/cpuidle/%s/name",
		cpu_path, cpu_id, state);

	(void)memset(buffer, 0, sizeof(buffer));
	if ((fp = fopen(path, "r")) != NULL) {
		if (fscanf(fp, "%63s", buffer) != 1)
			(void)strlcpy(buffer, "unknown", sizeof(buffer));
		(void)fclose(fp);
	} else {
		(void)strlcpy(buffer, state, sizeof(buffer));
	}
	if ((ci->cpu_state = cpu_state_get(buffer)) == NULL) {
		free(ci->state);
		free(ci);
		return NULL;
	}
	if (!ci->cpu_state->latency) {
		uint64_t val;
		(void)snprintf(path, sizeof(path), "%s/cpu%" PRIu32 "/cpuidle/%s/latency",
			cpu_path, cpu_id, state);
		if ((fp = fopen(path, "r")) != NULL) {
			if (fscanf(fp, "%" SCNu64, &val) == 1)
				ci->cpu_state->latency = val;
			(void)fclose(fp);
		}
	}

	ci->prev_time = 0;
	ci->time = 0;
	ci->hash_next = cpu_info[h];
	cpu_info[h] = ci;

	ci->list_next = ci->cpu_state->cpu_info_list;
	ci->cpu_state->cpu_info_list = ci;
	return ci;
}


/*
 *  cpu_info_update()
 *	update CPU state info
 */
static int cpu_info_update(cpu_info_t *ci)
{
	char path[PATH_MAX];
	FILE *fp;
	uint64_t val;

	(void)snprintf(path, sizeof(path), "%s/cpu%" PRIu32 "/cpuidle/%s/time",
		cpu_path, ci->cpu_id, ci->state);
	ci->prev_time = ci->time;
	ci->time = 0;
	if ((fp = fopen(path, "r")) != NULL) {
		if (fscanf(fp, "%" SCNu64, &val) == 1)
			ci->time = val;
		(void)fclose(fp);
	}
	ci->time_diff = ci->time - ci->prev_time;

	(void)snprintf(path, sizeof(path), "%s/cpu%" PRIu32 "/cpuidle/%s/usage",
		cpu_path, ci->cpu_id, ci->state);
	ci->prev_usage = ci->usage;
	ci->usage = 0;
	if ((fp = fopen(path, "r")) != NULL) {
		if (fscanf(fp, "%" SCNu64, &val) == 1)
			ci->usage = val;
		(void)fclose(fp);
	}
	ci->usage_diff = ci->usage - ci->prev_usage;

	ci->prev_tod = ci->tod;
	ci->tod = gettime_to_double();
	ci->tod_diff = ci->tod - ci->prev_tod;

	return 0;
}

/*
 *  cpu_states_update()
 *	update CPU state info for all CPUs
 */
static void cpu_states_update(void)
{
	struct dirent **cpu_list = NULL;
	int i, n_cpus;
	uint32_t max_cpu_id = 0;

	n_cpus = scandir(cpu_path, &cpu_list, NULL, alphasort);
	for (i = 0; i < n_cpus; i++) {
		char *name = cpu_list[i]->d_name;

		if (strlen(name) > 3 &&
		    !strncmp(name, "cpu", 3) &&
		    isdigit(name[3])) {
			int j, n_states;
			char path[PATH_MAX];
			struct dirent **states_list = NULL;
			uint32_t cpu_id = atoi(name + 3);

			if (max_cpu_id < cpu_id)
				max_cpu_id = cpu_id;

			(void)snprintf(path, sizeof(path), "%s/%s/cpuidle", cpu_path, name);

			n_states = scandir(path, &states_list, NULL, alphasort);
			for (j = 0; j < n_states; j++) {
				char *sname = states_list[j]->d_name;

				if (!strncmp("state", sname, 5)) {
					cpu_info_t *info;
					info = cpu_info_get(states_list[j]->d_name, cpu_id);
					if (info)
						cpu_info_update(info);
				}
				free(states_list[j]);
			}
			if (n_states > -1)
				free(states_list);
		}
		free(cpu_list[i]);
	}
	if (n_cpus > -1)
		free(cpu_list);
}

/*
 *  cpu_states_free()
 *	free cache of CPU state info
 */
static void cpu_states_free(void)
{
	cpu_state_t *s = cpu_states_list;

	while (s) {
		cpu_state_t *s_next = s->list_next;
		cpu_info_t *ci = s->cpu_info_list;

		while (ci) {
			cpu_info_t *ci_next = ci->list_next;

			free(ci->state);
			free(ci);
			ci = ci_next;
		}
		free(s->name);
		free(s->name_short);
		free(s);

		s = s_next;
	}
}

/*
 *  cpu_states_dump()
 *	dump out gather CPU state statistics
 */
static void cpu_states_dump(void)
{
	cpu_state_t *s;
	double c0_percent = 100.0;
	bool c0 = false;
	int name_width = 10;

	if (!cpu_states_list)
		return;

	for (s = cpu_states_list; s; s = s->list_next) {
		cpu_info_t *ci;
		uint64_t state_total = 0;
		double time_total = 0;
		const int len = strlen(s->name);

		if (len > name_width)
			name_width = len;

		for (ci = s->cpu_info_list; ci; ci = ci->list_next) {
			state_total += ci->time_diff;
			time_total += ci->tod_diff;
			s->usage_total += ci->usage_diff;
		}
		/* time_total into microseconds */
		time_total *= 1000000.0;
		s->resident = 100.0 * (double)state_total / (double)time_total;
		c0_percent -= s->resident;

		if (!strcmp(s->name_short, "C0"))
			c0 = true;
	}

	if (c0_percent >= 0.0) {
		(void)printf("\n%-*s %7s %10s %-8s\n",
			name_width, "C-State", "Resident", "Count", "Latency");
		for (s = cpu_states_list; s; s = s->list_next)
			(void)printf("%-*s %7.3f%% %10" PRIu64 "%8" PRIu64 "\n",
				name_width, s->name, s->resident, s->usage_total, s->latency);
		if (!c0)
			(void)printf("%-*s %7.3f%%\n", name_width, "C0", c0_percent);
	} else {
		(void)printf("\nUntrustworthy C-State states, ignoring.\n");
	}
}

/*
 *  proc_info_hash()
 * 	hash on PID
 */
static inline uint32_t proc_info_hash(const pid_t pid)
{
	return (uint32_t)pid % MAX_PIDS;
}

/*
 *  proc_cmdline()
 *	get a processes cmdline text
 */
static int proc_cmdline(
	const pid_t pid,
	char *const cmdline,
	const size_t size)
{
	int fd;
	char path[PATH_MAX];
	int n = 0;

	*cmdline = '\0';
	(void)snprintf(path, sizeof(path), "/proc/%d/cmdline", pid);
	if ((fd = open(path, O_RDONLY)) > -1) {
		n = read(fd, cmdline, size - 1);
		(void)close(fd);
		if (n > -1)
			cmdline[n] = '\0';
	}
	/*
	 * No cmdline, could be a kernel thread, so get the comm
	 * field instead
	 */
	if (!*cmdline) {
		(void)snprintf(path, sizeof(path), "/proc/%d/comm", pid);
		if ((fd = open(path, O_RDONLY)) > -1) {
			n = read(fd, cmdline, size - 1);
			(void)close(fd);
			if (n > 0)
				cmdline[n - 1] = '\0';	/* remove trailing \n */
		}
	}

	if (n < 1) {
		(void)strlcpy(cmdline, "<unknown>", size);
		n = 9;
	}

	return n;
}

/*
 *  proc_info_get()
 *	get proc info on a given pid
 */
static char *proc_info_get(const pid_t pid)
{
	uint32_t i = proc_info_hash(pid);
	int j;

	for (j = 0; j < MAX_PIDS; j++, i = (i + 1) % MAX_PIDS) {
		if ((proc_info[i] != NULL) && (proc_info[i]->pid == pid))
			return proc_info[i]->cmdline;
	}
	return "<unknown>";
}

/*
 *  proc_info_free()
 *	free cached process info and remove from hash table
 */
static void proc_info_free(const pid_t pid)
{
	unsigned int i = proc_info_hash(pid);
	int j;

	for (j = 0; j < MAX_PIDS; j++, i = (i + 1) % MAX_PIDS) {
		if ((proc_info[i] != NULL) && (proc_info[i]->pid == pid)) {
			free(proc_info[i]->cmdline);
			free(proc_info[i]);
			proc_info[i] = NULL;
			return;
		}
	}
}

/*
 *   proc_info_unload()
 *	free all hashed proc info entries
 */
static void proc_info_unload(void)
{
	int i;

	for (i = 0; i < MAX_PIDS; i++) {
		if (proc_info[i] != NULL) {
			free(proc_info[i]->cmdline);
			free(proc_info[i]);
			proc_info[i] = NULL;
		}
	}
}

/*
 *   proc_info_add()
 *	add processes info of a given pid to the hash table
 */
static int proc_info_add(const pid_t pid)
{
	uint32_t i;
	int j;
	proc_info_t *info;
	char path[PATH_MAX];
	char cmdline[1024];
	bool free_slot = false;

	i = proc_info_hash(pid);
	for (j = 0; j < MAX_PIDS; j++, i = (i + 1) % MAX_PIDS) {
		if (proc_info[i] == NULL) {
			free_slot = true;
			break;
		}
	}

	if (!free_slot)
		return -1;

	(void)memset(cmdline, 0, sizeof(cmdline));	/* keep valgrind happy */

	if ((info = calloc(1, sizeof(*info))) == NULL) {
		(void)fprintf(stderr, "Cannot allocate all proc info.\n");
		return -1;
	}
	info->pid = pid;

	(void)snprintf(path, sizeof(path), "/proc/%d/cmdline", info->pid);
	(void)proc_cmdline(pid, cmdline, sizeof(cmdline));

	if ((info->cmdline = malloc(strlen(cmdline)+1)) == NULL) {
		(void)fprintf(stderr, "Cannot allocate all proc info.\n");
		free(info);
		return -1;
	}
	(void)strcpy(info->cmdline, cmdline);
	proc_info[i] = info;

	return -1;
}

/*
 *  proc_info_load()
 *	load up all current processes info into hash table
 */
static int proc_info_load(void)
{
	DIR *dir;
	struct dirent *dirent;

	if ((dir = opendir("/proc")) == NULL)
		return -1;

	while ((dirent = readdir(dir))) {
		if (isdigit(dirent->d_name[0])) {
			errno = 0;
			pid_t pid = (pid_t)strtol(dirent->d_name, NULL, 10);
			if (!errno)
				proc_info_add(pid);
		}
	}

	(void)closedir(dir);
	return 0;
}

/*
 *   monitor()
 *	monitor system activity and power consumption
 */
static int monitor(const int sock)
{
	ssize_t len;
	int64_t t = 1;
	int redone = 0, row = 0;
	uint32_t readings = 0;
	stats_t *stats, s1, s2, average, stddev, min, max, geometric_mean;
	struct nlmsghdr *nlmsghdr;
	double time_start;

	if ((stats = calloc((size_t)max_readings, sizeof(*stats))) == NULL) {
		(void)fprintf(stderr, "Cannot allocate statistics table.\n");
		return -1;
	}

	stats_clear_all(stats, max_readings);
	stats_clear(&average);
	stats_clear(&stddev);
	stats_clear(&min);
	stats_clear(&max);
	stats_clear(&geometric_mean);

	stats_headings();
	row++;

	if ((time_start = gettime_to_double()) < 0.0) {
		free(stats);
		return -1;
	}

	(void)memset(&s1, 0, sizeof(s1));
	if (stats_read(&s1) < 0) {
		free(stats);
		return -1;
	}
	s2 = s1;

	if (opts & OPTS_CSTATES)
		cpu_states_update();
	if (opts & OPTS_GPU)
		power_get_gpu(&s2, &s1, &stats[readings]);

	while (!stop_recv && (readings < max_readings)) {
		double time_now, secs;
		int ret = 0;

		if ((time_now = gettime_to_double()) < 0.0) {
			free(stats);
			return -1;
		}
		/* Timeout to wait for in the future for this sample */
		secs = time_start + ((double)t * sample_delay) - time_now;

		if (secs > 0.0) {
			struct timeval tv;

			double_to_timeval(secs, &tv);

			if (opts & OPTS_USE_NETLINK) {
				fd_set readfds;
				FD_ZERO(&readfds);
				FD_SET(sock, &readfds);
				ret = select(sock+1, &readfds, NULL, NULL, &tv);
			} else {
				ret = select(0, NULL, NULL, NULL, &tv);
			}

			if (ret < 0) {
				if (errno == EINTR)
					break;
				(void)fprintf(stderr,"select failed: errno=%d (%s).\n",
					errno, strerror(errno));
				free(stats);
				return -1;
			}
		}

		/* Time out, so measure some more samples */
		if (ret == 0) {
			char tmbuffer[10];
			bool discharging;

			if (redone) {
				char buffer[80];
				int indent;
				(void)snprintf(buffer, sizeof(buffer),
					"--- Skipped samples(s) because of %s%s%s ---",
					(redone & OPTS_REDO_WHEN_NOT_IDLE) ? "low CPU idle" : "",
					(redone & (OPTS_REDO_WHEN_NOT_IDLE | OPTS_REDO_NETLINK_BUSY)) ==
					(OPTS_REDO_WHEN_NOT_IDLE | OPTS_REDO_NETLINK_BUSY) ? " and " : "",
					(redone & OPTS_REDO_NETLINK_BUSY) ? "fork/exec/exit activity" : "");
				indent = (80 - strlen(buffer)) / 2;
				row_increment(&row);
				(void)printf("%*.*s%s\n", indent, indent, "", buffer);
				redone = 0;
			}

			get_time(tmbuffer, sizeof(tmbuffer));
			if (stats_read(&s2) < 0) {
				free(stats);
				return -1;
			}
			/*
			 *  Total ticks was zero, something is broken,
			 *  so re-sample
			 */
			if (!stats_gather(&s1, &s2, &stats[readings])) {
				stats_clear(&stats[readings]);
				if (stats_read(&s1) < 0) {
					free(stats);
					return -1;
				}
				redone |= OPTS_REDO_WHEN_NOT_IDLE;
				continue;
			}


			if ((opts & OPTS_REDO_WHEN_NOT_IDLE) &&
			    (!stats[readings].inaccurate[CPU_IDLE]) &&
			    (stats[readings].value[CPU_IDLE] < idle_threshold)) {
				stats_clear(&stats[readings]);
				if (stats_read(&s1) < 0) {
					free(stats);
					return -1;
				}
				redone |= OPTS_REDO_WHEN_NOT_IDLE;
				continue;
			}

			if (power_get(&stats[readings], &discharging) < 0) {
				free(stats);
				return -1; 	/* Failure to read */
			}
			if (opts & OPTS_THERMAL_ZONE)
				tz_get_temperature(&stats[readings]);

			if (opts & OPTS_GPU)
				power_get_gpu(&s1, &s2, &stats[readings]);

			if (!discharging) {
				free(stats);
				return -1;	/* No longer discharging! */
			}

			row_increment(&row);
			stats_print(tmbuffer, false, &stats[readings]);
			readings++;
			s1 = s2;
			t++;
			continue;
		}

		if (opts & OPTS_USE_NETLINK) {
			bool redo = false;
			char __attribute__ ((aligned(NLMSG_ALIGNTO)))buf[4096];

        		if ((len = recv(sock, buf, sizeof(buf), 0)) == 0) {
				free(stats);
				return 0;
			}
			if (len == -1) {
				if ((errno == EINTR) ||
				    (errno == ENOBUFS) ||
				    (errno == ENOMEM)) {
					/* Retry */
					continue;
				} else {
					(void)fprintf(stderr,"recv failed: errno=%d (%s).\n",
						errno, strerror(errno));
					free(stats);
            				return -1;
				}
			}

			for (nlmsghdr = (struct nlmsghdr *)buf;
				NLMSG_OK (nlmsghdr, len);
				nlmsghdr = NLMSG_NEXT (nlmsghdr, len)) {
				struct cn_msg *cn_msg;
				struct proc_event *proc_ev;
				pid_t ppid;
				bool is_thread;

				if ((nlmsghdr->nlmsg_type == NLMSG_ERROR) ||
				    (nlmsghdr->nlmsg_type == NLMSG_NOOP))
      	                         	continue;

				cn_msg = NLMSG_DATA(nlmsghdr);

				if ((cn_msg->id.idx != CN_IDX_PROC) ||
				    (cn_msg->id.val != CN_VAL_PROC))
    	                            continue;

				proc_ev = (struct proc_event *)cn_msg->data;

       				switch (proc_ev->what) {
				case PROC_EVENT_FORK:
					stats[readings].value[PROC_FORK] += 1.0;
					ppid = get_parent_pid(proc_ev->event_data.fork.child_pid, &is_thread);

					proc_info_add(proc_ev->event_data.fork.child_pid);
					if (opts & OPTS_SHOW_PROC_ACTIVITY) {
						char *type = is_thread ? "clone" : "fork";
               					(void)log_printf("fork: parent pid=%d -> %s pid=%d (%s)\n",
							ppid,
							type,
                       					proc_ev->event_data.fork.child_pid,
							proc_info_get(proc_ev->event_data.fork.child_pid));
					}
					redo = true;
               				break;
           			case PROC_EVENT_EXEC:
					stats[readings].value[PROC_EXEC] += 1.0;
					if (opts & OPTS_SHOW_PROC_ACTIVITY) {
               					(void)log_printf("exec: pid=%d (%s)\n",
                       					proc_ev->event_data.exec.process_tgid,
							proc_info_get(proc_ev->event_data.exec.process_pid));
					}
					redo = true;
               				break;
           			case PROC_EVENT_EXIT:
					stats[readings].value[PROC_EXIT] += 1.0;
					if (opts & OPTS_SHOW_PROC_ACTIVITY) {
               					(void)log_printf("exit: pid=%d exit_code=%d (%s)\n",
                        				proc_ev->event_data.exit.process_pid,
                        				proc_ev->event_data.exit.exit_code,
							proc_info_get(proc_ev->event_data.exit.process_pid));
					}
					if (proc_ev->event_data.exit.process_pid ==
					    proc_ev->event_data.exit.process_tgid)
						proc_info_free(proc_ev->event_data.exit.process_pid);
					redo = true;
               				break;
				default:
               				break;
				}
			}

			/* Have we been asked to redo a sample on fork/exec/exit? */
			if (opts & OPTS_REDO_NETLINK_BUSY && redo) {
				stats_clear(&stats[readings]);
				if (stats_read(&s1) < 0) {
					free(stats);
					return -1;
				}
				redone |= OPTS_REDO_NETLINK_BUSY;
			}
        	}
	}
	if (opts & OPTS_CSTATES)
		cpu_states_update();

	/*
	 * Stats now gathered, calculate averages, stddev,
	 * min and max and display
	 */
	stats_average_stddev_min_max(stats, readings, &average,
		&stddev, &min, &max, &geometric_mean);
	if (readings > 0) {
		stats_ruler();
		stats_print("Average", true, &average);
		stats_print("GeoMean", true, &geometric_mean);
		stats_print("StdDev",  true, &stddev);
		stats_ruler();
		stats_print("Minimum", true, &min);
		stats_print("Maximum", true, &max);
		stats_ruler();
	}

	(void)printf("Summary:\n");
#if defined(POWERSTAT_X86)
	if (opts & OPTS_RAPL) {
		(void)printf("CPU: %6.2f Watts on average with standard deviation %-6.2f\n",
			average.value[POWER_TOTAL], stddev.value[POWER_TOTAL]);
	} else
#endif
	{
		(void)printf("System: %6.2f Watts on average with standard deviation %-6.2f\n",
			average.value[POWER_TOTAL], stddev.value[POWER_TOTAL]);
	}
	if (opts & OPTS_GPU) {
		(void)printf("GPU: %6.2f Watts on average with standard deviation %-6.2f\n",
			average.value[POWER_GPU], stddev.value[POWER_GPU]);
	}

#if defined(POWERSTAT_X86)
	if (opts & OPTS_RAPL) {
		char *names = power_get_rapl_domain_names();

		(void)printf("Note: power read from RAPL domains: %s.\n",
			names ? names : "unknown");
		(void)printf("These readings do not cover all the hardware in this device.\n");
		free(names);
	} else
#endif
	{
		if (power_calc_from_capacity) {
			(void)printf("Note: Power calculated from battery capacity drain, may not be accurate.\n");
		} else {
			if (opts & OPTS_STANDARD_AVERAGE)
				(void)printf("Note: The battery supplied suitable power data, -S option not required.\n");
		}
	}
	/* Just in case user wondered why no thermal zone info was displayed */
	if ((opts & OPTS_THERMAL_ZONE) && (!thermal_zones))
		(void)printf("Note: No thermal zones on this device.\n");

	if (opts & OPTS_CSTATES)
		cpu_states_dump();

	if (opts & OPTS_HISTOGRAM) {
		stats_histogram(stats, readings, POWER_TOTAL,
			"power measurements",
			"Histogram (of %d power measurements)\n\n",
			"Range (Watts)", 1.0);
		stats_histogram(stats, readings, CPU_TOTAL,
			"CPU utilization",
			"Histogram (of %d CPU utilization measurements)\n\n",
			"Range (%CPU)", 1.0);
		if (opts & OPTS_CPU_FREQ)
			stats_histogram(stats, readings, CPU_FREQ,
				"CPU average frequencies",
				"Histogram (of %d CPU average frequencies)\n\n",
				"Range (GHz)", 1000.0);
		if (opts & OPTS_THERMAL_ZONE) {
			uint8_t	tz;

			for (tz = 0; tz < thermal_zones; tz++) {
				char buf1[80], buf2[80];
				const char *type = tz_get_type(tz);

				(void)snprintf(buf1, sizeof(buf1), "thermal zone %s", type);
				(void)snprintf(buf2, sizeof(buf2), "Histogram (of %%d thermal zone %s readings)\n\n", type);
				stats_histogram(stats, readings, THERMAL_ZONE_0 + tz,
					buf1, buf2, "Range (°C)", 1.0);
			}
		}
	}

	free(stats);
	return 0;
}

/*
 *  show_help()
 *	simple help
 */
static void show_help(char *const argv[])
{
	(void)printf("%s, version %s\n\n", app_name, VERSION);
	(void)printf("usage: %s [options] [delay [count]]\n", argv[0]);
	(void)printf("\t-a enable all sampling collection options (-c, -f, -t and -H)\n");
	(void)printf("\t-b redo a sample if a system is busy, considered less than %d%% CPU idle\n", IDLE_THRESHOLD);
	(void)printf("\t-c show C-State statistics at end of the run\n");
	(void)printf("\t-d specify delay before starting, default is %" PRId32 " seconds\n", start_delay);
	(void)printf("\t-D show RAPL domain power measurements (enables -R option)\n");
	(void)printf("\t-f show average CPU frequency\n");
	(void)printf("\t-g show GPU power (currently just i915)\n");
	(void)printf("\t-h show help\n");
	(void)printf("\t-H show spread of measurements with power histogram\n");
	(void)printf("\t-i specify CPU idle threshold, used in conjunction with -b\n");
	(void)printf("\t-n no printing of table heading when screen scrolls\n");
	(void)printf("\t-p redo a sample if we see process fork/exec/exit activity\n");
	(void)printf("\t-r redo a sample if busy and we see process activity (same as -b -p)\n");
#if defined(POWERSTAT_X86)
	(void)printf("\t-R gather stats from Intel RAPL interface\n");
#endif
	(void)printf("\t-s show process fork/exec/exit activity log\n");
	(void)printf("\t-S calculate power from capacity drain using standard average\n");
	(void)printf("\t-t show Thermal Zone temperatures (in degrees C)\n");
	(void)printf("\t-z forcibly ignore zero power rate stats from the battery\n");
	(void)printf("\tdelay: delay between each sample, default is %.1f seconds\n", SAMPLE_DELAY);
	(void)printf("\tcount: number of samples to take\n");
}

int main(int argc, char * const argv[])
{
	int sock = -1, ret = EXIT_FAILURE, i;
	long int run_duration;
	bool discharging;
	struct sigaction new_action;
	stats_t dummy_stats;

	if (geteuid() == 0)
		opts |= OPTS_ROOT_PRIV;

	for (;;) {
#if defined(POWERSTAT_X86)
		int c = getopt(argc, argv, "abd:cDfghHi:nprszStR");
#else
		int c = getopt(argc, argv, "abd:cDfghHi:nprszSt");
#endif
		if (c == -1)
			break;
		switch (c) {
		case 'a':
			opts |= (OPTS_CSTATES |
				 OPTS_CPU_FREQ |
				 OPTS_HISTOGRAM |
				 OPTS_THERMAL_ZONE);
			if (opts & OPTS_ROOT_PRIV)
				 opts |= OPTS_GPU;
			break;
		case 'b':
			opts |= OPTS_REDO_WHEN_NOT_IDLE;
			break;
		case 'd':
			opts |= OPTS_START_DELAY;
			errno = 0;
			start_delay = strtol(optarg, NULL, 10);
			if (errno) {
				(void)fprintf(stderr, "Invalid value for start delay.\n");
				exit(EXIT_FAILURE);
			}
			if (start_delay < 0) {
				(void)fprintf(stderr, "Start delay must be 0 or more seconds.\n");
				exit(EXIT_FAILURE);
			}
			break;
		case 'c':
			opts |= OPTS_CSTATES;
			break;
		case 'D':
			opts |= (OPTS_DOMAIN_STATS | OPTS_RAPL);
			break;
		case 'f':
			opts |= OPTS_CPU_FREQ;
			break;
		case 'g':
			opts |= OPTS_GPU;
			break;
		case 'h':
			show_help(argv);
			exit(EXIT_SUCCESS);
		case 'H':
			opts |= OPTS_HISTOGRAM;
			break;
		case 'i':
			opts |= OPTS_REDO_WHEN_NOT_IDLE;
			idle_threshold = atof(optarg);
			if ((idle_threshold < 0.0) || (idle_threshold > 99.99)) {
				(void)fprintf(stderr, "Idle threshold must be between 0..99.99.\n");
				exit(EXIT_FAILURE);
			}
			break;
		case 'n':
			opts |= OPTS_NO_STATS_HEADINGS;
			break;
		case 'p':
			opts |= OPTS_REDO_NETLINK_BUSY;
			break;
		case 'r':
			opts |= (OPTS_REDO_NETLINK_BUSY | OPTS_REDO_WHEN_NOT_IDLE);
			break;
#if defined(POWERSTAT_X86)
		case 'R':
			opts |= OPTS_RAPL;
			break;
#endif
		case 's':
			opts |= OPTS_SHOW_PROC_ACTIVITY;
			break;
		case 'S':
			opts |= OPTS_STANDARD_AVERAGE;
			break;
		case 't':
			opts |= OPTS_THERMAL_ZONE;
			break;
		case 'z':
			opts |= OPTS_ZERO_RATE_ALLOW;
			break;
		case '?':
			(void)printf("Try '%s -h' for more information.\n", app_name);
			exit(EXIT_FAILURE);
		default:
			show_help(argv);
			exit(EXIT_FAILURE);
		}
	}

	if (optind < argc) {
		opts |= OPTS_SAMPLE_DELAY;
		errno = 0;
		sample_delay = atof(argv[optind++]);
		if (errno) {
			(void)fprintf(stderr, "Invalid value for start delay.\n");
			exit(EXIT_FAILURE);
		}
		if (sample_delay < MIN_SAMPLE_DELAY) {
			(void)fprintf(stderr, "Sample delay must be greater or equal "
				"to %.1f seconds.\n", MIN_SAMPLE_DELAY);
			exit(EXIT_FAILURE);
		}
	}
	if (opts & OPTS_THERMAL_ZONE)
		tz_get_zones();
	if ((opts & (OPTS_START_DELAY | OPTS_ZERO_RATE_ALLOW)) == OPTS_ZERO_RATE_ALLOW)
		start_delay = 0;
#if defined(POWERSTAT_X86)
	if ((opts & OPTS_RAPL) && (rapl_get_domains() < 1))
		exit(EXIT_FAILURE);
	if ((opts & (OPTS_START_DELAY | OPTS_RAPL)) == OPTS_RAPL)
		start_delay = 0;
	if ((opts & (OPTS_SAMPLE_DELAY | OPTS_RAPL)) == OPTS_RAPL)
		sample_delay = SAMPLE_DELAY_RAPL;

	if (opts & OPTS_RAPL)
		run_duration = MIN_RUN_DURATION_RAPL + START_DELAY_RAPL - start_delay;
	else
		run_duration = MIN_RUN_DURATION + START_DELAY - start_delay;
#else
	run_duration = MIN_RUN_DURATION + START_DELAY - start_delay;
#endif

	if (optind < argc) {
		errno = 0;
		max_readings = strtol(argv[optind++], NULL, 10);
		if (errno) {
			(void)fprintf(stderr, "Invalid value for maximum readings.\n");
			exit(EXIT_FAILURE);
		}
		if ((max_readings * sample_delay) < run_duration) {
			(void)fprintf(stderr, "Number of readings should be at least %ld.\n",
				(long int)(run_duration / sample_delay));
			exit(EXIT_FAILURE);
		}
	} else {
		max_readings = run_duration / sample_delay;
	}

	if (max_readings < 5) {
		(void)fprintf(stderr, "Number of readings too low.\n");
		exit(EXIT_FAILURE);
	}
	if (max_readings < 10)
		(void)fprintf(stderr, "Number of readings low, results may be inaccurate.\n");

	if (!(opts & OPTS_ROOT_PRIV) &&
	    (opts & (OPTS_USE_NETLINK | OPTS_GPU))) {
		(void)fprintf(stderr, "%s needs to be run with root privilege when using -g, -p, -r, -s options.\n", argv[0]);
		exit(ret);
	}

	if (power_get(&dummy_stats, &discharging) < 0)
		exit(ret);
	(void)printf("Running for %.1f seconds (%" PRIu32 " samples at %.1f second intervals).\n",
			sample_delay * max_readings, max_readings, sample_delay);
	(void)printf("Power measurements will start in %" PRId32 " seconds time.\n",
		start_delay);
	(void)printf("\n");

	if (start_delay > 0) {
		/* Gather up initial data */
		for (i = 0; i < start_delay; i++) {
			(void)printf("Waiting %" PRId32 " seconds before starting (gathering samples). \r", start_delay - i);
			(void)fflush(stdout);
			if (power_get(&dummy_stats, &discharging) < 0)
				exit(ret);
			if (sleep(1) || stop_recv)
				exit(ret);
			if (!discharging)
				exit(ret);
		}
		(void)printf("%79.79s\r", "");
	}

	(void)memset(&new_action, 0, sizeof(new_action));
	for (i = 0; i < (int)SIZEOF_ARRAY(signals); i++) {
		new_action.sa_handler = handle_sig;
		sigemptyset(&new_action.sa_mask);
		new_action.sa_flags = SA_RESTART;

		if (sigaction(signals[i], &new_action, NULL) < 0) {
			(void)fprintf(stderr, "sigaction failed: errno=%d (%s).\n",
				errno, strerror(errno));
			exit(EXIT_FAILURE);
		}
	}

	log_init();
	if (opts & OPTS_USE_NETLINK) {
    		sock = netlink_connect();
		if (sock == -EPROTONOSUPPORT) {
			if (opts & OPTS_SHOW_PROC_ACTIVITY)
				(void)printf("Cannot show process activity with this kernel.\n");
			opts &= ~OPTS_USE_NETLINK;
		} else if (sock < 0) {
			goto abort;
		} else {
			proc_info_load();

			if (netlink_listen(sock) < 0)
				goto abort_sock;
		}
	}

	if (power_get(&dummy_stats, &discharging) < 0)
		goto abort_sock;

	set_priority();

    	if (monitor(sock) ==  0)
		ret = EXIT_SUCCESS;

abort_sock:
	if (opts & OPTS_USE_NETLINK)
		proc_info_unload();
abort:
	if (opts & OPTS_USE_NETLINK) {
		log_dump();
		log_free();
		if (sock >= 0)
			(void)close(sock);
	}
#if defined(POWERSTAT_X86)
	if (opts & OPTS_RAPL)
		rapl_free_list();
#endif
	if (opts & OPTS_THERMAL_ZONE)
		tz_free_list();
	if (opts & OPTS_CSTATES)
		cpu_states_free();

	exit(ret);
}
