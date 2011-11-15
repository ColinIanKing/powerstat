/*
 * Copyright (C) 2011 Canonical
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
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>

#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <limits.h>
#include <dirent.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include <getopt.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <linux/connector.h>
#include <linux/netlink.h>
#include <linux/cn_proc.h>

#define START_DELAY	(15)		/* Delay to wait before sampling */
#define SAMPLE_DELAY	(10)		/* Delay between samples in seconds */
#define MAX_READINGS	(10)		/* Number of samples to take */
#define MAX_PIDS	(32769)		/* Hash Max PIDs */

#define CPU_USER	0
#define CPU_NICE	1
#define CPU_SYS		2
#define CPU_IDLE	3
#define CPU_IOWAIT	4
#define CPU_IRQ		5
#define CPU_SOFTIRQ	6
#define CPU_INTR	7
#define CPU_CTXT	8
#define CPU_PROCS_RUN	9
#define CPU_PROCS_BLK	10
#define POWER_RATE	11
#define PROC_FORK	12
#define PROC_EXEC	13
#define PROC_EXIT	14
#define MAX_VALUES	15

#define OPTS_SHOW_PROC_ACTIVITY	(0x0001)
#define OPTS_REDO_WHEN_BUSY	(0x0002)

/* Statistics entry */
typedef struct {
	double	value[MAX_VALUES];
} stats_t;

/* /proc info cache */
typedef struct {
	pid_t	pid;		/* Process ID */
	char	*cmdline;	/* /proc/pid/cmdline text */
} proc_info_t;

/* Log item link list */
typedef struct log_item_t {
	struct log_item_t *next;
	char *text;
} log_item_t;

/* Log list header */
typedef struct {
	log_item_t *head;
	log_item_t *tail;
} log_t;

static proc_info_t *proc_info[MAX_PIDS];	/* Proc hash table */
static int max_readings = MAX_READINGS;
static int sample_delay = SAMPLE_DELAY;
static int start_delay  = START_DELAY;
static log_t infolog;
static int opts;
static volatile int stop_recv;

/*
 *  time_now()
 *	Gather current time in buffer
 */
static void time_now(char *buffer, size_t buflen)
{
	struct tm tm;
	time_t now;

	time(&now);
	localtime_r(&now, &tm);

	snprintf(buffer, buflen, "%2.2d:%2.2d:%2.2d ",
		tm.tm_hour, tm.tm_min, tm.tm_sec);
}

/*
 *  log_init()
 *	Initialise log head
 */
static void log_init(void)
{
	infolog.head = NULL;
	infolog.tail = NULL;
}

/*
 *  log_printf()
 *	append log messages in log list
 */
static int log_printf(char *fmt, ...)
{
	char buffer[4096];
	char tmbuffer[10];
	va_list ap;
	va_start(ap, fmt);
	log_item_t *log_item;
	size_t len;

	time_now(tmbuffer, sizeof(tmbuffer));
	vsnprintf(buffer, sizeof(buffer), fmt, ap);

	if ((log_item = calloc(1, sizeof(log_t))) == NULL) {
		fprintf(stderr, "Out of memory allocating log item\n");
		return -1;
	}
	len = strlen(buffer) + strlen(tmbuffer) + 1;
	log_item->text = calloc(1, len);
	snprintf(log_item->text, len, "%s%s", tmbuffer, buffer);

	if (infolog.head == NULL) {
		infolog.head = log_item;
		infolog.tail = log_item;
	} else {
		infolog.tail->next = log_item;
		infolog.tail = log_item;
	}

	va_end(ap);

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
		printf("\nLog of fork()/exec()/exit() calls:\n");

	for (log_item = infolog.head; log_item != NULL; log_item = log_item->next)
		printf("%s", log_item->text);
}

/*
 *  log_free()
 *	free log messages
 */
static void log_free(void)
{
	log_item_t *log_item = infolog.head;

	while (log_item != NULL) {
		log_item_t *log_next = log_item->next;
		free(log_item->text);
		free(log_item);

		log_item = log_next;
	}
	infolog.head = NULL;
	infolog.tail = NULL;
}

/*
 *  handle_sigint()
 *	catch SIGINT and flag a stop
 */
static void handle_sigint(int dummy)
{
	stop_recv = 1;
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
		fprintf(stderr, "Socket failed: %s\n", strerror(errno));
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.nl_pid = getpid();
	addr.nl_family = AF_NETLINK;
	addr.nl_groups = CN_IDX_PROC;

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		fprintf(stderr, "Bind failed: %s\n", strerror(errno));
		return -1;
	}

	return sock;
}

/*
 *  netlink_listen()
 *	proc connector listen
 */
static int netlink_listen(int sock)
{
	struct iovec iov[3];
	struct nlmsghdr nlmsghdr;
	struct cn_msg cn_msg;
	enum proc_cn_mcast_op op;

	memset(&nlmsghdr, 0, sizeof(nlmsghdr));
	nlmsghdr.nlmsg_len = NLMSG_LENGTH(sizeof(cn_msg) + sizeof(op));
	nlmsghdr.nlmsg_pid = getpid();
	nlmsghdr.nlmsg_type = NLMSG_DONE;
	iov[0].iov_base = &nlmsghdr;
	iov[0].iov_len = sizeof(nlmsghdr);

	memset(&cn_msg, 0, sizeof(cn_msg));
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
 *  stats_clear()
 *	zero stats data
 */
static void stats_clear(stats_t *stats, int n)
{
	int i, j;

	for (j=0; j<n; j++)
		for (i=0; i<MAX_VALUES; i++)
			stats[j].value[i] = 0.0;
}

/*
 *  stats_read()
 *	gather pertinent /proc/stat data
 */
static int stats_read(stats_t *info)
{
	FILE *fp;
	char buf[4096];

	if ((fp = fopen("/proc/stat", "r")) == NULL)
		return -1;

	while (fgets(buf, sizeof(buf), fp) != NULL) {
		if (strncmp(buf, "cpu ", 4) == 0)
			sscanf(buf, "%*s %lf %lf %lf %lf %lf %lf %lf",
				&(info->value[CPU_USER]),
				&(info->value[CPU_NICE]),
				&(info->value[CPU_SYS]),
				&(info->value[CPU_IDLE]),
				&(info->value[CPU_IOWAIT]),
				&(info->value[CPU_IRQ]),
				&(info->value[CPU_SOFTIRQ]));
		if (strncmp(buf, "ctxt ", 5) == 0)
			sscanf(buf, "%*s %lf", &(info->value[CPU_CTXT]));
		if (strncmp(buf, "intr ", 5) == 0)
			sscanf(buf, "%*s %lf", &(info->value[CPU_INTR]));
		if (strncmp(buf, "procs_running ", 14) == 0)
			sscanf(buf, "%*s %lf", &(info->value[CPU_PROCS_RUN]));
		if (strncmp(buf, "procs_blocked ", 14) == 0)
			sscanf(buf, "%*s %lf", &(info->value[CPU_PROCS_BLK]));
	}
	fclose(fp);

	return 0;
}

/*
 *  stats_gather()
 *	gather up delta between last stats and current to get
 * 	some form of per sample accounting calculated.
 */
static void stats_gather(stats_t *s1, stats_t *s2, stats_t *res)
{
	double total;

	res->value[CPU_USER]    = s2->value[CPU_USER]    - s1->value[CPU_USER];
	res->value[CPU_NICE]    = s2->value[CPU_NICE]    - s1->value[CPU_NICE];
	res->value[CPU_SYS]     = s2->value[CPU_SYS]     - s1->value[CPU_SYS];
	res->value[CPU_IDLE]    = s2->value[CPU_IDLE]    - s1->value[CPU_IDLE];
	res->value[CPU_IOWAIT]  = s2->value[CPU_IOWAIT]  - s1->value[CPU_IOWAIT];
	res->value[CPU_IRQ]     = s2->value[CPU_IRQ]     - s1->value[CPU_IRQ];
	res->value[CPU_SOFTIRQ] = s2->value[CPU_SOFTIRQ] - s1->value[CPU_SOFTIRQ];
	res->value[CPU_CTXT]	= s2->value[CPU_CTXT]    - s1->value[CPU_CTXT];
	res->value[CPU_INTR]	= s2->value[CPU_INTR]    - s1->value[CPU_INTR];

	total = res->value[CPU_USER] + res->value[CPU_NICE] +
		res->value[CPU_SYS] + res->value[CPU_IDLE] + res->value[CPU_IOWAIT];

	res->value[CPU_USER]   = (100.0 * res->value[CPU_USER]) / total;
	res->value[CPU_NICE]   = (100.0 * res->value[CPU_NICE]) / total;
	res->value[CPU_SYS]    = (100.0 * res->value[CPU_SYS]) / total;
	res->value[CPU_IDLE]   = (100.0 * res->value[CPU_IDLE]) / total;
	res->value[CPU_IOWAIT] = (100.0 * res->value[CPU_IOWAIT]) / total;

	res->value[CPU_CTXT]   = res->value[CPU_CTXT] / sample_delay;
	res->value[CPU_INTR]   = res->value[CPU_INTR] / sample_delay;

	res->value[CPU_PROCS_RUN] = s2->value[CPU_PROCS_RUN];
	res->value[CPU_PROCS_BLK] = s2->value[CPU_PROCS_BLK];
}

/*
 *  stats_headings()
 *	dump heading columns
 */
static void stats_headings(void)
{
	printf("  Time    User  Nice   Sys  Idle    IO  Run Ctxt/s  IRQ/s Fork Exec Exit  Watts\n");
}

/*
 *  stats_ruler()
 *	pretty print ruler between rows
 */
static void stats_ruler(void)
{
	printf("-------- ----- ----- ----- ----- ----- ---- ------ ------ ---- ---- ---- ------\n");
}

/*
 *  stats_print()
 *	print out statistics with accuracy depending if it's a summary or not
 */
static void stats_print(char *prefix, bool summary, stats_t *s)
{
	char *fmt = summary ?
		"%8.8s %5.1f %5.1f %5.1f %5.1f %5.1f %4.1f %6.1f %6.1f %4.1f %4.1f %4.1f %6.2f\n" :
		"%8.8s %5.1f %5.1f %5.1f %5.1f %5.1f %4.0f %6.0f %6.0f %4.0f %4.0f %4.0f %6.2f\n";

	printf(fmt,
		prefix,
		s->value[CPU_USER], s->value[CPU_NICE],
		s->value[CPU_SYS], s->value[CPU_IDLE],
		s->value[CPU_IOWAIT], s->value[CPU_PROCS_RUN],
		s->value[CPU_CTXT], s->value[CPU_INTR],
		s->value[PROC_FORK], s->value[PROC_EXEC],
		s->value[PROC_EXIT], s->value[POWER_RATE]);
}

/*
 *  stats_average_stddev_min_max()
 *	calculate average, std deviation, min and max
 */
static void stats_average_stddev_min_max(stats_t *stats,
	int num,
	stats_t *average,
	stats_t *stddev,
	stats_t *min,
	stats_t *max)
{
	int i;
	int j;

	for (j=0; j<MAX_VALUES; j++) {
		double total = 0.0;

		max->value[j] = -1E6;
		min->value[j] = 1E6;

		for (i=0; i<num; i++) {
			if (stats[i].value[j] > max->value[j])
				max->value[j] = stats[i].value[j];
			if (stats[i].value[j] < min->value[j])
				min->value[j] = stats[i].value[j];
			total += stats[i].value[j];
		}
		average->value[j] = total / (double)num;

		total = 0.0;
		for (i=0; i<num; i++) {
			double diff = (double)stats[i].value[j] - average->value[j];
			diff = diff * diff;
			total += diff;
		}
		stddev->value[j] = total / (double)num;
		stddev->value[j] = sqrt(stddev->value[j]);
	}
}

/*
 *  power_rate_get()
 *	get power discharge rate from battery
 */
static int power_rate_get(double *rate, bool *discharging)
{
	DIR *dir;
	FILE *file;
	struct dirent *dirent;
	char filename[PATH_MAX];

	*rate = 0.0;
	*discharging = true;

	if ((dir = opendir("/proc/acpi/battery")) == NULL)
		return 0;

	while ((dirent = readdir(dir))) {
		double voltage = 0.0;
		double amps = 0.0;
		double power_rate = 0.0;
		char buffer[4096];
		char *ptr;

		if (strlen(dirent->d_name) < 3)
			continue;

		sprintf(filename, "/proc/acpi/battery/%s/state", dirent->d_name);
		if ((file = fopen(filename, "r")) == NULL)
			continue;

		memset(buffer, 0, sizeof(buffer));
		while (fgets(buffer, sizeof(buffer), file) != NULL) {
			if (strstr(buffer, "present:") &&
			    strstr(buffer, "no"))
				break;

			if (strstr(buffer, "charging state:") &&
			    !strstr(buffer, "discharging")) {
				printf("Machine is NOT running on battery and hence "
				       "we cannot measure power usage.\n");
				*discharging = false;
				break;
			}

			ptr = strchr(buffer, ':');
			if (ptr) {
				ptr++;
				if (strstr(buffer, "present voltage"))
					voltage = strtoull(ptr, NULL, 10) / 1000.0;
				if (strstr(buffer, "present rate") && strstr(ptr, "mW"))
					power_rate = strtoull(ptr, NULL, 10) / 1000.0 ;
				if (strstr(buffer, "present rate") && strstr(ptr, "mA"))
					amps = strtoull(ptr, NULL, 10) / 1000.0;
			}
		}
		fclose(file);
		*rate += power_rate + voltage * amps;
	}
	closedir(dir);

	return 0;
}

/*
 *  proc_info_hash()
 * 	hash on PID
 */
static int proc_info_hash(pid_t pid)
{
	return pid % MAX_PIDS;
}

/*
 *  proc_cmdline()
 *	get a processes cmdline text
 */
static int proc_cmdline(pid_t pid, char *cmdline, size_t size)
{
	FILE *fp;
	char path[PATH_MAX];
	int n = 0;

	*cmdline = '\0';
	snprintf(path, sizeof(path), "/proc/%d/cmdline", pid);
	if ((fp = fopen(path, "r")) != NULL) {
		n = fread(cmdline, size, 1, fp);
		fclose(fp);
	}
	return n;
}

/*
 *  proc_info_get()
 *	get proc info on a given pid
 */
static char *proc_info_get(pid_t pid)
{
	int i = proc_info_hash(pid);
	int j;

	for (j = 0; j<MAX_PIDS; j++, i = (i + 1) % MAX_PIDS) {
		if ((proc_info[i] != NULL) && (proc_info[i]->pid == pid))
			return proc_info[i]->cmdline;
	}
	return "<unknown>";
}

/*
 *  proc_info_free()
 *	free cached process info and remove from hash table
 */
static void proc_info_free(pid_t pid)
{
	int i = proc_info_hash(pid);
	int j;

	for (j = 0; j<MAX_PIDS; j++, i = (i + 1) % MAX_PIDS) {
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

	for (i = 0; i<MAX_PIDS; i++) {
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
static int proc_info_add(pid_t pid)
{
	int i, j;
	proc_info_t *info;
	char path[PATH_MAX];
	char cmdline[1024];

	memset(cmdline, 0, sizeof(cmdline));	/* keep valgrind happy */

	if ((info = calloc(1, sizeof(proc_info_t))) == NULL) {
		fprintf(stderr, "Cannot allocate all proc info\n");
		return -1;
	}
	info->pid = pid;

	snprintf(path, sizeof(path), "/proc/%d/cmdline", info->pid);
	(void)proc_cmdline(pid, cmdline, sizeof(cmdline));

	if ((info->cmdline = malloc(strlen(cmdline)+1)) == NULL) {
		fprintf(stderr, "Cannot allocate all proc info\n");
		free(info);
		return -1;
	}
	strcpy(info->cmdline, cmdline);

	i = proc_info_hash(pid);
	for (j = 0; j<MAX_PIDS; j++, i = (i + 1) % MAX_PIDS) {
		if (proc_info[i] == NULL) {
			proc_info[i] = info;
			return 0;
		}
	}
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
		if (isdigit(dirent->d_name[0]))
			proc_info_add(atoi(dirent->d_name));
	}

	closedir(dir);
	return 0;
}

/*
 *   monitor()
 *	monitor system activity and power consumption
 */
static int monitor(int sock)
{
	ssize_t len;
	struct nlmsghdr *nlmsghdr;
	struct timeval t1, t2;
	stats_t *stats;
	stats_t s1, s2;
	stats_t average, stddev, min, max;
	int readings = 0;
	bool redone = false;

	if ((stats = calloc(max_readings, sizeof(stats_t))) == NULL) {
		fprintf(stderr, "Cannot allocate statistics table.\n");
		return -1;
	}

	stats_clear(stats, max_readings);
	stats_headings();

	gettimeofday(&t1, NULL);
	stats_read(&s1);

	while (!stop_recv && (readings < max_readings)) {
		fd_set readfds;
		int ret;
		suseconds_t usec;
		struct timeval tv;
		char __attribute__ ((aligned(NLMSG_ALIGNTO)))buf[4096];
		bool redo = false;

		FD_ZERO(&readfds);
		FD_SET(sock, &readfds);
		gettimeofday(&t2, NULL);
		usec = ((t1.tv_sec + sample_delay - t2.tv_sec) * 1000000) + (t1.tv_usec - t2.tv_usec);
		tv.tv_sec = usec / 1000000;
		tv.tv_usec = usec % 1000000;

		if ((ret = select(sock+1, &readfds, NULL, NULL, &tv)) < 0) {
			if (errno == EINTR)
				break;
			fprintf(stderr,"select: %s\n", strerror(errno));
			free(stats);
			return -1;
		}

		/* Time out, so measure some more samples */
		if (ret == 0) {
			char tmbuffer[10];
			bool discharging;

			if (redone) {
				printf("          --- Skipped sample(s) because of fork/exec/exit activity ---\n");
				redone = false;
			}

			time_now(tmbuffer, sizeof(tmbuffer));
			gettimeofday(&t1, NULL);
			stats_read(&s2);
			stats_gather(&s1, &s2, &stats[readings]);
			power_rate_get(&stats[readings].value[POWER_RATE], &discharging);
			if (!discharging) {
				free(stats);
				return -1;
			}

			stats_print(tmbuffer, false, &stats[readings]);
			readings++;
			s1 = s2;
			continue;
		}

        	if ((len = recv(sock, buf, sizeof(buf), 0)) == 0) {
			free(stats);
			return 0;
		}
		if (len == -1) {
			if (errno == EINTR) {
				continue;
			} else {
				fprintf(stderr,"recv: %s\n", strerror(errno));
				free(stats);
            			return -1;
			}
		}

		for (nlmsghdr = (struct nlmsghdr *)buf; NLMSG_OK (nlmsghdr, len); nlmsghdr = NLMSG_NEXT (nlmsghdr, len)) {
			struct cn_msg *cn_msg;
			struct proc_event *proc_ev;

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
				proc_info_add(proc_ev->event_data.fork.child_pid);
				if (opts & OPTS_SHOW_PROC_ACTIVITY) {
               				log_printf("fork: parent tid=%d pid=%d -> child tid=%d pid=%d (%s)\n",
                       				proc_ev->event_data.fork.parent_pid,
                       				proc_ev->event_data.fork.parent_tgid,
                       				proc_ev->event_data.fork.child_pid,
                       				proc_ev->event_data.fork.child_tgid,
						proc_info_get(proc_ev->event_data.fork.child_pid));
				}
				redo = true;
               			break;
           		case PROC_EVENT_EXEC:
				stats[readings].value[PROC_EXEC] += 1.0;
				if (opts & OPTS_SHOW_PROC_ACTIVITY) {
               				log_printf("exec: tid=%d pid=%d (%s)\n",
                       				proc_ev->event_data.exec.process_pid,
                       				proc_ev->event_data.exec.process_tgid,
						proc_info_get(proc_ev->event_data.exec.process_pid));
				}
				redo = true;
               			break;
           		case PROC_EVENT_EXIT:
				stats[readings].value[PROC_EXIT] += 1.0;
				if (opts & OPTS_SHOW_PROC_ACTIVITY) {
               				log_printf("exit: tid=%d pid=%d exit_code=%d (%s)\n",
                        			proc_ev->event_data.exit.process_pid,
                        			proc_ev->event_data.exit.process_tgid,
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
		if (opts & OPTS_REDO_WHEN_BUSY && redo) {
			int i;
			char tmbuffer[10];

			time_now(tmbuffer, sizeof(tmbuffer));
			for (i=0; i<MAX_VALUES; i++)
				stats[readings].value[i] = 0.0;

			stats_read(&s1);
			gettimeofday(&t1, NULL);
			redone = true;
		}
        }

	/* Stats now gathered, calculate averages, stddev, min and max and display */
	stats_average_stddev_min_max(stats, readings, &average, &stddev, &min, &max);
	if (readings > 0) {
		stats_ruler();
		stats_print("Average", true, &average);
		stats_print("StdDev",  true, &stddev);
		stats_ruler();
		stats_print("Minimum", true, &min);
		stats_print("Maximum", true, &max);
		stats_ruler();
	}

	free(stats);

	return 0;
}


/*
 *  show_help()
 *	simple help
 */
void show_help(char * const argv[])
{
	printf("usage: %s [-s] [-h] [-d N] [delay [count]]\n", argv[0]);
	printf("\t-s show process fork/exec/exit activity log\n");
	printf("\t-d specify delay before stating, default is %d seconds\n", start_delay);
	printf("\t-r redo a sample if we see process fork/exec/exit activity\n");
	printf("\t-h show help\n");
	printf("\tdelay: delay between each sample, default is %d seconds\n", SAMPLE_DELAY);
	printf("\tcount: number of samples to take, default is %d\n", MAX_READINGS);
}

int main(int argc, char * const argv[])
{
    	int sock;
	double dummy_rate;
	bool discharging;
	int ret = EXIT_FAILURE;

    	signal(SIGINT, &handle_sigint);
    	siginterrupt(SIGINT, 1);

	for (;;) {
		int c = getopt(argc, argv, "srhd:");
		if (c == -1)
			break;
		switch (c) {
		case 'd':
			start_delay = atoi(optarg);
			break;
		case 's':
			opts |= OPTS_SHOW_PROC_ACTIVITY;
			break;
		case 'r':
			opts |= OPTS_REDO_WHEN_BUSY;
			break;
		case 'h':
			show_help(argv);
			exit(EXIT_SUCCESS);
		}
	}

	if (optind < argc)
		max_readings = atoi(argv[optind++]);
	if (optind < argc)
		sample_delay = atoi(argv[optind++]);

	if (geteuid() != 0) {
		fprintf(stderr, "%s needs to be run with root privilege\n", argv[0]);
		exit(ret);
	}

	power_rate_get(&dummy_rate, &discharging);
	if (!discharging)
		exit(ret);

	log_init();
	proc_info_load();

    	if ((sock = netlink_connect()) < 0)
		goto abort;

	if (netlink_listen(sock) < 0)
		goto abort_sock;

	printf("Waiting %d seconds before starting..\n", start_delay);
	if (sleep(start_delay) || stop_recv)
		goto abort_sock;

    	if (monitor(sock) ==  0)
		ret = EXIT_SUCCESS;
	
abort_sock:
	proc_info_unload();
abort:
	log_dump();
	log_free();
	close(sock);

	exit(ret);
}
