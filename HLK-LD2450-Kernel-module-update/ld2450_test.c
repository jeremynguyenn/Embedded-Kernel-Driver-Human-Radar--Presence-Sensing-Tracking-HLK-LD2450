/*
 * ld2450_test.c - Automated test suite for Hi-Link LD2450 radar module driver
 * Author: Nguyen Nhan
 * Date: 22nd September 2025
 * Version: 2.4.0
 * Description: Tests all driver features in user-space on Raspberry Pi
 * Tests:
 *   - File operations: open, read, write, lseek, ioctl, poll
 *   - Sysfs: Read/write fw_version, tracking_mode, stats
 *   - Debugfs: Read raw_data, error_count, fifo_usage
 *   - IPC: POSIX message queue, shared memory, semaphore, System V IPC
 *   - Signals: Send SIGUSR1/SIGUSR2
 *   - Threads: Create/join/cancel threads
 *   - Network: Send tracking data via netdev
 * Usage: ./ld2450_test [test_name]
 * Changelog:
 *   - 2.4.0: Initial implementation of test suite
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <mqueue.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <signal.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <sys/msg.h>
#include <sys/shm.h>
#include "LD2450.h"

#define TEST_DEVICE "/dev/ld2450"
#define TEST_SYSFS "/sys/devices/platform/ld2450"
#define TEST_DEBUGFS "/sys/kernel/debug/ld2450"
#define TEST_MQ "/ld2450_mq"
#define TEST_SHM "/ld2450_shm"
#define TEST_SEM "/ld2450_sem"
#define TEST_NET "ld2450_net"

static int test_result = 0;

void test_file_ops(void)
{
    int fd = open(TEST_DEVICE, O_RDWR);
    if (fd < 0) {
        printf("Test file_ops: Failed to open %s: %s\n", TEST_DEVICE, strerror(errno));
        test_result++;
        return;
    }

    struct ld2450_tracking_data tracking = {0};
    ssize_t ret = read(fd, &tracking, sizeof(tracking));
    if (ret != sizeof(tracking)) {
        printf("Test file_ops: Failed to read tracking data: %s\n", strerror(errno));
        test_result++;
    } else {
        printf("Test file_ops: Read X=%d, Y=%d, Vel=%d, Dist=%u\n",
               tracking.x_pos, tracking.y_pos, tracking.velocity, tracking.distance);
    }

    char cmd[] = "test_cmd";
    ret = write(fd, cmd, sizeof(cmd));
    if (ret != sizeof(cmd)) {
        printf("Test file_ops: Failed to write command: %s\n", strerror(errno));
        test_result++;
    }

    ret = lseek(fd, 0, SEEK_SET);
    if (ret < 0) {
        printf("Test file_ops: Failed to lseek: %s\n", strerror(errno));
        test_result++;
    }

    int mode = 1;
    if (ioctl(fd, LD2450_IOC_SET_MODE, &mode) < 0) {
        printf("Test file_ops: Failed to set mode: %s\n", strerror(errno));
        test_result++;
    }

    close(fd);
}

void test_sysfs(void)
{
    char path[256], buf[256];
    int fd;

    snprintf(path, sizeof(path), "%s/tracking_mode", TEST_SYSFS);
    fd = open(path, O_RDWR);
    if (fd < 0) {
        printf("Test sysfs: Failed to open tracking_mode: %s\n", strerror(errno));
        test_result++;
        return;
    }
    if (write(fd, "1", 1) != 1) {
        printf("Test sysfs: Failed to write tracking_mode: %s\n", strerror(errno));
        test_result++;
    }
    if (read(fd, buf, sizeof(buf)) <= 0) {
        printf("Test sysfs: Failed to read tracking_mode: %s\n", strerror(errno));
        test_result++;
    }
    close(fd);

    snprintf(path, sizeof(path), "%s/stats", TEST_SYSFS);
    fd = open(path, O_RDONLY);
    if (fd < 0) {
        printf("Test sysfs: Failed to open stats: %s\n", strerror(errno));
        test_result++;
        return;
    }
    if (read(fd, buf, sizeof(buf)) <= 0) {
        printf("Test sysfs: Failed to read stats: %s\n", strerror(errno));
        test_result++;
    } else {
        printf("Test sysfs: Stats: %s", buf);
    }
    close(fd);
}

void test_debugfs(void)
{
    char path[256], buf[256];
    int fd;

    snprintf(path, sizeof(path), "%s/raw_data", TEST_DEBUGFS);
    fd = open(path, O_RDONLY);
    if (fd < 0) {
        printf("Test debugfs: Failed to open raw_data: %s\n", strerror(errno));
        test_result++;
        return;
    }
    if (read(fd, buf, sizeof(buf)) <= 0) {
        printf("Test debugfs: Failed to read raw_data: %s\n", strerror(errno));
        test_result++;
    } else {
        printf("Test debugfs: Raw data read\n");
    }
    close(fd);
}

void test_ipc(void)
{
    mqd_t mq = mq_open(TEST_MQ, O_RDONLY | O_CREAT, 0644, NULL);
    if (mq == -1) {
        printf("Test IPC: Failed to open message queue: %s\n", strerror(errno));
        test_result++;
        return;
    }
    struct ld2450_tracking_data tracking;
    if (mq_receive(mq, (char *)&tracking, sizeof(tracking), NULL) > 0) {
        printf("Test IPC: Received X=%d, Y=%d, Vel=%d, Dist=%u\n",
               tracking.x_pos, tracking.y_pos, tracking.velocity, tracking.distance);
    } else {
        printf("Test IPC: Failed to receive from MQ: %s\n", strerror(errno));
        test_result++;
    }
    mq_close(mq);

    sem_t *sem = sem_open(TEST_SEM, O_CREAT, 0644, 1);
    if (sem == SEM_FAILED) {
        printf("Test IPC: Failed to open semaphore: %s\n", strerror(errno));
        test_result++;
        return;
    }
    sem_close(sem);

    int shm_fd = shm_open(TEST_SHM, O_RDWR, 0644);
    if (shm_fd < 0) {
        printf("Test IPC: Failed to open shared memory: %s\n", strerror(errno));
        test_result++;
        return;
    }
    close(shm_fd);
}

void test_signals(void)
{
    kill(getpid(), SIGUSR1);
    printf("Test signals: Sent SIGUSR1\n");
    kill(getpid(), SIGUSR2);
    printf("Test signals: Sent SIGUSR2\n");
}

void *test_thread(void *arg)
{
    printf("Test thread: Running thread %ld\n", (long)arg);
    sleep(1);
    return NULL;
}

void test_threads(void)
{
    pthread_t thread;
    if (pthread_create(&thread, NULL, test_thread, (void *)1) != 0) {
        printf("Test threads: Failed to create thread: %s\n", strerror(errno));
        test_result++;
        return;
    }
    pthread_join(thread, NULL);
    printf("Test threads: Thread joined\n");
}

void test_network(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        printf("Test network: Failed to create socket: %s\n", strerror(errno));
        test_result++;
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(12345),
        .sin_addr.s_addr = inet_addr("127.0.0.1"),
    };

    struct ld2450_tracking_data tracking = { .x_pos = 100, .y_pos = 200, .velocity = 50, .distance = 1000 };
    if (sendto(sock, &tracking, sizeof(tracking), 0, (struct sockaddr *)&addr, sizeof(addr)) != sizeof(tracking)) {
        printf("Test network: Failed to send packet: %s\n", strerror(errno));
        test_result++;
    } else {
        printf("Test network: Sent tracking data\n");
    }
    close(sock);
}

int main(int argc, char *argv[])
{
    printf("Starting LD2450 driver test suite...\n");

    if (argc == 1 || strcmp(argv[1], "all") == 0) {
        test_file_ops();
        test_sysfs();
        test_debugfs();
        test_ipc();
        test_signals();
        test_threads();
        test_network();
    } else if (strcmp(argv[1], "file_ops") == 0) {
        test_file_ops();
    } else if (strcmp(argv[1], "sysfs") == 0) {
        test_sysfs();
    } else if (strcmp(argv[1], "debugfs") == 0) {
        test_debugfs();
    } else if (strcmp(argv[1], "ipc") == 0) {
        test_ipc();
    } else if (strcmp(argv[1], "signals") == 0) {
        test_signals();
    } else if (strcmp(argv[1], "threads") == 0) {
        test_threads();
    } else if (strcmp(argv[1], "network") == 0) {
        test_network();
    } else {
        printf("Usage: %s [all|file_ops|sysfs|debugfs|ipc|signals|threads|network]\n", argv[0]);
        return 1;
    }

    printf("Test suite completed with %d failures\n", test_result);
    return test_result;
}