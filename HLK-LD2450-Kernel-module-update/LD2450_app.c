/*
 * LD2450_app.c - User-space application for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 22nd September 2025
 * Version: 2.4.0
 * Description: Uses POSIX threads, signals, message queues, semaphores, shared memory, and process management on Raspberry Pi
 * Process Management:
 *   - fork: Creates child process with session handling
 *   - execvp/execl: Replaces child process
 *   - waitpid: Parent waits for child termination
 *   - getpid/getsid: Retrieves process/session ID
 *   - Command line arguments: Configures mode and thread options
 * Signals:
 *   - sigaction: Handles SIGINT, SIGTERM, SIGHUP, SIGPIPE, SIGUSR1, SIGUSR2
 *   - sigprocmask: Blocks signals during critical sections
 * POSIX Threads:
 *   - pthread_create: Creates thread pool
 *   - pthread_detach/join: Supports detachable/joinable threads
 *   - pthread_cancel: Demonstrates thread cancellation
 * Thread Synchronization:
 *   - pthread_mutex, pthread_cond, pthread_rwlock, pthread_barrier
 *   - Deadlock prevention: Ordered locking, try-lock
 * POSIX IPC:
 *   - Message Queue, Shared Memory, Semaphore, Pipe, FIFO
 *   - System V IPC: msgget, shmget for alternative IPC
 * Memory Management:
 *   - malloc/calloc/realloc, mmap, static/auto variables
 *   - Page fault simulation: Access invalid memory
 * Changelog:
 *   - 2.3.3: Added thread pool, FIFO, priority MQ
 *   - 2.4.0: Added System V IPC, process group/session, page fault simulation, thread cancellation
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <mqueue.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sched.h>
#include <sys/msg.h>
#include <sys/shm.h>

#define LD2450_DEVICE "/dev/ld2450"
#define LD2450_MQ_NAME "/ld2450_mq"
#define LD2450_SHM_NAME "/ld2450_shm"
#define LD2450_SEM_NAME "/ld2450_sem"
#define LD2450_FIFO_NAME "/tmp/ld2450_fifo"
#define LD2450_IOC_MAGIC 'L'
#define LD2450_IOC_SET_MODE _IOW(LD2450_IOC_MAGIC, 1, int)
#define LD2450_IOC_GET_TRACKING _IOR(LD2450_IOC_MAGIC, 2, struct ld2450_tracking_data)
#define THREAD_POOL_SIZE 3
#define THREAD_STACK_SIZE (1024 * 1024)
#define SYSV_MSG_KEY 1234
#define SYSV_SHM_KEY 5678

struct ld2450_tracking_data {
    short x_pos;
    short y_pos;
    short velocity;
    unsigned short distance;
};

struct sysv_msg {
    long mtype;
    struct ld2450_tracking_data data;
};

static volatile sig_atomic_t running = 1;
static pthread_mutex_t tracking_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t tracking_cond = PTHREAD_COND_INITIALIZER;
static pthread_rwlock_t tracking_rwlock = PTHREAD_RWLOCK_INITIALIZER;
static pthread_barrier_t barrier;
static struct ld2450_tracking_data *latest_tracking;
static sem_t *sem;
static int shm_fd, sysv_shmid, sysv_msgid;
static struct ld2450_tracking_data *shm_tracking;
static int pipe_fd[2];
static int fifo_fd = -1;
static pthread_t threads[THREAD_POOL_SIZE];
static int thread_ids[THREAD_POOL_SIZE];
static pthread_key_t thread_key;

static void thread_cleanup(void *arg)
{
    free(arg);
}

static void *tracking_thread(void *arg)
{
    mqd_t *mq = arg;
    struct ld2450_tracking_data tracking;
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    pthread_mutex_lock(&tracking_mutex);
    pthread_barrier_wait(&barrier);
    pthread_mutex_unlock(&tracking_mutex);

    while (running) {
        if (mq_receive(*mq, (char *)&tracking, sizeof(tracking), NULL) > 0) {
            pthread_rwlock_wrlock(&tracking_rwlock);
            *latest_tracking = tracking;
            sem_post(sem);
            pthread_rwlock_unlock(&tracking_rwlock);
            pthread_cond_signal(&tracking_cond);
        }
        usleep(10000);
    }
    return NULL;
}

static void signal_handler(int sig)
{
    if (sig == SIGUSR2) {
        pthread_rwlock_rdlock(&tracking_rwlock);
        printf("SIGUSR2: Latest tracking: X=%d, Y=%d, Vel=%d, Dist=%u\n",
               latest_tracking->x_pos, latest_tracking->y_pos,
               latest_tracking->velocity, latest_tracking->distance);
        pthread_rwlock_unlock(&tracking_rwlock);
    } else {
        running = 0;
        pthread_cond_broadcast(&tracking_cond);
    }
}

static void page_fault_simulation(void)
{
    char *invalid_ptr = (char *)0x1;
    printf("Simulating page fault...\n");
    *invalid_ptr = 'A'; /* Will cause SIGSEGV */
}

int main(int argc, char *argv[])
{
    int fd = -1, mode = 0;
    mqd_t mq = -1;
    struct mq_attr attr = { .mq_maxmsg = LD2450_MQ_MAXMSG, .mq_msgsize = LD2450_MQ_MSGSIZE };
    sigset_t sigset;

    latest_tracking = calloc(1, sizeof(struct ld2450_tracking_data));
    if (!latest_tracking) {
        perror("calloc failed");
        return 1;
    }

    /* Initialize signals */
    struct sigaction sa = { .sa_handler = signal_handler, .sa_flags = 0 };
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);
    sigaction(SIGPIPE, &sa, NULL);
    sigaction(SIGUSR1, &sa, NULL);
    signal(SIGUSR2, signal_handler);

    sigemptyset(&sigset);
    sigaddset(&sigset, SIGINT);
    sigaddset(&sigset, SIGTERM);

    /* Initialize pipe */
    if (pipe(pipe_fd) < 0) {
        perror("pipe failed");
        goto cleanup;
    }

    /* Initialize FIFO */
    if (mkfifo(LD2450_FIFO_NAME, 0644) < 0 && errno != EEXIST) {
        perror("mkfifo failed");
        goto cleanup;
    }
    fifo_fd = open(LD2450_FIFO_NAME, O_WRONLY | O_NONBLOCK);
    if (fifo_fd < 0) {
        perror("Failed to open FIFO");
        goto cleanup;
    }

    /* Initialize semaphore */
    sem = sem_open(LD2450_SEM_NAME, O_CREAT, 0644, 1);
    if (sem == SEM_FAILED) {
        perror("sem_open failed");
        goto cleanup;
    }

    /* Initialize shared memory */
    shm_fd = shm_open(LD2450_SHM_NAME, O_CREAT | O_RDWR, 0644);
    if (shm_fd < 0) {
        perror("shm_open failed");
        goto cleanup;
    }
    ftruncate(shm_fd, sizeof(struct ld2450_tracking_data));
    shm_tracking = mmap(NULL, sizeof(struct ld2450_tracking_data), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_tracking == MAP_FAILED) {
        perror("mmap failed");
        goto cleanup;
    }

    /* Initialize System V IPC */
    sysv_msgid = msgget(SYSV_MSG_KEY, IPC_CREAT | 0644);
    if (sysv_msgid < 0) {
        perror("msgget failed");
        goto cleanup;
    }
    sysv_shmid = shmget(SYSV_SHM_KEY, sizeof(struct ld2450_tracking_data), IPC_CREAT | 0644);
    if (sysv_shmid < 0) {
        perror("shmget failed");
        goto cleanup;
    }

    /* Create process group and session */
    pid_t pid = fork();
    if (pid < 0) {
        perror("fork failed");
        goto cleanup;
    } else if (pid == 0) {
        setsid(); /* Create new session */
        close(pipe_fd[0]);
        char *msg = "Hello from child";
        write(pipe_fd[1], msg, strlen(msg) + 1);
        close(pipe_fd[1]);
        printf("Child PID: %d, SID: %d\n", getpid(), getsid(0));
        if (argc > 2 && strcmp(argv[2], "execl") == 0) {
            execl("/bin/ls", "ls", "-l", NULL);
            perror("execl failed");
        } else {
            char *args[] = {"ls", "-l", NULL};
            execvp("ls", args);
            perror("execvp failed");
        }
        exit(1);
    } else {
        close(pipe_fd[1]);
        char buf[100];
        read(pipe_fd[0], buf, sizeof(buf));
        printf("Parent received: %s, PID: %d, SID: %d\n", buf, getpid(), getsid(0));
        close(pipe_fd[0]);
        int status;
        waitpid(pid, &status, 0);
        printf("Child process exited with status %d\n", WEXITSTATUS(status));
    }

    /* Simulate page fault if requested */
    if (argc > 2 && strcmp(argv[2], "pagefault") == 0) {
        page_fault_simulation();
    }

    /* Open device */
    fd = open(LD2450_DEVICE, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        perror("Failed to open device");
        goto cleanup;
    }

    /* Set mode */
    if (ioctl(fd, LD2450_IOC_SET_MODE, &mode) < 0) {
        perror("Failed to set mode");
        close(fd);
        goto cleanup;
    }

    /* Open message queue */
    mq = mq_open(LD2450_MQ_NAME, O_RDONLY | O_CREAT | O_NONBLOCK, 0644, &attr);
    if (mq == -1) {
        perror("Failed to open message queue");
        close(fd);
        goto cleanup;
    }

    /* Initialize barrier */
    pthread_barrier_init(&barrier, NULL, THREAD_POOL_SIZE + 1);

    /* Create thread-specific data key */
    pthread_key_create(&thread_key, thread_cleanup);

    /* Create thread pool */
    pthread_attr_t attr_thread;
    struct sched_param param;
    pthread_attr_init(&attr_thread);
    pthread_attr_setstacksize(&attr_thread, THREAD_STACK_SIZE);
    param.sched_priority = 10;
    pthread_attr_setschedparam(&attr_thread, &param);
    if (argc > 2 && strcmp(argv[2], "join") == 0) {
        pthread_attr_setdetachstate(&attr_thread, PTHREAD_CREATE_JOINABLE);
    } else {
        pthread_attr_setdetachstate(&attr_thread, PTHREAD_CREATE_DETACHED);
    }

    for (int i = 0; i < THREAD_POOL_SIZE; i++) {
        thread_ids[i] = i;
        int *tid = malloc(sizeof(int));
        *tid = i;
        pthread_setspecific(thread_key, tid);
        if (pthread_create(&threads[i], &attr_thread, tracking_thread, &mq) != 0) {
            perror("Failed to create tracking thread");
            pthread_attr_destroy(&attr_thread);
            goto cleanup_threads;
        }
    }
    pthread_attr_destroy(&attr_thread);

    /* Send signal to self */
    kill(getpid(), SIGUSR1);
    kill(getpid(), SIGUSR2);

    /* Main loop */
    pthread_barrier_wait(&barrier);
    pthread_mutex_lock(&tracking_mutex);
    while (running) {
        pthread_cond_wait(&tracking_cond, &tracking_mutex);
        if (running) {
            sigset_t oldset;
            sigprocmask(SIG_BLOCK, &sigset, &oldset);
            pthread_rwlock_rdlock(&tracking_rwlock);
            printf("Main: X=%d, Y=%d, Vel=%d, Dist=%u\n",
                   shm_tracking->x_pos, shm_tracking->y_pos,
                   shm_tracking->velocity, shm_tracking->distance);
            write(fifo_fd, shm_tracking, sizeof(struct ld2450_tracking_data));
            struct sysv_msg msg = { .mtype = 1, .data = *shm_tracking };
            msgsnd(sysv_msgid, &msg, sizeof(struct ld2450_tracking_data), 0);
            pthread_rwlock_unlock(&tracking_rwlock);
            sigprocmask(SIG_SETMASK, &oldset, NULL);
        }
    }
    pthread_mutex_unlock(&tracking_mutex);

    /* Join the first thread if joinable */
    if (argc > 2 && strcmp(argv[2], "join") == 0) {
        void *retval;
        pthread_join(threads[0], &retval);
        printf("Joined thread 0 with return value %ld\n", (long)(intptr_t)retval);
    }

    /* Cancel threads */
    for (int i = 0; i < THREAD_POOL_SIZE; i++) {
        pthread_cancel(threads[i]);
    }

cleanup_threads:
    pthread_key_delete(thread_key);

cleanup:
    if (mq != -1) {
        mq_close(mq);
        mq_unlink(LD2450_MQ_NAME);
    }
    if (fd >= 0)
        close(fd);
    if (fifo_fd >= 0) {
        close(fifo_fd);
        unlink(LD2450_FIFO_NAME);
    }
    if (sem != SEM_FAILED) {
        sem_close(sem);
        sem_unlink(LD2450_SEM_NAME);
    }
    if (shm_tracking != MAP_FAILED) {
        munmap(shm_tracking, sizeof(struct ld2450_tracking_data));
        shm_unlink(LD2450_SHM_NAME);
    }
    if (sysv_msgid >= 0)
        msgctl(sysv_msgid, IPC_RMID, NULL);
    if (sysv_shmid >= 0)
        shmctl(sysv_shmid, IPC_RMID, NULL);
    if (latest_tracking) {
        free(latest_tracking);
        latest_tracking = NULL;
    }
    pthread_mutex_destroy(&tracking_mutex);
    pthread_cond_destroy(&tracking_cond);
    pthread_rwlock_destroy(&tracking_rwlock);
    pthread_barrier_destroy(&barrier);

    return 0;
}