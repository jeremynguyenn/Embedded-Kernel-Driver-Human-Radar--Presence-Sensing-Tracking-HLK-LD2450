/*
 * LD2450_app.c - User-space application for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 1st September 2025
 * Version: 2.3.3
 * Description: Uses POSIX threads, signals, message queues, semaphores, shared memory, and process management on Raspberry Pi
 * Process Management:
 *   - fork: Creates child process
 *   - execvp/execl: Replaces child process with new program (execvp for variable args, execl for fixed args)
 *   - waitpid: Parent waits for child termination
 *   - getpid: Retrieves process ID
 *   - Command line arguments: Used to configure device mode
 *   - Note: Process groups/sessions not used due to simple parent-child hierarchy; no need for complex process management
 * Signals:
 *   - sigaction: Modern signal handling for SIGINT, SIGTERM, SIGHUP, SIGPIPE, SIGUSR1
 *   - signal: Legacy signal handling for SIGUSR2 (logs tracking data)
 *   - kill: Sends signals to self or other processes
 *   - sigprocmask: Blocks signals during critical sections
 * POSIX Threads:
 *   - pthread_create: Creates multiple tracking threads (thread pool)
 *   - pthread_detach: Makes most threads detachable for scalability
 *   - pthread_join: Joins one thread to demonstrate joinable threads
 *   - pthread_setschedparam: Sets thread priority
 *   - pthread_attr_setstacksize: Customizes stack size for efficiency
 *   - Thread pool: Multiple threads process message queue data, improving throughput
 *   - Trade-offs: Thread pool adds complexity but enhances scalability for high data rates
 * Thread Synchronization:
 *   - pthread_mutex: Protects shared data (latest_tracking)
 *   - pthread_cond: Signals data availability
 *   - pthread_rwlock: Supports concurrent reads of shared memory
 *   - pthread_barrier: Synchronizes thread startup
 *   - Deadlock prevention: Uses try-lock and ordered locking
 * POSIX IPC:
 *   - Message Queue: Transfers tracking data with priority support
 *   - Shared Memory: Stores tracking data with mmap
 *   - Semaphore: Synchronizes shared memory access
 *   - Pipe: Simple parent-child communication
 *   - FIFO: Added for alternative IPC mechanism
 * Memory Management:
 *   - malloc/calloc/realloc: Dynamic memory allocation for flexibility and zero-initialization
 *   - mmap: Shared memory mapping for inter-process communication
 *   - static/auto variables: Demonstrated in example_function
 *   - Strategy: malloc for general allocation, calloc for zero-initialized memory, realloc for resizing
 * Changelog:
 *   - 2.2.0: Added POSIX threads, signal handling, and message queue
 *   - 2.3.0: Added fork/exec/wait, kill signal, detachable threads, rwlock, semaphore, shared memory, memory management
 *   - 2.3.1: Added execvp, getpid, signal(), thread priority, barrier, pipe, static/auto variables, detailed comments
 *   - 2.3.2: Optimized for Raspberry Pi
 *   - 2.3.3: Added calloc/realloc, memory documentation, leak checks, command line args, execl, enhanced signals, thread pool, stack size, FIFO, priority MQ
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

#define LD2450_DEVICE "/dev/ld2450"
#define LD2450_MQ_NAME "/ld2450_mq"
#define LD2450_SHM_NAME "/ld2450_shm"
#define LD2450_SEM_NAME "/ld2450_sem"
#define LD2450_FIFO_NAME "/tmp/ld2450_fifo"
#define LD2450_IOC_MAGIC 'L'
#define LD2450_IOC_SET_MODE _IOW(LD2450_IOC_MAGIC, 1, int)
#define LD2450_IOC_GET_TRACKING _IOR(LD2450_IOC_MAGIC, 2, struct ld2450_tracking_data)
#define THREAD_POOL_SIZE 3 /* Number of threads in pool */
#define THREAD_STACK_SIZE (1024 * 1024) /* 1MB stack size */

struct ld2450_tracking_data {
    short x_pos;
    short y_pos;
    short velocity;
    unsigned short distance;
};

static volatile sig_atomic_t running = 1;
static pthread_mutex_t tracking_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t tracking_cond = PTHREAD_COND_INITIALIZER;
static pthread_rwlock_t tracking_rwlock = PTHREAD_RWLOCK_INITIALIZER;
static pthread_barrier_t barrier;
static struct ld2450_tracking_data *latest_tracking;
static sem_t *sem;
static int shm_fd;
static struct ld2450_tracking_data *shm_tracking;
static int pipe_fd[2];
static int fifo_fd = -1;

static int global_counter = 0; /* Static variable, retains value across function calls */
void example_function(void)
{
    int auto_var = 0; /* Auto variable, destroyed when function exits */
    static struct ld2450_tracking_data *temp_data = NULL; /* Static for persistence */
    struct ld2450_tracking_data *calloc_data = NULL; /* For calloc example */
    struct ld2450_tracking_data *realloc_data = NULL; /* For realloc example */

    /* Demonstrate malloc */
    if (!temp_data) {
        temp_data = malloc(sizeof(struct ld2450_tracking_data));
        if (!temp_data) {
            perror("malloc failed in example_function");
            return;
        }
    }

    /* Demonstrate calloc (zero-initialized memory) */
    calloc_data = calloc(1, sizeof(struct ld2450_tracking_data));
    if (!calloc_data) {
        perror("calloc failed in example_function");
        free(temp_data);
        temp_data = NULL;
        return;
    }

    /* Demonstrate realloc (resize existing memory) */
    realloc_data = realloc(temp_data, 2 * sizeof(struct ld2450_tracking_data));
    if (!realloc_data) {
        perror("realloc failed in example_function");
        free(temp_data);
        free(calloc_data);
        temp_data = NULL;
        return;
    }
    temp_data = realloc_data;

    global_counter++;
    printf("Static: %d, Auto: %d, Malloc: %p, Calloc: %p, Realloc: %p\n",
           global_counter, auto_var, temp_data, calloc_data, realloc_data);

    /* Clean up to prevent memory leaks */
    free(calloc_data);
    free(temp_data);
    temp_data = NULL;
}

void signal_handler(int sig)
{
    running = 0;
    pthread_cond_broadcast(&tracking_cond); /* Signal all threads */
}

void ignore_signal(int sig)
{
    printf("Ignoring signal %d\n", sig);
}

void default_signal_handler(int sig)
{
    pthread_rwlock_rdlock(&tracking_rwlock);
    printf("SIGUSR2 received, current tracking: X=%d, Y=%d, Vel=%d, Dist=%u\n",
           shm_tracking->x_pos, shm_tracking->y_pos,
           shm_tracking->velocity, shm_tracking->distance);
    pthread_rwlock_unlock(&tracking_rwlock);
}

void *tracking_thread(void *arg)
{
    mqd_t mq = *(mqd_t *)arg;
    struct ld2450_tracking_data tracking;
    pthread_t tid = pthread_self();
    sigset_t sigset;
    int thread_id = *(int *)pthread_getspecific(thread_key);

    /* Block signals in thread */
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGINT);
    sigaddset(&sigset, SIGTERM);
    sigaddset(&sigset, SIGHUP);
    sigaddset(&sigset, SIGPIPE);
    sigaddset(&sigset, SIGUSR1);
    sigaddset(&sigset, SIGUSR2);
    pthread_sigmask(SIG_BLOCK, &sigset, NULL);

    printf("Tracking thread %d ID: %lu, PID: %d\n", thread_id, tid, getpid());
    pthread_barrier_wait(&barrier);

    while (running) {
        unsigned int prio;
        if (mq_receive(mq, (char *)&tracking, sizeof(tracking), &prio) == -1) {
            if (errno != EAGAIN)
                perror("mq_receive");
            continue;
        }
        printf("Thread %d received data with priority %u\n", thread_id, prio);

        sigset_t oldset;
        sigprocmask(SIG_BLOCK, &sigset, &oldset);
        if (sem_trywait(sem) == 0) {
            pthread_rwlock_wrlock(&tracking_rwlock);
            memcpy(shm_tracking, &tracking, sizeof(tracking));
            pthread_rwlock_unlock(&tracking_rwlock);
            sem_post(sem);
        } else {
            perror("sem_trywait failed, skipping update");
        }
        sigprocmask(SIG_SETMASK, &oldset, NULL);

        if (pthread_mutex_trylock(&tracking_mutex) == 0) {
            *latest_tracking = tracking;
            pthread_cond_signal(&tracking_cond);
            pthread_mutex_unlock(&tracking_mutex);
        }
    }

    return (thread_id == 0) ? (void *)(intptr_t)thread_id : NULL; /* Return for joinable thread */
}

static pthread_key_t thread_key;
static void thread_cleanup(void *arg)
{
    int *thread_id = (int *)arg;
    free(thread_id);
}

int main(int argc, char *argv[])
{
    int fd = -1, mode = 1; /* Default mode */
    mqd_t mq = -1;
    struct mq_attr attr = { .mq_maxmsg = 10, .mq_msgsize = sizeof(struct ld2450_tracking_data), .mq_flags = O_NONBLOCK };
    struct sigaction sa;
    pthread_t threads[THREAD_POOL_SIZE];
    int thread_ids[THREAD_POOL_SIZE];
    pid_t pid;
    sigset_t sigset;

    /* Initialize signal mask */
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGINT);
    sigaddset(&sigset, SIGTERM);
    sigaddset(&sigset, SIGHUP);
    sigaddset(&sigset, SIGPIPE);
    sigaddset(&sigset, SIGUSR1);
    sigaddset(&sigset, SIGUSR2);

    /* Parse command line arguments */
    if (argc > 1) {
        mode = atoi(argv[1]);
        if (mode < 0 || mode > 1) {
            fprintf(stderr, "Invalid mode: %d. Using default mode 1\n", mode);
            mode = 1;
        }
        printf("Using mode: %d\n", mode);
    }

    /* Allocate memory for latest_tracking */
    latest_tracking = malloc(sizeof(struct ld2450_tracking_data));
    if (!latest_tracking) {
        perror("Failed to allocate memory for latest_tracking");
        return 1;
    }

    /* Shared memory setup */
    shm_fd = shm_open(LD2450_SHM_NAME, O_CREAT | O_RDWR, 0644);
    if (shm_fd == -1) {
        perror("Failed to open shared memory");
        free(latest_tracking);
        return 1;
    }
    if (ftruncate(shm_fd, sizeof(struct ld2450_tracking_data)) == -1) {
        perror("Failed to set shared memory size");
        shm_unlink(LD2450_SHM_NAME);
        free(latest_tracking);
        return 1;
    }
    shm_tracking = mmap(NULL, sizeof(struct ld2450_tracking_data), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_tracking == MAP_FAILED) {
        perror("Failed to map shared memory");
        shm_unlink(LD2450_SHM_NAME);
        free(latest_tracking);
        return 1;
    }

    /* Semaphore setup */
    sem = sem_open(LD2450_SEM_NAME, O_CREAT, 0644, 1);
    if (sem == SEM_FAILED) {
        perror("Failed to open semaphore");
        munmap(shm_tracking, sizeof(struct ld2450_tracking_data));
        shm_unlink(LD2450_SHM_NAME);
        free(latest_tracking);
        return 1;
    }

    /* FIFO setup */
    if (mkfifo(LD2450_FIFO_NAME, 0644) == -1 && errno != EEXIST) {
        perror("Failed to create FIFO");
        goto cleanup;
    }
    fifo_fd = open(LD2450_FIFO_NAME, O_WRONLY | O_NONBLOCK);
    if (fifo_fd == -1) {
        perror("Failed to open FIFO");
        goto cleanup;
    }

    /* Signal handling */
    sa.sa_handler = signal_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);
    sigaction(SIGPIPE, &sa, NULL);
    sa.sa_handler = ignore_signal;
    sigaction(SIGUSR1, &sa, NULL);
    signal(SIGUSR2, default_signal_handler);

    /* Pipe setup */
    if (pipe(pipe_fd) == -1) {
        perror("pipe failed");
        goto cleanup;
    }

    /* Process management: fork and exec */
    pid = fork();
    if (pid < 0) {
        perror("fork failed");
        goto cleanup;
    } else if (pid == 0) {
        close(pipe_fd[0]);
        char *msg = "Hello from child";
        write(pipe_fd[1], msg, strlen(msg) + 1);
        close(pipe_fd[1]);
        printf("Child PID: %d\n", getpid());
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
        printf("Parent received: %s\n", buf);
        close(pipe_fd[0]);
        int status;
        waitpid(pid, &status, 0);
        printf("Child process exited with status %d\n", WEXITSTATUS(status));
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
        pthread_attr_setdetachstate(&attr_thread, PTHREAD_CREATE_JOINABLE); /* One joinable thread */
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
            /* Write to FIFO */
            write(fifo_fd, shm_tracking, sizeof(struct ld2450_tracking_data));
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

cleanup_threads:
    for (int i = 0; i < THREAD_POOL_SIZE; i++) {
        pthread_cancel(threads[i]);
    }
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