/* UARTLite receive only. Sensor arithmetic remains in the FPGA.
 * Each ring entry contains RX-valid/status bits and the received byte.
 * A native thread drains the FIFO independently of Python's GIL. */
#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#define CAPACITY 65536u
typedef struct {
    volatile uint32_t *regs;
    pthread_t thread;
    atomic_int running;
    atomic_int ready;
    uint16_t ring[CAPACITY];
    atomic_uint head, tail, dropped;
    atomic_uint max_gap_us, gaps_over_1ms;
    uint32_t affinity_error, realtime_error, policy, priority;
} Receiver;

static void *receive_loop(void *arg) {
    Receiver *r = arg;
    const struct timespec pause = {0, 20000};
    cpu_set_t allowed, selected;
    CPU_ZERO(&allowed);CPU_ZERO(&selected);
    int affinity = pthread_getaffinity_np(pthread_self(), sizeof(allowed), &allowed);
    if (!affinity) {
        int cpu = -1;
        for (int i=0;i<CPU_SETSIZE;i++) if (CPU_ISSET(i,&allowed)) cpu=i;
        if (cpu>=0) {
            CPU_SET(cpu,&selected);
            affinity=pthread_setaffinity_np(pthread_self(),sizeof(selected),&selected);
        }
    }
    r->affinity_error=(uint32_t)affinity;
    struct sched_param priority = {.sched_priority = 10};
    /* Optional on systems with CAP_SYS_NICE. Failure retains normal scheduling. */
    r->realtime_error=(uint32_t)pthread_setschedparam(pthread_self(), SCHED_FIFO, &priority);
    int policy=0;
    (void)pthread_getschedparam(pthread_self(),&policy,&priority);
    r->policy=(uint32_t)policy;r->priority=(uint32_t)priority.sched_priority;
    unsigned busy = 0;
    /* Load/create the thread before resetting idle-time history. Synchronize
     * in an inter-packet gap so a partial startup frame is never counted as
     * capture data. Startup has a strict 200 ms bound. */
    struct timespec began, quiet, now;
    clock_gettime(CLOCK_MONOTONIC, &began);
    quiet = began;
    r->regs[3] = 3;
    (void)r->regs[2];
    for (;;) {
        uint32_t status = r->regs[2];
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (status & 1u) {(void)r->regs[0];quiet = now;}
        double elapsed = now.tv_sec-began.tv_sec+(now.tv_nsec-began.tv_nsec)*1e-9;
        double idle = now.tv_sec-quiet.tv_sec+(now.tv_nsec-quiet.tv_nsec)*1e-9;
        if (idle >= .003) break;
        if (elapsed >= .2) {atomic_store(&r->ready, -1);return NULL;}
        nanosleep(&pause, NULL);
    }
    atomic_store(&r->ready, 1);
    struct timespec previous;
    clock_gettime(CLOCK_MONOTONIC,&previous);
    while (atomic_load(&r->running)) {
        clock_gettime(CLOCK_MONOTONIC,&now);
        int64_t gap_ns=(int64_t)(now.tv_sec-previous.tv_sec)*1000000000+(now.tv_nsec-previous.tv_nsec);
        uint32_t gap_us=(uint32_t)(gap_ns/1000);
        if (gap_us>atomic_load(&r->max_gap_us)) atomic_store(&r->max_gap_us,gap_us);
        if (gap_us>=1000) atomic_fetch_add(&r->gaps_over_1ms,1);
        previous=now;
        uint32_t status = r->regs[2];
        if (status & 0xe1u) {
            uint16_t event = (uint16_t)((status & 0xe1u) << 8);
            if (status & 1u) event |= (uint16_t)(r->regs[0] & 255u);
            uint32_t head=atomic_load_explicit(&r->head,memory_order_relaxed);
            uint32_t next=(head+1u)%CAPACITY;
            if (next==atomic_load_explicit(&r->tail,memory_order_acquire)) atomic_fetch_add(&r->dropped,1);
            else {
                r->ring[head]=event;
                atomic_store_explicit(&r->head,next,memory_order_release);
            }
            /* Bound CPU ownership even if a faulty peripheral stays RX-valid. */
            if (++busy < 256) continue;
        }
        busy = 0;
        nanosleep(&pause, NULL);
    }
    return NULL;
}

void *uart_rx_start(uintptr_t address) {
    Receiver *r = calloc(1, sizeof(*r));
    if (!r) return NULL;
    r->regs = (volatile uint32_t *)address;
    atomic_init(&r->running, 1);
    atomic_init(&r->ready, 0);
    atomic_init(&r->head,0);atomic_init(&r->tail,0);atomic_init(&r->dropped,0);
    atomic_init(&r->max_gap_us,0);atomic_init(&r->gaps_over_1ms,0);
    if (pthread_create(&r->thread, NULL, receive_loop, r)) {
        free(r);return NULL;
    }
    const struct timespec pause = {0, 100000};
    while (!atomic_load(&r->ready)) nanosleep(&pause, NULL);
    if (atomic_load(&r->ready) < 0) {
        pthread_join(r->thread, NULL);free(r);return NULL;
    }
    return r;
}

uint32_t uart_rx_read(void *handle, uint16_t *out, uint32_t limit) {
    Receiver *r = handle;
    uint32_t n = 0;
    uint32_t tail=atomic_load_explicit(&r->tail,memory_order_relaxed);
    while (n<limit && tail!=atomic_load_explicit(&r->head,memory_order_acquire)) {
        out[n++]=r->ring[tail];
        tail=(tail+1u)%CAPACITY;
    }
    atomic_store_explicit(&r->tail,tail,memory_order_release);
    return n;
}

uint64_t uart_rx_dropped(void *handle) {
    Receiver *r = handle;
    return atomic_load(&r->dropped);
}

void uart_rx_metrics(void *handle,uint32_t *out) {
    Receiver *r=handle;
    out[0]=atomic_load(&r->max_gap_us);out[1]=atomic_load(&r->gaps_over_1ms);
    out[2]=r->affinity_error;out[3]=r->realtime_error;out[4]=r->policy;out[5]=r->priority;
}

void uart_rx_stop(void *handle) {
    Receiver *r = handle;
    atomic_store(&r->running, 0);
    pthread_join(r->thread, NULL);
    free(r);
}
