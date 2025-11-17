/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_OSAL_H
#define EC_OSAL_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#ifdef __INCLUDE_NUTTX_CONFIG_H
#define CONFIG_EC_OSAL_THREAD_SET_ARGV int argc, char **argv
#define CONFIG_EC_OSAL_THREAD_GET_ARGV ((uintptr_t)strtoul(argv[1], NULL, 16))
#elif defined(__ZEPHYR__)
#define CONFIG_EC_OSAL_THREAD_SET_ARGV void *p1, void *p2, void *p3
#define CONFIG_EC_OSAL_THREAD_GET_ARGV ((uintptr_t)p1)
#else
#define CONFIG_EC_OSAL_THREAD_SET_ARGV void *argument
#define CONFIG_EC_OSAL_THREAD_GET_ARGV ((uintptr_t)argument)
#endif

#define EC_OSAL_WAITING_FOREVER (0xFFFFFFFFU)

typedef void *ec_osal_thread_t;
typedef void *ec_osal_sem_t;
typedef void *ec_osal_mutex_t;
typedef void *ec_osal_mq_t;
typedef void (*ec_thread_entry_t)(CONFIG_EC_OSAL_THREAD_SET_ARGV);
typedef void (*ec_timer_handler_t)(void *argument);
struct ec_osal_timer {
    ec_timer_handler_t handler;
    void *argument;
    bool is_period;
    uint32_t timeout_ms;
    void *timer;
};

/*
 * Task with smaller priority value indicates higher task priority
*/
ec_osal_thread_t ec_osal_thread_create(const char *name, uint32_t stack_size, uint32_t prio, ec_thread_entry_t entry, void *args);
void ec_osal_thread_delete(ec_osal_thread_t thread);
void ec_osal_thread_suspend(ec_osal_thread_t thread);
void ec_osal_thread_resume(ec_osal_thread_t thread);

ec_osal_sem_t ec_osal_sem_create(uint32_t max_count, uint32_t initial_count);
void ec_osal_sem_delete(ec_osal_sem_t sem);
int ec_osal_sem_take(ec_osal_sem_t sem, uint32_t timeout);
int ec_osal_sem_give(ec_osal_sem_t sem);
void ec_osal_sem_reset(ec_osal_sem_t sem);

ec_osal_mutex_t ec_osal_mutex_create(void);
void ec_osal_mutex_delete(ec_osal_mutex_t mutex);
int ec_osal_mutex_take(ec_osal_mutex_t mutex);
int ec_osal_mutex_give(ec_osal_mutex_t mutex);

ec_osal_mq_t ec_osal_mq_create(uint32_t max_msgs);
void ec_osal_mq_delete(ec_osal_mq_t mq);
int ec_osal_mq_send(ec_osal_mq_t mq, uintptr_t addr);
int ec_osal_mq_recv(ec_osal_mq_t mq, uintptr_t *addr, uint32_t timeout);

struct ec_osal_timer *ec_osal_timer_create(const char *name, uint32_t timeout_ms, ec_timer_handler_t handler, void *argument, bool is_period);
void ec_osal_timer_delete(struct ec_osal_timer *timer);
void ec_osal_timer_start(struct ec_osal_timer *timer);
void ec_osal_timer_stop(struct ec_osal_timer *timer);

size_t ec_osal_enter_critical_section(void);
void ec_osal_leave_critical_section(size_t flag);

void ec_osal_msleep(uint32_t delay);

void *ec_osal_malloc(size_t size);
void ec_osal_free(void *ptr);

#endif /* EC_OSAL_H */
