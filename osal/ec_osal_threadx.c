/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"
#include "tx_api.h"

extern TX_BYTE_POOL ec_byte_pool; // define ec_byte_pool and call ec_osal_init first

TX_QUEUE *ec_osal_mq = TX_NULL;

ec_osal_thread_t ec_osal_thread_create(const char *name, uint32_t stack_size, uint32_t prio, ec_thread_entry_t entry, void *args)
{
    TX_THREAD *thread_ptr = TX_NULL;

    tx_byte_allocate(&ec_byte_pool, (VOID **)&thread_ptr, EC_ALIGN_UP(sizeof(TX_THREAD), 4) + stack_size, TX_NO_WAIT);
    if (thread_ptr == TX_NULL) {
        EC_LOG_ERR("Create thread %s failed\r\n", name);
        while (1) {
        }
    }

    tx_thread_create(thread_ptr, (CHAR *)name, (VOID(*)(ULONG))entry, (uintptr_t)args,
                     ((CHAR *)thread_ptr + EC_ALIGN_UP(sizeof(TX_THREAD), 4)), stack_size,
                     prio, prio, TX_NO_TIME_SLICE, TX_AUTO_START);

    return (ec_osal_thread_t)thread_ptr;
}

void ec_osal_thread_delete(ec_osal_thread_t thread)
{
    if (thread == NULL) {
        thread = tx_thread_identify();

        uintptr_t addr = (uintptr_t)thread;
        tx_queue_send((TX_QUEUE *)ec_osal_mq, &addr, TX_NO_WAIT);
        tx_thread_terminate(thread);

        return;
    }

    tx_thread_terminate(thread);
    tx_thread_delete(thread);
    tx_byte_release(thread);
}

void ec_osal_thread_suspend(ec_osal_thread_t thread)
{
    tx_thread_suspend((TX_THREAD *)thread);
}

void ec_osal_thread_resume(ec_osal_thread_t thread)
{
    tx_thread_resume((TX_THREAD *)thread);
}

ec_osal_sem_t ec_osal_sem_create(uint32_t max_count, uint32_t initial_count)
{
    TX_SEMAPHORE *sem_ptr = TX_NULL;

    tx_byte_allocate(&ec_byte_pool, (VOID **)&sem_ptr, sizeof(TX_SEMAPHORE), TX_NO_WAIT);
    if (sem_ptr == TX_NULL) {
        EC_LOG_ERR("Create semaphore failed\r\n");
        while (1) {
        }
    }

    tx_semaphore_create(sem_ptr, "ec_sem", initial_count);
    return (ec_osal_sem_t)sem_ptr;
}

void ec_osal_sem_delete(ec_osal_sem_t sem)
{
    tx_semaphore_delete((TX_SEMAPHORE *)sem);
    tx_byte_release(sem);
}

int ec_osal_sem_take(ec_osal_sem_t sem, uint32_t timeout)
{
    int ret = 0;

    ret = tx_semaphore_get((TX_SEMAPHORE *)sem, timeout);
    if (ret == TX_SUCCESS) {
        ret = 0;
    } else if ((ret == TX_WAIT_ABORTED) || (ret == TX_NO_INSTANCE)) {
        ret = -EC_ERR_TIMEOUT;
    } else {
        ret = -EC_ERR_INVAL;
    }

    return (int)ret;
}

int ec_osal_sem_give(ec_osal_sem_t sem)
{
    return (tx_semaphore_put((TX_SEMAPHORE *)sem) == TX_SUCCESS) ? 0 : -EC_ERR_INVAL;
}

void ec_osal_sem_reset(ec_osal_sem_t sem)
{
    tx_semaphore_get((TX_SEMAPHORE *)sem, 0);
}

ec_osal_mutex_t ec_osal_mutex_create(void)
{
    TX_MUTEX *mutex_ptr = TX_NULL;

    tx_byte_allocate(&ec_byte_pool, (VOID **)&mutex_ptr, sizeof(TX_MUTEX), TX_NO_WAIT);
    if (mutex_ptr == TX_NULL) {
        EC_LOG_ERR("Create mutex failed\r\n");
        while (1) {
        }
    }

    tx_mutex_create(mutex_ptr, "ec_mutex", TX_INHERIT);
    return (ec_osal_mutex_t)mutex_ptr;
}

void ec_osal_mutex_delete(ec_osal_mutex_t mutex)
{
    tx_mutex_delete((TX_MUTEX *)mutex);
    tx_byte_release(mutex);
}

int ec_osal_mutex_take(ec_osal_mutex_t mutex)
{
    int ret = 0;

    ret = tx_mutex_get((TX_MUTEX *)mutex, TX_WAIT_FOREVER);
    if (ret == TX_SUCCESS) {
        ret = 0;
    } else if ((ret == TX_WAIT_ABORTED) || (ret == TX_NO_INSTANCE)) {
        ret = -EC_ERR_TIMEOUT;
    } else {
        ret = -EC_ERR_INVAL;
    }

    return (int)ret;
}

int ec_osal_mutex_give(ec_osal_mutex_t mutex)
{
    return (tx_mutex_put((TX_MUTEX *)mutex) == TX_SUCCESS) ? 0 : -EC_ERR_INVAL;
}

ec_osal_mq_t ec_osal_mq_create(uint32_t max_msgs)
{
    TX_QUEUE *queue_ptr = TX_NULL;

    tx_byte_allocate(&ec_byte_pool, (VOID **)&queue_ptr, EC_ALIGN_UP(sizeof(TX_QUEUE), 4) + sizeof(uintptr_t) * max_msgs, TX_NO_WAIT);
    if (queue_ptr == TX_NULL) {
        EC_LOG_ERR("Create TX_QUEUE failed\r\n");
        while (1) {
        }
    }

    tx_queue_create(queue_ptr, "ec_mq", sizeof(uintptr_t) / 4, (CHAR *)queue_ptr + EC_ALIGN_UP(sizeof(TX_QUEUE), 4), sizeof(uintptr_t) * max_msgs);
    return (ec_osal_mq_t)queue_ptr;
}

void ec_osal_mq_delete(ec_osal_mq_t mq)
{
    tx_queue_delete((TX_QUEUE *)mq);
    tx_byte_release(mq);
}

int ec_osal_mq_send(ec_osal_mq_t mq, uintptr_t addr)
{
    return (tx_queue_send((TX_QUEUE *)mq, &addr, TX_NO_WAIT) == TX_SUCCESS) ? 0 : -EC_ERR_INVAL;
}

int ec_osal_mq_recv(ec_osal_mq_t mq, uintptr_t *addr, uint32_t timeout)
{
    int ret = 0;

    ret = tx_queue_receive((TX_QUEUE *)mq, addr, timeout);
    if (ret == TX_SUCCESS) {
        ret = 0;
    } else if (ret == TX_QUEUE_EMPTY) {
        ret = -EC_ERR_TIMEOUT;
    } else {
        ret = -EC_ERR_INVAL;
    }

    return (int)ret;
}

struct ec_osal_timer *ec_osal_timer_create(const char *name, uint32_t timeout_ms, ec_timer_handler_t handler, void *argument, bool is_period)
{
    TX_TIMER *timer_ptr = TX_NULL;
    struct ec_osal_timer *timer;

    tx_byte_allocate(&ec_byte_pool, (VOID **)&timer, sizeof(struct ec_osal_timer), TX_NO_WAIT);
    if (timer == TX_NULL) {
        EC_LOG_ERR("Create ec_osal_timer failed\r\n");
        while (1) {
        }
    }
    memset(timer, 0, sizeof(struct ec_osal_timer));

    tx_byte_allocate(&ec_byte_pool, (VOID **)&timer_ptr, sizeof(TX_TIMER), TX_NO_WAIT);
    if (timer_ptr == TX_NULL) {
        EC_LOG_ERR("Create TX_TIMER failed\r\n");
        while (1) {
        }
    }

    timer->timer = timer_ptr;
    timer->timeout_ms = timeout_ms;
    timer->is_period = is_period;
    if (tx_timer_create(timer_ptr, (CHAR *)name, (void (*)(ULONG))handler, (uintptr_t)argument, 1, is_period ? 1 : 0,
                        TX_NO_ACTIVATE) != TX_SUCCESS) {
        return NULL;
    }
    return timer;
}

void ec_osal_timer_delete(struct ec_osal_timer *timer)
{
    tx_timer_deactivate((TX_TIMER *)timer->timer);
    tx_timer_delete((TX_TIMER *)timer->timer);
    tx_byte_release(timer->timer);
    tx_byte_release(timer);
}

void ec_osal_timer_start(struct ec_osal_timer *timer)
{
    if (tx_timer_change((TX_TIMER *)timer->timer, timer->timeout_ms, timer->is_period ? timer->timeout_ms : 0) == TX_SUCCESS) {
        /* Call the tx_timer_activate to activates the specified application
            timer. The expiration routines of timers that expire at the same
            time are executed in the order they were activated. */
        if (tx_timer_activate((TX_TIMER *)timer->timer) == TX_SUCCESS) {
            /* Return osOK for success */
        } else {
            /* Return osErrorResource in case of error */
        }
    } else {
    }
}

void ec_osal_timer_stop(struct ec_osal_timer *timer)
{
    tx_timer_deactivate((TX_TIMER *)timer->timer);
}

size_t ec_osal_enter_critical_section(void)
{
    TX_INTERRUPT_SAVE_AREA

    TX_DISABLE

    return interrupt_save;
}

void ec_osal_leave_critical_section(size_t flag)
{
    TX_INTERRUPT_SAVE_AREA

    interrupt_save = flag;
    TX_RESTORE
}

void ec_osal_msleep(uint32_t delay)
{
#if TX_TIMER_TICKS_PER_SECOND != 1000
#error "TX_TIMER_TICKS_PER_SECOND must be 1000"
#endif
    tx_thread_sleep(delay);
}

void *ec_osal_malloc(size_t size)
{
    CHAR *pointer = TX_NULL;

    tx_byte_allocate(&ec_byte_pool, (VOID **)&pointer, size, TX_WAIT_FOREVER);

    return pointer;
}

void ec_osal_free(void *ptr)
{
    tx_byte_release(ptr);
}

static void ec_osal_thread(CONFIG_EC_OSAL_THREAD_SET_ARGV)
{
    int ret;
    ec_osal_thread_t thread;

    while (1) {
        ret = tx_queue_receive(ec_osal_mq, (uintptr_t *)&thread, TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS) {
            continue;
        }
        tx_thread_delete(thread);
        tx_byte_release(thread);
    }
}

void ec_osal_init(uint8_t *mem, uint32_t mem_size)
{
    ec_osal_thread_t thread;

    tx_byte_pool_create(&ec_byte_pool, "ec byte pool", mem, mem_size);

    thread = ec_osal_thread_create("ec_osal", 2048, 10, ec_osal_thread, NULL);
    if (thread == NULL) {
        EC_LOG_ERR("Create ec_osal_thread failed\r\n");
        while (1) {
        }
    }

    tx_byte_allocate(&ec_byte_pool, (VOID **)&ec_osal_mq, EC_ALIGN_UP(sizeof(TX_QUEUE), 4) + sizeof(uintptr_t) * max_msgs, TX_NO_WAIT);
    if (ec_osal_mq == TX_NULL) {
        EC_LOG_ERR("Create ec_osal_mq failed\r\n");
        while (1) {
        }
    }

    tx_queue_create(ec_osal_mq, "ec_mq", sizeof(uintptr_t) / 4, (CHAR *)ec_osal_mq + EC_ALIGN_UP(sizeof(TX_QUEUE), 4), sizeof(uintptr_t) * 32);
}
