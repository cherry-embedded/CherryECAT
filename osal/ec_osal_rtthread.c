/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"
#include <rtthread.h>
#include <rthw.h>

ec_osal_thread_t ec_osal_thread_create(const char *name, uint32_t stack_size, uint32_t prio, ec_thread_entry_t entry, void *args)
{
    rt_thread_t htask;
    htask = rt_thread_create(name, entry, args, stack_size, prio, 10);
    if (htask == NULL) {
        EC_LOG_ERR("Create thread %s failed\r\n", name);
        while (1) {
        }
    }
    rt_thread_startup(htask);
    return (ec_osal_thread_t)htask;
}

void ec_osal_thread_delete(ec_osal_thread_t thread)
{
    if (thread == NULL) {
        rt_thread_t self = rt_thread_self();
        rt_thread_control(self, RT_THREAD_CTRL_CLOSE, RT_NULL);
        return;
    }

    rt_thread_delete(thread);
}

void ec_osal_thread_suspend(ec_osal_thread_t thread)
{
    rt_thread_suspend((rt_thread_t)thread);
}

void ec_osal_thread_resume(ec_osal_thread_t thread)
{
    rt_thread_resume((rt_thread_t)thread);
}

ec_osal_sem_t ec_osal_sem_create(uint32_t max_count, uint32_t initial_count)
{
    ec_osal_sem_t sem = (ec_osal_sem_t)rt_sem_create("ec_sem", initial_count, RT_IPC_FLAG_FIFO);
    if (sem == NULL) {
        EC_LOG_ERR("Create semaphore failed\r\n");
        while (1) {
        }
    }
    return sem;
}

void ec_osal_sem_delete(ec_osal_sem_t sem)
{
    rt_sem_delete((rt_sem_t)sem);
}

int ec_osal_sem_take(ec_osal_sem_t sem, uint32_t timeout)
{
    int ret = 0;
    rt_err_t result = RT_EOK;

    if (timeout == EC_OSAL_WAITING_FOREVER) {
        result = rt_sem_take((rt_sem_t)sem, RT_WAITING_FOREVER);
    } else {
        result = rt_sem_take((rt_sem_t)sem, rt_tick_from_millisecond(timeout));
    }
    if (result == -RT_ETIMEOUT) {
        ret = -EC_ERR_TIMEOUT;
    } else if (result == -RT_ERROR) {
        ret = -EC_ERR_INVAL;
    } else {
        ret = 0;
    }

    return (int)ret;
}

int ec_osal_sem_give(ec_osal_sem_t sem)
{
    return (int)rt_sem_release((rt_sem_t)sem);
}

void ec_osal_sem_reset(ec_osal_sem_t sem)
{
    rt_sem_control((rt_sem_t)sem, RT_IPC_CMD_RESET, (void *)0);
}

ec_osal_mutex_t ec_osal_mutex_create(void)
{
    ec_osal_mutex_t mutex = (ec_osal_mutex_t)rt_mutex_create("ec_mutex", RT_IPC_FLAG_FIFO);
    if (mutex == NULL) {
        EC_LOG_ERR("Create mutex failed\r\n");
        while (1) {
        }
    }
    return mutex;
}

void ec_osal_mutex_delete(ec_osal_mutex_t mutex)
{
    rt_mutex_delete((rt_mutex_t)mutex);
}

int ec_osal_mutex_take(ec_osal_mutex_t mutex)
{
    return (int)rt_mutex_take((rt_mutex_t)mutex, RT_WAITING_FOREVER);
}

int ec_osal_mutex_give(ec_osal_mutex_t mutex)
{
    return (int)rt_mutex_release((rt_mutex_t)mutex);
}


ec_osal_mq_t ec_osal_mq_create(uint32_t max_msgs)
{
    return (ec_osal_mq_t)rt_mq_create("ec_mq", sizeof(uintptr_t), max_msgs, RT_IPC_FLAG_FIFO);
}

void ec_osal_mq_delete(ec_osal_mq_t mq)
{
    rt_mq_delete((rt_mq_t)mq);
}

int ec_osal_mq_send(ec_osal_mq_t mq, uintptr_t addr)
{
    return rt_mq_send((rt_mq_t)mq, &addr, sizeof(uintptr_t));
}

int ec_osal_mq_recv(ec_osal_mq_t mq, uintptr_t *addr, uint32_t timeout)
{
    int ret = 0;
    rt_err_t result = RT_EOK;

    if (timeout == EC_OSAL_WAITING_FOREVER) {
        result = rt_mq_recv((rt_mq_t)mq, addr, sizeof(uintptr_t), RT_WAITING_FOREVER);
    } else {
        result = rt_mq_recv((rt_mq_t)mq, addr, sizeof(uintptr_t), rt_tick_from_millisecond(timeout));
    }
    if (result == -RT_ETIMEOUT) {
        ret = -EC_ERR_TIMEOUT;
    } else if (result == -RT_ERROR) {
        ret = -EC_ERR_INVAL;
    } else {
        ret = 0;
    }

    return (int)ret;
}

struct ec_osal_timer *ec_osal_timer_create(const char *name, uint32_t timeout_ms, ec_timer_handler_t handler, void *argument, bool is_period)
{
    struct ec_osal_timer *timer;

    timer = rt_malloc(sizeof(struct ec_osal_timer));
    if (timer == NULL) {
        EC_LOG_ERR("Create ec_osal_timer failed\r\n");
        while (1) {
        }
    }
    memset(timer, 0, sizeof(struct ec_osal_timer));

    timer->timer = (void *)rt_timer_create(name, handler, argument, timeout_ms, is_period ? (RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER) : (RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER));
    if (timer->timer == NULL) {
        EC_LOG_ERR("Create timer failed\r\n");
        while (1) {
        }
    }
    return timer;
}

void ec_osal_timer_delete(struct ec_osal_timer *timer)
{
    rt_timer_stop(timer->timer);
    rt_timer_delete(timer->timer);
    rt_free(timer);
}

void ec_osal_timer_start(struct ec_osal_timer *timer)
{
    rt_timer_start(timer->timer);
}

void ec_osal_timer_stop(struct ec_osal_timer *timer)
{
    rt_timer_stop(timer->timer);
}

size_t ec_osal_enter_critical_section(void)
{
    return rt_hw_interrupt_disable();
}

void ec_osal_leave_critical_section(size_t flag)
{
    rt_hw_interrupt_enable(flag);
}

void ec_osal_msleep(uint32_t delay)
{
    rt_thread_mdelay(delay);
}

void *ec_osal_malloc(size_t size)
{
    return rt_malloc(size);
}

void ec_osal_free(void *ptr)
{
    rt_free(ptr);
}
