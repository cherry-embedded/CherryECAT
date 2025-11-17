/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"
#include <FreeRTOS.h>
#include "semphr.h"
#include "event_groups.h"

ec_osal_thread_t ec_osal_thread_create(const char *name, uint32_t stack_size, uint32_t prio, ec_thread_entry_t entry, void *args)
{
    TaskHandle_t htask = NULL;
    stack_size /= sizeof(StackType_t);
    xTaskCreate(entry, name, stack_size, args, configMAX_PRIORITIES - 1 - prio, &htask);
    if (htask == NULL) {
        EC_LOG_ERR("Create thread %s failed\r\n", name);
        while (1) {
        }
    }
    return (ec_osal_thread_t)htask;
}

void ec_osal_thread_delete(ec_osal_thread_t thread)
{
    vTaskDelete(thread);
}

void ec_osal_thread_suspend(ec_osal_thread_t thread)
{
    vTaskSuspend((TaskHandle_t)thread);
}

void ec_osal_thread_resume(ec_osal_thread_t thread)
{
    vTaskResume((TaskHandle_t)thread);
}

ec_osal_sem_t ec_osal_sem_create(uint32_t max_count, uint32_t initial_count)
{
    ec_osal_sem_t sem = (ec_osal_sem_t)xSemaphoreCreateCounting(max_count, initial_count);
    if (sem == NULL) {
        EC_LOG_ERR("Create semaphore failed\r\n");
        while (1) {
        }
    }
    return sem;
}

void ec_osal_sem_delete(ec_osal_sem_t sem)
{
    vSemaphoreDelete((SemaphoreHandle_t)sem);
}

int ec_osal_sem_take(ec_osal_sem_t sem, uint32_t timeout)
{
    if (timeout == EC_OSAL_WAITING_FOREVER) {
        return (xSemaphoreTake((SemaphoreHandle_t)sem, portMAX_DELAY) == pdPASS) ? 0 : -EC_ERR_TIMEOUT;
    } else {
        return (xSemaphoreTake((SemaphoreHandle_t)sem, pdMS_TO_TICKS(timeout)) == pdPASS) ? 0 : -EC_ERR_TIMEOUT;
    }
}

int ec_osal_sem_give(ec_osal_sem_t sem)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int ret;

    if (xPortIsInsideInterrupt()) {
        ret = xSemaphoreGiveFromISR((SemaphoreHandle_t)sem, &xHigherPriorityTaskWoken);
        if (ret == pdPASS) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    } else {
        ret = xSemaphoreGive((SemaphoreHandle_t)sem);
    }

    return (ret == pdPASS) ? 0 : -EC_ERR_TIMEOUT;
}

void ec_osal_sem_reset(ec_osal_sem_t sem)
{
    xQueueReset((QueueHandle_t)sem);
}

ec_osal_mutex_t ec_osal_mutex_create(void)
{
    ec_osal_mutex_t mutex = (ec_osal_mutex_t)xSemaphoreCreateMutex();
    if (mutex == NULL) {
        EC_LOG_ERR("Create mutex failed\r\n");
        while (1) {
        }
    }
    return mutex;
}

void ec_osal_mutex_delete(ec_osal_mutex_t mutex)
{
    vSemaphoreDelete((SemaphoreHandle_t)mutex);
}

int ec_osal_mutex_take(ec_osal_mutex_t mutex)
{
    return (xSemaphoreTake((SemaphoreHandle_t)mutex, portMAX_DELAY) == pdPASS) ? 0 : -EC_ERR_TIMEOUT;
}

int ec_osal_mutex_give(ec_osal_mutex_t mutex)
{
    return (xSemaphoreGive((SemaphoreHandle_t)mutex) == pdPASS) ? 0 : -EC_ERR_TIMEOUT;
}

ec_osal_mq_t ec_osal_mq_create(uint32_t max_msgs)
{
    return (ec_osal_mq_t)xQueueCreate(max_msgs, sizeof(uintptr_t));
}

void ec_osal_mq_delete(ec_osal_mq_t mq)
{
    vQueueDelete((QueueHandle_t)mq);
}

int ec_osal_mq_send(ec_osal_mq_t mq, uintptr_t addr)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int ret;

    if (xPortIsInsideInterrupt()) {
        ret = xQueueSendFromISR((ec_osal_mq_t)mq, &addr, &xHigherPriorityTaskWoken);
        if (ret == pdPASS) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    } else {
        ret = xQueueSend((ec_osal_mq_t)mq, &addr, 0);
    }

    return (ret == pdPASS) ? 0 : -EC_ERR_TIMEOUT;
}

int ec_osal_mq_recv(ec_osal_mq_t mq, uintptr_t *addr, uint32_t timeout)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int ret;

    if (xPortIsInsideInterrupt()) {
        ret = xQueueReceiveFromISR((ec_osal_mq_t)mq, addr, &xHigherPriorityTaskWoken);
        if (ret == pdPASS) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        return (ret == pdPASS) ? 0 : -EC_ERR_TIMEOUT;
    } else {
        if (timeout == EC_OSAL_WAITING_FOREVER) {
            return (xQueueReceive((ec_osal_mq_t)mq, addr, portMAX_DELAY) == pdPASS) ? 0 : -EC_ERR_TIMEOUT;
        } else {
            return (xQueueReceive((ec_osal_mq_t)mq, addr, pdMS_TO_TICKS(timeout)) == pdPASS) ? 0 : -EC_ERR_TIMEOUT;
        }
    }
}

static void __ec_timeout(TimerHandle_t *handle)
{
    struct ec_osal_timer *timer = (struct ec_osal_timer *)pvTimerGetTimerID((TimerHandle_t)handle);

    timer->handler(timer->argument);
}

struct ec_osal_timer *ec_osal_timer_create(const char *name, uint32_t timeout_ms, ec_timer_handler_t handler, void *argument, bool is_period)
{
    struct ec_osal_timer *timer;
    (void)name;

    timer = pvPortMalloc(sizeof(struct ec_osal_timer));

    if (timer == NULL) {
        EC_LOG_ERR("Create ec_osal_timer failed\r\n");
        while (1) {
        }
    }
    memset(timer, 0, sizeof(struct ec_osal_timer));

    timer->handler = handler;
    timer->argument = argument;

    timer->timer = (void *)xTimerCreate("ec_tim", pdMS_TO_TICKS(timeout_ms), is_period, timer, (TimerCallbackFunction_t)__ec_timeout);
    if (timer->timer == NULL) {
        EC_LOG_ERR("Create timer failed\r\n");
        while (1) {
        }
    }
    return timer;
}

void ec_osal_timer_delete(struct ec_osal_timer *timer)
{
    xTimerStop(timer->timer, 0);
    xTimerDelete(timer->timer, 0);
    vPortFree(timer);
}

void ec_osal_timer_start(struct ec_osal_timer *timer)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int ret;

    if (xPortIsInsideInterrupt()) {
        ret = xTimerStartFromISR(timer->timer, &xHigherPriorityTaskWoken);
        if (ret == pdPASS) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    } else {
        xTimerStart(timer->timer, 0);
    }
}

void ec_osal_timer_stop(struct ec_osal_timer *timer)
{
    xTimerStop(timer->timer, 0);
}

size_t ec_osal_enter_critical_section(void)
{
    size_t ret;

    if (xPortIsInsideInterrupt()) {
        ret = taskENTER_CRITICAL_FROM_ISR();
    } else {
        taskENTER_CRITICAL();
        ret = 1;
    }

    return ret;
}

void ec_osal_leave_critical_section(size_t flag)
{
    if (xPortIsInsideInterrupt()) {
        taskEXIT_CRITICAL_FROM_ISR(flag);
    } else {
        taskEXIT_CRITICAL();
    }
}

void ec_osal_msleep(uint32_t delay)
{
    vTaskDelay(pdMS_TO_TICKS(delay));
}

void *ec_osal_malloc(size_t size)
{
    return pvPortMalloc(size);
}

void ec_osal_free(void *ptr)
{
    vPortFree(ptr);
}