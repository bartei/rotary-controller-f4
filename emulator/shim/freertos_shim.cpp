/*
 * FreeRTOS / CMSIS-RTOS v2 shim implementation for emulator.
 *
 * Maps FreeRTOS primitives to POSIX threads + C++ sync primitives.
 * - osThreadNew   → pthread_create
 * - osDelay       → usleep
 * - xTimerCreate  → background timer thread with condition variable
 * - xSemaphore*   → pthread_mutex + condition variable (counting semaphore)
 * - xTaskNotify   → per-task condition variable + notification value
 * - xQueue*       → mutex-protected ring buffer
 */

#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <vector>
#include <deque>
#include <atomic>
#include <map>
#include <unistd.h>
#include <pthread.h>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "cmsis_os2.h"
}

/* ========================================================================
 * Task Notification
 * ======================================================================== */

struct TaskControlBlock {
    std::mutex mtx;
    std::condition_variable cv;
    uint32_t notifyValue;
    bool notifyPending;
    pthread_t threadId;

    TaskControlBlock() : notifyValue(0), notifyPending(false), threadId(0) {}
};

/* Map pthread_t → TaskControlBlock for task notifications */
static std::mutex g_taskMapMutex;
static std::map<pthread_t, TaskControlBlock*> g_taskMap;

static TaskControlBlock* getOrCreateTCB(pthread_t tid) {
    std::lock_guard<std::mutex> lock(g_taskMapMutex);
    auto it = g_taskMap.find(tid);
    if (it != g_taskMap.end()) return it->second;
    auto *tcb = new TaskControlBlock();
    tcb->threadId = tid;
    g_taskMap[tid] = tcb;
    return tcb;
}

static TaskControlBlock* getCurrentTCB() {
    return getOrCreateTCB(pthread_self());
}

extern "C" TaskHandle_t xTaskGetCurrentTaskHandle(void) {
    return (TaskHandle_t)getCurrentTCB();
}

extern "C" BaseType_t xTaskNotify(TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction) {
    auto *tcb = (TaskControlBlock*)xTaskToNotify;
    if (!tcb) return pdFAIL;
    {
        std::lock_guard<std::mutex> lock(tcb->mtx);
        switch (eAction) {
            case eSetValueWithOverwrite:
                tcb->notifyValue = ulValue;
                break;
            case eSetValueWithoutOverwrite:
                if (!tcb->notifyPending) tcb->notifyValue = ulValue;
                break;
            case eIncrement:
                tcb->notifyValue++;
                break;
            case eNoAction:
            default:
                break;
        }
        tcb->notifyPending = true;
    }
    tcb->cv.notify_one();
    return pdPASS;
}

extern "C" BaseType_t xTaskNotifyFromISR(TaskHandle_t xTaskToNotify, uint32_t ulValue,
                                          eNotifyAction eAction, BaseType_t *pxHigherPriorityTaskWoken) {
    if (pxHigherPriorityTaskWoken) *pxHigherPriorityTaskWoken = pdFALSE;
    return xTaskNotify(xTaskToNotify, ulValue, eAction);
}

extern "C" uint32_t ulTaskNotifyTake(BaseType_t xClearCountOnExit, TickType_t xTicksToWait) {
    auto *tcb = getCurrentTCB();
    std::unique_lock<std::mutex> lock(tcb->mtx);

    if (!tcb->notifyPending) {
        if (xTicksToWait == portMAX_DELAY) {
            tcb->cv.wait(lock, [&]{ return tcb->notifyPending; });
        } else {
            auto ms = std::chrono::milliseconds(xTicksToWait);
            tcb->cv.wait_for(lock, ms, [&]{ return tcb->notifyPending; });
        }
    }

    uint32_t val = tcb->notifyValue;
    if (tcb->notifyPending) {
        if (xClearCountOnExit) {
            tcb->notifyValue = 0;
        } else {
            if (tcb->notifyValue > 0) tcb->notifyValue--;
        }
        tcb->notifyPending = false;
    }
    return val;
}

/* ========================================================================
 * Threads (osThreadNew)
 * ======================================================================== */

struct ThreadArgs {
    osThreadFunc_t func;
    void *arg;
    TaskControlBlock *tcb;
};

static void *threadEntry(void *rawArgs) {
    auto *args = (ThreadArgs*)rawArgs;
    /* Register this thread's TCB in the global map */
    {
        std::lock_guard<std::mutex> lock(g_taskMapMutex);
        args->tcb->threadId = pthread_self();
        g_taskMap[pthread_self()] = args->tcb;
    }
    args->func(args->arg);
    delete args;
    return nullptr;
}

extern "C" osThreadId_t osThreadNew(osThreadFunc_t func, void *argument, const osThreadAttr_t *attr) {
    (void)attr;
    auto *tcb = new TaskControlBlock();
    auto *args = new ThreadArgs{func, argument, tcb};

    pthread_t tid;
    pthread_attr_t pattr;
    pthread_attr_init(&pattr);
    pthread_attr_setdetachstate(&pattr, PTHREAD_CREATE_DETACHED);
    int ret = pthread_create(&tid, &pattr, threadEntry, args);
    pthread_attr_destroy(&pattr);

    if (ret != 0) {
        delete args;
        delete tcb;
        return nullptr;
    }

    /* Temporarily store tid so the TCB is findable even before the thread runs */
    tcb->threadId = tid;
    {
        std::lock_guard<std::mutex> lock(g_taskMapMutex);
        g_taskMap[tid] = tcb;
    }

    return (osThreadId_t)tcb;
}

extern "C" osThreadId_t osThreadGetId(void) {
    return (osThreadId_t)getCurrentTCB();
}

/* ========================================================================
 * Delay
 * ======================================================================== */

extern "C" osStatus_t osDelay(uint32_t ticks) {
    /* ticks are in ms (configTICK_RATE_HZ = 1000) */
    usleep(ticks * 1000);
    return osOK;
}

/* ========================================================================
 * Software Timers
 * ======================================================================== */

struct SoftwareTimer {
    std::string name;
    uint32_t periodMs;
    bool autoReload;
    TimerCallbackFunction_t callback;
    std::atomic<bool> running;
    std::atomic<bool> alive;
    std::mutex mtx;
    std::condition_variable cv;
    std::thread worker;

    SoftwareTimer(const char *n, uint32_t period, bool reload, TimerCallbackFunction_t cb)
        : name(n ? n : ""), periodMs(period), autoReload(reload), callback(cb),
          running(false), alive(true)
    {
        worker = std::thread([this]() { timerLoop(); });
        worker.detach();
    }

    void timerLoop() {
        while (alive.load()) {
            {
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock, [this]{ return running.load() || !alive.load(); });
            }
            if (!alive.load()) break;

            /* Wait for the timer period */
            {
                std::unique_lock<std::mutex> lock(mtx);
                bool expired = !cv.wait_for(lock, std::chrono::milliseconds(periodMs),
                                              [this]{ return !running.load() || !alive.load(); });
                if (!alive.load()) break;
                if (!running.load()) continue;  /* Timer was stopped/reset during wait */

                if (expired) {
                    /* Timer fired */
                    if (!autoReload) running.store(false);
                    lock.unlock();
                    /* The Modbus library declares its timer callbacks as
                     * void cb(TimerHandle_t *pxTimer) but in real FreeRTOS
                     * TimerCallbackFunction_t passes the handle by value.
                     * The library then compares the received value against the
                     * stored handle, so we must pass the handle value itself
                     * (cast to match our typedef). */
                    callback((TimerHandle_t*)this);
                }
            }
        }
    }
};

extern "C" TimerHandle_t xTimerCreate(const char *pcTimerName, TickType_t xTimerPeriodInTicks,
                                       UBaseType_t uxAutoReload, void *pvTimerID,
                                       TimerCallbackFunction_t pxCallbackFunction) {
    (void)pvTimerID;
    auto *t = new SoftwareTimer(pcTimerName, xTimerPeriodInTicks, uxAutoReload != 0, pxCallbackFunction);
    return (TimerHandle_t)t;
}

extern "C" BaseType_t xTimerStart(TimerHandle_t xTimer, TickType_t xBlockTime) {
    (void)xBlockTime;
    auto *t = (SoftwareTimer*)xTimer;
    t->running.store(true);
    t->cv.notify_all();
    return pdPASS;
}

extern "C" BaseType_t xTimerStop(TimerHandle_t xTimer, TickType_t xBlockTime) {
    (void)xBlockTime;
    auto *t = (SoftwareTimer*)xTimer;
    t->running.store(false);
    t->cv.notify_all();
    return pdPASS;
}

extern "C" BaseType_t xTimerReset(TimerHandle_t xTimer, TickType_t xBlockTime) {
    (void)xBlockTime;
    auto *t = (SoftwareTimer*)xTimer;
    /* Reset = stop + restart, which re-arms the period */
    t->running.store(false);
    t->cv.notify_all();
    /* Brief yield to let the timer thread notice the stop */
    usleep(100);
    t->running.store(true);
    t->cv.notify_all();
    return pdPASS;
}

extern "C" BaseType_t xTimerResetFromISR(TimerHandle_t xTimer, BaseType_t *pxHigherPriorityTaskWoken) {
    if (pxHigherPriorityTaskWoken) *pxHigherPriorityTaskWoken = pdFALSE;
    return xTimerReset(xTimer, 0);
}

/* ========================================================================
 * Semaphores
 * ======================================================================== */

struct CountingSemaphore {
    std::mutex mtx;
    std::condition_variable cv;
    uint32_t count;
    uint32_t maxCount;

    CountingSemaphore(uint32_t max, uint32_t initial) : count(initial), maxCount(max) {}
};

extern "C" SemaphoreHandle_t xSemaphoreCreateBinary(void) {
    return (SemaphoreHandle_t) new CountingSemaphore(1, 0);
}

extern "C" SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t uxMaxCount, UBaseType_t uxInitialCount) {
    return (SemaphoreHandle_t) new CountingSemaphore(uxMaxCount, uxInitialCount);
}

extern "C" BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xBlockTime) {
    auto *s = (CountingSemaphore*)xSemaphore;
    std::unique_lock<std::mutex> lock(s->mtx);

    if (xBlockTime == portMAX_DELAY) {
        s->cv.wait(lock, [&]{ return s->count > 0; });
    } else {
        if (!s->cv.wait_for(lock, std::chrono::milliseconds(xBlockTime), [&]{ return s->count > 0; })) {
            return pdFAIL;
        }
    }
    s->count--;
    return pdPASS;
}

extern "C" BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore) {
    auto *s = (CountingSemaphore*)xSemaphore;
    {
        std::lock_guard<std::mutex> lock(s->mtx);
        if (s->count < s->maxCount) s->count++;
    }
    s->cv.notify_one();
    return pdPASS;
}

/* CMSIS osSemaphoreNew maps to counting semaphore */
extern "C" osSemaphoreId_t osSemaphoreNew(uint32_t max_count, uint32_t initial_count,
                                            const osSemaphoreAttr_t *attr) {
    (void)attr;
    return (osSemaphoreId_t) new CountingSemaphore(max_count, initial_count);
}

/* ========================================================================
 * Message Queues
 * ======================================================================== */

struct MessageQueue {
    std::mutex mtx;
    std::condition_variable cvNotEmpty;
    std::deque<std::vector<uint8_t>> items;
    uint32_t maxItems;
    uint32_t itemSize;

    MessageQueue(uint32_t max, uint32_t size) : maxItems(max), itemSize(size) {}
};

extern "C" QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize) {
    return (QueueHandle_t) new MessageQueue(uxQueueLength, uxItemSize);
}

extern "C" osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size,
                                                  const osMessageQueueAttr_t *attr) {
    (void)attr;
    return (osMessageQueueId_t) new MessageQueue(msg_count, msg_size);
}

extern "C" BaseType_t xQueueSendToBack(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait) {
    (void)xTicksToWait;
    auto *q = (MessageQueue*)xQueue;
    std::lock_guard<std::mutex> lock(q->mtx);
    if (q->items.size() >= q->maxItems) return pdFAIL;
    std::vector<uint8_t> item(q->itemSize);
    memcpy(item.data(), pvItemToQueue, q->itemSize);
    q->items.push_back(std::move(item));
    q->cvNotEmpty.notify_one();
    return pdPASS;
}

extern "C" BaseType_t xQueueSendToFront(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait) {
    (void)xTicksToWait;
    auto *q = (MessageQueue*)xQueue;
    std::lock_guard<std::mutex> lock(q->mtx);
    if (q->items.size() >= q->maxItems) return pdFAIL;
    std::vector<uint8_t> item(q->itemSize);
    memcpy(item.data(), pvItemToQueue, q->itemSize);
    q->items.push_front(std::move(item));
    q->cvNotEmpty.notify_one();
    return pdPASS;
}

extern "C" BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait) {
    auto *q = (MessageQueue*)xQueue;
    std::unique_lock<std::mutex> lock(q->mtx);

    if (xTicksToWait == portMAX_DELAY) {
        q->cvNotEmpty.wait(lock, [&]{ return !q->items.empty(); });
    } else {
        if (!q->cvNotEmpty.wait_for(lock, std::chrono::milliseconds(xTicksToWait),
                                      [&]{ return !q->items.empty(); })) {
            return pdFAIL;
        }
    }

    auto &front = q->items.front();
    memcpy(pvBuffer, front.data(), q->itemSize);
    q->items.pop_front();
    return pdPASS;
}

extern "C" BaseType_t xQueueReset(QueueHandle_t xQueue) {
    auto *q = (MessageQueue*)xQueue;
    std::lock_guard<std::mutex> lock(q->mtx);
    q->items.clear();
    return pdPASS;
}

/* ========================================================================
 * Kernel
 * ======================================================================== */

extern "C" osStatus_t osKernelInitialize(void) { return osOK; }
extern "C" osStatus_t osKernelStart(void) { return osOK; }
