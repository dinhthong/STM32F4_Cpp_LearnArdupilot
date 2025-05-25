#include "AP_Scheduler.h"

#define MAX_TASKS 10
#define MAX_FAST_TASK_PRIORITIES 5
#define LOOP_PERIOD_US 10000 // 10 ms loop


// Dummy micros() implementation (to be replaced with a real one based on DWT or TIM)
uint32_t micros() {
    // This should return system uptime in microseconds
    // Implement using DWT->CYCCNT or TIMx->CNT in your STM32 project
    return (TIM5->CNT);
}

// constructor
AP_Scheduler::AP_Scheduler()
{
    if (_singleton) {
        return;
    }
    _singleton = this;

    //AP_Param::setup_object_defaults(this, var_info);
}


/*
 * Get the AP_Scheduler singleton
 */
AP_Scheduler *AP_Scheduler::_singleton;
AP_Scheduler *AP_Scheduler::get_singleton()
{
    return _singleton;
}

void AP_Scheduler::init(Task *tasks, uint8_t num_tasks, uint8_t fast_task_count) {
        _tasks = tasks;
        _num_tasks = num_tasks;
        _fast_task_count = fast_task_count;

        memset(_last_run, 0, sizeof(_last_run));
        memset(_last_run_usec, 0, sizeof(_last_run_usec));
        _last_loop_us = micros();
}

void AP_Scheduler::loop() {
    uint32_t now = micros();
    uint32_t dt = now - _last_loop_us;
    if (dt < LOOP_PERIOD_US) {
        return; // Maintain loop timing
    }
    _last_loop_us = now;

    uint16_t time_available = LOOP_PERIOD_US;
    for (uint8_t i = 0; i < _num_tasks; i++) {
        uint16_t rate_div = 1000000UL / _tasks[i].rate_hz;
        if ((now - _last_run_usec[i]) >= rate_div) {
            uint32_t tstart = micros();
            _tasks[i].function();
            uint32_t telapsed = micros() - tstart;
            _last_run_usec[i] = now;

            if (telapsed < time_available) {
                time_available -= telapsed;
            } else {
                break; // Not enough time for more tasks
            }
        }
    }
}
