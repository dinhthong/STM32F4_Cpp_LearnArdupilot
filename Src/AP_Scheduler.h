// Minimal AP_Scheduler-like task scheduler for STM32F4
// No dynamic memory, no HAL dependencies, uses static task array

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "stm32f4xx_hal.h"
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  main loop scheduler for APM
 *  Author: Andrew Tridgell, January 2013
 *
 */
#pragma once

#define LOOP_RATE 0
#define MAX_TASKS 10
#define MAX_FAST_TASK_PRIORITIES 5
#define LOOP_PERIOD_US 10000 // 10 ms loop
/*
  useful macro for creating scheduler task table
 */
#define SCHED_TASK_CLASS(classname, classptr, func, _rate_hz, _max_time_micros, _priority) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    AP_SCHEDULER_NAME_INITIALIZER(classname, func)\
    .rate_hz = _rate_hz,\
    .max_time_micros = _max_time_micros,        \
    .priority = _priority \
}

/*
  useful macro for creating the fastloop task table
 */
#define FAST_TASK_CLASS(classname, classptr, func) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    AP_FAST_NAME_INITIALIZER(classname, func)\
    .rate_hz = 0,\
    .max_time_micros = 0,\
    .priority = AP_Scheduler::FAST_TASK_PRI0 \
}

/*
  A task scheduler for APM main loops

  Sketches should call scheduler.init() on startup, then call
  scheduler.tick() at regular intervals (typically every 10ms).

  To run tasks use scheduler.run(), passing the amount of time that
  the scheduler is allowed to use before it must return
 */
struct Task {
		uint8_t priority;
		uint16_t rate_hz;
		uint16_t max_time_micros;
		const char *name;
		void (*function)(void);
};
	
class AP_Scheduler
{
public:
    AP_Scheduler();

    /* Do not allow copies */
    //CLASS_NO_COPY(AP_Scheduler);

    static AP_Scheduler *get_singleton();
    static AP_Scheduler *_singleton;

    //FUNCTOR_TYPEDEF(task_fn_t, void);



//    enum class Options : uint8_t {
//        RECORD_TASK_INFO = 1 << 0
//    };

//    enum FastTaskPriorities {
//        FAST_TASK_PRI0 = 0,
//        FAST_TASK_PRI1 = 1,
//        FAST_TASK_PRI2 = 2,
//        MAX_FAST_TASK_PRIORITIES = 3
//    };

    // initialise scheduler
    void init(Task *tasks, uint8_t num_tasks, uint8_t fast_task_count);
	
    // called by vehicle's main loop - which should be the only thing
    // that function does
    void loop();

private:
    Task *_tasks = nullptr;
    uint8_t _num_tasks = 0;
    uint8_t _fast_task_count = 0;
    uint16_t _last_run[MAX_TASKS] = {0};
    uint32_t _last_run_usec[MAX_TASKS] = {0};
    uint32_t _last_loop_us = 0;
};

