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

#include "AP_Scheduler.h"

// constructor
AP_Scheduler::AP_Scheduler()
{
}

// initialise the scheduler
void AP_Scheduler::init(const AP_Scheduler::Task *tasks, uint8_t num_tasks, uint32_t log_performance_bit)
{
    // grab the semaphore before we start anything
    //_rsem.take_blocking();

    // only allow 50 to 2000 Hz
    if (_loop_rate_hz < 50) {
        _loop_rate_hz = 50 ;
    } else if (_loop_rate_hz > 2000) {
        _loop_rate_hz  = 2000;
    }
    _last_loop_time_s = 1.0 / _loop_rate_hz;

    _vehicle_tasks = tasks;
    _num_vehicle_tasks = num_tasks;

#if AP_VEHICLE_ENABLED
    AP_Vehicle* vehicle = AP::vehicle();
    if (vehicle != nullptr) {
        vehicle->get_common_scheduler_tasks(_common_tasks, _num_common_tasks);
    }
#endif

    _num_tasks = _num_vehicle_tasks + _num_common_tasks;

   _last_run = NEW_NOTHROW uint16_t[_num_tasks];
    _tick_counter = 0;

    _log_performance_bit = log_performance_bit;

    // sanity check the task lists to ensure the priorities are
    // never decrease
    uint8_t old = 0;
    for (uint8_t i=0; i<_num_common_tasks; i++) {
        if (_common_tasks[i].priority < old){
            //INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }
        old = _common_tasks[i].priority;
    }
    old = 0;
    for (uint8_t i=0; i<_num_vehicle_tasks; i++) {
        if (_vehicle_tasks[i].priority < old) {
            //INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }
        old = _vehicle_tasks[i].priority;
    }
}

// one tick has passed
void AP_Scheduler::tick(void)
{
    _tick_counter++;
    _tick_counter32++;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void AP_Scheduler::run(uint32_t time_available)
{
    uint32_t run_started_usec = AP_HAL_micro();
    uint32_t now = run_started_usec;

    uint8_t vehicle_tasks_offset = 0;
    uint8_t common_tasks_offset = 0;

    for (uint8_t i=0; i<_num_tasks; i++) {
        // determine which of the common task / vehicle task to run
        bool run_vehicle_task = false;
        if (vehicle_tasks_offset < _num_vehicle_tasks &&
            common_tasks_offset < _num_common_tasks) {
            // still have entries on both lists; compare the
            // priorities.  In case of a tie the vehicle-specific
            // entry wins.
            const Task &vehicle_task = _vehicle_tasks[vehicle_tasks_offset];
            const Task &common_task = _common_tasks[common_tasks_offset];
            if (vehicle_task.priority <= common_task.priority) {
                run_vehicle_task = true;
            }
        } else if (vehicle_tasks_offset < _num_vehicle_tasks) {
            // out of common tasks to run
            run_vehicle_task = true;
        } else if (common_tasks_offset < _num_common_tasks) {
            // out of vehicle tasks to run
            run_vehicle_task = false;
        } else {
            // this is an error; the outside loop should have terminated
            //INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }

        const AP_Scheduler::Task &task = run_vehicle_task ? _vehicle_tasks[vehicle_tasks_offset] : _common_tasks[common_tasks_offset];
        if (run_vehicle_task) {
            vehicle_tasks_offset++;
        } else {
            common_tasks_offset++;
        }

        if (task.priority > MAX_FAST_TASK_PRIORITIES) {
            const uint16_t dt = _tick_counter - _last_run[i];
            // we allow 0 to mean loop rate
            uint32_t interval_ticks = (is_zero(task.rate_hz) ? 1 : _loop_rate_hz / task.rate_hz);
            if (interval_ticks < 1) {
                interval_ticks = 1;
            }
            if (dt < interval_ticks) {
                // this task is not yet scheduled to run again
                continue;
            }
            // this task is due to run. Do we have enough time to run it?
            _task_time_allowed = task.max_time_micros;

            if (dt >= interval_ticks*2) {
                //perf_info.task_slipped(i);
            }

            if (dt >= interval_ticks*max_task_slowdown) {
                // we are going beyond the maximum slowdown factor for a
                // task. This will trigger increasing the time budget
                task_not_achieved++;
            }

            if (_task_time_allowed > time_available) {
                // not enough time to run this task.  Continue loop -
                // maybe another task will fit into time remaining
                continue;
            }
        } else {
            _task_time_allowed = get_loop_period_us();
        }

        // run it
        _task_time_started = now;
        hal.util->persistent_data.scheduler_task = i;

        task.function();
        hal.util->persistent_data.scheduler_task = -1;

        // record the tick counter when we ran. This drives
        // when we next run the event
        _last_run[i] = _tick_counter;

        // work out how long the event actually took
        now = AP_HAL_micro();
        uint32_t time_taken = now - _task_time_started;
        bool overrun = false;
        if (time_taken > _task_time_allowed) {
            overrun = true;
            // the event overran!
            // debug(3, "Scheduler overrun task[%u-%s] (%u/%u)\n",
            //       (unsigned)i,
            //       task.name,
            //       (unsigned)time_taken,
            //       (unsigned)_task_time_allowed);
        }

        //perf_info.update_task_info(i, time_taken, overrun);

        if (time_taken >= time_available) {
            /*
              we are out of time, but we need to keep walking the task
              table in case there is another fast loop task after this
              task, plus we need to update the accouting so we can
              work out if we need to allocate extra time for the loop
              (lower the loop rate)
              Just set time_available to zero, which means we will
              only run fast tasks after this one
             */
            time_available = 0;
        } else {
            time_available -= time_taken;
        }
    }

    // update number of spare microseconds
    _spare_micros += time_available;

    _spare_ticks++;
    if (_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }
}

/*
  return number of micros until the current task reaches its deadline
 */
uint16_t AP_Scheduler::time_available_usec(void) const
{
    uint32_t dt = AP_HAL_micro() - _task_time_started;
    if (dt > _task_time_allowed) {
        return 0;
    }
    return _task_time_allowed - dt;
}

/*
  calculate load average as a number from 0 to 1
 */
float AP_Scheduler::load_average()
{
    // return 1 if filtered main loop rate is 5% below the configured rate
    if (get_filtered_loop_rate_hz() < get_loop_rate_hz() * 0.95) {
        return 1.0;
    }
    if (_spare_ticks == 0) {
        return 0.0f;
    }
    const uint32_t loop_us = get_loop_period_us();
    const uint32_t used_time = loop_us - (_spare_micros/_spare_ticks);
    return constrain_float(used_time / (float)loop_us, 0, 1);
}

void AP_Scheduler::loop()
{
    // wait for an INS sample
    //hal.util->persistent_data.scheduler_task = -3;
    //_rsem.give();
    //AP::ins().wait_for_sample();
    //_rsem.take_blocking();
    //hal.util->persistent_data.scheduler_task = -1;

    _loop_sample_time_us = AP_HAL_micro64();
    const uint32_t sample_time_us = uint32_t(_loop_sample_time_us);
    
    if (_loop_timer_start_us == 0) {
        _loop_timer_start_us = sample_time_us;
        _last_loop_time_s = get_loop_period_s();
    } else {
        _last_loop_time_s = (sample_time_us - _loop_timer_start_us) * 1.0e-6;
    }

    // tell the scheduler one tick has passed
    tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    const uint32_t loop_us = get_loop_period_us();
    uint32_t now = AP_HAL_micro();
    uint32_t time_available = 0;
    const uint32_t loop_tick_us = now - sample_time_us;
    if (loop_tick_us < loop_us) {
        // get remaining time available for this loop
        time_available = loop_us - loop_tick_us;
    }

    // add in extra loop time determined by not achieving scheduler tasks
    time_available += extra_loop_us;

    // run the tasks
    run(time_available);

    if (task_not_achieved > 0) {
        // add some extra time to the budget
        extra_loop_us = MIN(extra_loop_us+100U, 5000U);
        task_not_achieved = 0;
        task_all_achieved = 0;
    } else if (extra_loop_us > 0) {
        task_all_achieved++;
        if (task_all_achieved > 50) {
            // we have gone through 50 loops without a task taking too
            // long. CPU pressure has eased, so drop the extra time we're
            // giving each loop
            task_all_achieved = 0;
            // we are achieving all tasks, slowly lower the extra loop time
            extra_loop_us = MAX(0U, extra_loop_us-50U);
        }
    }

    _loop_timer_start_us = sample_time_us;
}

// display task statistics as text buffer for @SYS/tasks.txt
// void AP_Scheduler::task_info(ExpandingString &str)
// {
//     // a header to allow for machine parsers to determine format

//     // dynamically enable statistics collection
//     // if (!(_options & uint8_t(Options::RECORD_TASK_INFO))) {
//     //     _options.set(_options | uint8_t(Options::RECORD_TASK_INFO));
//     //     return;
//     // }

//     // baseline the total time taken by all tasks
//     float total_time = 1.0f;
//     for (uint8_t i = 0; i < _num_tasks + 1; i++) {
//         // const AP::PerfInfo::TaskInfo* ti = perf_info.get_task_info(i);
//         // if (ti != nullptr && ti->tick_count > 0) {
//         //     total_time += ti->elapsed_time_us;
//         // }
//     }

//     uint8_t vehicle_tasks_offset = 0;
//     uint8_t common_tasks_offset = 0;

//     for (uint8_t i = 0; i < _num_tasks; i++) {
//         const char *task_name;

//         // determine which of the common task / vehicle task to run
//         bool run_vehicle_task = false;
//         if (vehicle_tasks_offset < _num_vehicle_tasks &&
//             common_tasks_offset < _num_common_tasks) {
//             // still have entries on both lists; compare the
//             // priorities.  In case of a tie the vehicle-specific
//             // entry wins.
//             const Task &vehicle_task = _vehicle_tasks[vehicle_tasks_offset];
//             const Task &common_task = _common_tasks[common_tasks_offset];
//             if (vehicle_task.priority <= common_task.priority) {
//                 run_vehicle_task = true;
//             }
//         } else if (vehicle_tasks_offset < _num_vehicle_tasks) {
//             // out of common tasks to run
//             run_vehicle_task = true;
//         } else if (common_tasks_offset < _num_common_tasks) {
//             // out of vehicle tasks to run
//             run_vehicle_task = false;
//         } else {
//             // this is an error; the outside loop should have terminated
//             //INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
//             return;
//         }

//         if (run_vehicle_task) {
//             task_name = _vehicle_tasks[vehicle_tasks_offset++].name;
//         } else {
//             task_name = _common_tasks[common_tasks_offset++].name;
//         }

//     }
// }