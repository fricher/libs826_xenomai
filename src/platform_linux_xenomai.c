// Copyright Sensoray Company Inc. 2011
// platform specific code
// critical section and sleep threads
//   ini file reading
#include <string.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <linux/types.h>
#include <stdint.h>
#include "assert.h"
#include "platform.h"
#include <alchemy/task.h>
#include <alchemy/timer.h>

/** sleep for x milliseconds
 *  @param time time in milliseconds to sleep the thread
 *  @return none
 */
void thread_sleep_msec( int time)
{
	rt_task_sleep(rt_timer_ns2ticks(time*1000000));
	return;
}

/** retrive current time
 *  @ret
*/
uint64_t current_time()
{
    RTIME current_ticks = rt_timer_read();
    SRTIME current_ns = rt_timer_ticks2ns((SRTIME)current_ticks);
    return (uint64_t)(current_ns/1000);
}


/** returns elapsed_time from starttime in ms 
 *  @param starttime time to compare from
 *  @return time difference
 */
uint64_t elapsed_time( uint64_t starttime) 
{
	return current_time() - starttime;

}
