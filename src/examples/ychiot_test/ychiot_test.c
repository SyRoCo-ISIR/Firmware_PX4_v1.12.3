/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/uwb_msg.h>

__EXPORT int ychiot_test_main(int argc, char *argv[]);

int ychiot_test_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");

    /* subscribe to sensor_combined topic */
    int uwb_sub = orb_subscribe(ORB_ID(uwb_msg));
    /* limit the update rate to 100 Hz */
    orb_set_interval(uwb_sub, 10);
        
    hrt_abstime now = hrt_absolute_time();
    int id=0;
    while (hrt_absolute_time()-now<1000000) {
	bool updated;
	orb_check(uwb_sub, &updated);
	if(updated){
		id++;
		// obtained data for the first file descriptor
		struct uwb_msg_s raw;
		// copy sensors raw data into local buffer
		orb_copy(ORB_ID(uwb_msg), uwb_sub, &raw);
                PX4_INFO("Time:%2llu buffer: %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
                        (uint64_t)raw.timestamp,
                        raw.buffer[0],raw.buffer[1],raw.buffer[2],raw.buffer[3],raw.buffer[4],raw.buffer[5],raw.buffer[6],raw.buffer[7],raw.buffer[8],raw.buffer[9],raw.buffer[10],
                        raw.buffer[11],raw.buffer[12],raw.buffer[13],raw.buffer[14],raw.buffer[15],raw.buffer[16],raw.buffer[17],raw.buffer[18],raw.buffer[19],raw.buffer[20],
                        raw.buffer[21],raw.buffer[22],raw.buffer[23],raw.buffer[24],raw.buffer[25],raw.buffer[26],raw.buffer[27],raw.buffer[28],raw.buffer[29],raw.buffer[30],
                        raw.buffer[31],raw.buffer[32],raw.buffer[33]);
                PX4_INFO("T_0:%8.4f T_1:%8.4f T_2:%8.4f T_3:%8.4f T_4:%8.4f T_5:%8.4f T_6:%8.4f T_7:%8.4f",
                        (double)raw.distance[0],
                        (double)raw.distance[1],
                        (double)raw.distance[2],
                        (double)raw.distance[3],
                        (double)raw.distance[4],
                        (double)raw.distance[5],
                        (double)raw.distance[6],
                        (double)raw.distance[7]);
	}
    }
    PX4_INFO("with PX4_INFO, total updates %d times in 1 second",id);
    id=0;
    now = hrt_absolute_time();
    while (hrt_absolute_time()-now<1000000) {
	bool updated;
	orb_check(uwb_sub, &updated);
	if(updated){
		// obtained data for the first file descriptor
		struct uwb_msg_s raw;
		// copy sensors raw data into local buffer
		orb_copy(ORB_ID(uwb_msg), uwb_sub, &raw);
		id++;
	}
    }
    PX4_INFO("without PX4_INFO, total updates %d times in 1 second",id);
    
    return 0;
}
