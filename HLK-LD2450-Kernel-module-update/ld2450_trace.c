/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_trace.c - Tracepoints for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 22nd September 2025
 * Version: 2.4.0
 * Description: Implements tracepoints for profiling and debugging on Raspberry Pi
 * Tracepoints:
 *   - ld2450_data_received: Logs received tracking data
 *   - ld2450_error: Logs errors with error code
 *   - ld2450_race_detected: Logs race condition events
 * Usage:
 *   - Enable with: echo 1 > /sys/kernel/debug/tracing/events/ld2450/enable
 *   - View with: cat /sys/kernel/debug/tracing/trace
 * Changelog:
 *   - 2.4.0: Initial implementation of tracepoints
 */

#include <linux/tracepoint.h>
#include "LD2450.h"

DEFINE_TRACEPOINT(ld2450_data_received);
DEFINE_TRACEPOINT(ld2450_error);
DEFINE_TRACEPOINT(ld2450_race_detected);

TRACE_EVENT(ld2450_data_received,
    TP_PROTO(struct ld2450_data *data, s16 x_pos, s16 y_pos, s16 velocity, u16 distance),
    TP_ARGS(data, x_pos, y_pos, velocity, distance),
    TP_STRUCT__entry(
        __field(s16, x_pos)
        __field(s16, y_pos)
        __field(s16, velocity)
        __field(u16, distance)
    ),
    TP_fast_assign(
        __entry->x_pos = x_pos;
        __entry->y_pos = y_pos;
        __entry->velocity = velocity;
        __entry->distance = distance;
    ),
    TP_printk("x_pos=%d y_pos=%d velocity=%d distance=%u",
              __entry->x_pos, __entry->y_pos, __entry->velocity, __entry->distance)
);

TRACE_EVENT(ld2450_error,
    TP_PROTO(struct ld2450_data *data, int error_code),
    TP_ARGS(data, error_code),
    TP_STRUCT__entry(
        __field(int, error_code)
    ),
    TP_fast_assign(
        __entry->error_code = error_code;
    ),
    TP_printk("error_code=%d", __entry->error_code)
);

TRACE_EVENT(ld2450_race_detected,
    TP_PROTO(struct ld2450_data *data, const char *context),
    TP_ARGS(data, context),
    TP_STRUCT__entry(
        __string(context, context)
    ),
    TP_fast_assign(
        __assign_str(context, context);
    ),
    TP_printk("race_detected: %s", __get_str(context))
);

void ld2450_trace_data_received(struct ld2450_data *data)
{
    trace_ld2450_data_received(data, data->x_pos, data->y_pos, data->velocity, data->distance);
}

void ld2450_trace_error(struct ld2450_data *data, int error_code)
{
    trace_ld2450_error(data, error_code);
}

void ld2450_trace_race_detected(struct ld2450_data *data, const char *context)
{
    trace_ld2450_race_detected(data, context);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Tracepoints for Hi-Link LD2450 radar module on Raspberry Pi");
MODULE_VERSION("2.4.0");