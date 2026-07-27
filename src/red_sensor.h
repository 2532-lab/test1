/*
 * red_sensor.h
 *
 * 对射式红外传感器计数和事件缓存接口。
 *
 * 当前 SysConfig 配置为 PA31 上升沿中断，因此本模块每检测到一次
 * 有效上升沿就认为有一个小球通过，并将事件放入环形缓存。
 */

#ifndef RED_SENSOR_H
#define RED_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/* 可按实际小球的最小合法间隔调整，消除传感器重复边沿。 */
#define RED_SENSOR_DEBOUNCE_MS       (50U)

/* 缓存突发触发事件，环形队列最多同时保存 63 条未发送记录。 */
#define RED_SENSOR_EVENT_QUEUE_SIZE  (64U)

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化毫秒时间基准、清空计数并使能传感器 GPIO 中断。 */
void red_sensor_init(void);

/* 返回已经检测到的小球数量。 */
uint32_t red_sensor_get_count(void);

/* 清零计数、消抖状态和事件缓存。 */
void red_sensor_reset_count(void);

/* 返回最近两次有效触发之间的时间，单位 ms；不足两次触发时返回 0。 */
uint32_t red_sensor_get_interval_ms(void);

typedef struct {
    uint32_t count;
    uint32_t interval_ms;
} red_sensor_event_t;

/* 取出一条按触发顺序保存的事件；无事件时返回 false。 */
bool red_sensor_pop_event(red_sensor_event_t *event);

/* 返回因缓存已满而丢弃的事件数量。 */
uint32_t red_sensor_get_event_overflow_count(void);

#ifdef __cplusplus
}
#endif

#endif /* RED_SENSOR_H */
