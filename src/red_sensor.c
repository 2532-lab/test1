/*
 * red_sensor.c
 *
 * 对射式红外传感器计数模块。
 * 硬件: MSPM0G3507, PA31 上升沿 GPIO 中断。
 */

#include "red_sensor.h"
#include "ti_msp_dl_config.h"

#define RED_SENSOR_TIMEBASE_HZ       (1000U)

static volatile uint32_t s_pass_count;
static volatile uint32_t s_system_ms;
static volatile uint32_t s_last_trigger_ms;
static volatile uint32_t s_interval_ms;
static volatile bool s_have_trigger;

static volatile red_sensor_event_t s_event_queue[RED_SENSOR_EVENT_QUEUE_SIZE];
static volatile uint32_t s_event_head;
static volatile uint32_t s_event_tail;
static volatile uint32_t s_event_overflow_count;

/* SysTick 是整个模块共用的 1 ms 时间基准。 */
void SysTick_Handler(void)
{
    s_system_ms++;
}

static void red_sensor_enqueue_event(uint32_t count, uint32_t interval_ms)
{
    uint32_t next_head = s_event_head + 1U;

    if (next_head >= RED_SENSOR_EVENT_QUEUE_SIZE) {
        next_head = 0U;
    }

    if (next_head == s_event_tail) {
        /* 保留计数结果，同时记录缓存溢出，便于调试串口吞吐不足。 */
        s_event_overflow_count++;
        return;
    }

    s_event_queue[s_event_head].count = count;
    s_event_queue[s_event_head].interval_ms = interval_ms;
    s_event_head = next_head;
}

/* MSPM0G3507 的 GPIOA 中断由 GROUP1 向量入口分发。 */
void GROUP1_IRQHandler(void)
{
    uint32_t interrupt_status;

    interrupt_status = DL_GPIO_getEnabledInterruptStatus(
        Outred_PORT, Outred_PIN_red_PIN);

    if ((interrupt_status & Outred_PIN_red_PIN) != 0U) {
        uint32_t now_ms = s_system_ms;
        uint32_t interval_ms = 0U;
        bool accept_trigger = false;

        DL_GPIO_clearInterruptStatus(Outred_PORT, Outred_PIN_red_PIN);

        if (!s_have_trigger) {
            accept_trigger = true;
        } else {
            interval_ms = now_ms - s_last_trigger_ms;
            if (interval_ms >= RED_SENSOR_DEBOUNCE_MS) {
                accept_trigger = true;
            }
        }

        if (accept_trigger) {
            s_have_trigger = true;
            s_last_trigger_ms = now_ms;
            s_interval_ms = interval_ms;

            if (s_pass_count != UINT32_MAX) {
                s_pass_count++;
                red_sensor_enqueue_event(s_pass_count, interval_ms);
            }
        }
    }
}

void red_sensor_init(void)
{
    s_pass_count = 0U;
    s_system_ms = 0U;
    s_last_trigger_ms = 0U;
    s_interval_ms = 0U;
    s_have_trigger = false;
    s_event_head = 0U;
    s_event_tail = 0U;
    s_event_overflow_count = 0U;

    /* SysConfig 已完成 PA31 的输入、电阻、边沿和 GPIO 标志配置。 */
    DL_GPIO_clearInterruptStatus(Outred_PORT, Outred_PIN_red_PIN);

    /* 1 ms tick at the SysConfig-generated 32 MHz CPU clock. */
    (void)SysTick_Config(CPUCLK_FREQ / RED_SENSOR_TIMEBASE_HZ);

    NVIC_ClearPendingIRQ(Outred_INT_IRQN);
    NVIC_EnableIRQ(Outred_INT_IRQN);
}

void red_sensor_reset_count(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_pass_count = 0U;
    s_last_trigger_ms = s_system_ms;
    s_interval_ms = 0U;
    s_have_trigger = false;
    s_event_head = 0U;
    s_event_tail = 0U;
    s_event_overflow_count = 0U;

    if (primask == 0U) {
        __enable_irq();
    }
}

uint32_t red_sensor_get_count(void)
{
    return s_pass_count;
}

uint32_t red_sensor_get_interval_ms(void)
{
    return s_interval_ms;
}

bool red_sensor_pop_event(red_sensor_event_t *event)
{
    uint32_t primask;
    bool has_event = false;

    if (event == (red_sensor_event_t *)0) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (s_event_tail != s_event_head) {
        event->count = s_event_queue[s_event_tail].count;
        event->interval_ms = s_event_queue[s_event_tail].interval_ms;

        s_event_tail++;
        if (s_event_tail >= RED_SENSOR_EVENT_QUEUE_SIZE) {
            s_event_tail = 0U;
        }
        has_event = true;
    }

    if (primask == 0U) {
        __enable_irq();
    }

    return has_event;
}

uint32_t red_sensor_get_event_overflow_count(void)
{
    return s_event_overflow_count;
}
