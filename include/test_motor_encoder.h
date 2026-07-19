#ifndef TEST_MOTOR_ENCODER_H_
#define TEST_MOTOR_ENCODER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TEST_TELEM_INTERVAL_MS   50
#define TEST_TELEM_CHANNELS      6
#define TEST_PHASE_DURATION_MS   3000
#define TEST_STOP_DURATION_MS    2000

typedef enum {
    TEST_PHASE_STOP = 0,
    TEST_PHASE_FWD_30,
    TEST_PHASE_FWD_50,
    TEST_PHASE_FWD_70,
    TEST_PHASE_PIVOT_LEFT,
    TEST_PHASE_PIVOT_RIGHT,
    TEST_PHASE_COUNT
} TestPhase_t;

void Test_MotorEncoder_Init(void);
void Test_MotorEncoder_Run(void);

#ifdef __cplusplus
}
#endif

#endif
