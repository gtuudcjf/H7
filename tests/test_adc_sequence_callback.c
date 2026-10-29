#include <assert.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
    void *Instance;
    uint32_t flags;
    uint32_t jdr1;
    uint32_t jdr2;
} ADC_HandleTypeDef;
#define ADC1 ((void *)1)
#define ADC_INJECTED_RANK_1 1U
#define ADC_INJECTED_RANK_2 2U
#define ADC_FLAG_JEOC 1U
#define ADC_FLAG_JEOS 2U
#define RESET 0U
#define MOTOR_FAST_TICK_S 0.00005f
#define __HAL_ADC_GET_FLAG(handle, flag) ((handle)->flags & (flag))

static uint32_t reads, control_calls, last_a, last_b;
uint32_t HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef *adc, uint32_t rank)
{
    ++reads;
    return rank == ADC_INJECTED_RANK_1 ? adc->jdr1 : adc->jdr2;
}
void MotorControl_CurrentSampleComplete(uint32_t a, uint32_t b, float dt)
{
    ++control_calls;
    last_a = a;
    last_b = b;
    assert(dt == MOTOR_FAST_TICK_S);
}

/* The runner copies the production callback verbatim from main.c. */
#include "adc_callback_under_test.h"

int main(void)
{
    ADC_HandleTypeDef adc = {ADC1, ADC_FLAG_JEOC, 111U, 222U};
    ADC_HandleTypeDef other = {(void *)2, ADC_FLAG_JEOS, 11U, 22U};
    /* Rank 1 is new, but rank 2 still belongs to the preceding PWM period. */
    HAL_ADCEx_InjectedConvCpltCallback(&adc);
    assert(reads == 0U && control_calls == 0U);
    HAL_ADCEx_InjectedConvCpltCallback(&other);
    assert(reads == 0U && control_calls == 0U);
    /* Only the sequence-complete callback may read the new coherent pair. */
    adc.jdr2 = 333U;
    adc.flags = ADC_FLAG_JEOC | ADC_FLAG_JEOS;
    HAL_ADCEx_InjectedConvCpltCallback(&adc);
    assert(reads == 2U && control_calls == 1U);
    assert(last_a == 111U && last_b == 333U);
    /* HAL clears the flags after the callback; a spurious call does nothing. */
    adc.flags = 0U;
    HAL_ADCEx_InjectedConvCpltCallback(&adc);
    assert(control_calls == 1U);
    adc.jdr1 = 444U;
    adc.flags = ADC_FLAG_JEOC;
    HAL_ADCEx_InjectedConvCpltCallback(&adc);
    assert(control_calls == 1U);
    adc.jdr2 = 555U;
    adc.flags = ADC_FLAG_JEOS;
    HAL_ADCEx_InjectedConvCpltCallback(&adc);
    assert(control_calls == 2U && reads == 4U);
    assert(last_a == 444U && last_b == 555U);
    puts("ADC sequence-only control callback tests passed");
    return 0;
}
