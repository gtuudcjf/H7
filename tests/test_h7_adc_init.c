/* Exercise MX_ADC1_Init itself; only the peripheral HAL boundary is replaced.
 * Regression: SINGLE_CONV enables JEOC before both injected ranks are ready.
 * This checks the HAL configuration contract, not physical ADC timing.
 */
#include <assert.h>
#include <stdio.h>

enum {
    HAL_OK, DISABLE, ENABLE, ADC1, ADC_CLOCK_ASYNC_DIV1, ADC_CLOCK_ASYNC_DIV2,
    ADC_RESOLUTION_12B,
    ADC_SCAN_ENABLE, ADC_EOC_SINGLE_CONV, ADC_EOC_SEQ_CONV,
    ADC_SOFTWARE_START, ADC_EXTERNALTRIGCONVEDGE_NONE, ADC_CONVERSIONDATA_DR,
    ADC_OVR_DATA_PRESERVED, ADC_LEFTBITSHIFT_NONE, ADC_MODE_INDEPENDENT,
    ADC_CHANNEL_16, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_32CYCLES_5,
    ADC_SINGLE_ENDED, ADC_OFFSET_NONE, ADC_CHANNEL_5, ADC_CHANNEL_9,
    ADC_INJECTED_RANK_1, ADC_INJECTED_RANK_2,
    ADC_EXTERNALTRIGINJEC_T8_TRGO2, ADC_EXTERNALTRIGINJECCONV_EDGE_RISING
};
typedef struct {
    int ClockPrescaler, Resolution, ScanConvMode, EOCSelection;
    int LowPowerAutoWait, ContinuousConvMode, NbrOfConversion;
    int DiscontinuousConvMode, ExternalTrigConv, ExternalTrigConvEdge;
    int ConversionDataManagement, Overrun, LeftBitShift, OversamplingMode;
    struct { int Ratio; } Oversampling;
} ADC_InitTypeDef;
typedef struct { int Instance; ADC_InitTypeDef Init; } ADC_HandleTypeDef;
typedef struct { int Mode; } ADC_MultiModeTypeDef;
typedef struct {
    int Channel, Rank, SamplingTime, SingleDiff, OffsetNumber, Offset;
    int OffsetSignedSaturation;
} ADC_ChannelConfTypeDef;
typedef struct {
    int InjectedChannel, InjectedRank, InjectedSamplingTime, InjectedSingleDiff;
    int InjectedOffsetNumber, InjectedOffset, InjectedNbrOfConversion;
    int InjectedDiscontinuousConvMode, AutoInjectedConv, QueueInjectedContext;
    int ExternalTrigInjecConv, ExternalTrigInjecConvEdge, InjecOversamplingMode;
} ADC_InjectionConfTypeDef;

static ADC_HandleTypeDef hadc1;
static ADC_InitTypeDef configured_init;
static ADC_InjectionConfTypeDef configured_rank[2];
static unsigned configured_rank_count;

static int HAL_ADC_Init(ADC_HandleTypeDef *adc)
{
    configured_init = adc->Init;
    return HAL_OK;
}
static int HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *adc,
                                           ADC_MultiModeTypeDef *config)
{
    (void)adc; (void)config;
    return HAL_OK;
}
static void HAL_ADCEx_DisableInjectedQueue(ADC_HandleTypeDef *adc) { (void)adc; }
static int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *adc, ADC_ChannelConfTypeDef *config)
{
    (void)adc; (void)config;
    return HAL_OK;
}
static int HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef *adc,
                                          ADC_InjectionConfTypeDef *config)
{
    (void)adc;
    assert(configured_rank_count < 2);
    configured_rank[configured_rank_count++] = *config;
    return HAL_OK;
}
static void Error_Handler(void) { assert(!"ADC initialization failed"); }

#include "adc_init_under_test.inc"

int main(void)
{
    MX_ADC1_Init();
    assert(configured_rank_count == 2);
#if ETHERCAT_MOTOR_OPEN_ANGLE && ETHERCAT_MOTOR_POWER_ENABLED
    /* The clock comparison must reach the HAL only in the powered bench. */
    assert(configured_init.ClockPrescaler == ADC_CLOCK_ASYNC_DIV2);
#else
    /* Do not change the working local mode or either locked DryRun profile. */
    assert(configured_init.ClockPrescaler == ADC_CLOCK_ASYNC_DIV1);
#endif
    assert(configured_init.Resolution == ADC_RESOLUTION_12B);
    /* Both JDRs are consumed together: HAL must enable sequence completion. */
    assert(configured_init.EOCSelection == ADC_EOC_SEQ_CONV);
    assert(configured_rank[0].InjectedRank == ADC_INJECTED_RANK_1);
    assert(configured_rank[0].InjectedChannel == ADC_CHANNEL_5);
    assert(configured_rank[1].InjectedRank == ADC_INJECTED_RANK_2);
    assert(configured_rank[1].InjectedChannel == ADC_CHANNEL_9);
    for (unsigned i = 0; i < 2; ++i) {
        assert(configured_rank[i].InjectedSamplingTime == ADC_SAMPLETIME_32CYCLES_5);
        assert(configured_rank[i].InjectedNbrOfConversion == 2);
        assert(configured_rank[i].InjectedDiscontinuousConvMode == DISABLE);
        assert(configured_rank[i].AutoInjectedConv == DISABLE);
        assert(configured_rank[i].ExternalTrigInjecConv == ADC_EXTERNALTRIGINJEC_T8_TRGO2);
        assert(configured_rank[i].ExternalTrigInjecConvEdge == ADC_EXTERNALTRIGINJECCONV_EDGE_RISING);
    }
    puts("Production ADC1 init: profile-isolated clock and unchanged A/B trigger contract passed");
    return 0;
}
