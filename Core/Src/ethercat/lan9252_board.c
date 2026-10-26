#include "lan9252_board.h"
#include "fmc.h"

typedef struct {
    uint32_t last_cycles, last_tick, cycles_per_us, fractional_cycles, microseconds;
    uint8_t initialized;
} BoardClock;
static BoardClock clock_state;

static uint32_t board_time_us(void *context)
{
    BoardClock *clock = (BoardClock *)context;
    uint32_t tick = HAL_GetTick();
    uint32_t cycles = DWT->CYCCNT;
    uint64_t delta = (uint32_t)(cycles - clock->last_cycles);
    uint64_t coarse = (uint64_t)(uint32_t)(tick - clock->last_tick) * clock->cycles_per_us * 1000U;
    /* Extend DWT across its 8.95s wrap using the millisecond tick. Sampling
     * gaps must be shorter than one HAL tick wrap (49.7 days). */
    if (coarse > delta)
        delta += ((coarse - delta + UINT64_C(0x80000000)) >> 32) << 32;
    clock->last_cycles = cycles;
    clock->last_tick = tick;
    delta += clock->fractional_cycles;
    clock->microseconds += (uint32_t)(delta / clock->cycles_per_us);
    clock->fractional_cycles = (uint32_t)(delta % clock->cycles_per_us);
    return clock->microseconds;
}

static LAN9252Status board_read16(void *context, uint16_t offset, uint16_t *value)
{
    (void)context;
    *value = *(volatile const uint16_t *)(uintptr_t)(LAN9252_HBI_BASE + offset);
    __DSB();
    return LAN9252_OK;
}

static LAN9252Status board_write16(void *context, uint16_t offset, uint16_t value)
{
    (void)context;
    *(volatile uint16_t *)(uintptr_t)(LAN9252_HBI_BASE + offset) = value;
    __DSB();
    return LAN9252_OK;
}

static void board_finish_dword(void *context)
{
    BoardClock *clock = (BoardClock *)context;
    uint32_t start;
    __DSB(); /* Last write has completed before measuring the recovery time. */
    start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < clock->cycles_per_us) { __NOP(); }
    __DSB();
}

const LAN9252Transport *LAN9252_BoardTransport(void)
{
    static const LAN9252Transport transport = {
        board_read16, board_write16, board_finish_dword, board_time_us, &clock_state
    };
    if (!clock_state.initialized) {
        uint32_t start;
        unsigned i;
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->LAR = UINT32_C(0xc5acce55); /* Cortex-M7 software unlock, also without SWD. */
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        __DSB(); __ISB();
        /* A finite instruction loop checks the timebase BEFORE MMIO or delay
         * polling can use it. A stopped/locked counter returns a clock error. */
        start = DWT->CYCCNT;
        for (i = 0; i < 32; ++i) { __NOP(); }
        if (DWT->CYCCNT == start || SystemCoreClock < 1000000U) return NULL;
        clock_state.cycles_per_us = SystemCoreClock / 1000000U;
        clock_state.last_cycles = DWT->CYCCNT;
        clock_state.last_tick = HAL_GetTick();
        clock_state.initialized = 1;
    }
    return &transport;
}
