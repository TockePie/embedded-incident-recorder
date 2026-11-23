
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

static TIM_HandleTypeDef htim4;

/*
 * TIM4 is used as the HAL 1 ms time base.
 * SysTick remains reserved for FreeRTOS.
 */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    uint32_t pclk1;
    uint32_t timclk;
    uint32_t tick_hz;
    uint32_t prescaler;
    uint32_t period;

    if (TickPriority >= (1UL << __NVIC_PRIO_BITS))
    {
        return HAL_ERROR;
    }

    /*
     * If TIM4 was already initialized, stop and reinitialize it.
     * HAL_RCC_ClockConfig() calls HAL_InitTick() after clock changes.
     */
    if (htim4.Instance == TIM4)
    {
        (void)HAL_TIM_Base_Stop_IT(&htim4);

        if (HAL_TIM_Base_DeInit(&htim4) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    pclk1 = HAL_RCC_GetPCLK1Freq();

    /*
     * APB1 timer clock is doubled when APB1 prescaler is not DIV1.
     */
    timclk = pclk1;

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    {
        timclk *= 2U;
    }

    /*
     * Use a 10 kHz timer counter.
     * This allows all supported HAL tick frequencies to fit
     * into the 16-bit TIM4 auto-reload register.
     */
    if ((timclk < 10000U) || ((timclk % 10000U) != 0U))
    {
        return HAL_ERROR;
    }

    tick_hz = 1000U / (uint32_t)uwTickFreq;

    if ((tick_hz == 0U) || ((10000U % tick_hz) != 0U))
    {
        return HAL_ERROR;
    }

    prescaler = (timclk / 10000U) - 1U;
    period = (10000U / tick_hz) - 1U;

    if ((prescaler > 0xFFFFU) || (period > 0xFFFFU))
    {
        return HAL_ERROR;
    }

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = prescaler;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = period;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        return HAL_ERROR;
    }

    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

    HAL_NVIC_SetPriority(TIM4_IRQn, TickPriority, 0U);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
    {
        return HAL_ERROR;
    }

    uwTickPrio = TickPriority;

    return HAL_OK;
}

void HAL_SuspendTick(void)
{
    __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
}

/*
 * TIM4 interrupt handler.
 * It is deliberately independent of HAL_TIM_PeriodElapsedCallback()
 * to avoid conflicts with callbacks used by application peripherals.
 */
void TIM4_IRQHandler(void)
{
    if (((TIM4->SR & TIM_SR_UIF) != 0U) &&
        ((TIM4->DIER & TIM_DIER_UIE) != 0U))
    {
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
        HAL_IncTick();
    }
}
