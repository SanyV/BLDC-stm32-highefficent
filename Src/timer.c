/*
    This file is part of AutoQuad ESC32.

    AutoQuad ESC32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad ESC32 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad ESC32.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#include "timer.h"
#include "stm32f4xx_ll_tim.h"
#include "tim.h"

timerStruct_t timerData;
volatile uint32_t timerMicros;

// must be called at least once every 65536 ticks in order for this strategy to work
// TODO - consider interrupt interference
uint32_t timerGetMicros(void) {
    static uint16_t timerLast;
    static uint16_t hiBits;
    register uint16_t tmp;

    tmp = TIMER_TIM->CNT;

    if (tmp < timerLast)
	    hiBits++;

    timerLast = tmp;

    timerMicros = (hiBits<<16 | tmp) & TIMER_MASK;

    return timerMicros;
}

void timerDelay(uint32_t us) {
    uint32_t cnt = TIMER_TIM->CNT;
    uint32_t targetTimerVal = cnt + us*TIMER_MULT;

    if (targetTimerVal < cnt)
	// wait till timer rolls over
	while (TIMER_TIM->CNT > targetTimerVal)
	    ;

    while (TIMER_TIM->CNT < targetTimerVal)
	    ;
}

void timerCancelAlarm1(void) {
	LL_TIM_CC_DisableChannel(TIMER_TIM,LL_TIM_CHANNEL_CH1);
	LL_TIM_DisableIT_CC1(TIMER_TIM);
}

void timerCancelAlarm2(void) {
	LL_TIM_CC_DisableChannel(TIMER_TIM,LL_TIM_CHANNEL_CH2);
	LL_TIM_DisableIT_CC2(TIMER_TIM);
}

void timerCancelAlarm3(void) {
	LL_TIM_CC_DisableChannel(TIMER_TIM,LL_TIM_CHANNEL_CH3);
	LL_TIM_DisableIT_CC3(TIMER_TIM);
}

uint8_t timerAlarmActive3(void) {
    return LL_TIM_IsEnabledIT_CC3(TIMER_TIM);
}

// Create a timer
void timerInit(void) {
	LL_TIM_InitTypeDef TIM_InitStruct;
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;

	  /* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    // Enable the TIMER_TIM global Interrupt
	NVIC_SetPriority(TIM2_IRQn, 1);
	NVIC_EnableIRQ(TIM2_IRQn);

    // Time base configuration
    TIM_InitStruct.Prescaler = (120/TIMER_MULT) - 1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 0xFFFFFFFF;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM2, &TIM_InitStruct);

    LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);

    timerCancelAlarm1();
    timerCancelAlarm2();
    timerCancelAlarm3();

    // Output Compare for alarm
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_INACTIVE;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;

    LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_DisablePreload(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

    LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_DisablePreload(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);

    LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_DisablePreload(TIM2, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);

    LL_TIM_EnableARRPreload(TIM2);

    // go...
    /* Enable counter */
    LL_TIM_EnableCounter(TIM2);

    /* Force update generation */
 //   LL_TIM_GenerateEvent_UPDATE(TIM2);
}

// TODO: worry about 32 bit rollover
void timerSetAlarm1(int32_t ticks, timerCallback_t *callback, int parameter) {
    // do it now
    if (ticks <= TIMER_MULT) {
	// Disable the Interrupt
	LL_TIM_EnableIT_CC1(TIMER_TIM);
	callback(parameter);
    }
    // otherwise, schedule it
    else {
	timerData.alarm1Callback = callback;
	timerData.alarm1Parameter = parameter;

	LL_TIM_OC_SetCompareCH1(TIMER_TIM,TIMER_TIM->CNT + ticks);
	LL_TIM_ClearFlag_CC1(TIMER_TIM);
	LL_TIM_EnableIT_CC1(TIMER_TIM);
    }
}

void timerSetAlarm2(int32_t ticks, timerCallback_t *callback, int parameter) {
    // do it now
    if (ticks <= TIMER_MULT) {
	// Disable the Interrupt
    LL_TIM_EnableIT_CC2(TIMER_TIM);

	callback(parameter);
    }
    // otherwise, schedule it
    else {
	timerData.alarm2Callback = callback;
	timerData.alarm2Parameter = parameter;

	LL_TIM_OC_SetCompareCH2(TIMER_TIM,TIMER_TIM->CNT + ticks);
	LL_TIM_ClearFlag_CC2(TIMER_TIM);
	LL_TIM_EnableIT_CC2(TIMER_TIM);
    }
}

void timerSetAlarm3(int32_t ticks, timerCallback_t *callback, int parameter) {
    // do it now
    if (ticks <= TIMER_MULT) {
	// Disable the Interrupt
    LL_TIM_EnableIT_CC3(TIMER_TIM);

	callback(parameter);
    }
    // otherwise, schedule it
    else {
	timerData.alarm3Callback = callback;
	timerData.alarm3Parameter = parameter;

	LL_TIM_OC_SetCompareCH3(TIMER_TIM,TIMER_TIM->CNT + ticks);
	LL_TIM_ClearFlag_CC3(TIMER_TIM);
	LL_TIM_EnableIT_CC3(TIMER_TIM);
    }
}

void TIMER_ISR(void) {
    if (LL_TIM_IsActiveFlag_CC1(TIMER_TIM) == 1) {
    LL_TIM_ClearFlag_CC1(TIMER_TIM);;

	// Disable the Interrupt
	LL_TIM_EnableIT_CC1(TIMER_TIM);

	timerData.alarm1Callback(timerData.alarm1Parameter);
    }
    else if (LL_TIM_IsActiveFlag_CC2(TIMER_TIM) == 1) {
    LL_TIM_ClearFlag_CC2(TIMER_TIM);

	// Disable the Interrupt
	LL_TIM_EnableIT_CC2(TIMER_TIM);

	timerData.alarm2Callback(timerData.alarm2Parameter);
    }
    else if (LL_TIM_IsActiveFlag_CC3(TIMER_TIM) == 1) {
    LL_TIM_ClearFlag_CC3(TIMER_TIM);

	// Disable the Interrupt
	LL_TIM_EnableIT_CC3(TIMER_TIM);

	timerData.alarm3Callback(timerData.alarm3Parameter);
    }
}
