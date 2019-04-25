#pragma once

#include "stddef.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

#define VERSION	1
#define BUILDNUMBER 1
#define ADC_CHANNELS	1
#define RUN_FREQ		2000		    // Hz
#define MotorPole	14
#define	PWMFreq		48000.0f
#define max_potenciomentr	4000.0f
#define min_potenciomentr	0.0f
#define scale_potenciomentr	(max_potenciomentr - min_potenciomentr)

#define max_sine_freq		2000.0f	                                //defines the top frecuency
#define min_sine_freq		50.0f	                                //defines the bottom frecuency
#define sine_freq_scale  (max_sine_freq - min_sine_freq)

#define max_RPM		10000.0f	                                //defines the top frecuency
#define min_RPM		10.0f	                                //defines the bottom frecuency
#define RPM_scale  (max_RPM - min_RPM)

#define ADC_DETECTION_TIME	(uint32_t) (15*4*(1/60e6))  // 1.5T Sampling time + 12.5T ConversionTime at 12bit * Number of channels * Tick time  ADC Clock
#define ADC_COMMUTATION_ADVANCE	(crossingPeriod/adcAdvance)		    // variable
#define ADC_HIST_SIZE		64
#define ADC_MIN_COMP	30
#define N_Slope	0 //0					   // порог колебаний нулевой точки Back-EMF
//#define IntegrateTreshold (uint32_t) 4500 //0
#define GuardInterval	30

#define ADC_MIN_PERIOD          150       // us
#define ADC_MAX_PERIOD          20000    // us
#define DEFAULT_BLANKING_MICROS     30       // us

#define RAD2DEG 				(float)(180.0/M_PI)
#define DEG2RAD					(float)(M_PI/180.0)
#define R2D 					(float)(180.0/M_PI)
#define D2R						(float)(M_PI/180.0)

#define SQR(x)  ((x) * (x))
#define __sqrtf(a)	sqrtf(a)
#ifndef MIN
#define MIN(a, b) ((a < b) ? a : b)
#endif
#ifndef MAX
#define MAX(a, b) ((a > b) ? a : b)
#endif


//��������� ������� ��� ��������� ��������
enum STATE {
    STATE_NORMAL	= 0x00,
    STATE_ALIGN		= 0x01,
    STATE_STARTING	= 0x02,
    STATE_RUNING	= 0x04,
    STATE_SYNCLOSS	= 0x08,
    STATE_STOP	= 0x10,
};
enum STATE STATE;


enum configParameters {
    CONFIG_VERSION = 0,
    STARTUP_MODE,
    BAUD_RATE,
    PTERM,
    ITERM,
    FF1TERM,
    FF2TERM,
    CL1TERM,
    CL2TERM,
    CL3TERM,
    CL4TERM,
    CL5TERM,
    SHUNT_RESISTANCE,
    MIN_PERIOD,
    MAX_PERIOD,
    BLANKING_MICROS,
    ADVANCE,
    START_VOLTAGE,
    GOOD_DETECTS_START,
    BAD_DETECTS_DISARM,
    MAX_CURRENT,
    SWITCH_FREQ,
    MOTOR_POLES,
    PWM_MIN_PERIOD,
    PWM_MAX_PERIOD,
    PWM_MIN_VALUE,
    PWM_LO_VALUE,
    PWM_HI_VALUE,
    PWM_MAX_VALUE,
    PWM_MIN_START,
    PWM_RPM_SCALE,
    FET_BRAKING,
    PNFAC,
    INFAC,
    THR1TERM,
    THR2TERM,
    START_ALIGN_TIME,
    START_ALIGN_VOLTAGE,
    START_STEPS_NUM,
    START_STEPS_PERIOD,
    START_STEPS_ACCEL,
    PWM_LOWPASS,
    RPM_MEAS_LP,
    SERVO_DUTY,
    SERVO_P,
    SERVO_D,
    SERVO_MAX_RATE,
    SERVO_SCALE,
    ESC_ID,
    DIRECTION,
    CONFIG_NUM_PARAMS
};


#define FET_A_L_PORT	GPIOA
#define FET_B_L_PORT	GPIOB
#define FET_C_L_PORT	GPIOB
#define FET_A_H_PORT	GPIOB
#define FET_B_H_PORT	GPIOB
#define FET_C_H_PORT	GPIOB

#define FET_A_L_PIN		LL_GPIO_PIN_7
#define FET_B_L_PIN		LL_GPIO_PIN_0
#define FET_C_L_PIN		LL_GPIO_PIN_1
#define FET_A_H_PIN		LL_GPIO_PIN_6
#define FET_B_H_PIN		LL_GPIO_PIN_7
#define FET_C_H_PIN		LL_GPIO_PIN_8

#define FET_A_H_ON 	LL_GPIO_SetOutputPin(FET_A_H_PORT,FET_A_H_PIN)
#define FET_B_H_ON 	LL_GPIO_SetOutputPin(FET_B_H_PORT,FET_B_H_PIN)
#define FET_C_H_ON 	LL_GPIO_SetOutputPin(FET_C_H_PORT,FET_C_H_PIN)

#define FET_A_H_OFF LL_GPIO_ResetOutputPin(FET_A_H_PORT,FET_A_H_PIN)
#define FET_B_H_OFF LL_GPIO_ResetOutputPin(FET_B_H_PORT,FET_B_H_PIN)
#define FET_C_H_OFF LL_GPIO_ResetOutputPin(FET_C_H_PORT,FET_C_H_PIN)

#define FET_A_L_ON 	LL_GPIO_SetOutputPin(FET_A_L_PORT,FET_A_L_PIN)
#define FET_B_L_ON	LL_GPIO_SetOutputPin(FET_B_L_PORT,FET_B_L_PIN)
#define FET_C_L_ON	LL_GPIO_SetOutputPin(FET_C_L_PORT,FET_C_L_PIN)

#define FET_A_L_OFF LL_GPIO_ResetOutputPin(FET_A_L_PORT,FET_A_L_PIN)
#define FET_B_L_OFF LL_GPIO_ResetOutputPin(FET_B_L_PORT,FET_B_L_PIN)
#define FET_C_L_OFF LL_GPIO_ResetOutputPin(FET_C_L_PORT,FET_C_L_PIN)

#define FET_H_PWM LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6|LL_GPIO_PIN_7|LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE)
#define FET_H_GPIO LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6|LL_GPIO_PIN_7|LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT)


// turn switch on or off PWM output
// HI side MODER Bits for 407 and CNF for 103
//Тут какая то херня на 407 камне просиходит через MODER BITs нормально не управляется как остальные пины почему то инверсное состояние то есть в "1" когда выключен что только ен пробовал пришлось через AFRL его дергать
#define AH_BITBAND  	((uint32_t *)(PERIPH_BB_BASE+32*((uint32_t)(&(GPIOB->AFR[0]))-PERIPH_BASE) + (25*4)))

#define FET_A_H_PWM_ON  *AH_BITBAND = 1
#define FET_A_H_PWM_OFF	*AH_BITBAND = 0

// HI side
//#define FET_A_H_PWM_ON		LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1)
#define FET_B_H_PWM_ON		LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2)
#define FET_C_H_PWM_ON		LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3)
//#define FET_A_H_PWM_OFF		LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH1)
#define FET_B_H_PWM_OFF		LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH2)
#define FET_C_H_PWM_OFF		LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3)
// LO side
#define FET_A_L_PWM_ON		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1)
#define FET_B_L_PWM_ON		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2)
#define FET_C_L_PWM_ON		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3)
#define FET_A_L_PWM_OFF		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1)
#define FET_B_L_PWM_OFF		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH2)
#define FET_C_L_PWM_OFF		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH3)
