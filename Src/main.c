
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "serial.h"
#include "system_defines.h"
#include "cli.h"
#include "timer.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void SystemClock_Config(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void ADC_Callback(void);
void Commutate(int period);
void TIM1_CC_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler(void);
void align(void);
void fetstep(uint8_t step);
void fetBeep(uint32_t freq, uint32_t duration);

/* USER CODE END PFP */
extern volatile uint32_t runCount;
/* USER CODE BEGIN 0 */
uint32_t oldIdleCounter;
float idlePercent;
volatile uint32_t minCycles, idleCounter, totalCycles;
volatile uint8_t state, inputMode;
volatile uint32_t cycles;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *SCB_DEMCR = (uint32_t *)0xE000EDFC;

bool frame_500Hz;
bool frame_400Hz;
bool frame_200Hz;
bool frame_100Hz;
bool frame_50Hz ;
bool frame_10Hz ;
bool frame_5Hz  ;
bool frame_1Hz  ;
///////////////////////////////////////
// Frame Timing Variables
///////////////////////////////////////
#define FRAME_COUNT   (TIMER_MULT*1000)

#define COUNT_1000HZ  (TIMER_MULT*1)         // Number of 1000 Hz frames for 1000 Hz Loop
#define COUNT_500HZ   (TIMER_MULT*2)         // Number of 1000 Hz frames for 500 Hz Loop
#define COUNT_400HZ   5         		   // 5/2=2.5*2 Number of 1000 Hz frames for 400 Hz Loop
#define COUNT_200HZ   (TIMER_MULT*5)         // Number of 1000 Hz frames for 200 Hz Loop
#define COUNT_100HZ   (TIMER_MULT*10)        // Number of 1000 Hz frames for 100 Hz Loop
#define COUNT_50HZ    (TIMER_MULT*20)        // Number of 1000 Hz frames for  50 Hz Loop
#define COUNT_10HZ    (TIMER_MULT*100)       // Number of 1000 Hz frames for  10 Hz Loop
#define COUNT_5HZ     (TIMER_MULT*200)       // Number of 1000 Hz frames for   5 Hz Loop
#define COUNT_1HZ     (TIMER_MULT*1000)      // Number of 1000 Hz frames for   1 Hz Loop

//----------------
int16_t histIndex=0;
int16_t histSize;
uint16_t histA[ADC_HIST_SIZE];
uint16_t histB[ADC_HIST_SIZE];
uint16_t histC[ADC_HIST_SIZE];
//----------------
int16_t adcAdvance;
int32_t angle=0;
uint32_t IntegrateTreshold=500;
uint32_t fetCommutationMicros;
uint32_t currentMicros;
uint32_t oldMicros;
uint32_t PeriodMicros;
int32_t IntA,IntB,IntC;
uint8_t IntergrateA=0;
uint8_t IntergrateB=0;
uint8_t IntergrateC=0;
uint8_t StartCommutate=0;
uint32_t avgA, avgB, avgC,valN;
int32_t adcAmpsOffset;
int32_t adcMinPeriod,adcMaxPeriod;
uint32_t DeadTime;
uint32_t Pulse_width;
uint32_t Period_PWM;
uint32_t frameCounter;
float RPM;
float rpm;
float duty;
float mainDuty;
float runRPMFactor;
float runRpmLPF;

volatile uint32_t detectedCrossing;
volatile uint32_t crossingPeriod;
volatile int32_t crossingPeriod2;
volatile int32_t adcCrossingPeriod;
uint32_t nextCrossingDetect;

uint8_t adcStateA, adcStateB, adcStateC ,fetStepDir, fetStep;
volatile uint32_t GoodDetected;
volatile uint32_t BadDetected;
uint32_t OldGood;
volatile uint8_t oldStep=0;
volatile uint8_t nextStep=0;
uint8_t firsttime=1;

extern uint16_t adcRawData[ADC_CHANNELS*2];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t Message[]="\rCLI Init...\r\n";
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  //SystemCoreClockUpdate();
  /* USER CODE BEGIN SysInit */
  adcMinPeriod = ADC_MIN_PERIOD;
  adcMaxPeriod = ADC_MAX_PERIOD;
  adcCrossingPeriod = (adcMaxPeriod/2)<<15;
  crossingPeriod = (adcMaxPeriod/2);
  adcAdvance = 100.0f / (5.0f * (50.0f / 30.0f));
  nextCrossingDetect = adcMaxPeriod;
  RPM = 50.0f;
  duty = 0.1f;
  DeadTime = 10;
  histSize = ADC_HIST_SIZE;
  BadDetected =0;
  GoodDetected =0;
  runRPMFactor = (1e6f * (float)TIMER_MULT * 120.0f) / (MotorPole * 6.0f);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
 // MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  fetstep(0); // вырубаем ключи
 // MX_ADC2_Init();
//  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  //pwminit();
  timerInit();
  serialInit();
  cliInit();
 // serialPrint(Message);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

   STATE = STATE_ALIGN;

   align();

   STATE = STATE_STARTING;
   /* Enable counter */
   LL_ADC_REG_StartConversionSWStart(ADC1);
   LL_TIM_EnableCounter(TIM1);

	uint32_t lastRunCount;
	uint32_t thisCycles, lastCycles;
    *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
    *DWT_CONTROL = *DWT_CONTROL | 1;	// enable the counter

	minCycles = 0xffff;
      while (1) {
        idleCounter++;
	    if (runCount != lastRunCount && !(runCount % (RUN_FREQ / 1000))) {
		    cliCheck();

		lastRunCount = runCount;
	    }
	    rpm = runRpmLPF * rpm + ((32768.0f * runRPMFactor) / (float)adcCrossingPeriod) * (1.0f - runRpmLPF); // increased resolution, variable filter here

		  if (frame_1Hz){
			  frame_1Hz  = false;
			  LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		  }

		  if (frame_200Hz) {
			  frame_200Hz =false;
			    {
				//   LL_GPIO_TogglePin(TP3_GPIO_Port, TP3_Pin);

			    	if (BadDetected > 80 ){ //&& STATE >= STATE_STARTING
			    		__asm volatile ("cpsid i");
			    		fetstep(0);
			    		STATE = STATE_SYNCLOSS;
			      		duty = 0.1f;
			    		GoodDetected = 0;

			    		align();

			    		LL_TIM_OC_SetCompareCH1(TIM4,(uint32_t)  Period_PWM*duty);
			    		LL_TIM_OC_SetCompareCH2(TIM4,(uint32_t)  Period_PWM*duty);
			    		LL_TIM_OC_SetCompareCH3(TIM4,(uint32_t)  Period_PWM*duty);
/*
			    		LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
			    		LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2);
			    		LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3);
			    		LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH4);
/*/
			    		TIM1->CNT = 0;
			    		LL_TIM_EnableCounter(TIM1);
			    		 __asm volatile ("cpsie i");
			    		STATE = STATE_STARTING;
			    	}

					if (GoodDetected > 80) // && STATE > STATE_STARTING
					{
					//	LL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
						__asm volatile ("cpsid i");
						LL_TIM_DisableCounter(TIM1);
					    STATE = STATE_RUNING;
/*
						LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH1);
						LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH2);
						LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH3);
						LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH4);
/*/
	//					LL_TIM_DisableCounter(TIM1);
						 __asm volatile ("cpsie i");
					}
			    	if (GoodDetected==OldGood) {
			    		BadDetected++;
			    	}
			    	OldGood=GoodDetected;
			    }

		  }


        thisCycles = *DWT_CYCCNT;
	    cycles = thisCycles - lastCycles;
	    lastCycles = thisCycles;
          // record shortest number of instructions for loop
	    totalCycles += cycles;
          if (cycles < minCycles)
              minCycles = cycles;
      }
}

static void LL_Init(void)
{
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* BusFault_IRQn interrupt configuration */
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* UsageFault_IRQn interrupt configuration */
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SVCall_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* DebugMonitor_IRQn interrupt configuration */
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_7, 420, LL_RCC_PLLP_DIV_2);

  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);

  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_4);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(240000000);

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

  LL_SetSystemCoreClock(240000000);

  /* SysTick_IRQn interrupt configuration */
  SysTick_Config(SystemCoreClock / RUN_FREQ);
  NVIC_SetPriority(SysTick_IRQn, 10);	    // lower priority
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	//Аварийное отключение ключей в случае аппаратных проблем
	 FET_A_L_PWM_OFF;
	 FET_B_L_PWM_OFF;
	 FET_C_L_PWM_OFF;
 	 FET_A_H_PWM_OFF;
	 FET_B_H_PWM_OFF;
	 FET_C_H_PWM_OFF;
	 FET_A_L_OFF;
	 FET_B_L_OFF;
	 FET_C_L_OFF;
	 FET_A_H_OFF;
	 FET_B_H_OFF;
	 FET_C_H_OFF;
	 while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
/**
* @brief This function handles System tick timer.
*/

void SysTick_Handler(void)
{
//	LL_GPIO_TogglePin(TP3_GPIO_Port, TP3_Pin);
	if (frameCounter > FRAME_COUNT)
		frameCounter = 1;
	if ((frameCounter % COUNT_200HZ) == 0)
		frame_200Hz = true;
	if ((frameCounter % COUNT_1HZ) == 0)
		frame_1Hz = true;

    if (!(runCount % (10 * 1000 / RUN_FREQ))) {
	idlePercent = 100.0f * (idleCounter-oldIdleCounter) / (SystemCoreClock * 10 / RUN_FREQ / minCycles);
	oldIdleCounter = idleCounter;
	totalCycles = 0;
    }
	frameCounter++;
    runCount++;
}

void DMA2_Stream0_IRQHandler(void)
{
	  /* Check whether DMA transfer complete caused the DMA interruption */
	  if(LL_DMA_IsActiveFlag_TC0(DMA2) == 1)
	  {
	    /*  Clear Stream  transfer complete flag*/
	    LL_DMA_ClearFlag_TC0(DMA2);
	    /* Call interruption treatment function */
	//	LL_GPIO_TogglePin(TP1_GPIO_Port, TP1_Pin);

	    ADC_Callback();
	  }

	  /* Check whether DMA transfer error caused the DMA interruption */
	  if(LL_DMA_IsActiveFlag_TE0(DMA2) == 1)
	  {
	    /* Clear flag DMA transfer error */
	    LL_DMA_ClearFlag_TE0(DMA2);

	    /* Call interruption treatment function */
	//    AdcDmaTransferError_Callback();
	  }
}

static inline void adcGrowHist(void) {
    register int i;

    avgA += histA[histIndex];
    avgB += histB[histIndex];
    avgC += histC[histIndex];

    for (i = histSize; i > histIndex; i--) {
	histA[i] = histA[i-1];
	histB[i] = histB[i-1];
	histC[i] = histC[i-1];
    }

    histSize++;
}

static inline void adcShrinkHist(void) {
    register int i;

    for (i = histIndex; i < histSize-1; i++) {
	histA[i] = histA[i+1];
	histB[i] = histB[i+1];
	histC[i] = histC[i+1];
    }

    histSize--;

    if (histIndex == histSize)
	histIndex = 0;

    avgA -= histA[histIndex];
    avgB -= histB[histIndex];
    avgC -= histC[histIndex];
}

static inline void adcEvaluateHistSize(void) {
    int16_t sizeNeeded;

    sizeNeeded = crossingPeriod/32/TIMER_MULT;

    if (sizeNeeded > (histSize+1) && histSize < ADC_HIST_SIZE)
	adcGrowHist();
    else if (sizeNeeded < (histSize-1) && sizeNeeded > 1)
	adcShrinkHist();
}

void Commutate(int period)
{

	if (STATE == STATE_RUNING) {
		nextStep++;
		if (nextStep > 6 ) nextStep=1;
		fetstep(nextStep);
	}
}


void fetstep(uint8_t step)
{
	__asm volatile ("cpsid i");
	switch (step)
	 {
	 // шаг 0 - исходное состояние - все ключи выключены
	 case 0:
	  {
		 FET_A_H_PWM_OFF;
		 FET_B_H_PWM_OFF;
		 FET_C_H_PWM_OFF;
		 FET_A_L_OFF;
		 FET_B_L_OFF;
		 FET_C_L_OFF;
		 break;
	  }
	 // шаг 1
	 case 1:
	  {
		 FET_C_H_PWM_OFF;
		 FET_B_H_PWM_ON;
		 break;
	  }
	 // шаг 2
	 case 2:
	  {

		 FET_A_L_OFF;
		 FET_C_L_ON;
		 break;
	  }
	 // шаг 3
	 case 3:
	  {

		 FET_B_H_PWM_OFF;
		 FET_A_H_PWM_ON;
		 break;
	  }
	 // шаг 4
	 case 4:
	  {
		 FET_C_L_OFF;
		 FET_B_L_ON;
		 break;
	  }
	 // шаг 5
	 case 5:
	  {
		 FET_A_H_PWM_OFF;
		 FET_C_H_PWM_ON;
		 break;
	  }
	 // шаг 6
	 case 6:
	  {
		 FET_B_L_OFF;
		 FET_A_L_ON;
		 break;
	  }
	 // шаг 7 - тормоз все нижние ключи включены
	 case 7:
	  {
		 FET_A_H_PWM_OFF;
		 FET_B_H_PWM_OFF;
		 FET_C_H_PWM_OFF;
		 FET_A_L_ON;//!
		 FET_B_L_ON;//!
		 FET_C_L_ON;//!
		 break;
	  }

	 // подстраховка в случае неправильного состояния переменной шага - отключаем все ключи
	 default:
	  {
		 FET_A_H_PWM_OFF;
		 FET_B_H_PWM_OFF;
		 FET_C_H_PWM_OFF;
		 FET_A_L_OFF;
		 FET_B_L_OFF;
		 FET_C_L_OFF;
		 break;
	  }
	 }
	fetCommutationMicros = timerGetMicros();
	__asm volatile ("cpsie i");
}


// this assume that one low FET is conducting (s/b B)
void fetBeep(uint32_t freq, uint32_t duration) {
    uint32_t prevReloadVal;
    int i;
    fetstep(0);
    FET_H_GPIO;
    __asm volatile ("cpsid i");
    FET_B_L_ON;
    for (i = 0; i < duration; i++) {
	// reload the hardware watchdog

	FET_A_H_ON;
	//timerDelay(8);


	timerDelay((1/freq)/2);
	FET_A_H_OFF;
/*
	FET_C_L_ON;
	timerDelay(8);
	FET_C_H_OFF;
*/
	timerDelay((1/freq)/2);
    }
    fetstep(0);

    __asm volatile ("cpsie i");

}
void ADC_Callback(void) {
    register uint16_t *raw = (uint16_t *)adcRawData;
    register uint32_t valA, valB, valC;
    __asm volatile ("cpsid i");
    currentMicros = timerGetMicros();
    __asm volatile ("cpsie i");

	// LL_GPIO_TogglePin(TP3_GPIO_Port, TP2_Pin);
    // blanking time after commutation
	if (!fetCommutationMicros || ((currentMicros >= fetCommutationMicros) ? (currentMicros - fetCommutationMicros) : (TIMER_MASK - fetCommutationMicros + currentMicros)) > GuardInterval)
	{

    histA[histIndex] = valA = (raw[0]);
    histB[histIndex] = valB = (raw[1]);
    histC[histIndex] = valC = (raw[2]);

    histIndex = (histIndex + 1) %  histSize;

    avgA += valA - histA[histIndex];
    avgB += valB - histB[histIndex];
    avgC += valC - histC[histIndex];

/*
	avgA = medfilter1(raw[0]);
	avgB = medfilter2(raw[1]);
	avgC = medfilter3(raw[2]);
*/
//	valN = (avgA+avgB+avgC)/3;

    	if (((avgA+avgB+avgC)/histSize) > (ADC_MIN_COMP*3))
    	{
        	register int32_t periodMicros;

            periodMicros = (currentMicros >= detectedCrossing) ? (currentMicros - detectedCrossing) : (TIMER_MASK - detectedCrossing + currentMicros);
    		// LL_GPIO_TogglePin(TP3_GPIO_Port, TP2_Pin);
            if (periodMicros > nextCrossingDetect) {//periodMicros > nextCrossingDetect) {

                register int8_t Start = 0;

                if (!adcStateA && avgA >= ((avgB+avgC)>>1)+N_Slope) {
                    adcStateA = 1;
                    Start=1;
                }
                else if (adcStateA && avgA <= ((avgB+avgC)>>1)-N_Slope) { //-N_Slope)
                     adcStateA = 0;
                    Start=1;
                }
                else if (!adcStateB && avgB >= ((avgA+avgC)>>1)+N_Slope) {
                    adcStateB = 1;
                    Start=1;
                }
                else if (adcStateB && avgB <= ((avgA+avgC)>>1)-N_Slope) {
                    adcStateB = 0;
                    Start=1;
                }
                else if (!adcStateC && avgC >= ((avgA+avgB)>>1)+N_Slope) {
                    adcStateC = 1;
                    Start=1;
                }
                else if (adcStateC && avgC <= ((avgA+avgB)>>1)-N_Slope) {
                    adcStateC = 0;
                    Start=1;
                }

        		if (Start && periodMicros > 300)
	//    if ( Start  && periodMicros > 400 ) //StartCommutate && periodMicros > adcMinPeriod
        	  {
        		 if (periodMicros > adcMaxPeriod)
        		 {
        			 periodMicros = adcMaxPeriod;
        		 }
        		 	 adcCrossingPeriod += ((periodMicros<<15) - adcCrossingPeriod)>>3;
        		 	 crossingPeriod = adcCrossingPeriod>>15;
        		 	 // schedule next commutation
        		     //Commutate(1);
        		     //LL_GPIO_TogglePin(TP3_GPIO_Port, TP2_Pin);
        		    // if (STATE > STATE_STARTING) LL_GPIO_TogglePin(TP1_GPIO_Port, TP1_Pin);
        	         //timerSetAlarm1((uint32_t)(crossingPeriod*0.5f)-angle, Commutate, crossingPeriod);
        	         timerSetAlarm1((uint32_t)crossingPeriod/2 - (ADC_DETECTION_TIME*ADC_HIST_SIZE)/2 - angle, Commutate, 1);
        	         //timerSetAlarm1((crossingPeriod>>1) - (ADC_DETECTION_TIME*(histSize+2))/2 - ADC_COMMUTATION_ADVANCE, Commutate, crossingPeriod);

        	         GoodDetected++;
        		 	// crossingPeriod2 = currentMicros - fetCommutationMicros;
        		 	 BadDetected = 0;
        		 	 adcEvaluateHistSize();
        		 	 fetCommutationMicros = 0;
        		 	 // record crossing time
        		 	 detectedCrossing = currentMicros;
        		 	 // calculate next crossing detection time
        		 	 nextCrossingDetect = crossingPeriod*3/4;

                   //  StartCommutate=0;
        	  }
        		else if (Start)  // Prevent False positive ZC detection
        		{
        			detectedCrossing = currentMicros;
        			//BadDetected++;
        		}
	    Start=0;
	}
    	}
   }

}
void TIM1_UP_TIM10_IRQHandler(void)
{
	  if(LL_TIM_IsActiveFlag_UPDATE(TIM1) == 1)
	  {
		  LL_TIM_ClearFlag_UPDATE(TIM1);
		  LL_GPIO_TogglePin(LED1_GPIO_Port, LED2_Pin);
	  }
}
void TIM1_CC_IRQHandler(void)
{
	  //Capture compare 1 = Первый канал используется для начальных отчетов на ОС1,2,3
	  if(LL_TIM_IsActiveFlag_CC1(TIM1) == 1)
	  {
		  LL_TIM_ClearFlag_CC1(TIM1);
	    	if (TIM1->CCR1 == 0+DeadTime)
	    	{
	    		TIM1->CCR1=(Pulse_width+DeadTime);
	    		FET_B_H_PWM_ON;
	    		nextStep = 1;
	    	}
	    	else if (TIM1->CCR1 == (uint32_t)(Pulse_width)+DeadTime)
	    	{
	    		TIM1->CCR1=(uint32_t) (2*Pulse_width)+DeadTime;
	    		FET_A_H_PWM_ON;
	    		nextStep = 3;
	    	}

	    	else if (TIM1->CCR1 == (uint32_t)(2*Pulse_width)+DeadTime)
	    	{
	    		TIM1->CCR1= 0+DeadTime;
	    		FET_C_H_PWM_ON;
	    		nextStep = 5;
	    	}
    		fetCommutationMicros = timerGetMicros();
	  }
	  //Capture Compare 2 = Второй канал используется для конечных отчетов на ОС1,2,3
	  if(LL_TIM_IsActiveFlag_CC2(TIM1) == 1)
	  {
		  LL_TIM_ClearFlag_CC2(TIM1);
	    	if (TIM1->CCR2 == (uint32_t)(2*Pulse_width+Pulse_width*mainDuty)-DeadTime)
	    	{
	    		TIM1->CCR2=(uint32_t)(Pulse_width*mainDuty)-DeadTime;
	    		FET_C_H_PWM_OFF;
	    	}
	    	else if (TIM1->CCR2 == ((uint32_t)(Pulse_width*mainDuty)-DeadTime))
	    	{
	    		TIM1->CCR2= (uint32_t)(Pulse_width+Pulse_width*mainDuty)-DeadTime;
	    		FET_B_H_PWM_OFF;
	    	}
	    	else if (TIM1->CCR2 == ((uint32_t)(Pulse_width+Pulse_width*mainDuty)-DeadTime))
	    	{
	    		TIM1->CCR2= (uint32_t)(2*Pulse_width+Pulse_width*mainDuty)-DeadTime;
	    		FET_A_H_PWM_OFF;
	    	}
	  }
	  //Capture Compare 3 = Третий канал используется для начальных отчетов на ОСN1,2,3
	  if(LL_TIM_IsActiveFlag_CC3(TIM1) == 1)
	  {
		  LL_TIM_ClearFlag_CC3(TIM1);

	    	if (TIM1->CCR3 == (uint32_t)(Pulse_width/2)+DeadTime)
	    	{
	    		TIM1->CCR3=(uint32_t)(Pulse_width/2)+Pulse_width+DeadTime;
				FET_C_L_ON;
				nextStep = 2;
	    	}
	    	else if (TIM1->CCR3 == ((uint32_t)(Pulse_width/2)+Pulse_width)+DeadTime)
	    	{
	    		TIM1->CCR3= ((uint32_t)(Pulse_width/2)+(uint32_t)2*Pulse_width+DeadTime);
	    		FET_B_L_ON;
	    		nextStep = 4;
	    	}

	    	else if (TIM1->CCR3 == ((uint32_t)(Pulse_width/2)+(uint32_t)2*Pulse_width+DeadTime))
	    	{
	    		TIM1->CCR3= (uint32_t)(Pulse_width/2)+DeadTime;
	    		FET_A_L_ON;
	    		nextStep = 6;
	    	}
    		fetCommutationMicros = timerGetMicros();
	  }
	  //Capture compare 4 = Четвертый канал используется для конечных отчетов на ОСN1,2,3
	  if(LL_TIM_IsActiveFlag_CC4(TIM1) == 1)
	  {
		  LL_TIM_ClearFlag_CC4(TIM1);
	    	if (TIM1->CCR4 == (uint32_t)(Pulse_width/2)-DeadTime)
	    	{
	    		TIM1->CCR4=(uint32_t)(Pulse_width/2)+Pulse_width-DeadTime;
	    		FET_A_L_OFF;
	    	}
	    	else if (TIM1->CCR4 == ((uint32_t)(Pulse_width/2)+Pulse_width)-DeadTime)
	    	{
	    		TIM1->CCR4=(uint32_t) ((Pulse_width/2)+(uint32_t)2*Pulse_width)-DeadTime;
				FET_C_L_OFF;
	    	}

	    	else if (TIM1->CCR4 == ((uint32_t)(Pulse_width/2)+(uint32_t)2*Pulse_width)-DeadTime)
	    	{
	    		TIM1->CCR4= (uint32_t)(Pulse_width/2)-DeadTime;
	    		FET_B_L_OFF;
	    	}

	  }
}

void align(void)
{
	//Shutdown all switched
	 fetstep(0);
	 timerDelay(500);
	 fetstep(6);
	 timerDelay(500);
	 fetstep(1);
	 timerDelay(500);
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
