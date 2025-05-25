/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <cstdio>
#include "LEDcpp.hpp"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define micros() TIM5->CNT
void Initial_System_Timer(void)
{
	RCC->APB1ENR |= 0x0008;	
	TIM5->CR1 = 0x0080; 
	TIM5->CR2 = 0x0000;
	TIM5->CNT = 0x0000;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->PSC = 84 - 1;	
	TIM5->EGR = 0x0001;
	TIM5->CR1 |= 0x0001; 
}
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define TASK_COUNT 4
uint32_t u32_debug_vars[10];
void TaskA(void) {
    printf("Task A is running\n\r");
		u32_debug_vars[0]++;
}
void TaskB(void) {
		printf("Task B is running\n\r");
		u32_debug_vars[1]++;
    //usleep(500);
}
void TaskC(void) {
		printf("Task C is running\n\r");
		u32_debug_vars[2]++;
    //usleep(1000);
}
void TaskD(void) {
		printf("Task D is running\n\r");
	   u32_debug_vars[3]++;
}
void fast_loop(void) {
    printf("This is fast loop \n\r");
		u32_debug_vars[4]++;
}

unsigned long AP_HAL_micro(void);
void scheduler_run(unsigned int t_available);
unsigned int main_loop_tick = 0, loop_rate_hz = 400;
unsigned long time_available, sample_time_us, loop_period_us;

struct task {
    void (*task_fnc)(void); //pointer to function
    unsigned int task_rate_hz;
    unsigned int last_run; // tick count for each task, used for ensuring task_rate_hz
    unsigned int max_time_micros; // maximum execution time in microseconds.
} task_list[]={
    {TaskA, 50, 0 , 100},
    {TaskB, 100, 0, 500},
    {TaskC, 200, 0, 200},
    {TaskD, 400, 0, 200}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
CLed led3(GPIOD, GPIO_PIN_13, 50);
CLed led4(GPIOD, GPIO_PIN_12, 500);
CLed led5(GPIOD, GPIO_PIN_14, 200);
CLed led6(GPIOD, GPIO_PIN_15, 400);
/* USER CODE END 0 */
uint8_t pd12_status;
unsigned long time_usecond;
unsigned long start_time_usecond;
double time_sec;
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	GPIO gpio_d12(GPIO_PIN_12);
	
  /* USER CODE BEGIN 2 */
	Initial_System_Timer();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* USER CODE END WHILE */
			pd12_status = gpio_d12.read();
		/* USER CODE BEGIN 3 */
			/*HAL_Delay(1000);  //delay 1s
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);*/
			
			printf("----------------------------------------\n");
			sample_time_us = AP_HAL_micro();    
			fast_loop();
			main_loop_tick++;
			time_available = (sample_time_us + loop_period_us) - AP_HAL_micro();
			printf("time available is: %lu [us]\n", time_available);
			scheduler_run((unsigned int)time_available);
			time_usecond = AP_HAL_micro();
			time_usecond -= start_time_usecond;
			time_sec = (double)time_usecond/1000000;
			printf ("Current tick = %lu microseconds\n", time_usecond);
			printf ("Current tick = %lf seconds\n", time_sec);
    }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 252;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//was "__weak void HAL_SYSTICK_Callback(void)" in stm32f4xx_hal_cortex.c
void HAL_SYSTICK_Callback(void)
{
  led3.runToggle();
  led4.runToggle();
  led5.runToggle();
  led6.runToggle();
}

void scheduler_run(unsigned int t_available) {
    printf("In the scheduler function!\n");
    for (int i=0; i< TASK_COUNT; i++) {
        unsigned int dt = main_loop_tick - task_list[i].last_run;
        printf("dt[%d] has value: %u\n", i, dt);
        unsigned int interval_ticks = loop_rate_hz/task_list[i].task_rate_hz;
        if (interval_ticks < 1) {
            interval_ticks = 1;
        }
        if (dt < interval_ticks) {
            continue;
        }
        unsigned int task_start = AP_HAL_micro();
        task_list[i].task_fnc();
        task_list[i].last_run = main_loop_tick;
        unsigned int time_taken = AP_HAL_micro() - task_start;
        //printf("Time taken for the task is : %u [us]\n", time_taken);
        if (time_taken > task_list[i].max_time_micros) {
            // the event overran!
            printf("Scheduler overrun task\n");
        }
        if (time_taken >= t_available) {
            t_available = 0;
            break;
        }
        t_available -= time_taken;
    }
}
unsigned long AP_HAL_micro(void) {
    return (TIM5->CNT);
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
