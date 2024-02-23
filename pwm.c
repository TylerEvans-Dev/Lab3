/**
  *
  * Brandon Mouser & Tyler Evans
  * U0962682, u1313811
  *
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f0xx_hal.h"
void _Error_Handler(char * file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
//
/**
 @brief function is the handler of the timer 2 used when intrupt is triggred
 @function TIM2_IRQHandler
 */
void TIM2_IRQHandler(void){
    GPIOC->ODR ^= (1<<8);
    GPIOC->ODR ^= (1<<9);
    TIM2->SR ^= (1<<0);
}

int main(void)
{
    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); //Configure the system clock
    /* This example uses HAL library calls to control
     the GPIOC peripheral. You’ll be redoing this code
     with hardware register access. */
    
    //__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
    //HERE IS THE RCC CLOCK ENABLE PIN REGEISTER DONE
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // done to find the clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // this inits the clock for GPIO A
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // sets up the timer 2
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enables timer 3;
    
    //sets everything to zero in the pins
    // GPIOC is the GPIO_x where the pin is located.
    GPIOC->MODER &= 0; // sets the mode
    GPIOC->OTYPER &= 0; // sets what type
    GPIOC->OSPEEDR &= 0; // sets the speed
    GPIOC->PUPDR &= 0; // sets the pulldown/pullup resitor
    
    //sets all the values in modder to the correct pin into input mode.
    //               10                     10                 10                10
    GPIOC->MODER |= (0<< 12) |(1<<13)| (0 << 14)| (1<<15)| (1<< 16)|(0<<17) | (1 << 18) | (0<<19); //configures what pins for use setting up the mode
    /*here is GPIOA stuff*/
    GPIOA->MODER &= 0; // resets the Moder regiester. this sets it to input  mode so it is not to be messed with
    GPIOA->OSPEEDR &= 0; //sets the clock to slowest option
    GPIOA->PUPDR &= 0; // sets it to a default mode
    GPIOA->PUPDR |= (1 << 2);// this sets it to the pulldown state. this is 1 0
    GPIOC->ODR |= (1<<9);
    GPIOC->ODR |= (1<<8);
    /* using the equation I decided ARR arbitaurly to ensure that it runs at 4hz
     */
    TIM2->ARR = 800;
    TIM2->PSC = 2499;
    /* This is the NVIC once again going into what we weould like
     */
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);
    
    /*TIMER 3 setup 800 hz wanted.
     the arr and psc is set using
     psc = 99
     arr = 100
     99 = (8*10^6 / (100 * 800)) -1
     */
    
    TIM3->ARR  = 100;
    TIM3->PSC = 99;
    //configure output
    TIM3->CCMR1 |= (0<<9) | (0<<8); // setting bits zero and 1 to be 00 for output mode on chan. 2
    TIM3->CCMR1 |= (0<<1) | (0<<0); // setting bits to be zero on here 00 sets t
    //PWM mode section
    TIM3->CCMR1 |= (1<<6) | (1<<5) | (1<< 4); // setting channel 1 to be in mode PWM mode 2 111 inbits 6,5 4
    //selecting channel 2
    TIM3->CCMR1 |= (1<<14) | (1<<13) | (0<< 12); // setting to be in mode PWM mode 1 110 in bits 6,5 4
    //enabling preload on the chanle pins 11 to enable 2; 3 for chan 1
    TIM3->CCMR1 |= (1<<11) | (1<<3);
    //here is the TIM3 part were we are enabling the
    TIM3->CCER |= (1<<0) | (1 << 4); // 1 at zero for chan. 1 output and 1  at 4 for chan 2
    //HERE IS THE CAPTURE 2
    int lightVal = 10;
    TIM3->CCR1 = lightVal;
    TIM3->CCR2 = lightVal; // value is 100 so 20 for both.
    TIM3->CR1 |= (1<<0);
    //PC7
    GPIOC->AFR[0] |= (0<<31) | (0<<30) |  (0<<29) | (0<<28); // for pin 7 I am going to use 0000 for # pin
    //PC6
    GPIOC->AFR[0]|= (0<<27) | (0<<26) | (0<<25) | (0<<24); // 0000 in # pin shfited right
    
    //one mistake I made here was not put the right bit of information in the
    //regiester.
    TIM2->DIER |= (1<<0);
    // this is done on the bottom becasue it enables the timer.
    TIM2->CR1 |= (1<<0); // this is the counter enable bit don't copy above
        while(1) {
            
     }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

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
