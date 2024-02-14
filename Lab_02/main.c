/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	
void EXTI0_1_IRQHandler(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config(); // Configure the system clock

  // Enable the GPIOC clock
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  // Enable the GPIOA clock
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  // Enable the SYSCFG clock
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Clear the bits for PC6, PC7, PC8 and PC9
  //GPIOC->MODER &= ~(3 << 12);
  //GPIOC->MODER &= ~(3 << 14);
  //GPIOC->MODER &= ~(3 << 16);
  //GPIOC->MODER &= ~(3 << 18);
  // Setting PC6, PC7, PC8 and PC9 to General-Purpose Output Mode
  GPIOC->MODER |= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);
  // Setting PC6, PC7, PC8 and PC9 to Push-Pull Output Type
  GPIOC->OTYPER &= ~(1 << 6);
  GPIOC->OTYPER &= ~(1 << 7);
  GPIOC->OTYPER &= ~(1 << 8);
  GPIOC->OTYPER &= ~(1 << 9);
  // Set PC6, PC7, PC8 and PC9 to Low Speed
  GPIOC->OSPEEDR &= ~(0 << 12);
  GPIOC->OSPEEDR &= ~(0 << 14);
  GPIOC->OSPEEDR &= ~(0 << 16);
  GPIOC->OSPEEDR &= ~(0 << 18);
  // Clear the bits for PC6, PC7, PC8 and PC9
  // This also sets the pull-up/pull-down resistors to no pull-up/pull-down since the bits are 00
  GPIOC->PUPDR &= ~(3 << 12);
  GPIOC->PUPDR &= ~(3 << 14);
  GPIOC->PUPDR &= ~(3 << 16);
  GPIOC->PUPDR &= ~(3 << 18);

  // Setting PA0 to Digital Input Mode
  GPIOA->MODER &= ~(3 << 0);
  // Set PA0 to Low Speed
  GPIOA->OSPEEDR &= ~(3 << 0);
  // Set PA0 to Pull Down Resistor
  GPIOA->PUPDR &= ~(3 << 0);

  //Set EXTI0 to allow for interrupts
  EXTI->IMR |= (1 << 0);

  //Set EXTI0 to trigger on a rising edge
  EXTI->RTSR |= (1 << 0);

  //Set SYSCFG to allow for EXTI0
  SYSCFG->EXTICR[0] |= (0 << 0);

  //Enable the EXTI0 interrupt and set the priority to 1
  NVIC_EnableIRQ(EXTI0_1_IRQn);
	//Part 2.1: Change Priorities of  EXTI interrupt handler and systick to resolve starving interrupt issue
	//NVIC_SetPriority(EXTI0_1_IRQn, 3);
  NVIC_SetPriority(EXTI0_1_IRQn, 1);
	
	//Part 2.2 - Set appropriate priority to systick handler interrupt
	//NVIC_SetPriority(SysTick_IRQn, 2);
	
	
  //Set PC9 Green to High
  GPIOC->ODR |= (1 << 9);
  
  while (1)
  {
		
    HAL_Delay(500); // Delay for 500ms
    GPIOC->ODR ^= (1 << 6); // Toggle PC6 Red
		
  }
}

/** System Clock Configuration
 */
/**
  * @brief System Clock Configuration
  * @retval None
  */

void EXTI0_1_IRQHandler(void){
	
	EXTI->PR |= EXTI_PR_PR0;
	
	GPIOC->ODR ^= ((1 << 8) | (1 << 9));
	//Part2
	
	//delay loop
	*/
	volatile uint32_t count1 = 0;
	while (count1 < 1500000) {
		count1++;// This loop provides a rough delay of 1-2 seconds
  }
	GPIOC->ODR ^= ((1 << 8) | (1 << 9));
	//clear flag
	*/
	//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
