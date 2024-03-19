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
void GPIO_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
RCC->APB1ENR  |= RCC_APB1ENR_I2C2EN; //ENABLE I2C2
SystemClock_Config();


// Configure the leds in output mode
GPIOC->MODER |= (1<<12) | (1<<14) | (1<<16) | (1<<18) | (1<<0);
GPIOC->MODER &= ~((1<<13) | (1<<15) | (1<<17) | (1<<19) | (1<<1));
GPIOC->OTYPER &= ~((1<<0) | (1<<6) | (1<<7) | (1<<8) | (1<<9));
GPIOC->OSPEEDR &= ~((1<<0) |(1<<12) | (1<<14) | (1<<16) | (1<<18));
GPIOC->PUPDR &= ~((1<<0) |(1<<12) | (1<<14) | (1<<16) | (1<<18)	| (1<<1) | (1<<13) | (1<<15) | (1<<17) | (1<<19));
//Part 1
GPIO_Init(); //configure required pins in alternate modes and output modes
//i2c config
I2C2->TIMINGR |= ((0x01<<28)| (0x04<<20) | (0x02<<16) | (0xF<<8) | (0x13<<0)); //set timing parameters for 100Khz
I2C2->CR1 |= (1<<0); //enable i2c by enabling PE bit in cr1 
	
//I2C Comm flow steps
	
//STEP1: Configure i2c parameters, slave address, number of bytes and master to write mode
I2C2->CR2 = 0;
I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //clear sadd and nbyte 
I2C2->CR2 |= ((0x69 << 1)); //set sadd (slave address)
I2C2->CR2 |= (1<<16); //set nbytes =1
I2C2->CR2 |= (1<<10); //set rd_wrn for write
I2C2->CR2 |= (1<<13); //set start bit
	
//STEP 2: Check status of NACKF (not set)and TXIS(set) & step 3 (address of who register)
	
while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {}
//check if TXIS flag is set, turn blue led on to indicate ok and continue
if(I2C2->ISR & I2C_ISR_TXIS){
	GPIOC->ODR |= (1<<7);
}
I2C2->TXDR = 0x0F; //set address of WHO_AM_I register
		
//STEP 4:CHECK TC for end of transmission until end of nbytes (byte -=1) & step 5 set parameters again	
while (!(I2C2->ISR & I2C_ISR_TC)) {}
	
//step 5:Reload parameters and set master to read mode
	
I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //clear sadd and nbytes
I2C2->CR2 |= ((0x69 << 1)); //set sadd
I2C2->CR2 |= (1<<16); //set nbytes =1
I2C2->CR2 |= (0<<10); //set rd_wrn for read
	
//restart
I2C2->CR2 |= (1<<13); //set start bit
	
//step 6: check flags RXNE & NACKF, step 7 check tc, step 8 check contents of rxdr with who address
	
//step 6: Check rxne flag and NACKF
while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF))) {}
//check if RXNE flag is set, turn blue led on to indicate ok and continue
if(I2C2->ISR & I2C_ISR_RXNE){
	GPIOC->ODR |= (1<<7);
}
 
//step 7: Check TC
while (!(I2C2->ISR & I2C_ISR_TC)) {}	

//logic for verifying contents of RDR register
if(I2C2->RXDR == 0xD3){
	GPIOC->ODR |= (1<<9);
}
else{                                  
	GPIOC->ODR |= (1<<8);
}
 
//STEP 9 SET STOP BIT
I2C2->CR2 |= (1<<14);
 
/* USER CODE END 1 */

/* MCU Configuration--------------------------------------------------------*/

/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
//function for GPIO pins initialization, configuring pb11,pb13 in alternate modes, pb14 and pc0 in output mode
void GPIO_Init(void) {
//pb14 - 29,28 - 01
GPIOB->MODER |= (1<<28);
GPIOB->MODER &= ~(1<<29);
//GPIOB->MODER &= ~((1<<0) | (1<<1));
GPIOB->OTYPER &= ~ (1<<14);
GPIOC->OSPEEDR &= ~((1<<28));
GPIOC->PUPDR &= ~((1<<28) | (1<<29));
//GPIOB->PUPDR &= ~((1<<0));
//GPIOB->PUPDR |= (1<<1);	
	
//Alternate mode PINS: PB11 ALTERNATE MODE- I2C_SDA, PB13 ALTERNATE MODE- I2C_SDL, PB14 & PC0 OUTPUT
GPIOB->MODER |= (1<<27) | (1<<23); 
GPIOB->MODER &= ~((1<<22) | (1<<26));
//set pb11 & pb13 to open drain for i2c slave
GPIOB->OTYPER |= ((1<<11) | (1<<13));
GPIOB->OSPEEDR &= ~((1<<26) | (1<<27) | (1<<22) | (1<<23));
GPIOB->PUPDR &= ~((1<<26) | (1<<27) | (1<<22) | (1<<23));
//confgiguring i2c sda,, slc alternate modes
GPIOB->AFR[1] |= (1<<12) | (1<<20) | (1<<22);
GPIOB->AFR[1] &= ~((0<<15) | (0<<13) | (0<<14) | (0<<21) | (0<<22) | (0<<23));
//set o/p pins high and reset led pins
GPIOB->ODR |= (1<<14);
GPIOC->ODR |= (1<<0);
GPIOC->ODR &= ~((1<<6) | (1<<7) | (1<<8) | (1<<9));
}
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
