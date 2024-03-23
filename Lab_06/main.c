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
void LED_Init(void);
void ADC_Init(void);
uint16_t ADC_Read(void);
void DAC_Init(void);
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
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 clock
	LED_Init();
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
	//Part 2
	DAC_Init();
	// Sine Wave: 8-bit, 32 samples/cycle
	const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	// Triangle Wave: 8-bit, 32 samples/cycle
	const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
	// Sawtooth Wave: 8-bit, 32 samples/cycle
	const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};
	uint8_t index = 0;
  	/* USER CODE END 2 */

  	/* Infinite loop */
  	/* USER CODE BEGIN WHILE */
	//Part 1: ADC
	/*
  	while (1)
  	{
  		//Read ADC value
    		uint16_t adcValue = ADC_Read();  // Turn on/off LEDs based on ADC value
        	if (adcValue >= 10) {
            		GPIOC->ODR |= (1<<6); // Turn on LED connected to pin PC6
        	} 
		else {
            		GPIOC->ODR |= (0<<6); // Turn off LED connected to pin PC6
        	}
        	if (adcValue >= 20) {
            		GPIOC->ODR |= (1<<7); // Turn on LED connected to pin PC7
        	} 
		else {
            		GPIOC->ODR |= (0<<7); // Turn off LED connected to pin PC7
        	}
        	if (adcValue >= 30) {
            		GPIOC->ODR |= (1<<8); // Turn on LED connected to pin PC8
        	} 
		else {
            		GPIOC->ODR |= (0<<8); // Turn off LED connected to pin PC8
        	}
        	if (adcValue >= 40) {
            		GPIOC->ODR |= (1<<9); // Turn on LED connected to pin PC9
        	} 
		else {
            		GPIOC->ODR |= (0<<9); // Turn off LED connected to pin PC9
        	}
    	}
     	*/
	//Part 2: DAC
	while (1) {			
        // Write the next value in the wave-table to the DAC data register
        	DAC->DHR8R1 = triangle_table[index];
		HAL_Delay(1);
        // Increment index for next value
        	index = (index + 1) % 32;		
        }
	
    	/* USER CODE END WHILE */

    	/* USER CODE BEGIN 3 */
  
  	/* USER CODE END 3 */


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
  		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  		{
    			Error_Handler();
  		}
	}
}

/* USER CODE BEGIN 4 */
//function for LED initialization
void LED_Init(void) {
     // Enable GPIOC clock
    
    // Set PC6, PC7, PC8, PC9 as output
    GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
    
    // No pull-up/down resistors for PC6, PC7, PC8, PC9
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7 | GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);

}

//Function for ADC initialization, calibration, and enable
void ADC_Init(void) {
	GPIOC->MODER |= GPIO_MODER_MODER0; // Set PC0 as analog mode
	//set adc 8 bit bit 3&4, continuos conversion mode bit13, hardware trigge disabled - 10,11
	ADC1->CFGR1 |= ((1<<4) | (1<<3)| (1<<13) | (0<<10) | (0<<11));
	ADC1->CHSELR |= (1<<10); //enable input channel 10 for pin 0
  	ADC1->CR |= (0<<0); //clear aden
	ADC1->CFGR1 |= (0<<0); //clear dmaen
		
	//ADC calibration
		if ((ADC1->CR & ADC_CR_ADEN) != 0)
		{
			ADC1->CR |= ADC_CR_ADDIS; //set ADDIS
		}
		while ((ADC1->CR & ADC_CR_ADEN) != 0){};
 
		ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; //clear DMAEN
		ADC1->CR |= ADC_CR_ADCAL; //set ADCAL
	
		while ((ADC1->CR & ADC_CR_ADCAL) != 0){}; 
		if ((ADC1->ISR & ADC_ISR_ADRDY) != 0)
		{
			ADC1->ISR |= ADC_ISR_ADRDY;
		}
		ADC1->CR |= ADC_CR_ADEN; //enable ADCEN
		while ((ADC1->ISR & ADC_ISR_ADRDY) == 0){};
}

// Function to read ADC value
uint16_t ADC_Read(void) {
	ADC1->CR |= ADC_CR_ADSTART; // Start conversion
    
	while (!(ADC1->ISR & ADC_ISR_EOC)){}; // Wait until the conversion is complete
    
	return ADC1->DR; // Return converted value
}

// Function to initialize DAC
void DAC_Init(void) {
	//RCC->APB1ENR |= RCC_APB1ENR_DACEN; // Enable DAC clock
	// Configure GPIO pin PA5 as analog mode (DAC_OUT2)
   	GPIOA->MODER |= GPIO_MODER_MODER5; // Set PA5 as analog mode
	// Set DAC channel 2 to software trigger mode
	DAC->CR |= DAC_CR_TEN2 | DAC_CR_TSEL2_1; // Enable trigger and select software trigger
	DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;
	DAC->CR |= DAC_CR_EN2; // Enable DAC channel 2
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
