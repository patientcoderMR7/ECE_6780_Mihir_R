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
//Functions & Global Variable declarations
void Transmit_char(char transmitted);
void Transmit_string(char* string);
void Wait_for_input(void);
void led_character_toggle(void);

volatile char input;
volatile int flag;
volatile char read;
volatile char received, received1, received2;

int main(void)
{
  /* USER CODE BEGIN 1 */
RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
//Enable system clock for USART3 in RCC
RCC->APB1ENR  |= RCC_APB1ENR_USART3EN;
SystemClock_Config();

// Configure the leds and button
GPIOC->MODER |= (1<<12) | (1<<14) | (1<<16) | (1<<18);
GPIOC->MODER &= ~((1<<13) | (1<<15) | (1<<17) | (1<<19));
GPIOC->OTYPER &= ~((1<<6) | (1<<7) | (1<<8) | (1<<9));
GPIOC->OSPEEDR &= ~((1<<12) | (1<<14) | (1<<16) | (1<<18));
GPIOC->PUPDR &= ~((1<<12) | (1<<14) | (1<<16) | (1<<18)| (1<<13) | (1<<15) | (1<<17) | (1<<19));
GPIOA->MODER &= ~((1<<0) | (1<<1));
GPIOC->OSPEEDR &= ~((1<<0) | (1<<1));
GPIOA->PUPDR &= ~((1<<0));
GPIOA->PUPDR |= (1<<1);
	
//Alternate mode
GPIOB->MODER |= (1<<23) | (1<<21);
GPIOB->MODER &= ~((1<<22) | (1<<20));
GPIOB->OTYPER &= ~((1<<10) | (1<<11));
GPIOB->OSPEEDR &= ~((1<<20) | (1<<21) | (1<<22) | (1<<23));
GPIOB->PUPDR &= ~((1<<20) | (1<<21) | (1<<22) | (1<<23));
GPIOB->AFR[1] |= (1<<14) | (1<<10);
GPIOB->AFR[1] &= ~((1<<15) | (1<<13) | (1<<12) | (1<<11)| (1<<9) | (1<<8));

//USART 3 config
//Calculationf for Baud Rate 115200
USART3->BRR = 69;
//Enable Tx & Rx
USART3->CR1 |= (1<<2) | (1<<3);
//Set Enable bit USART3
USART3->CR1 |= (1<<0);
Set Interupt
USART3->CR1 |= (1<<5);
//Enable Interrupt & Set priority
NVIC_EnableIRQ(29);
NVIC_SetPriority(29,1);
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
//Reset pins
GPIOC->ODR &= ~((1<<6) | (1<<7) | (1<<8) | (1<<9));	
//Part 1: Switch case for all LEDs and error message
while (1)
{
	//Part 1 function
	//led_character_toggle();
	//Part 2
	Transmit_string("Please enter input\r\nPress r or b to select one led.\r\n Then press 0,1 or 2 to - On,Off,Toggle\n\r");
	Wait_for_input();
	
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
//part 1 function
void led_character_toggle(){
	//check the status flag and check receive data register bit for not empty
	if (USART3->ISR & (1<<5))
		{
			//read character and store
			received = USART3->RDR;
			switch(received){
				case 'r':
				GPIOC->ODR ^= (1<<6);	
				break;
				case 'b':
				GPIOC->ODR ^= (1<<7);
				break;
				case 'g':
				GPIOC->ODR ^= (1<<9);
				break;
				case 'o':
				GPIOC->ODR ^= (1<<8);
				break;
				default:
				Transmit_string("error\n");
			}					
		}
}
//function for transmitting single character
void Transmit_char(char transmitted)
{
 //check flag
	while((USART3->ISR & (1<<7)) == 0)
	{
	}
	//read character
	USART3->TDR = transmitted;
}
//function for transmitting string
void Transmit_string(char* string)
{
	int i = 0;
	//parse end of string
	while(*string != '\0')
	{
		Transmit_char(*string);
		string++;
	}
	Transmit_char('\r');
}

void Wait_for_input(void)
{
	while (!(USART3->ISR & USART_ISR_RXNE))
    {
    }
    // Read the character
    input = USART3->RDR;
}
//Define Interrupt function for USART3 Interrupt Handler. Implement Part 2 logic for Red & Blue LEDs
void USART3_4_IRQHandler(void)
{
    if (USART3->ISR & USART_ISR_RXNE)
    {
        if (flag == 0) // First character
        {
            received1 = USART3->RDR; // Read first character
            flag = 1; // Set flag to indicate the first character received
        }
        else // Second character
        {
            received2 = USART3->RDR; // Read second character
            flag = 0; // Reset flag for next command
            //
            switch (received1)
            {
                case 'r':
                case 'R':
                    // Command for red LED
                    if (received2 == '0')
                    {
                        // Turn off red LED
                        GPIOC->ODR &= ~(1 << 6);
                        Transmit_string("Turn off red LED\n");
                    }
                    else if (received2 == '1')
                    {
                        // Turn on red LED
                        GPIOC->ODR |= (1 << 6);
                        Transmit_string("Turn on red LED\n");
                    }
                    else if (received2 == '2')
                    {
                        // Toggle red LED
                        GPIOC->ODR ^= (1 << 6);
                        Transmit_string("Toggle red LED\n");
                    }
                    else
                    {
                        // Invalid command
                        Transmit_string("Invalid command for red LED\n");
                    }
                    break;
                case 'b':
                case 'B':
                    // Command for blue LED
                    if (received2 == '0')
                    {
                        // Turn off blue LED
                        GPIOC->ODR &= ~(1 << 7);
                        Transmit_string("Turn off blue LED\n");
                    }
                    else if (received2 == '1')
                    {
                        // Turn on blue LED
                        GPIOC->ODR |= (1 << 7);
                        Transmit_string("Turn on blue LED\n");
                    }
                    else if (received2 == '2')
                    {
                        // Toggle blue LED
                        GPIOC->ODR ^= (1 << 7);
                        Transmit_string("Toggle blue LED\n");
                    }
                    else
                    {
                        // Invalid command
                        Transmit_string("Invalid command for blue LED\n");
                    }
                    break;
                default:
                	// Unknown command
			Transmit_string("\rError\n");
                    	//Transmit_string("Unknown command\n\rError. Please read carefully and give the right input!!\n\r");
			//Transmit_string("Please enter input\r\nPress r or b to select one led.\r\n Then press 0,1 or 2 to - On,Off,Toggle\n\r");
                	break;
            }
        }
    }
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
