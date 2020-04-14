#include "main.h"

void SystemClock_Config(void);
void Button_UpDown_Init(void);
void Button_OpenClose_Init(void);
void Motor_Init(void);
void delay(float);
void ADC_Init(void);
int ADC_Read(void);

int main(void)
{
  int ADC_Val;
  HAL_Init();
  SystemClock_Config();
  Button_UpDown_Init();
  Button_OpenClose_Init();
  Motor_Init();
  ADC_Init();

  TIM2->CCR3 = 50;

  while (1)
  {
	  TIM2->CCR1 = 0;
	  TIM2->CCR2 = 0;
	  ADC_Val = ADC_Read();
  }
  /* USER CODE END 3 */
}

void ADC_Init(void)
{
	GPIOB->MODER |= (3 << (2 * 4));			//Set PB4 to analog

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;		//Enable ADC in RCC

	RCC->CR2 |= RCC_CR2_HSI14ON;

	while(!(RCC->CR2 & RCC_CR2_HSI14RDY));

	ADC1->CR |= ADC_CR_ADEN;

	while(!(ADC1->ISR & ADC_ISR_ADRDY));
	while((ADC1->CR & ADC_CR_ADSTART));
}

int ADC_Read(void)
{
	ADC1->CHSELR = 0;
	ADC1->CHSELR |= 1 << 9;
	while(!(ADC1->ISR & ADC_ISR_ADRDY));
	ADC1->CR |= ADC_CR_ADSTART;
	while(!(ADC1->ISR & ADC_ISR_EOC));
	int x = ADC1->DR;
	return x;
}

void Motor_Init(void)
{
	/* Previously used:
	   	   TIM2->CH1 for motor up (PA5)
		   TIM2->CH2 for motor down (PA1)
	       TIM2->CH3 for motor open/close (PA2)
	       TIM2->CH4 for LED (PA3)
	 */

	/* Currently used:
	 	 PA0: Motor Up (TIM2->CH1)
	 	 PA1: Motor Down (TIM2->CH2)
	 	 PA2: Motor Open (TIM3->CH3)
	 	 PA3: Motor Close (TIM3->CH4l8
	 */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;


	GPIOA->MODER &= ~(3 << (2 * 0));	//Clear Moder 0
	GPIOA->MODER &= ~(3 << (2 * 1));	//Clear Moder 1
	GPIOA->MODER &= ~(3 << (2 * 2));	//Clear Moder 2
	GPIOA->MODER &= ~(3 << (2 * 3));	//Clear Moder 3

	GPIOA->MODER |= (2 << (2 * 0));		//Set PA5 to Alternate Function Mode
	GPIOA->MODER |= (2 << (2 * 1));		//Set PA1 Alternative Function Mode
	GPIOA->MODER |= (2 << (2 * 2));		//Set PA2 to Alternative Function Mode
	GPIOA->MODER |= (2 << (2 * 3));		//Set PA3 to Alternative Function Mode

	GPIOA->AFR[0] &= ~(0xF << (4 * 0));		//Clear AF5
	GPIOA->AFR[0] &= ~(0xF << (4 * 1));		//Clear AF3
	GPIOA->AFR[0] &= ~(0xF << (4 * 2));		//Clear AF2
	GPIOA->AFR[0] &= ~(0xF << (4 * 3));		//Clear AF1

	GPIOA->AFR[0] |= (2 << (4 * 0));	//Set to PA5 to AF1
	GPIOA->AFR[0] |= (2 << (4 * 1));	//Set PA1 to AF1
	GPIOA->AFR[0] |= (2 << (4 * 2));	//Set PA2 to AF1
	GPIOA->AFR[0] |= (2 << (4 * 3));	//Set PA3 to AF1

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;		//Enable Timer2 in RCC
	TIM2->PSC = 480 - 1;
	TIM2->ARR = 100 - 1;

	TIM2->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;	//TIM2->CH1
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;	//TIM2->CH2
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;	//TIM2->CH3
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;	//TIM2->CH4


	TIM2->CCER |= TIM_CCER_CC1E;	//Enable output in capture/control register 1
	TIM2->CCER |= TIM_CCER_CC2E;	//Enable output in capture/control register 2
	TIM2->CCER |= TIM_CCER_CC3E;
	TIM2->CCER |= TIM_CCER_CC4E;

	TIM2->CR1 |= TIM_CR1_CEN;		//Enable timer counter

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

	return;
}

void Button_OpenClose_Init(void)
{
	/*Previously used:
			PC6 to manually operate blinds up	(TIM3->CH1)
			PC7 to manually operate blinds down (TIM3->CH2)
			PC8 to manually operate blinds open (TIM3->CH3)
			PC9 to manually operate blinds close (TIM3->CH4)
		*/

		/*Currently used:
		 	 PA6 to manually operate blinds up (TIM3->CH1)
		 	 PA7 to manually operate blinds down (TIM3->CH2)
		 	 PB0 to manually operate blinds open (TIM3->CH3)
		 	 PB1 to manually operate blinds close (TIM3->CH4)


		 */

		//Timer
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

		GPIOB->MODER &= ~(3 << (2 * 0));	//Clear PB0
		GPIOB->MODER &= ~(3 << (2 * 1));	//Clear PB1

		GPIOB->MODER |= (2 << (2 * 0));		//Set PB0 to AF
		GPIOB->MODER |= (2 << (2 * 1));		//Set PB1 to AF

		GPIOB->AFR[0] &= ~(0xF << (4 * 0));	//Clear AFR[0]
		GPIOB->AFR[0] &= ~(0xF << (4 * 1));	//Clear AFR[1]

		GPIOB->AFR[0] |= (1 << (4 * 0));	//Set AFR[6] to AF1
		GPIOB->AFR[0] |= (1 << (4 * 1));	//Set AFR[7] to AF1

		GPIOB->PUPDR &= ~(3 << (2 * 0));	//Clear PUPDR 6
		GPIOB->PUPDR &= ~(3 << (2 * 1));	//Clear PUPDR 7

		GPIOB->PUPDR |= (2 << (2 * 0));		//Set Pull-Down
		GPIOB->PUPDR |= (2 << (2 * 1));		//Set Pull-Down

		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;	//Enable Timer3 in RCC

		TIM3->PSC = 1 - 1;
		TIM3->ARR = 0xFFFFFFFF;

		TIM3->CCMR2 &= ~TIM_CCMR2_CC3S;
		TIM3->CCMR2 &= ~TIM_CCMR2_CC4S;

		TIM3->CCMR2 |= TIM_CCMR2_CC3S_0;
		TIM3->CCMR2 |= TIM_CCMR2_CC4S_0;

		TIM3->CCER &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
		TIM3->CCER &= ~(TIM_CCER_CC4P | TIM_CCER_CC4NP);

		TIM3->CCER |= TIM_CCER_CC3E;
		TIM3->CCER |= TIM_CCER_CC4E;

		TIM3->DIER |= TIM_DIER_CC3IE;
		TIM3->DIER |= TIM_DIER_CC4IE;

		TIM3->CCMR2 |= TIM_CCMR2_IC3F_3 | TIM_CCMR2_IC3F_2 | TIM_CCMR2_IC3F_1 | TIM_CCMR2_IC3F_0;
		TIM3->CCMR2 |= TIM_CCMR2_IC4F_3 | TIM_CCMR2_IC4F_2 | TIM_CCMR2_IC4F_1 | TIM_CCMR2_IC4F_0;

}

void Button_UpDown_Init(void)
{
	/*Previously used:
		PC6 to manually operate blinds up	(TIM3->CH1)
		PC7 to manually operate blinds down (TIM3->CH2)
		PC8 to manually operate blinds open (TIM3->CH3)
		PC9 to manually operate blinds close (TIM3->CH4)
	*/

	/*Currently used:
	 	 PA6 to manually operate blinds up (TIM3->CH1)
	 	 PA7 to manually operate blinds down (TIM3->CH2)
	 	 PB0 to manually operate blinds open (TIM3->CH3)
	 	 PB1 to manually operate blinds close (TIM3->CH4)


	 */

	//Timer
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER &= ~(3 << (2 * 6));	//Clear PA6
	GPIOA->MODER &= ~(3 << (2 * 7));	//Clear PA7

	GPIOA->MODER |= (2 << (2 * 6));		//Set PA6 to AF
	GPIOA->MODER |= (2 << (2 * 7));		//Set PA7 to AF

	GPIOA->AFR[0] &= ~(0xF << (4 * 6));	//Clear AFR[6]
	GPIOA->AFR[0] &= ~(0xF << (4 * 7));	//Clear AFR[7]

	GPIOA->AFR[0] |= (1 << (4 * 6));	//Set AFR[6] to AF1
	GPIOA->AFR[0] |= (1 << (4 * 7));	//Set AFR[7] to AF1

	GPIOA->PUPDR &= ~(3 << (2 * 6));	//Clear PUPDR 6
	GPIOA->PUPDR &= ~(3 << (2 * 7));	//Clear PUPDR 7

	GPIOA->PUPDR |= (2 << (2 * 6));		//Set Pull-Down
	GPIOA->PUPDR |= (2 << (2 * 7));		//Set Pull-Down

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;	//Enable Timer3 in RCC

	TIM3->PSC = 1 - 1;
	TIM3->ARR = 0xFFFFFFFF;

	TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM3->CCMR1 &= ~TIM_CCMR1_CC2S;

	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;
	TIM3->CCMR1 |= TIM_CCMR1_CC2S_0;

	TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
	TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2NP);

	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER |= TIM_CCER_CC2E;

	TIM3->DIER |= TIM_DIER_CC1IE;
	TIM3->DIER |= TIM_DIER_CC2IE;

	TIM3->CR1 |= (2 << 8);	//Set clock div4

	NVIC->ISER[0] = 1 << TIM3_IRQn;

	TIM3->CCMR1 |= TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0;
	TIM3->CCMR1 |= TIM_CCMR1_IC2F_3 | TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_0;

}

void TIM3_IRQHandler()
{
	if((TIM3->SR & TIM_SR_UIF) != 0)
	{
		TIM3->SR &= ~1;
		return;
	}

	if(TIM3->SR & TIM_SR_CC1IF)
	{
		TIM2->CCR1 = 100;
		int __attribute((unused)) useless;
		useless = TIM3->CCR1;
		//return;
	}
	if(TIM3->SR & TIM_SR_CC2IF)
	{
		TIM2->CCR2 = 100;
		int __attribute((unused)) useless;
		useless = TIM3->CCR2;
	}
	if(TIM3->SR & TIM_SR_CC3IF)		//Blinds open
	{
		while((GPIOB->IDR & (1 << 0)) != 0)
		{
			delay(2);
			if(TIM2->CCR3 < 100)
			{
				TIM2->CCR3 += 10;
			}
		}
		int __attribute((unused)) useless;
		useless = TIM3->CCR3;
	}
	if(TIM3->SR & TIM_SR_CC4IF)		//Blinds Close
	{
		while((GPIOB->IDR & (1 << 1)) != 0)
		{
			delay(2);
			if(TIM2->CCR3 > 0)
			{
				TIM2->CCR3 -= 10;
			}
		}
		int __attribute((unused)) useless;
		useless = TIM3->CCR4;
	}

	while((GPIOA->IDR & (1 << 6)) != 0);		//Wait for button to stop being pressed
	while((GPIOA->IDR & (1 << 7)) != 0);
	while((GPIOB->IDR & (1 << 0)) != 0);
	while((GPIOB->IDR & (1 << 1)) != 0);

	int __attribute((unused)) useless;
	useless = TIM3->CCR1;
	useless = TIM3->CCR2;
	useless = TIM3->CCR3;
	useless = TIM3->CCR4;
	return;
}

void delay(float val)
{
	int i, max = (int)(val * 50000);
	for(i = 0; i < max; i++);
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
