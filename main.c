/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @author			: Artur Moreno, Aryel Barbosa
  * @file           : main.c
  * @brief          : Main program body
  * @date			: 2025-04-22
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PWM_PInit(void);
void GPIO_PInit(void);
void Iniciar(void);

typedef struct {
	uint8_t pwm;
	uint8_t dir;
	uint8_t increase;
	uint8_t flag;
} ControlePWM;

ControlePWM motor = {
	.pwm = 0,
	.dir = 1,
	.increase = 1,
	.flag = 0
};

void leCicloTrabalho(uint8_t *dir, uint8_t *pwm, uint8_t *increase){
	*dir = motor.dir;
	*pwm = motor.pwm;
	*increase = motor.increase;
	return;
};


/*
 * @brief Atualiza o valor da flag do motor
 * @param Novo valor da flag
 * @returns None
 * */
void atualizaFlag(uint8_t estado_atual){
	motor.flag = estado_atual;
};


/*
 * @brief Le o valor da flag
 * @param None
 * @returns Valor da flag atual
 * */
uint8_t leFlag(){
	return motor.flag;
}

void EXTI15_10_IRQHandler(void);
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
	//	uint8_t increase_motor = 1; // 1 - cresce velocidade; 0 - reduz velocidade
	/*
	 * @brief esturura para armazenar as informacoes de operacao do motor
	 * */


	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	/* USER CODE BEGIN 2 */
	PWM_PInit();
	GPIO_PInit();
	Iniciar();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//Ting: Quando ativado mecanismo de interrupcao, nao eh mais
		//necessario fazer polling do estado do botao ...
		//	  while(!(GPIOC->IDR & GPIO_IDR_ID13));
		if (leFlag()) {
			leCicloTrabalho(&motor.dir, &motor.pwm, &motor.increase);

			// Atualiza a direcao do motor
			if (motor.dir){// Altera a direção do motor via ponte H: BR2 = 0, BS13 = 1
				  GPIOB->BSRR = GPIO_BSRR_BR2; // P1 da ponte em nivel baixo
				  GPIOD->BSRR = GPIO_BSRR_BS13; // N1 da ponte em nivel alto
				  motor.dir = 0;
			} else {// Altera de volta via ponte H: BS2 = 1, BR13 = 0
				  GPIOB->BSRR = GPIO_BSRR_BS2; // P1 da ponte em nivel alto
				  GPIOD->BSRR = GPIO_BSRR_BR13; // N1 da ponte em nivel baixo
				  motor.dir = 1;
			}

			// Atualiza a velocidade do motor
			if (motor.increase) {
				if (motor.pwm < 100) {
					motor.pwm += 10;
				} else {
					motor.increase = 0;
					motor.pwm -= 10;
				}
			} else {
				if (motor.pwm > 0) {
					motor.pwm -= 10;
				} else {
					motor.increase = 1;
					motor.pwm += 10;
				}
			}

			// Atualiza o duty cycle dos timers
			TIM3->CCR2 = motor.pwm; // PWM do motor aumenta/diminui 10%
			TIM4->CCR1 = motor.pwm; // PWM do LED vermelho diminui/aumenta em 10%
			TIM4->CCR3 = motor.pwm; // PWM do LED verde diminui/aumenta em 10%
			atualizaFlag(0); // sistema atualizado em funcao dos novos valores
		}
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

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
* @brief: Inicialização do PWM para o motor e para os LEDs
* @param None
* @returns None
*/
void PWM_PInit(void) {
	// Habilitar o clock para o Timer 3 (alimenta o motor)
	RCC->APB1LENR |= RCC_APB1LENR_TIM3EN_Msk;

	// Ting: Resetar os registradores de TIM3
	TIM3->EGR |= TIM_EGR_UG_Msk;
	while (TIM3->EGR & TIM_EGR_UG);

	// Configurar PSC e ARR
	//Ting: 1/5000 = (TIMx_ARR+1)/(96000000/(TIMx_PSC+1)) => TIMx_PSC+1 = 96000000/(5000*100) = 192 => TMIx_PSC = (192 -1)
	TIM3->PSC = 9600 - 1; // Prescaler
	TIM3->ARR = 100 - 1; // Período
	TIM3->CCR2 = 0; // Duty cycle 0%

	// Configuração do modo PWM para o motor
	TIM3->CCMR1 &= ~(TIM_CCMR1_CC2S_Msk); // Direção do canal Output
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk); // Modo PWM 1
	TIM3->CCMR1 |= (TIM_CCMR1_OC2M_1 |
					TIM_CCMR1_OC2M_2 |
					TIM_CCMR1_OC2M_3); // Modo PWM 1 assimétrico
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE; // Habilita preload para o canal 2
	TIM3->CCMR1 &= ~TIM_CCMR1_OC2FE; // Desabilita modo rápido
	//Direcao de contagem
	TIM3->CR1 &= ~TIM_CR1_DIR_Msk;

	// Habilitar o clock para o Timer 4 (alimenta os LEDs)
	RCC->APB1LENR |= RCC_APB1LENR_TIM4EN_Msk;
	// Ting: Resetar os registradores de TIM4
	TIM4->EGR |= TIM_EGR_UG_Msk;
	while (TIM4->EGR & TIM_EGR_UG);

	// Configurar PSC e ARR
	//Ting: seguindo a especificacao
	TIM3->PSC = 192 - 1; // Prescaler
	TIM4->ARR = 100 - 1; // Período
	TIM4->CCR1 = 0; // Duty cycle 0% para LED vermelho
	TIM4->CCR3 = 0; // Duty cycle 0% para LED verde

	// Configuração do modo PWM para o LED vermelho
	TIM4->CCMR1 &= ~TIM_CCMR1_CC1S_Msk; // Direção do canal Output
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk); // Modo PWM 1
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_1 |
					TIM_CCMR1_OC1M_2 |
					TIM_CCMR1_OC1M_3); // Modo PWM 1 assimétrico
	TIM4->CCMR1 |= TIM_CCMR1_OC1PE; // Habilita preload para canal 1
	TIM4->CCMR1 &= ~TIM_CCMR1_OC1FE; // Desabilita modo rápido

	// Configuração do modo PWM para o LED verde
	TIM4->CCMR2 &= ~TIM_CCMR2_CC3S_Msk; // Direção do canal Output
	//Ting: Mesmo modo do motor ...
	TIM4->CCMR2 &= ~TIM_CCMR2_OC3M_Msk; // Modo PWM 1
	TIM4->CCMR2 |= (TIM_CCMR2_OC3M_1 |
					TIM_CCMR2_OC3M_2 |
					TIM_CCMR2_OC3M_3); // Modo PWM 1 assimétrico
	TIM4->CCMR2 |= TIM_CCMR2_OC3PE; // Habilita preload para canal 1
	TIM4->CCMR2 &= ~TIM_CCMR2_OC3FE; // Desabilita modo rápido
	//Direcao de contagem
	TIM4->CR1 &= ~TIM_CR1_DIR_Msk;
}

/**
* @brief Inicialização de PC7, PD12 e PD14
* @param None
* @returns None
*/
void GPIO_PInit(void) {
	// Habilitar o clock para o GPIOB, GPIOC e GPIOD
	RCC->AHB4ENR |= (RCC_AHB4ENR_GPIOBEN |
					RCC_AHB4ENR_GPIOCEN |
					RCC_AHB4ENR_GPIODEN);

	// Motor
	// Configurar o pino correspondente ao TIM3_CH2 (PC7)
	GPIOC->MODER &= ~(GPIO_MODER_MODE7_Msk); // Limpar modos
	GPIOC->MODER |= GPIO_MODER_MODE7_1; // Modo alternativo
	// Selecionar a função alternativa para TIM3_CH2 (AF2)
	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL7); // AF2 para PC7
	GPIOC->AFR[0] |= (GPIO_AFRL_AFSEL7_1); // AF2 para PC7
	// Configurar o pino correspondente a ponte H
	GPIOB->MODER &= ~(GPIO_MODER_MODE2_Msk); // Limpar modos
	GPIOB->MODER |= GPIO_MODER_MODE2_0; // GP Output
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT2_Msk; // Push-pull
	GPIOD->MODER &= ~(GPIO_MODER_MODE13_Msk); // Limpar modos
	GPIOD->MODER |= GPIO_MODER_MODE13_0; // GP Output
	GPIOD->OTYPER &= ~GPIO_OTYPER_OT13_Msk; // Push-pull

	// PD12 (LED vermelho)
	// Configurar o pino correspondente ao TIM4_CH1 (PD12)
	GPIOD->MODER &= ~(GPIO_MODER_MODE12_Msk); // Limpar modos
	GPIOD->MODER |= GPIO_MODER_MODE12_1; // Modo alternativo
	// Selecionar a função alternativa para TIM4_CH1 (AF2)
	GPIOD->AFR[1] &= ~(GPIO_AFRH_AFSEL12); // AF2 para PD12
	GPIOD->AFR[1] |= GPIO_AFRH_AFSEL12_1; // AF2 para PD12

	// PD14 (LED verde)
	// Configurar o pino correspondente ao TIM4_CH1 (PD12)
	GPIOD->MODER &= ~(GPIO_MODER_MODE14_Msk); // Limpar modos
	GPIOD->MODER |= GPIO_MODER_MODE14_1; // Modo alternativo
	// Selecionar a função alternativa para TIM4_CH1 (AF2)
	GPIOD->AFR[1] &= ~(GPIO_AFRH_AFSEL14); // AF2 para PD12
	GPIOD->AFR[1] |= GPIO_AFRH_AFSEL14_1; // AF2 para PD12

	// Configurar o botao azul
	// Configurar o pino correspondente a ponte H
	GPIOC -> MODER &= ~(GPIO_MODER_MODE13_Msk); // Limpar modos (Ting: equivale ao modo de entrada)

	// Ativa o clock para o periférico SYSCFG
	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN_Msk;
	// Configura a linha de interrupção EXTI13 para ser acionada pelo pino PC13
	SYSCFG->EXTICR[3]&= ~SYSCFG_EXTICR4_EXTI13_Msk;
	SYSCFG->EXTICR[3]|=SYSCFG_EXTICR4_EXTI13_PC;
	//Ativar a borda de subida e destivar borda de descida
	EXTI->RTSR1 |=EXTI_RTSR1_TR13_Msk;
	EXTI->FTSR1 &=~EXTI_FTSR1_TR13_Msk;

	// Habilita a interrupção na linha EXTI13, permitindo que eventos nela gerem uma IRQ
	EXTI->IMR1|=EXTI_IMR1_IM13_Msk;

	// Define prioridade 6 para a interrupção EXTI15_10, que é a utilizada pelo PC13
	NVIC_SetPriority(EXTI15_10_IRQn, 6);
	// Habilita a interrupção EXTI15_10 no NVIC
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}


/*
 * @brief Habilita a ponte H e os timers para o motor e para os LEDs
 * @param None
 * @returns None
 * */
void Iniciar(void) {
	GPIOB->BSRR = GPIO_BSRR_BS2_Msk; // P1 da ponte H em nivel alto
	GPIOD->BSRR = GPIO_BSRR_BR13_Msk; // N1 da ponte em nivel baixo
	TIM3->CCER |= TIM_CCER_CC2E_Msk; // Habilitar saída do canal 2
	TIM4->CCER |= TIM_CCER_CC1E_Msk; // Habilitar saída do canal 1
	TIM4->CCER |= TIM_CCER_CC3E_Msk; // Habilitar saída do canal 3
	// Habilitar o Timer
	TIM3->CR1 |= TIM_CR1_CEN_Msk; // Iniciar o timer para PWM do motor
	TIM4->CR1 |= TIM_CR1_CEN_Msk; // Iniciar o timer para PWM dos LEDs
}


/*
 * @brief Fornece os valores do ciclo de trabalho do motor, necessario para integrar com a biblioteca
 * @param Ponteiro para direcao, ponteiro para pwm e ponteiro para increase
 * @returns None
 * */



/*
 * @brief Atualiza a flag do motor quando ha uma interrupcao pelo botao
 * @param None
 * @returns None
 * */
void EXTI15_10_IRQHandler(void) {
	if (EXTI->PR1 & EXTI_PR1_PR13_Msk) {
			EXTI->PR1 |= EXTI_PR1_PR13_Msk;
			atualizaFlag(1);
}};




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



