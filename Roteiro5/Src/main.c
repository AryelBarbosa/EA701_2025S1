/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Artur Moreno, Aryel Barbosa - Grupo 11
 * @brief          : Main program body
 * @date		   : 2025-03-17
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

// Inclusão das bibliotecas usadas no projeto

#include <stdint.h>	// Permite o uso de valores inteiros com tamanhos específicos. Por exemplo, usamos uint32_t e uint8_t
#include <stm32h7a3xxq.h>
	/*
	 * As bibliotecas stm32h7a3xxq.h e core_cm7.h são responsáveis por criar máscaras que facilitam a manipulação dos bits dos
	 * registradores do microcontrolador. A abstração de mais alto nível facilita o entendimento do código ao simplificar,
	 * principalmente, expressões de shift de bits.
	 */

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
	#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


// Definição da estrutura otimizada
typedef struct {
char identificador[50]; // Artur e Aryel
uint8_t flagEV;    // Indica ocorrência de um novo evento
uint32_t contador;        // Número de acionamentos do botão
enum COR {PRETO, VERMELHO, VERDE, AZUL, AMARELO, CIANO, MAGENTA, BRANCO} cor_led; // Cor atual do LED
uint8_t estado_leds[5][8];      //Matriz de padrões de cores predefinidos
} Perifericos_t;

// Instância global da estrutura
Perifericos_t LEDInterativo = {
	.identificador = "Artur e Aryel",
	.estado_leds = {
			{0b10,0b100,0b1000000,0b10,0b1},
			{0b1000,0b1000,0b100000,0b1,0b10},
			{0b10,0b100,0b1000000,0b10,0b100},
			{0b1000,0b1000,0b100000,0b1,0b1000},
			{0b10,0b100,0b1000000,0b10,0b10000},
			{0b1000,0b1000,0b100000,0b1,0b100000},
			{0b10,0b100,0b1000000,0b10,0b1000000},
			{0b1000,0b1000,0b100000,0b1,0b10000000}
	},
	.contador = 0,
	.cor_led = VERMELHO,
	.flagEV = 0

};

//Variáveis auxiliares
uint8_t i=0;
uint8_t j=0;

// Rotina de Interrupção para mudar as sequências
void EXTI15_10_IRQHandler(void) {
if (EXTI->PR1 & EXTI_PR1_PR13_Msk) {
	EXTI->PR1 |= EXTI_PR1_PR13_Msk; // Limpa a flag de interrupção
	LEDInterativo.contador++;
	i = LEDInterativo.contador %5;
	LEDInterativo.flagEV = 1;
	j = 0;
}
}

// Rotina de Interrupção para o timer
void TIM6_DAC_IRQHandler(void) {
if (TIM6->SR & TIM_SR_UIF) { // Testa a flag de atualizacao
// Limpa a flag de atualizacao escrevendo ZERO
TIM6->SR &= ~TIM_SR_UIF;
j = (j+1)%8;
LEDInterativo.flagEV = 1;

}

// Atualização dos LEDs
void AtualizarLEDs(void) {
switch (LEDInterativo.estado_leds[i][j]) {
	case 0b00000001: // Preto: todos apagados
		GPIOD->ODR = (GPIO->ODR & ~(GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk | GPIO_ODR_OD15_Msk));
		LEDInterativo.cor_led = PRETO;
		break;
	case 0b00000010: // Vermelho: R = 1, G = B = 0
		GPIOD->ODR = (GPIOD->ODR & ~(GPIO_ODR_OD14_Msk | GPIO_ODR_OD15_Msk)) | GPIO_ODR_OD12_Msk;
		LEDInterativo.cor_led = VERMELHO;
		break;
	case 0b00000100: // Verde: G = 1, R=B=0
		GPIOD->ODR = (GPIOD->ODR & ~(GPIO_ODR_OD12_Msk | GPIO_ODR_OD15_Msk)) | GPIO_ODR_OD14_Msk;
		LEDInterativo.cor_led = VERDE;
		break;
	case 0b00001000: // Azul: R = G = 0, B = 1
		GPIOD->ODR = (GPIO->ODR & ~(GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk)) | GPIO_ODR_OD15_Msk;
		LEDInterativo.cor_led = AZUL;
		break;
	case 0b00010000: // Amarelo: R = G = 1, B = 0
		GPIOD->ODR = (GPIO->ODR &~(GPIO_ODR_OD15_Msk)) | GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk;
		LEDInterativo.cor_led = AMARELO;
		break;
	case 0b00100000: // Ciano: G = B = 1, R = 0
		GPIOD->ODR = (GPIO->ODR & ~(GPIO_ODR_OD12_Msk)) | GPIO_ODR_OD14_Msk | GPIO_ODR_OD15_Msk;
		LEDInterativo.cor_led = CIANO;
		break;
	case 0b01000000: // Magenta: R = B = 1, G = 0
		GPIOD->ODR = (GPIO->ODR & ~(GPIO_ODR_OD14_Msk)) | GPIO_ODR_OD12_Msk | GPIO_ODR_OD15_Msk;
		LEDInterativo.cor_led = MAGENTA;
		break;
	case 0b10000000: // Branco: todos acesos
		GPIOD->ODR = GPIO->ODR | (GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk| GPIO_ODR_OD15_Msk);
		LEDInterativo.cor_led = BRANCO;
		break;
}
}



int main(void)	{
// Ativa GPIOD15 (entrada azul do LED RGB) e GPIOC13 (botão de usuário)
RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN_Msk | RCC_AHB4ENR_GPIODEN_Msk;

/*
* O uso das máscaras facilita a compreensão, já que o nome atribuído a cada máscara representa a ação que
* ela realiza em alto nível. Por exemplo, RCC_AHB4ENR_GPIOCEN_Msk realiza a operação que habilita (enable) o clock
* RCC_AHB4ENR para as portas GPIOC. A operação realizada (0x1UL << RCC_AHB4ENR_GPIOCEN_Pos) é bem menos clara.
* Máscaras são usadas por todo o código para facilitar a compreensão das operações.
*/

//Configuração de cada periférico
// PD12 (LED vermelho) como saida digital
GPIOD->MODER &= ~(GPIO_MODER_MODE12_Msk);
GPIOD->MODER |= GPIO_MODER_MODE12_0;
GPIOD->OTYPER &= ~GPIO_OTYPER_OT12_Msk; // PD12 como push-pull
// PD14 (LED verde) como saida digital
GPIOD->MODER &= ~(GPIO_MODER_MODE14_Msk);
GPIOD->MODER |= GPIO_MODER_MODE14_0;
GPIOD->OTYPER &= ~GPIO_OTYPER_OT14_Msk; // PD14 como push-pull
// PD15 (LED azul) como saida digital
GPIOD->MODER &= ~(GPIO_MODER_MODE15_Msk);
GPIOD->MODER |= GPIO_MODER_MODE15_0;
GPIOD->OTYPER &= ~GPIO_OTYPER_OT15_Msk; // PD15 como push-pull
// PC13 (botão) como entrada digital
GPIOC->MODER &= ~(GPIO_MODER_MODE13_Msk);

//Configuração do SYSCFG
// Ativa o SYSCFG
RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN_Msk;
// Conectar PC13 com EXTI13
SYSCFG->EXTICR[3] &= 0xFFFFFF0F; // Define 8 LSB como 0000 1111
SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; // Define que o registrador SYSCFG_EXTICR3 será conectado à porta PC13

//Configuração do EXTI
// Ativa a borda de subida e desativa borda de descida
EXTI->RTSR1 |= EXTI_RTSR1_TR13_Msk;
EXTI->FTSR1 &= ~EXTI_FTSR1_TR13_Msk;
// Desmascara EXTI13
EXTI->IMR1 |= EXTI_IMR1_IM13_Msk;

//Configuração de prioridade da interrupção
NVIC_SetPriority(EXTI15_10_IRQn, 6);
NVIC_EnableIRQ(EXTI15_10_IRQn);
//Define de que forma tudo vai começar a rodar
// Inicializa PD12, PD14 e PD15 apagados
	GPIOD->ODR &= ~GPIO_ODR_OD12_Msk;
	GPIOD->ODR &= ~GPIO_ODR_OD14_Msk;
	GPIOD->ODR &= ~GPIO_ODR_OD15_Msk;


//Configuração do timer
int main(void)
	{
	//Inicializa Atualizar LEDS
	//Inicializa o TIM6
	RCC->APB1LENR |= RCC_APB1LENR_TIM6EN; // clock gating
	TIM6->PSC = 64000 - 1; // prescaler
	TIM6->ARR = 500 - 1; // Auto-reload, modulo de contagem
	TIM6->CNT = 0; // Zerando o conrador
	// Inicia a contagem de tempo
	TIM6->CR1 |= TIM_CR1_CEN;
	/* Loop forever */
	for(;;){
		if(LEDInterativo.flagEV){
			AtualizaLEDs();
			LEDInterativo.flagEV = 0;
		}
	};
	}
	}
