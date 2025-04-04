/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Artur Moreno e Aryel Pereira
 * @brief          : Main program body
 * @date		   : 2025/03/31
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

#include <stdint.h>
#include <stm32h7a3xxq.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


/*					BLOCO 1
Definição da estrutura otimizada, aqui definimos as variáveis  e identificadores do código.
Nessa parte do código definimos as sequências de 8 variações para 5 grupos.
Para as cores utilizamos a seguinte definição binária que foi definida como "estado_leds":

    0b00000001: Apagar todos os canais do LED.
    0b00000010: Acender o canal vermelho.
    0b00000100: Acender o canal verde.
    0b00001000: Acender o canal azul.
    0b00010000: Acender os canais vermelho e verde (amarelo).
    0b00100000: Acender os canais verde e azul (ciano).
    0b01000000: Acender os canais azul e vermelho (magenta).
    0b10000000: Acender todos os canais (branco).
*/


typedef struct {
char identificador[50]; // Artur e Aryel
uint8_t flagEV;    // Indica ocorrência de um novo evento
uint32_t contador;        // Número de acionamentos do botão
enum COR {PRETO, VERMELHO, VERDE, AZUL, AMARELO, CIANO, MAGENTA, BRANCO} cor_led; // Cor atual do LED
uint8_t estado_leds[5][8];      //Matriz de padrões de cores predefinidos
} Perifericos_t;

Perifericos_t LEDInterativo = {
	.identificador = "Artur e Aryel",
	.estado_leds = {
			{0b10, 0b1000, 0b10, 0b1000, 0b10, 0b1000, 0b10, 0b1000}, // Vermelho e azul
			{0b100, 0b1000, 0b100, 0b1000, 0b100, 0b1000, 0b100, 0b1000}, // Verde e azul
			{0b1000000, 0b100000, 0b1000000, 0b100000, 0b1000000, 0b100000, 0b1000000, 0b100000}, // Magenta e ciano
			{0b10, 0b1, 0b10, 0b1, 0b10, 0b1, 0b10, 0b1}, // Vermelho e preto
			{0b1, 0b10, 0b100, 0b1000, 0b10000, 0b100000, 0b1000000, 0b10000000} // Todas as cores
	},
	.contador = 0,
	.cor_led = VERMELHO, // Primeira cor
	.flagEV = 1 // Começa o ciclo de cores assim que inicia o programa
};

//Variáveis auxiliares
uint8_t i = 0; // Define a sequência de cores
uint8_t j = 0; // Define a cor dentro da sequência


/*                 BLOCO 2
 *  Nessa parte do código configuramos duas interrupções,
 *  uma que é ativada pelo botão e que é responsável
 *  por mudar o grupo que queremos ativar
 *  e a outra que é ativada automaticamente pelas configurações
 *  definidas pelo TIM6 e é responsável por mudar as
 *  cores dentro de cada sequência em determinado tempo,
 *  no nosso caso, em 500ms  */


// Rotina de Interrupção para mudar as sequências
void EXTI15_10_IRQHandler(void) {
	if (EXTI->PR1 & EXTI_PR1_PR13_Msk) {
		EXTI->PR1 |= EXTI_PR1_PR13_Msk; // Limpa a flag de interrupção
		LEDInterativo.contador++;  //Atualiza o contador para que possamos saber em que sequência está
		i = LEDInterativo.contador % 5; //Módulo 5 para que o contador vá somente até a quantidade de grupos que queremos
		LEDInterativo.flagEV = 1;
		j = 0; // Garante que será a primeira cor da sequência
	}
}

// Rotina de Interrupção para o timer
void TIM6_DAC1_IRQHandler(void) {
	if (TIM6->SR & TIM_SR_UIF) { // Testa a flag de atualizacao
		// Limpa a flag de atualizacao escrevendo ZERO
		TIM6->SR &= ~TIM_SR_UIF;
		j = (j + 1) % 8;
		LEDInterativo.flagEV = 1;
	}
}


/*                 BLOCO 3
 *  Nesta parte dizemos para o microcontrolador como ativar
 *  cada cor na linguagem que ele entende que é em binário.
 *  Para facilitar o processo nos utilizamos os recursos das
 *  máscaras.
 *
 * O uso das máscaras facilita a compreensão, já que o nome atribuído a cada máscara
 * representa a ação que ela realiza em alto nível. Por exemplo,
 * mais a frente no código teremos a máscara RCC_AHB4ENR_GPIOCEN_Msk
 * que realiza a operação que habilita (enable) o clock RCC_AHB4ENR para as portas GPIOC.
 * A operação realizada (0x1UL << RCC_AHB4ENR_GPIOCEN_Pos) é bem menos clara.
 * Máscaras são usadas por todo o código para facilitar a compreensão das operações.
 */


// Atualização dos LEDs
void AtualizaLEDs(void) {
	switch (LEDInterativo.estado_leds[i][j]) {
		case 0b00000001: // Preto: todos apagados
			GPIOD->ODR = (GPIOD->ODR & ~(GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk | GPIO_ODR_OD15_Msk));
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
			GPIOD->ODR = (GPIOD->ODR & ~(GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk)) | GPIO_ODR_OD15_Msk;
			LEDInterativo.cor_led = AZUL;
			break;
		case 0b00010000: // Amarelo: R = G = 1, B = 0
			GPIOD->ODR = (GPIOD->ODR &~(GPIO_ODR_OD15_Msk)) | GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk;
			LEDInterativo.cor_led = AMARELO;
			break;
		case 0b00100000: // Ciano: G = B = 1, R = 0
			GPIOD->ODR = (GPIOD->ODR & ~(GPIO_ODR_OD12_Msk)) | GPIO_ODR_OD14_Msk | GPIO_ODR_OD15_Msk;
			LEDInterativo.cor_led = CIANO;
			break;
		case 0b01000000: // Magenta: R = B = 1, G = 0
			GPIOD->ODR = (GPIOD->ODR & ~(GPIO_ODR_OD14_Msk)) | GPIO_ODR_OD12_Msk | GPIO_ODR_OD15_Msk;
			LEDInterativo.cor_led = MAGENTA;
			break;
		case 0b10000000: // Branco: todos acesos
			GPIOD->ODR = GPIOD->ODR | (GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk| GPIO_ODR_OD15_Msk);
			LEDInterativo.cor_led = BRANCO;
			break;
	}
}


/*                 BLOCO 4
 *  Nesta parte ativamos os protagonistas do nosso código,
 *  os periféricos, como os LEDs e o botão,
 *  o timer que vai ser crucial para nossa interrupção e
 *  configuramos também a prioridade da interrupção
 *  o que é crucial quando um sistema possui mais de uma interrupção, que é o nosso caso,
 *  porque se elas acontecerem ao mesmo tempo o microcontrolador vai saber qual atender primeiro
 *  através da prioridade definida.
 */


int main(void)	{
	// Ativa GPIOD15 (entrada azul do LED RGB) e GPIOC13 (botão de usuário)
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN_Msk | RCC_AHB4ENR_GPIODEN_Msk;
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

	// Inicializa o TIM6
	RCC->APB1LENR |= RCC_APB1LENR_TIM6EN; // clock gating
	TIM6->PSC = 64000 - 1; // prescaler
	TIM6->ARR = 500 - 1; // Auto-reload, modulo de contagem
	TIM6->CNT = 0; // Zerando o contador
	// Inicia a contagem de tempo
	TIM6->CR1 |= TIM_CR1_CEN;
	TIM6->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM6_DAC_IRQn, 1);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);


/*                 BLOCO 5
 *  O bloco a seguir é o que o microcontrolador ficará rodando até que aconteça uma
 *  interrupção. É simplesmente um loop infinito que apenas ativa
 *  a configuração inicial dos leds e mantém a flag de interrupção
 *  desativada, pois ela só será ativada quando a interrupção ocorrer
*/


	for(;;){
		if(LEDInterativo.flagEV){
			AtualizaLEDs();
			LEDInterativo.flagEV = 0;
		}
	}
};
