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
	#include <core_cm7.h>
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
    char identificador[50];  // Nome do proprietário
    uint8_t estado_botao;    // Indica se há evento de pressionamento
    uint8_t estado_leds;     // Controle dos estados do LED RGB
    uint32_t contador;       // Número de acionamentos do botão
    enum COR {PRETO, VERMELHO, VERDE, AZUL, AMARELO, CIANO, MAGENTA, BRANCO} cor_led;
} Perifericos_t;

// Instância global da estrutura
Perifericos_t LEDInterativo = {
		.identificador = "Artur e Aryel",
		.estado_leds = 0b00000001,
		.estado_botao = 0,
		.contador = 0,
		.cor_led = PRETO };

// Rotina de Interrupção
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR1 & EXTI_PR1_PR13_Msk) {
        EXTI->PR1 |= EXTI_PR1_PR13_Msk; // Limpa a flag de interrupção
        AtualizarLEDs();
        LEDInterativo.estado_leds = (LEDInterativo.estado_leds << 1) | ((LEDInterativo.estado_leds >> 7) & 1);
        LEDInterativo.contador++;
        LEDInterativo.estado_botao = 1;
    }
}

// Atualização dos LEDs
void AtualizarLEDs(void) {
    if (LEDInterativo.estado_botao) {
        switch (LEDInterativo.estado_leds) {
            case 0b00000001: // Preto: todos apagados
                GPIOD->ODR &= ~GPIO_ODR_12_Msk; // Apaga LED vermelho
                GPIOD->ODR &= ~GPIO_ODR_14_Msk; // Apaga LED verde
                GPIOD->ODR &= ~GPIO)ODR_15_Msk; // Apaga LED azul
                LEDInterativo.cor_led = PRETO;
                break;
            case 0b00000010: // Vermelho: R = 1, G = B = 0
                GPIOD->ODR = (GPIOD->ODR & ~(GPIO_ODR_OD14_Msk | GPIO_ODR_OD15_Msk)) | GPIO_ODR_OD12_Msk;
                LEDInterativo.cor_led = VERMELHO;
                break;
            case 0b00000100:
                GPIOD->ODR = (GPIOD->ODR & ~(GPIO_ODR_OD12_Msk | GPIO_ODR_OD15_Msk)) | GPIO_ODR_OD14_Msk;
                LEDInterativo.cor_led = VERDE;
                break;
            case 0b00001000:
                GPIOD->ODR = (GPIOD->ODR & ~(GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk)) | GPIO_ODR_OD15_Msk;
                LEDInterativo.cor_led = AZUL;
                break;
            case 0b00010000:
                GPIOD->ODR = (GPIOD->ODR & ~GPIO_ODR_OD15_Msk) | (GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk);
                LEDInterativo.cor_led = AMARELO;
                break;
            case 0b00100000:
                GPIOD->ODR = (GPIOD->ODR & ~GPIO_ODR_OD12_Msk) | (GPIO_ODR_OD14_Msk | GPIO_ODR_OD15_Msk);
                LEDInterativo.cor_led = CIANO;
                break;
            case 0b01000000:
                GPIOD->ODR = (GPIOD->ODR & ~GPIO_ODR_OD14_Msk) | (GPIO_ODR_OD12_Msk | GPIO_ODR_OD15_Msk);
                LEDInterativo.cor_led = MAGENTA;
                break;
            case 0b10000000:
                GPIOD->ODR |= (GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk | GPIO_ODR_OD15_Msk);
                LEDInterativo.cor_led = BRANCO;
                break;
        }
        LEDInterativo.estado_botao = 0; // Resetar estado do botão
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
	// Ativa IRQ 40 do NVIC com prioridade 6 (macros definidos em core_cm7.h)
	uint8_t prioridade = 6;
	// Monta a palavra para escrever os bits simultaneamente
	uint32_t reg_value;
	reg_value = SCB->AIRCR;
	reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk));
	reg_value = (reg_value | ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | ((0b11 & (uint32_t)0x07UL) << SCB_AIRCR_PRIGROUP_Pos) );
	SCB->AIRCR = reg_value;
	// Seta a prioridade nos bits [7:5] do byte de prioridade no registrador NVIC_IPRm
	NVIC->IP[40]=(uint8_t)((prioridade << (8U-__NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
	// Seta o bit 40%32=(40&0x01) do registrador NVIC_ISERn, onde n = 40/32 = 40<<5.
	NVIC->ISER[(((uint32_t)40) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)40) & 0x1FUL));


//Define de que forma tudo vai começar a rodar, acredito que nesse código será desligando todos os leds
	// Inicializa PD12, PD14 e PD15 apagados
	GPIOD->ODR &= ~GPIO_ODR_OD12_Msk;
	GPIOD->ODR &= ~GPIO_ODR_OD14_Msk;
	GPIOD->ODR &= ~GPIO_ODR_OD15_Msk;
	uint32_t d;
	for(;;) {
		LEDInterativo.identificador;
		LEDInterativo.contador;
		LEDInterativo.estado_botao;
		LEDInterativo.estado_leds;
		for(d = 0; d < 16000000; d++); // O processador somente espera a interrupção do botão
	}
}
