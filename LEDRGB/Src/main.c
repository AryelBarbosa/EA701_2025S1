#include "stm32h7xx.h"

// Definição da estrutura otimizada
typedef struct {
    char identificador[50];  // Nome do proprietário
    uint32_t contador;       // Número de acionamentos do botão
    uint8_t estado_botao;    // Indica se há evento de pressionamento
    uint8_t estado_leds;     // Controle dos estados do LED RGB
    enum COR {PRETO, VERMELHO, VERDE, AZUL, AMARELO, CIANO, MAGENTA, BRANCO} cor_led;
} Perifericos_t;

// Instância global da estrutura
Perifericos_t LEDInterativo = { .estado_leds = 0b00000001, .estado_botao = 0, .contador = 0, .cor_led = PRETO };

// Configuração de pinos
void ConfigurarGPIO(void) {
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOCEN; // Habilita GPIOC e GPIOD

    // Configuração dos LEDs (PD12, PD14, PD15) como saída
    GPIOD->MODER &= ~(GPIO_MODER_MODE12_Msk | GPIO_MODER_MODE14_Msk | GPIO_MODER_MODE15_Msk);
    GPIOD->MODER |= (GPIO_MODER_MODE12_0 | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0);

    // Configuração do botão (PC13) como entrada com pull-up
    GPIOC->MODER &= ~GPIO_MODER_MODE13_Msk;
    GPIOC->PUPDR |= GPIO_PUPDR_PUPD13_0;
}

// Configuração da interrupção
void ConfigurarInterrupcao(void) {
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN; // Habilita SYSCFG
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; // Conectar PC13 à EXTI13

    EXTI->RTSR1 |= EXTI_RTSR1_TR13_Msk; // Habilita interrupção por borda de subida
    EXTI->FTSR1 &= ~EXTI_FTSR1_TR13_Msk; // Desabilita borda de descida

    EXTI->IMR1 |= EXTI_IMR1_IM13_Msk; // Habilita interrupção para EXTI13

    NVIC_SetPriority(EXTI15_10_IRQn, 6); // Configura prioridade da interrupção
    NVIC_EnableIRQ(EXTI15_10_IRQn); // Habilita interrupção no NVIC
}

// Rotina de Interrupção
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR1 & EXTI_PR1_PR13_Msk) {
        EXTI->PR1 |= EXTI_PR1_PR13_Msk; // Limpa a flag de interrupção

        LEDInterativo.estado_leds = (LEDInterativo.estado_leds << 1) | ((LEDInterativo.estado_leds >> 7) & 1);
        LEDInterativo.contador++;
        LEDInterativo.estado_botao = 1;
    }
}

// Atualização dos LEDs
void AtualizarLEDs(void) {
    if (LEDInterativo.estado_botao) {
        switch (LEDInterativo.estado_leds) {
            case 0b00000001:
                GPIOD->ODR &= ~(GPIO_ODR_OD12_Msk | GPIO_ODR_OD14_Msk | GPIO_ODR_OD15_Msk);
                LEDInterativo.cor_led = PRETO;
                break;
            case 0b00000010:
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

// Função Principal
int main(void) {
    ConfigurarGPIO();
    ConfigurarInterrupcao();

    while (1) {
        AtualizarLEDs();
    }
}
