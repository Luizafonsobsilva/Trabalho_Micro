/*
 * AP2 - BRUCUTU Sistemas Eletronicos S.A.
 * Teste isolado do modulo ADC.
 *
 * Le dois sensores analogicos e imprime as leituras a cada 500 ms:
 *   - LM35 (temperatura)  -> GP26 / ADC0   (alimentar o LM35 com 5V / VBUS)
 *   - Sensor de umidade   -> GP27 / ADC1   (resistivo analogico, alimentar 3V3)
 *
 * Saida pela USB-CDC (monitor serial do VS Code).
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

// --- Pinos ADC ---
#define PINO_LM35      26   // ADC0
#define PINO_UMIDADE   27   // ADC1
#define CANAL_LM35      0
#define CANAL_UMIDADE   1

// --- Referencia do ADC ---
// O ADC do RP2350 e de 12 bits (0..4095) com referencia de 3.3 V.
#define ADC_VREF       3.3f
#define ADC_RESOLUCAO  4096.0f   // 2^12

// Converte leitura bruta (0..4095) em tensao (volts).
static float adc_para_tensao(uint16_t bruto) {
    return (bruto * ADC_VREF) / ADC_RESOLUCAO;
}

// LM35: 10 mV por grau Celsius -> temp = Vout / 0.01
static float tensao_para_celsius(float tensao) {
    return tensao / 0.01f;
}

int main(void) {
    stdio_init_all();

    // Inicializa o hardware de ADC e habilita os dois pinos.
    adc_init();
    adc_gpio_init(PINO_LM35);
    adc_gpio_init(PINO_UMIDADE);

    // Pequena espera para o monitor serial conectar antes dos primeiros prints.
    sleep_ms(2000);
    printf("=== AP2 Teste ADC - BRUCUTU ===\n");
    printf("LM35 em GP%d (ADC%d), Umidade em GP%d (ADC%d)\n\n",
           PINO_LM35, CANAL_LM35, PINO_UMIDADE, CANAL_UMIDADE);

    while (true) {
        // --- Leitura do LM35 (temperatura) ---
        adc_select_input(CANAL_LM35);
        uint16_t bruto_temp = adc_read();
        float tensao_temp = adc_para_tensao(bruto_temp);
        float temperatura = tensao_para_celsius(tensao_temp);

        // --- Leitura do sensor de umidade ---
        adc_select_input(CANAL_UMIDADE);
        uint16_t bruto_umid = adc_read();
        float tensao_umid = adc_para_tensao(bruto_umid);
        // Percentual relativo simples (0 V = 0%, 3.3 V = 100%).
        // A curva real depende do sensor; calibrar depois.
        float umidade_pct = (tensao_umid / ADC_VREF) * 100.0f;

        printf("TEMP: %5.1f C  (bruto=%4u, %4.2f V)   |   "
               "UMID: %5.1f %%  (bruto=%4u, %4.2f V)\n",
               temperatura, bruto_temp, tensao_temp,
               umidade_pct, bruto_umid, tensao_umid);

        sleep_ms(500);
    }

    return 0;
}
