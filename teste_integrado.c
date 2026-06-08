/*
 * AP2 - BRUCUTU Sistemas Eletronicos S.A.
 * Teste INTEGRADO: ADC + UART funcionando juntos no mesmo firmware.
 *
 * O que faz, num unico laco (sem timer/interrupcao, so polling):
 *   - Le LM35 (temperatura, GP26/ADC0) e sensor de umidade (GP27/ADC1).
 *   - A cada 1 s:
 *       * imprime as leituras na USB-CDC (debug, monitor serial do VS Code);
 *       * envia uma linha de status pela UART0 fisica (GP0/GP1):
 *             STATUS TEMP=24.7C UMID=41.2% SET=25.0C
 *   - A cada volta do laco, le comandos pela UART0 (nao bloqueante):
 *             SET TEMP <valor>   -> ajusta o setpoint e confirma pela UART.
 *
 * Ligacao da UART0: Pico (TTL 3V3) -> MAX3232 -> DB9 -> USB-RS232 -> PC.
 *   Cruzamento: TX do Pico na entrada T do MAX3232; saida R do MAX3232 no RX
 *   do Pico. GND comum no lado TTL. VCC do MAX3232 em 3V3. Monitor: 115200 8N1.
 *
 * Alimentacao dos sensores: LM35 em VBUS (5V); umidade em 3V3.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"

// --- Pinos ADC ---
#define PINO_LM35      26   // ADC0
#define PINO_UMIDADE   27   // ADC1
#define CANAL_LM35      0
#define CANAL_UMIDADE   1

// --- Referencia do ADC (12 bits, 0..4095, 3.3 V) ---
#define ADC_VREF       3.3f
#define ADC_RESOLUCAO  4096.0f   // 2^12

// --- Configuracao da UART0 ---
#define UART_ID        uart0
#define BAUD_RATE      115200
#define PINO_TX        0    // GP0
#define PINO_RX        1    // GP1

// --- Tempo entre envios de status ---
#define PERIODO_MS     1000

// Tamanho maximo de um comando recebido.
#define TAM_BUFFER     64

// "Setpoint" de temperatura ajustavel via comando SET TEMP.
static float setpoint_temp = 25.0f;

// Buffer de recepcao de comando e indice atual.
static char buffer_rx[TAM_BUFFER];
static int  idx_rx = 0;

// Converte leitura bruta (0..4095) em tensao (volts).
static float adc_para_tensao(uint16_t bruto) {
    return (bruto * ADC_VREF) / ADC_RESOLUCAO;
}

// LM35: 10 mV por grau Celsius -> temp = Vout / 0.01
static float tensao_para_celsius(float tensao) {
    return tensao / 0.01f;
}

// Le um canal do ADC e devolve a leitura bruta (0..4095).
static uint16_t ler_canal(uint canal) {
    adc_select_input(canal);
    return adc_read();
}

// Interpreta uma linha de comando ja completa (sem o terminador).
static void processar_comando(const char *cmd) {
    // Espera o formato: SET TEMP <valor>
    if (strncmp(cmd, "SET TEMP ", 9) == 0) {
        setpoint_temp = strtof(cmd + 9, NULL);
        char resp[64];
        int n = snprintf(resp, sizeof(resp),
                         "OK SETPOINT TEMP = %.1f C\r\n", setpoint_temp);
        uart_write_blocking(UART_ID, (const uint8_t *)resp, n);
        printf("[debug] novo setpoint = %.1f C\n", setpoint_temp);
    } else {
        const char *erro = "ERRO COMANDO DESCONHECIDO\r\n";
        uart_write_blocking(UART_ID, (const uint8_t *)erro, strlen(erro));
    }
}

// Le os caracteres disponiveis na UART e monta o comando ate o terminador.
static void ler_uart_nao_bloqueante(void) {
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);

        if (c == '\n' || c == '\r') {
            if (idx_rx > 0) {
                buffer_rx[idx_rx] = '\0';  // termina a string
                processar_comando(buffer_rx);
                idx_rx = 0;
            }
        } else if (idx_rx < (TAM_BUFFER - 1)) {
            buffer_rx[idx_rx++] = c;
        } else {
            // Buffer cheio sem terminador: descarta para evitar overflow.
            idx_rx = 0;
        }
    }
}

int main(void) {
    stdio_init_all();   // USB-CDC para debug, separado da UART0 fisica

    // --- Inicializa o ADC ---
    adc_init();
    adc_gpio_init(PINO_LM35);
    adc_gpio_init(PINO_UMIDADE);

    // --- Inicializa a UART0 fisica ---
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(PINO_TX, GPIO_FUNC_UART);
    gpio_set_function(PINO_RX, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);  // 8N1
    uart_set_fifo_enabled(UART_ID, true);

    // Espera o monitor USB conectar antes dos primeiros prints de debug.
    sleep_ms(2000);
    printf("=== AP2 Teste INTEGRADO (ADC+UART) - BRUCUTU ===\n");

    const char *banner = "=== AP2 Teste INTEGRADO (ADC+UART) - BRUCUTU ===\r\n";
    uart_write_blocking(UART_ID, (const uint8_t *)banner, strlen(banner));

    absolute_time_t proximo_status = make_timeout_time_ms(PERIODO_MS);

    while (true) {
        // 1) Trata comandos recebidos pela UART (nao bloqueante).
        ler_uart_nao_bloqueante();

        // 2) A cada PERIODO_MS, le os sensores e publica nos dois canais.
        if (absolute_time_diff_us(get_absolute_time(), proximo_status) <= 0) {
            uint16_t bruto_temp = ler_canal(CANAL_LM35);
            float tensao_temp = adc_para_tensao(bruto_temp);
            float temperatura = tensao_para_celsius(tensao_temp);

            uint16_t bruto_umid = ler_canal(CANAL_UMIDADE);
            float tensao_umid = adc_para_tensao(bruto_umid);
            float umidade_pct = (tensao_umid / ADC_VREF) * 100.0f;

            // Debug legivel pela USB-CDC.
            printf("TEMP: %5.1f C  (bruto=%4u, %4.2f V)   |   "
                   "UMID: %5.1f %%  (bruto=%4u, %4.2f V)   |   SET: %.1f C\n",
                   temperatura, bruto_temp, tensao_temp,
                   umidade_pct, bruto_umid, tensao_umid, setpoint_temp);

            // Status pela UART0 fisica (protocolo do professor).
            char status[80];
            int n = snprintf(status, sizeof(status),
                             "STATUS TEMP=%.1fC UMID=%.1f%% SET=%.1fC\r\n",
                             temperatura, umidade_pct, setpoint_temp);
            uart_write_blocking(UART_ID, (const uint8_t *)status, n);

            proximo_status = make_timeout_time_ms(PERIODO_MS);
        }

        sleep_ms(10);   // alivia o laco; nao e timer/interrupcao, so polling
    }

    return 0;
}
