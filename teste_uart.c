/*
 * AP2 - BRUCUTU Sistemas Eletronicos S.A.
 * Teste isolado do modulo UART (hardware).
 *
 * Usa a UART0 fisica nos pinos:
 *   - GP0 -> TX
 *   - GP1 -> RX
 * Ligada ao MAX3232 (TTL 3V3 <-> RS-232) e dai ao adaptador USB-RS232 ate o PC.
 * Lembrar do cruzamento: TX do Pico vai na entrada T do MAX3232; saida R do
 * MAX3232 vai no RX do Pico. GND comum no lado TTL. VCC do MAX3232 em 3V3.
 *
 * Funcionamento:
 *   - Envia uma string de status periodica (a cada 1 s) pela UART0.
 *   - Recebe comandos ASCII terminados em '\n' e os interpreta.
 *     Comando suportado nesta etapa:  SET TEMP <valor>
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// --- Configuracao da UART0 ---
#define UART_ID        uart0
#define BAUD_RATE      115200
#define PINO_TX        0    // GP0
#define PINO_RX        1    // GP1

// Tamanho maximo de um comando recebido.
#define TAM_BUFFER     64

// "Setpoint" de temperatura ajustavel via comando SET TEMP.
static float setpoint_temp = 25.0f;

// Buffer de recepcao de comando e indice atual.
static char buffer_rx[TAM_BUFFER];
static int  idx_rx = 0;

// Interpreta uma linha de comando ja completa (sem o '\n').
static void processar_comando(const char *cmd) {
    // Espera o formato: SET TEMP <valor>
    if (strncmp(cmd, "SET TEMP ", 9) == 0) {
        float valor = strtof(cmd + 9, NULL);
        setpoint_temp = valor;
        char resp[64];
        int n = snprintf(resp, sizeof(resp),
                         "OK SETPOINT TEMP = %.1f C\r\n", setpoint_temp);
        uart_write_blocking(UART_ID, (const uint8_t *)resp, n);
    } else {
        const char *erro = "ERRO COMANDO DESCONHECIDO\r\n";
        uart_write_blocking(UART_ID, (const uint8_t *)erro, strlen(erro));
    }
}

// Le os caracteres disponiveis na UART e monta o comando ate encontrar '\n'.
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

    // --- Inicializa a UART0 fisica ---
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(PINO_TX, GPIO_FUNC_UART);
    gpio_set_function(PINO_RX, GPIO_FUNC_UART);

    // 8 bits de dados, 1 stop bit, sem paridade (8N1).
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    // Mensagem inicial pela propria UART0.
    const char *banner = "=== AP2 Teste UART - BRUCUTU ===\r\n";
    uart_write_blocking(UART_ID, (const uint8_t *)banner, strlen(banner));

    absolute_time_t proximo_status = make_timeout_time_ms(1000);

    while (true) {
        // 1) Trata comandos recebidos (nao bloqueante).
        ler_uart_nao_bloqueante();

        // 2) Envia status periodico a cada 1 s.
        if (absolute_time_diff_us(get_absolute_time(), proximo_status) <= 0) {
            char status[80];
            int n = snprintf(status, sizeof(status),
                             "STATUS SETPOINT=%.1f C\r\n", setpoint_temp);
            uart_write_blocking(UART_ID, (const uint8_t *)status, n);
            proximo_status = make_timeout_time_ms(1000);
        }

        sleep_ms(10);   // alivia o laco; nao e timer/interrupcao, so polling
    }

    return 0;
}
