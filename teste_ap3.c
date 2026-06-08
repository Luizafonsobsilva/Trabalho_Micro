/*
 * AP3 - BRUCUTU Sistemas Eletronicos S.A.
 * Teste integrado dos modulos da AP3: PWM, Timer/Timer Interrupt e Interrupcao
 * Externa. ADC e UART (AP2) entram como fontes de entrada de apoio.
 *
 * Atuadores (todos por PWM):
 *   - Aquecedor (LED vermelho)   -> GP15  slice 7 B  ~1 kHz
 *   - Ventilador (cooler 12V)    -> GP16  slice 0 A  ~25 kHz  (gate do IRLZ44N)
 *   - Buzzer de alarme (passivo) -> GP12  slice 6 A  ~2 kHz
 *   - Umidificador (LED azul)    -> GP11  slice 5 B  ~1 kHz
 *
 * Entradas digitais (interrupcao externa, borda de descida, pull-up):
 *   - Botao POWER -> GP14  liga/desliga o sistema
 *   - Botao MODO  -> GP13  alterna AUTOMATICO <-> MANUAL
 *
 * Heartbeat:
 *   - LED status onboard -> GP25  piscado pela ISR do timer (250 ms)
 *
 * Reaproveitado da AP2:
 *   - LM35 (temperatura) -> GP26 / ADC0   (alimentar com 5V / VBUS)
 *   - Umidade            -> GP27 / ADC1   (alimentar 3V3)
 *   - UART0 fisica       -> GP0 (TX) / GP1 (RX), 115200 8N1, via MAX3232
 *
 * Regras de ISR: ISR curta (so flag/contador/toggle); ADC, UART, PWM e printf
 * ficam no main; variaveis compartilhadas ISR<->main sao volatile; debounce
 * por time_us_64 (nunca sleep).
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/uart.h"

// ==========================================================================
// Mapa de pinos
// ==========================================================================
#define PINO_AQUECEDOR   15   // PWM aquecedor (LED vermelho)
#define PINO_VENTILADOR  16   // PWM ventilador (MOSFET IRLZ44N)
#define PINO_BUZZER      12   // PWM buzzer (alarme)
#define PINO_UMIDIFIC    11   // PWM umidificador (LED azul)

#define PINO_BTN_POWER   14   // botao POWER (liga/desliga)
#define PINO_BTN_MODO    13   // botao MODO  (AUTO <-> MANUAL)

#define PINO_HEARTBEAT   25   // LED status onboard

#define PINO_LM35        26   // ADC0
#define PINO_UMIDADE     27   // ADC1
#define CANAL_LM35        0
#define CANAL_UMIDADE     1

#define UART_ID          uart0
#define BAUD_RATE        115200
#define PINO_TX           0
#define PINO_RX           1

// ==========================================================================
// Frequencias de PWM (uma por slice)
// ==========================================================================
#define FREQ_AQUECEDOR   1000     // ~1 kHz  (LED sem cintilacao visivel)
#define FREQ_VENTILADOR  25000    // ~25 kHz (acima do audivel, sem zumbido)
#define FREQ_BUZZER      2000     // ~2 kHz  (tom audivel no alarme)
#define FREQ_UMIDIFIC    1000     // ~1 kHz  (LED)

#define PWM_CLKDIV       4.0f     // divisor comum no calculo do wrap
#define DUTY_BUZZER_ON   50       // duty do buzzer quando o alarme dispara

// ==========================================================================
// ADC
// ==========================================================================
#define ADC_VREF         3.3f
#define ADC_RESOLUCAO    4096.0f  // 2^12

// ==========================================================================
// Controle proporcional simples
// ==========================================================================
#define K_TEMP           20.0f    // %/C   (satura ~5 C de erro)
#define K_UMID            4.0f    // %/%   (satura ~25 % de erro)
#define LIMIAR_INTERLOCK 90       // % a partir do qual o intertravamento age

#define SETPOINT_TEMP_PADRAO  25.0f
#define SETPOINT_UMID_PADRAO  50.0f

// ==========================================================================
// Amostragem / debounce
// ==========================================================================
#define PERIODO_AMOSTRAGEM_MS  250
#define DEBOUNCE_US            (200 * 1000)  // ~200 ms

// ==========================================================================
// Modos de operacao
// ==========================================================================
typedef enum { MODO_AUTOMATICO = 0, MODO_MANUAL = 1 } modo_t;

// ==========================================================================
// Estado compartilhado ISR <-> main (volatile)
// ==========================================================================
static volatile bool    sistema_ligado   = false;
static volatile modo_t  modo             = MODO_AUTOMATICO;
static volatile bool    tick_amostragem  = false;

// Setpoints: no modo AUTO valem os padroes; no MANUAL vem da UART.
static float setpoint_temp = SETPOINT_TEMP_PADRAO;
static float setpoint_umid = SETPOINT_UMID_PADRAO;

// Buffer de recepcao da UART.
#define TAM_BUFFER 64
static char buffer_rx[TAM_BUFFER];
static int  idx_rx = 0;

// Debounce: ultimo instante aceito de cada botao.
static volatile uint64_t ultimo_power_us = 0;
static volatile uint64_t ultimo_modo_us  = 0;

// ==========================================================================
// Helpers de PWM
// ==========================================================================

// Configura o pino para PWM com a frequencia desejada.
// wrap = clk_sys / (clkdiv * freq) - 1, com clk_sys lido em runtime.
static void pwm_init_pino(uint gpio, uint freq) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);

    uint32_t clk_sys = clock_get_hz(clk_sys);
    uint32_t wrap = (uint32_t)((float)clk_sys / (PWM_CLKDIV * (float)freq)) - 1;

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, PWM_CLKDIV);
    pwm_config_set_wrap(&cfg, (uint16_t)wrap);
    pwm_init(slice, &cfg, true);

    pwm_set_gpio_level(gpio, 0);   // comeca desligado
}

// Aplica um duty (0..100 %) ao pino, convertendo para nivel pelo wrap do slice.
static void pwm_set_duty(uint gpio, int duty_pct) {
    if (duty_pct < 0)   duty_pct = 0;
    if (duty_pct > 100) duty_pct = 100;

    uint slice = pwm_gpio_to_slice_num(gpio);
    uint16_t wrap = pwm_hw->slice[slice].top;   // valor de wrap configurado
    uint32_t nivel = ((uint32_t)wrap + 1) * (uint32_t)duty_pct / 100;
    pwm_set_gpio_level(gpio, (uint16_t)nivel);
}

// Desliga todos os atuadores (sistema OFF ou seguranca).
static void atuadores_off(void) {
    pwm_set_duty(PINO_AQUECEDOR, 0);
    pwm_set_duty(PINO_VENTILADOR, 0);
    pwm_set_duty(PINO_UMIDIFIC, 0);
    pwm_set_duty(PINO_BUZZER, 0);
}

// ==========================================================================
// ISR do timer (250 ms): so heartbeat + flag de amostragem
// ==========================================================================
static bool cb_timer_amostragem(repeating_timer_t *t) {
    (void)t;
    tick_amostragem = true;
    gpio_xor_mask(1u << PINO_HEARTBEAT);   // toggle do LED status
    return true;   // mantem o timer repetindo
}

// ==========================================================================
// ISR unica dos botoes (borda de descida): distingue pelo argumento gpio
// ==========================================================================
static void cb_botoes(uint gpio, uint32_t eventos) {
    (void)eventos;
    uint64_t agora = time_us_64();

    if (gpio == PINO_BTN_POWER) {
        if (agora - ultimo_power_us < DEBOUNCE_US) return;
        ultimo_power_us = agora;
        sistema_ligado = !sistema_ligado;
    } else if (gpio == PINO_BTN_MODO) {
        if (agora - ultimo_modo_us < DEBOUNCE_US) return;
        ultimo_modo_us = agora;
        modo = (modo == MODO_AUTOMATICO) ? MODO_MANUAL : MODO_AUTOMATICO;
    }
}

// ==========================================================================
// Leitura dos sensores (ADC) - reaproveitado da AP2
// ==========================================================================
static float ler_temperatura(void) {
    adc_select_input(CANAL_LM35);
    uint16_t bruto = adc_read();
    float tensao = (bruto * ADC_VREF) / ADC_RESOLUCAO;
    return tensao / 0.01f;   // LM35: 10 mV/C
}

static float ler_umidade(void) {
    adc_select_input(CANAL_UMIDADE);
    uint16_t bruto = adc_read();
    float tensao = (bruto * ADC_VREF) / ADC_RESOLUCAO;
    return (tensao / ADC_VREF) * 100.0f;   // 0 V = 0 %, 3.3 V = 100 %
}

// ==========================================================================
// UART - parser (reaproveitado da AP2, estendido com SET UMID)
// So aceita comandos no modo MANUAL.
// ==========================================================================
static void uart_envia(const char *s) {
    uart_write_blocking(UART_ID, (const uint8_t *)s, strlen(s));
}

static void processar_comando(const char *cmd) {
    if (modo != MODO_MANUAL) {
        uart_envia("ERRO COMANDOS SO NO MODO MANUAL\r\n");
        return;
    }

    char resp[64];
    if (strncmp(cmd, "SET TEMP ", 9) == 0) {
        setpoint_temp = strtof(cmd + 9, NULL);
        snprintf(resp, sizeof(resp), "OK SETPOINT TEMP = %.1f C\r\n", setpoint_temp);
        uart_envia(resp);
    } else if (strncmp(cmd, "SET UMID ", 9) == 0) {
        setpoint_umid = strtof(cmd + 9, NULL);
        snprintf(resp, sizeof(resp), "OK SETPOINT UMID = %.1f %%\r\n", setpoint_umid);
        uart_envia(resp);
    } else {
        uart_envia("ERRO COMANDO DESCONHECIDO\r\n");
    }
}

static void ler_uart_nao_bloqueante(void) {
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        if (c == '\n' || c == '\r') {
            if (idx_rx > 0) {
                buffer_rx[idx_rx] = '\0';
                processar_comando(buffer_rx);
                idx_rx = 0;
            }
        } else if (idx_rx < (TAM_BUFFER - 1)) {
            buffer_rx[idx_rx++] = c;
        } else {
            idx_rx = 0;   // overflow sem terminador: descarta
        }
    }
}

// ==========================================================================
// Inicializacao
// ==========================================================================
static void init_pwm(void) {
    pwm_init_pino(PINO_AQUECEDOR,  FREQ_AQUECEDOR);
    pwm_init_pino(PINO_VENTILADOR, FREQ_VENTILADOR);
    pwm_init_pino(PINO_BUZZER,     FREQ_BUZZER);
    pwm_init_pino(PINO_UMIDIFIC,   FREQ_UMIDIFIC);
}

static void init_botoes(void) {
    gpio_init(PINO_BTN_POWER);
    gpio_set_dir(PINO_BTN_POWER, GPIO_IN);
    gpio_pull_up(PINO_BTN_POWER);

    gpio_init(PINO_BTN_MODO);
    gpio_set_dir(PINO_BTN_MODO, GPIO_IN);
    gpio_pull_up(PINO_BTN_MODO);

    // Callback unico para a porta; o primeiro habilitado registra o handler.
    gpio_set_irq_enabled_with_callback(PINO_BTN_POWER, GPIO_IRQ_EDGE_FALL,
                                       true, &cb_botoes);
    gpio_set_irq_enabled(PINO_BTN_MODO, GPIO_IRQ_EDGE_FALL, true);
}

static void init_heartbeat(void) {
    gpio_init(PINO_HEARTBEAT);
    gpio_set_dir(PINO_HEARTBEAT, GPIO_OUT);
    gpio_put(PINO_HEARTBEAT, 0);
}

static void init_adc(void) {
    adc_init();
    adc_gpio_init(PINO_LM35);
    adc_gpio_init(PINO_UMIDADE);
}

static void init_uart(void) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(PINO_TX, GPIO_FUNC_UART);
    gpio_set_function(PINO_RX, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
}

// ==========================================================================
// Ciclo de controle (executado a cada tick de amostragem)
// ==========================================================================
static void ciclo_controle(void) {
    float temp = ler_temperatura();
    float umid = ler_umidade();

    // Lei de controle proporcional simples.
    int duty_aq = 0, duty_vt = 0, duty_um = 0;

    float erro_temp = setpoint_temp - temp;
    if (erro_temp > 0.0f) {
        duty_aq = (int)(K_TEMP * erro_temp);          // abaixo do setpoint: aquece
    } else if (erro_temp < 0.0f) {
        duty_vt = (int)(K_TEMP * (-erro_temp));       // acima do setpoint: ventila
    }

    float erro_umid = setpoint_umid - umid;
    if (erro_umid > 0.0f) {
        duty_um = (int)(K_UMID * erro_umid);          // seco: umidifica
    }

    // Satura em 0..100.
    if (duty_aq > 100) duty_aq = 100;
    if (duty_vt > 100) duty_vt = 100;
    if (duty_um > 100) duty_um = 100;

    // Intertravamento de seguranca (sempre ativo, nos dois modos):
    // aquecedor e ventilador nunca ambos no maximo.
    bool alarme = false;
    if (duty_aq >= LIMIAR_INTERLOCK && duty_vt >= LIMIAR_INTERLOCK) {
        duty_aq = 0;
        alarme = true;
    }

    // Aplica aos atuadores.
    pwm_set_duty(PINO_AQUECEDOR, duty_aq);
    pwm_set_duty(PINO_VENTILADOR, duty_vt);
    pwm_set_duty(PINO_UMIDIFIC, duty_um);
    pwm_set_duty(PINO_BUZZER, alarme ? DUTY_BUZZER_ON : 0);

    // Status pela UART0 fisica.
    char status[120];
    snprintf(status, sizeof(status),
             "STATUS MODO=%s T=%.1f/%.1fC U=%.1f/%.1f%% "
             "AQ=%d VT=%d UM=%d%s\r\n",
             (modo == MODO_AUTOMATICO) ? "AUTO" : "MAN",
             temp, setpoint_temp, umid, setpoint_umid,
             duty_aq, duty_vt, duty_um, alarme ? " ALARME" : "");
    uart_envia(status);
}

// ==========================================================================
// main
// ==========================================================================
int main(void) {
    stdio_init_all();   // USB-CDC so para debug

    init_heartbeat();
    init_pwm();
    init_adc();
    init_uart();
    init_botoes();

    atuadores_off();

    // Timer periodico de amostragem (250 ms), callback em IRQ.
    repeating_timer_t timer;
    add_repeating_timer_ms(PERIODO_AMOSTRAGEM_MS, cb_timer_amostragem, NULL, &timer);

    uart_envia("=== AP3 Teste integrado - BRUCUTU ===\r\n");
    uart_envia("POWER=GP14  MODO=GP13  (AUTO/MANUAL)\r\n");

    bool estava_ligado = false;

    while (true) {
        // Comandos da UART tratados sempre (nao-bloqueante).
        ler_uart_nao_bloqueante();

        if (tick_amostragem) {
            tick_amostragem = false;

            if (sistema_ligado) {
                ciclo_controle();
                estava_ligado = true;
            } else {
                // DESLIGADO: tudo zerado; heartbeat (ISR) segue piscando.
                if (estava_ligado) {
                    atuadores_off();
                    uart_envia("STATUS SISTEMA DESLIGADO\r\n");
                    estava_ligado = false;
                }
            }
        }

        tight_loop_contents();
    }

    return 0;
}
