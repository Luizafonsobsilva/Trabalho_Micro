/*
 * AP3 - BRUCUTU Sistemas Eletronicos S.A.
 * Teste integrado dos modulos da AP3: PWM, Timer/Timer Interrupt e Interrupcao
 * Externa. ADC e UART (AP2) entram como fontes de entrada de apoio.
 *
 * Atuadores (todos por PWM):
 *   - Aquecedor (LED vermelho)   -> GP15  slice 7 B  ~1 kHz
 *   - Ventilador (cooler 12V)    -> GP16  slice 0 A  ~25 kHz  (gate do IRLZ44N)
 *   - Buzzer de alarme (ATIVO)   -> GP12  GPIO digital on/off (oscilador interno)
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
#include "pico/cyw43_arch.h"   // LED onboard do Pico 2 W (via chip CYW43)
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
#define PINO_BUZZER      12   // buzzer ATIVO (alarme) - GPIO digital on/off
#define PINO_UMIDIFIC    11   // PWM umidificador (LED azul)

#define PINO_BTN_POWER   14   // botao POWER (liga/desliga)
#define PINO_BTN_MODO    13   // botao MODO  (AUTO <-> MANUAL)

// Heartbeat: no Pico 2 W o LED onboard NAO e um GPIO comum (o GP25 e o
// chip-select do CYW43). Usamos CYW43_WL_GPIO_LED_PIN via cyw43_arch.

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
#define FREQ_UMIDIFIC    1000     // ~1 kHz  (LED)

#define PWM_CLKDIV       4.0f     // divisor comum no calculo do wrap

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

// [DIAGNOSTICO] modo de teste manual dos atuadores (via USB-CDC).
// Quando ligado, o ciclo de controle automatico fica suspenso e os
// atuadores so respondem aos comandos de teste.
static volatile bool modo_teste = false;

// Setpoints: no modo AUTO valem os padroes; no MANUAL vem da UART.
static float setpoint_temp = SETPOINT_TEMP_PADRAO;
static float setpoint_umid = SETPOINT_UMID_PADRAO;

// Buffer de recepcao da UART.
#define TAM_BUFFER 64
static char buffer_rx[TAM_BUFFER];
static int  idx_rx = 0;

// [DIAGNOSTICO] buffer de comandos de teste vindos da USB-CDC.
static char buffer_usb[TAM_BUFFER];
static int  idx_usb = 0;

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
    gpio_put(PINO_BUZZER, 0);
}

// ==========================================================================
// ISR do timer (250 ms): so heartbeat + flag de amostragem
// ==========================================================================
static bool cb_timer_amostragem(repeating_timer_t *t) {
    (void)t;
    tick_amostragem = true;   // o pisca do LED e o controle sao feitos no main
    return true;              // mantem o timer repetindo
    // OBS: nada de cyw43_arch_gpio_put aqui - faz SPI lenta/bloqueante.
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
// [DIAGNOSTICO] Modo de teste manual dos atuadores (via USB-CDC)
// Permite acionar cada atuador isoladamente, sem depender dos sensores nem
// da UART fisica. Comandos (digite no Serial Monitor USB do Pico + Enter):
//   TEST ON    -> entra no modo de teste (suspende o controle automatico)
//   TEST OFF   -> sai do modo de teste
//   AQ <0-100> -> aquecedor  (LED vermelho, GP15)
//   VT <0-100> -> ventilador (GP16)
//   UM <0-100> -> umidificador (LED azul, GP11)
//   BZ <0|1>   -> buzzer (GP12)
//   SWEEP      -> varre todos os atuadores em sequencia (0->100->0) + beeps
// ==========================================================================
static void sweep_atuadores(void) {
    struct { uint gpio; const char *nome; } a[] = {
        { PINO_AQUECEDOR,  "aquecedor   (GP15, vermelho)" },
        { PINO_VENTILADOR, "ventilador  (GP16)" },
        { PINO_UMIDIFIC,   "umidificador(GP11, azul)" },
    };
    for (int i = 0; i < 3; i++) {
        printf("[TEST] SWEEP %s : 0 -> 100\n", a[i].nome);
        for (int d = 0; d <= 100; d += 10) { pwm_set_duty(a[i].gpio, d); sleep_ms(80); }
        for (int d = 100; d >= 0; d -= 10) { pwm_set_duty(a[i].gpio, d); sleep_ms(80); }
        pwm_set_duty(a[i].gpio, 0);
    }
    printf("[TEST] SWEEP buzzer (GP12): 3 beeps\n");
    for (int i = 0; i < 3; i++) {
        gpio_put(PINO_BUZZER, 1); sleep_ms(150);
        gpio_put(PINO_BUZZER, 0); sleep_ms(150);
    }
    printf("[TEST] SWEEP fim\n");
}

static void processar_comando_usb(const char *cmd) {
    int v;
    if (strcmp(cmd, "TEST ON") == 0) {
        modo_teste = true;
        atuadores_off();
        printf("[TEST] modo de teste LIGADO (controle automatico suspenso)\n");
    } else if (strcmp(cmd, "TEST OFF") == 0) {
        modo_teste = false;
        atuadores_off();
        printf("[TEST] modo de teste DESLIGADO\n");
    } else if (!modo_teste) {
        printf("[TEST] envie 'TEST ON' antes de testar atuadores\n");
    } else if (strcmp(cmd, "SWEEP") == 0) {
        sweep_atuadores();
    } else if (sscanf(cmd, "AQ %d", &v) == 1) {
        pwm_set_duty(PINO_AQUECEDOR, v);
        printf("[TEST] aquecedor (GP15) = %d%%\n", v);
    } else if (sscanf(cmd, "VT %d", &v) == 1) {
        pwm_set_duty(PINO_VENTILADOR, v);
        printf("[TEST] ventilador (GP16) = %d%%\n", v);
    } else if (sscanf(cmd, "UM %d", &v) == 1) {
        pwm_set_duty(PINO_UMIDIFIC, v);
        printf("[TEST] umidificador (GP11) = %d%%\n", v);
    } else if (sscanf(cmd, "BZ %d", &v) == 1) {
        gpio_put(PINO_BUZZER, v ? 1 : 0);
        printf("[TEST] buzzer (GP12) = %d\n", v ? 1 : 0);
    } else {
        printf("[TEST] comando desconhecido: '%s'\n", cmd);
    }
}

static void ler_usb_nao_bloqueante(void) {
    int c = getchar_timeout_us(0);
    while (c != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (idx_usb > 0) {
                buffer_usb[idx_usb] = '\0';
                processar_comando_usb(buffer_usb);
                idx_usb = 0;
            }
        } else if (idx_usb < (TAM_BUFFER - 1)) {
            buffer_usb[idx_usb++] = (char)c;
        } else {
            idx_usb = 0;
        }
        c = getchar_timeout_us(0);
    }
}

// ==========================================================================
// Inicializacao
// ==========================================================================
static void init_pwm(void) {
    pwm_init_pino(PINO_AQUECEDOR,  FREQ_AQUECEDOR);
    pwm_init_pino(PINO_VENTILADOR, FREQ_VENTILADOR);
    pwm_init_pino(PINO_UMIDIFIC,   FREQ_UMIDIFIC);
}

// Buzzer ATIVO: oscilador interno proprio, acionado por GPIO digital on/off.
static void init_buzzer(void) {
    gpio_init(PINO_BUZZER);
    gpio_set_dir(PINO_BUZZER, GPIO_OUT);
    gpio_put(PINO_BUZZER, 0);   // alarme desligado
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

// LED de status onboard do Pico 2 W (controlado pelo chip CYW43).
// Retorna true se o chip inicializou; se falhar, o resto do firmware segue
// funcionando, so sem o LED de heartbeat.
static bool init_heartbeat(void) {
    if (cyw43_arch_init() != 0) {
        return false;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);   // comeca apagado
    return true;
}

static void heartbeat_set(bool ligado) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, ligado);
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
    gpio_put(PINO_BUZZER, alarme ? 1 : 0);

    const char *nome_modo = (modo == MODO_AUTOMATICO) ? "AUTO" : "MAN";

    // Status pela UART0 fisica (formato do protocolo - nao alterar).
    char status[120];
    snprintf(status, sizeof(status),
             "STATUS MODO=%s T=%.1f/%.1fC U=%.1f/%.1f%% "
             "AQ=%d VT=%d UM=%d%s\r\n",
             nome_modo, temp, setpoint_temp, umid, setpoint_umid,
             duty_aq, duty_vt, duty_um, alarme ? " ALARME" : "");
    uart_envia(status);

    // Versao legivel pela USB-CDC (so debug).
    printf("[%-4s] Temp %5.1f/%-4.1f C | Umid %5.1f/%-4.1f %% | "
           "Aquec %3d%%  Vent %3d%%  Umid %3d%%%s\n",
           nome_modo, temp, setpoint_temp, umid, setpoint_umid,
           duty_aq, duty_vt, duty_um,
           alarme ? "  *** ALARME ***" : "");
}

// ==========================================================================
// main
// ==========================================================================
int main(void) {
    stdio_init_all();   // USB-CDC so para debug

    bool hb_ok = init_heartbeat();
    init_pwm();
    init_buzzer();
    init_adc();
    init_uart();
    init_botoes();

    atuadores_off();

    // Timer periodico de amostragem (250 ms), callback em IRQ.
    repeating_timer_t timer;
    add_repeating_timer_ms(PERIODO_AMOSTRAGEM_MS, cb_timer_amostragem, NULL, &timer);

    uart_envia("=== AP3 Teste integrado - BRUCUTU ===\r\n");
    uart_envia("POWER=GP14  MODO=GP13  (AUTO/MANUAL)\r\n");

    // Banner/ajuda na USB-CDC (debug).
    printf("\n=== AP3 - BRUCUTU =========================================\n");
    printf("Heartbeat (LED onboard): %s\n", hb_ok ? "ok" : "FALHOU cyw43");
    printf("POWER=GP14 (liga/desliga)   MODO=GP13 (AUTO/MANUAL)\n");
    printf("Teste manual de atuadores pela USB:\n");
    printf("  TEST ON | TEST OFF | SWEEP\n");
    printf("  AQ <0-100> | VT <0-100> | UM <0-100> | BZ <0|1>\n");
    printf("===========================================================\n\n");

    bool estava_ligado = false;

    while (true) {
        // Comandos da UART tratados sempre (nao-bloqueante).
        ler_uart_nao_bloqueante();
        // Comandos de teste pela USB.
        ler_usb_nao_bloqueante();

        if (tick_amostragem) {
            tick_amostragem = false;

            // Heartbeat: toggle do LED onboard a cada tick (250 ms).
            if (hb_ok) {
                static bool hb = false;
                hb = !hb;
                heartbeat_set(hb);
            }

            if (modo_teste) {
                // Atuadores controlados manualmente pelos comandos de teste.
            } else if (sistema_ligado) {
                ciclo_controle();
                estava_ligado = true;
            } else {
                // DESLIGADO: tudo zerado; heartbeat (ISR) segue piscando.
                if (estava_ligado) {
                    atuadores_off();
                    uart_envia("STATUS SISTEMA DESLIGADO\r\n");
                    printf("[OFF] sistema desligado\n");
                    estava_ligado = false;
                }
            }
        }

        tight_loop_contents();
    }

    return 0;
}
