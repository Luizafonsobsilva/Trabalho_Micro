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
#include <stdarg.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"   // LED onboard do Pico 2 W (via chip CYW43)
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"      // LCD 16x2 via backpack PCF8574

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

    uint32_t f_clk = clock_get_hz(clk_sys);   // enum clk_sys (nao sombrear!)
    uint32_t wrap = (uint32_t)((float)f_clk / (PWM_CLKDIV * (float)freq)) - 1;

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

// ==========================================================================
// LCD 16x2 (HD44780) via backpack I2C PCF8574
// 4 fios: VCC(3V3), GND, SDA=GP4, SCL=GP5. Endereco detectado no boot.
// Mapa do PCF8574: P0=RS P1=RW P2=EN P3=Backlight P4..P7=D4..D7
// ==========================================================================
#define LCD_I2C   i2c0
#define PINO_SDA  4
#define PINO_SCL  5
#define LCD_BL    0x08   // backlight ligado
#define LCD_EN    0x04   // enable
#define LCD_RS    0x01   // 1=dado, 0=comando

static bool    lcd_ok   = false;
static uint8_t lcd_addr = 0x27;

static void lcd_raw(uint8_t b) {
    i2c_write_blocking(LCD_I2C, lcd_addr, &b, 1, false);
}

static void lcd_pulse(uint8_t data) {
    lcd_raw(data | LCD_EN | LCD_BL);
    sleep_us(1);
    lcd_raw((data & ~LCD_EN) | LCD_BL);
    sleep_us(50);
}

static void lcd_send(uint8_t val, uint8_t mode) {
    lcd_pulse((val & 0xF0) | mode);          // nibble alto
    lcd_pulse(((val << 4) & 0xF0) | mode);   // nibble baixo
}

static void lcd_cmd(uint8_t c)  { lcd_send(c, 0); }
static void lcd_data(uint8_t d) { lcd_send(d, LCD_RS); }

static void lcd_set_cursor(int linha, int col) {
    static const uint8_t base[2] = { 0x00, 0x40 };
    lcd_cmd(0x80 | (base[linha & 1] + col));
}

// Escreve uma linha inteira (16 chars, completando com espacos para limpar
// o que sobrou da leitura anterior). printf-like.
static void lcd_linha(int linha, const char *fmt, ...) {
    if (!lcd_ok) return;
    char buf[17];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) n = 0;
    for (int i = n; i < 16; i++) buf[i] = ' ';
    buf[16] = '\0';
    lcd_set_cursor(linha, 0);
    for (const char *p = buf; *p; p++) lcd_data((uint8_t)*p);
}

// Varre o barramento e devolve o 1o endereco que responde (0 = nenhum).
static uint8_t i2c_buscar(void) {
    for (uint8_t a = 0x08; a < 0x78; a++) {
        uint8_t z = 0;
        if (i2c_write_blocking(LCD_I2C, a, &z, 1, false) >= 0) return a;
    }
    return 0;
}

// Inicializa I2C + LCD. Retorna false se nenhum backpack respondeu.
static bool init_lcd(void) {
    i2c_init(LCD_I2C, 100 * 1000);
    gpio_set_function(PINO_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PINO_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PINO_SDA);
    gpio_pull_up(PINO_SCL);

    uint8_t a = i2c_buscar();
    if (a == 0) return false;
    lcd_addr = a;

    sleep_ms(50);                       // sequencia de init em 4 bits do HD44780
    lcd_pulse(0x30); sleep_ms(5);
    lcd_pulse(0x30); sleep_us(150);
    lcd_pulse(0x30); sleep_us(150);
    lcd_pulse(0x20); sleep_us(150);     // passa para 4 bits
    lcd_cmd(0x28);   // 4 bits, 2 linhas, fonte 5x8
    lcd_cmd(0x0C);   // display on, cursor off
    lcd_cmd(0x06);   // incrementa o cursor a cada caractere
    lcd_cmd(0x01);   // limpa
    sleep_ms(2);

    lcd_ok = true;
    return true;
}

// Desliga todos os atuadores (sistema OFF ou seguranca).
static void atuadores_off(void) {
    pwm_set_duty(PINO_AQUECEDOR, 0);
    pwm_set_duty(PINO_VENTILADOR, 0);
    pwm_set_duty(PINO_UMIDIFIC, 0);
    gpio_put(PINO_BUZZER, 0);
}

// Intertravamento de seguranca: aquecedor e ventilador nunca ambos no maximo.
// Se os dois passarem do limiar, zera o aquecedor e retorna true (alarme).
static bool aplicar_interlock(int *duty_aq, int *duty_vt) {
    if (*duty_aq >= LIMIAR_INTERLOCK && *duty_vt >= LIMIAR_INTERLOCK) {
        *duty_aq = 0;
        return true;
    }
    return false;
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
// Comandos: um unico parser atende a UART fisica E a USB-CDC.
// As respostas saem nos DOIS canais (responder()).
//
// Operacao:
//   MODO AUTO | MODO MAN   -> troca o modo (mesmo efeito do botao GP13)
//   SET TEMP <v>           -> setpoint de temperatura (so no modo MANUAL)
//   SET UMID <v>           -> setpoint de umidade     (so no modo MANUAL)
// Teste de atuadores (precisa de TEST ON; suspende o controle automatico):
//   TEST ON | TEST OFF
//   AQ <0-100> | VT <0-100> | UM <0-100> | BZ <0|1> | SWEEP
//   INTERLOCK  -> forca AQ=VT=100% e dispara o intertravamento (buzzer)
// ==========================================================================
static void uart_envia(const char *s) {
    uart_write_blocking(UART_ID, (const uint8_t *)s, strlen(s));
}

// Resposta de comando: ecoa nos dois canais (UART fisica + USB-CDC).
static void responder(const char *s) {
    uart_envia(s);
    printf("%s", s);
}

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

static void processar_comando(const char *cmd) {
    int v;
    char resp[80];

    // ---- modo de operacao (mesmo efeito do botao MODO/GP13) ----
    if (strcmp(cmd, "MODO AUTO") == 0) {
        modo = MODO_AUTOMATICO;
        responder("OK MODO = AUTO\r\n");
        return;
    }
    if (strcmp(cmd, "MODO MAN") == 0 || strcmp(cmd, "MODO MANUAL") == 0) {
        modo = MODO_MANUAL;
        responder("OK MODO = MANUAL\r\n");
        return;
    }

    // ---- setpoints: so valem no modo MANUAL ----
    if (strncmp(cmd, "SET TEMP ", 9) == 0) {
        if (modo != MODO_MANUAL) { responder("ERRO SET SO NO MODO MANUAL\r\n"); return; }
        setpoint_temp = strtof(cmd + 9, NULL);
        snprintf(resp, sizeof(resp), "OK SETPOINT TEMP = %.1f C\r\n", setpoint_temp);
        responder(resp);
        return;
    }
    if (strncmp(cmd, "SET UMID ", 9) == 0) {
        if (modo != MODO_MANUAL) { responder("ERRO SET SO NO MODO MANUAL\r\n"); return; }
        setpoint_umid = strtof(cmd + 9, NULL);
        snprintf(resp, sizeof(resp), "OK SETPOINT UMID = %.1f %%\r\n", setpoint_umid);
        responder(resp);
        return;
    }

    // ---- teste de atuadores: precisa de TEST ON ----
    if (strcmp(cmd, "TEST ON") == 0) {
        modo_teste = true;
        atuadores_off();
        responder("OK TESTE ON (controle automatico suspenso)\r\n");
        return;
    }
    if (strcmp(cmd, "TEST OFF") == 0) {
        modo_teste = false;
        atuadores_off();
        responder("OK TESTE OFF\r\n");
        return;
    }
    if (strcmp(cmd, "SWEEP") == 0) {
        if (!modo_teste) { responder("ERRO ENVIE TEST ON ANTES\r\n"); return; }
        sweep_atuadores();
        return;
    }
    if (strcmp(cmd, "INTERLOCK") == 0) {
        if (!modo_teste) { responder("ERRO ENVIE TEST ON ANTES\r\n"); return; }
        // Forca a condicao perigosa (AQ e VT no maximo) e aplica a MESMA
        // logica de seguranca da operacao real: zera o aquecedor + alarme.
        int aq = 100, vt = 100;
        bool alarme = aplicar_interlock(&aq, &vt);
        pwm_set_duty(PINO_AQUECEDOR, aq);
        pwm_set_duty(PINO_VENTILADOR, vt);
        gpio_put(PINO_BUZZER, alarme ? 1 : 0);
        snprintf(resp, sizeof(resp),
                 "OK INTERLOCK forcado: AQ=100 VT=100 -> AQ=%d VT=%d BUZZER=%d%s\r\n",
                 aq, vt, alarme ? 1 : 0, alarme ? " ALARME" : "");
        responder(resp);
        return;
    }
    if (sscanf(cmd, "AQ %d", &v) == 1) {
        if (!modo_teste) { responder("ERRO ENVIE TEST ON ANTES\r\n"); return; }
        pwm_set_duty(PINO_AQUECEDOR, v);
        snprintf(resp, sizeof(resp), "OK AQ = %d%%\r\n", v);
        responder(resp);
        return;
    }
    if (sscanf(cmd, "VT %d", &v) == 1) {
        if (!modo_teste) { responder("ERRO ENVIE TEST ON ANTES\r\n"); return; }
        pwm_set_duty(PINO_VENTILADOR, v);
        snprintf(resp, sizeof(resp), "OK VT = %d%%\r\n", v);
        responder(resp);
        return;
    }
    if (sscanf(cmd, "UM %d", &v) == 1) {
        if (!modo_teste) { responder("ERRO ENVIE TEST ON ANTES\r\n"); return; }
        pwm_set_duty(PINO_UMIDIFIC, v);
        snprintf(resp, sizeof(resp), "OK UM = %d%%\r\n", v);
        responder(resp);
        return;
    }
    if (sscanf(cmd, "BZ %d", &v) == 1) {
        if (!modo_teste) { responder("ERRO ENVIE TEST ON ANTES\r\n"); return; }
        gpio_put(PINO_BUZZER, v ? 1 : 0);
        snprintf(resp, sizeof(resp), "OK BZ = %d\r\n", v ? 1 : 0);
        responder(resp);
        return;
    }

    responder("ERRO COMANDO DESCONHECIDO\r\n");
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

static void ler_usb_nao_bloqueante(void) {
    int c = getchar_timeout_us(0);
    while (c != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (idx_usb > 0) {
                buffer_usb[idx_usb] = '\0';
                processar_comando(buffer_usb);
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
    bool alarme = aplicar_interlock(&duty_aq, &duty_vt);

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

    // LCD 16x2: temperatura e umidade, atual > alvo.
    lcd_linha(0, "T%5.1f>%5.1fC", temp, setpoint_temp);
    if (alarme) {
        lcd_linha(1, "U%5.1f>%5.1f ALM", umid, setpoint_umid);
    } else {
        lcd_linha(1, "U%5.1f>%5.1f %%", umid, setpoint_umid);
    }
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
    bool lcd_found = init_lcd();

    atuadores_off();

    // Timer periodico de amostragem (250 ms), callback em IRQ.
    repeating_timer_t timer;
    add_repeating_timer_ms(PERIODO_AMOSTRAGEM_MS, cb_timer_amostragem, NULL, &timer);

    uart_envia("=== AP3 Teste integrado - BRUCUTU ===\r\n");
    uart_envia("POWER=GP14  MODO=GP13  (AUTO/MANUAL)\r\n");

    // Banner/ajuda na USB-CDC (debug). Os mesmos comandos valem pela UART.
    printf("\n=== AP3 - BRUCUTU =========================================\n");
    printf("Heartbeat (LED onboard): %s\n", hb_ok ? "ok" : "FALHOU cyw43");
    if (lcd_found) printf("LCD I2C: ok no endereco 0x%02X (GP4=SDA GP5=SCL)\n", lcd_addr);
    else           printf("LCD I2C: NAO encontrado (cheque fiacao/alimentacao)\n");
    printf("Botoes: POWER=GP14 (liga/desliga)  MODO=GP13 (AUTO/MANUAL)\n");
    printf("Comandos (UART fisica OU USB):\n");
    printf("  MODO AUTO | MODO MAN\n");
    printf("  SET TEMP <v> | SET UMID <v>   (so no modo MANUAL)\n");
    printf("  TEST ON | TEST OFF | SWEEP | INTERLOCK\n");
    printf("  AQ <0-100> | VT <0-100> | UM <0-100> | BZ <0|1>\n");
    printf("===========================================================\n\n");

    // Splash no LCD.
    lcd_linha(0, "AP3 - BRUCUTU");
    lcd_linha(1, "Aperte POWER");

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
                    lcd_linha(0, "SISTEMA");
                    lcd_linha(1, "DESLIGADO");
                    estava_ligado = false;
                }
            }
        }

        tight_loop_contents();
    }

    return 0;
}
