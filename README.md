# BRUCUTU Sistemas Eletronicos S.A. — Controle de Clima (AP3)

Projeto da disciplina de Sistemas Embarcados. Firmware em C, com o **Pico SDK**,
para um **Raspberry Pi Pico 2W (RP2350)** rodando em modo **RISC-V (nucleos
Hazard3)**.

A entrega anterior (AP2) cobriu apenas **ADC** e **UART**. Esta entrega (AP3)
amplia o sistema com **PWM**, **Timer / Timer Interrupt** e **Interrupcao
Externa**, mantendo ADC e UART como entradas/saidas de apoio. O codigo de
referencia desta entrega esta em [`Codigo_P2.c`](./Codigo_P2.c).

## Visao geral do sistema

O firmware le um sensor de temperatura (LM35) e um sensor de umidade
resistivo, e atua sobre tres cargas para tentar levar as leituras aos
*setpoints* definidos:

- **Aquecedor** (LED vermelho, simulando uma resistencia) — PWM ~1 kHz.
- **Ventilador** (cooler 12 V, via MOSFET IRLZ44N) — PWM ~25 kHz.
- **Umidificador** (LED azul, simulando o atomizador) — PWM ~1 kHz.
- **Buzzer ativo** de alarme — GPIO digital on/off (oscilador interno do buzzer).

Dois botoes externos com pull-up controlam a operacao em **interrupcao por
borda de descida**:

- **POWER (GP14)** — liga/desliga o sistema.
- **MODO (GP13)** — alterna entre **AUTOMATICO** e **MANUAL**.

Um **timer periodico de 250 ms** (Timer Interrupt) levanta uma flag de
amostragem; o ciclo de controle roda no `main()`. O LED onboard do Pico 2 W
(via chip CYW43) e usado como heartbeat para sinalizar que o firmware esta vivo.

### Logica de controle

Em modo AUTOMATICO os setpoints sao 25 °C e 50 % UR. Em MANUAL eles vem da
UART (`SET TEMP <v>` / `SET UMID <v>`). A lei de controle e proporcional:

- `duty_aq = K_TEMP * (setpoint_temp - temp)` quando esta frio.
- `duty_vt = K_TEMP * (temp - setpoint_temp)` quando esta quente.
- `duty_um = K_UMID * (setpoint_umid - umid)` quando esta seco.

Os duties saturam em 0..100 %.

### Intertravamento de seguranca

Aquecedor e ventilador nunca podem estar simultaneamente em PWM alto — isso
poderia significar uma falha de sensor ou comando inconsistente. Se os dois
passarem do limiar `LIMIAR_INTERLOCK` (90 %), o firmware **zera o aquecedor
e dispara o buzzer**. A mesma logica vale tanto em operacao normal quanto no
modo de teste (comando `INTERLOCK`).

## Mapa de pinos

| Funcao                      | Pino       | Observacao                             |
|-----------------------------|------------|----------------------------------------|
| LM35 (temperatura)          | GP26 / ADC0| Alimentar com **VBUS 5 V**             |
| Sensor de umidade           | GP27 / ADC1| Alimentar com **3,3 V**                |
| UART0 TX                    | GP0        | Via MAX3232 → DB9 → USB-RS232          |
| UART0 RX                    | GP1        | Via MAX3232 → DB9 → USB-RS232          |
| I2C0 SDA (LCD 16x2)         | GP4        | Backpack PCF8574, endereco autodetect  |
| I2C0 SCL (LCD 16x2)         | GP5        | Backpack PCF8574                       |
| Umidificador (LED azul)     | GP11       | PWM ~1 kHz                             |
| Buzzer ativo                | GP12       | GPIO digital on/off                    |
| Botao MODO                  | GP13       | Entrada com pull-up, IRQ borda descida |
| Botao POWER                 | GP14       | Entrada com pull-up, IRQ borda descida |
| Aquecedor (LED vermelho)    | GP15       | PWM ~1 kHz                             |
| Ventilador (MOSFET IRLZ44N) | GP16       | PWM ~25 kHz (acima do audivel)         |
| Heartbeat (LED onboard)     | CYW43      | Via `cyw43_arch_gpio_put`              |

## Fiacao da UART (exigencia do professor)

A comunicacao do protocolo **tem que** sair pela UART por hardware, **nao
pela USB-CDC**. A cadeia fisica e:

```
Pico (TTL 3,3 V) -> MAX3232 -> cabo serial DB9 -> adaptador USB-RS232 -> PC
```

Cuidados:

- `VCC` do MAX3232 em **3,3 V** (nao 5 V).
- TX do Pico vai na entrada `T` do MAX3232; saida `R` do MAX3232 vai no RX do Pico (cruzamento).
- GND comum no lado TTL.
- Monitor serial no PC em **115200 8N1**.

A USB-CDC (`stdio_usb`) continua habilitada, mas **so para debug**: imprime
status legivel, aceita os mesmos comandos da UART e funciona como console de
desenvolvimento. O protocolo oficial trafega pela UART0.

## Build e gravacao

Ambiente:

- VS Code com a extensao oficial **Raspberry Pi Pico SDK**.
- Toolchain RISC-V (`PICO_PLATFORM=rp2350-riscv`, `PICO_BOARD=pico2_w`),
  definidos antes de `pico_sdk_init()` no `CMakeLists.txt`.
- `pico_sdk_import.cmake` no diretorio raiz (gerado automaticamente pela
  extensao; se faltar, copiar de `$PICO_SDK_PATH/external/`).

Fluxo:

1. `Raspberry Pi Pico: Configure CMake`
2. Selecionar o alvo (`teste_ap3`, `teste_integrado`, `teste_adc` ou `teste_uart`) e Compile.
3. Pico em modo **BOOTSEL** → Flash, ou arrastar manualmente o `.uf2` de
   `build/` para a unidade `RPI-RP2`.

Alvos definidos em `CMakeLists.txt`:

- `teste_ap3` — firmware principal (codigo deste repositorio).
- `teste_integrado`, `teste_adc`, `teste_uart` — alvos de apoio / fallback da AP2.

## Protocolo de comandos (UART e USB)

Um unico parser trata os dois canais; respostas saem nos dois. Os comandos
sao terminados em `\n` ou `\r` e tem o seguinte conjunto:

| Comando              | Efeito                                                                 |
|----------------------|------------------------------------------------------------------------|
| `MODO AUTO`          | Mesmo efeito do botao MODO: muda para AUTOMATICO.                      |
| `MODO MAN` / `MODO MANUAL` | Mesmo efeito do botao MODO: muda para MANUAL.                    |
| `SET TEMP <v>`       | Define o setpoint de temperatura (so em MANUAL).                       |
| `SET UMID <v>`       | Define o setpoint de umidade (so em MANUAL).                           |
| `TEST ON` / `TEST OFF` | Entra/sai do modo de teste de atuadores (suspende o controle auto).  |
| `AQ <0-100>`         | (modo teste) Forca duty do aquecedor.                                  |
| `VT <0-100>`         | (modo teste) Forca duty do ventilador.                                 |
| `UM <0-100>`         | (modo teste) Forca duty do umidificador.                               |
| `BZ <0\|1>`          | (modo teste) Liga/desliga o buzzer.                                    |
| `SWEEP`              | (modo teste) Varre 0→100→0 % em cada atuador.                          |
| `INTERLOCK`          | (modo teste) Forca AQ=VT=100 % para validar o intertravamento.         |

A cada amostragem (250 ms) o firmware tambem **emite** um pacote de status:

```
STATUS MODO=AUTO T=24.3/25.0C U=42.7/50.0% AQ=14 VT=0 UM=29
```

Quando o intertravamento dispara, o pacote termina em `ALARME`.

## LCD 16x2 (I2C)

LCD HD44780 com backpack PCF8574 (4 fios: 3V3, GND, SDA=GP4, SCL=GP5). O
endereco e descoberto no boot por varredura do barramento (0x08..0x77). Se
nenhum backpack responder, o firmware continua funcionando sem o LCD; o
ciclo de controle e a UART nao dependem dele.

Layout:

```
T 24.3> 25.0C
U 42.7> 50.0 %     (ou "ALM" no lugar de "%" em alarme)
```

## Notas de hardware aprendidas

- IRF520/IRF540N **nao** saturam com 3,3 V de GPIO. Usar MOSFETs de
  **nivel-logico**: **IRLZ44N**, IRLB8721 ou IRL520.
- Tacometro de ventilador (entregas futuras): pull-up para **3,3 V**, nunca
  5 V, para nao danificar o GPIO.
- **Wokwi** so simula RP2040 — da para usar como stand-in para
  ADC/UART/PWM/I2C (API do SDK compativel), mas nao testa o RP2350 em si.

## Regras de ISR (resumo)

- ISR **curta**: so flag/contador/toggle. ADC, UART, PWM e `printf` ficam no main.
- Variaveis compartilhadas ISR ↔ main marcadas `volatile`.
- Debounce dos botoes por `time_us_64` (nao usar `sleep`).
- A ISR do timer **nao** chama `cyw43_arch_gpio_put` (SPI lenta/bloqueante);
  o heartbeat e feito no main em resposta ao tick.

## Estrutura do repositorio

```
Trabalho_Micro/
├── CMakeLists.txt         # alvos teste_ap3, teste_integrado, teste_adc, teste_uart
├── pico_sdk_import.cmake  # gerado pela extensao do Pico SDK
├── Codigo_P2.c            # firmware principal (AP3)
├── CLAUDE.md              # contexto interno do projeto
└── README.md              # este arquivo
```
