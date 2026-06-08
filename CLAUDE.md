# CLAUDE.md — Projeto AP2 (BRUCUTU Sistemas Eletrônicos S.A.)

Este arquivo dá o contexto do projeto para o Claude Code. Leia antes de qualquer
alteração.

## Visão geral

Sistema de controle de clima (ventilação, aquecimento e umidade) baseado no
**Raspberry Pi Pico 2W (RP2350)**, configurado em **modo RISC-V (núcleos
Hazard3)**. Projeto de disciplina de sistemas embarcados, entregue em entregas
parciais (APs) para um professor.

A disciplina exige o uso de seis módulos do RP2350: ADC, Timer, Timer
Interrupt, PWM, External Interrupt e UART. **Esta entrega (AP2) cobre apenas
ADC e UART** — PWM, interrupções e timers ficam para entregas posteriores.

## Escopo desta entrega (AP2)

Dois programas de teste **isolados**, sem PWM, sem interrupções e sem timers:

- `teste_adc.c` — lê o LM35 (temperatura, GP26/ADC0) e um sensor de umidade
  resistivo analógico (GP27/ADC1), imprimindo as leituras a cada 500 ms pela
  USB-CDC.
- `teste_uart.c` — envia uma string de status periódica pela UART0 física e
  interpreta comandos recebidos (ex.: `SET TEMP <valor>`).

Não adicionar funcionalidades fora desse escopo nesta entrega.

## Decisões de projeto e restrições (NÃO alterar sem necessidade)

- Firmware em **C** com o **Pico SDK oficial** (não MicroPython).
- Compilação com o **toolchain RISC-V**:
  `PICO_PLATFORM=rp2350-riscv`, `PICO_BOARD=pico2_w`.
  Essas definições estão no `CMakeLists.txt` e devem vir **antes** de
  `pico_sdk_init()`.
- **UART por hardware obrigatória** (exigência explícita do professor):
  UART0 nos pinos **GP0 (TX) / GP1 (RX)**. Não usar USB-CDC como UART do
  protocolo. A USB-CDC (`stdio_usb`) fica habilitada apenas para debug,
  separada do protocolo.
- Cadeia física da UART (o professor pediu os dois adaptadores):
  Pico (TTL 3,3 V) → **MAX3232** → cabo serial DB9 → **adaptador USB-RS232** → PC.
  - VCC do MAX3232 em **3,3 V** (não 5 V).
  - Cruzamento: TX do Pico → entrada T do MAX3232; saída R do MAX3232 → RX do Pico.
  - GND comum no lado TTL.
  - Monitor serial do lado do PC: **115200 8N1**.
- **LM35 alimentado por VBUS (5 V)**, não pelo 3,3 V, para a faixa de saída
  ficar correta. A saída (10 mV/°C) entra no GP26/ADC0.
- **Sensor de umidade alimentado por 3,3 V** para proteger o pino do ADC; saída
  no GP27/ADC1.
- Recepção UART por **polling não-bloqueante** com `uart_is_readable` (nada de
  interrupções nesta etapa). Temporização por `absolute_time_diff_us` /
  `make_timeout_time_ms` (nada de timers de hardware nesta etapa).

## Estrutura de arquivos

```
AP2_Micro/
├── CMakeLists.txt          # dois alvos: teste_adc e teste_uart
├── pico_sdk_import.cmake   # gerado pela extensão do Pico SDK no VS Code
├── teste_adc.c
├── teste_uart.c
└── CLAUDE.md
```

## Ambiente e build

- IDE: **VS Code** com a extensão oficial **Raspberry Pi Pico SDK**.
- O `pico_sdk_import.cmake` é colocado automaticamente pela extensão ao
  criar/importar o projeto. Se faltar, copiar de
  `$PICO_SDK_PATH/external/pico_sdk_import.cmake`. Sem esse arquivo, o SDK
  nunca carrega e aparecem erros enganosos de linker (ex.:
  `undefined reference to stdio_init_all`).
- Fluxo na extensão:
  1. `Raspberry Pi Pico: Configure CMake`
  2. Escolher o alvo (`teste_adc` ou `teste_uart`) e Compile
  3. Pico em modo BOOTSEL → Flash, ou arrastar o `.uf2` de `build/` para
     a unidade `RPI-RP2`
- `teste_adc`: saída visível no **Serial Monitor** da porta USB-CDC do Pico.
- `teste_uart`: status e comandos pelo **monitor ligado ao adaptador
  USB-RS232** (não pela USB do Pico), a 115200 8N1. Enviar `SET TEMP 25`
  com quebra de linha (`\n` ou `\r`).

## Notas de hardware aprendidas (para etapas futuras, fora do escopo da AP2)

- **MOSFET para o ventilador**: IRF520/IRF540N **não** saturam com 3,3 V de
  GPIO. Usar nível-lógico: **IRLZ44N**, IRLB8721 ou IRL520.
- **Tacômetro do ventilador**: pull-up para **3,3 V** (nunca 5 V), para não
  danificar o GPIO.
- **Wokwi** só simula RP2040, não o RP2350. Dá para usar o RP2040 como
  stand-in para ADC/UART/PWM/I2C (API do SDK compatível).

## Sistema completo (horizonte, NÃO implementar na AP2)

Arquitetura em camadas (HAL), máquina de estados completa, controle de
atuadores por PWM, lógica de intertravamento de segurança (impedir
aquecedor + ventilador no máximo simultaneamente), rotina de intertravamento
em **Assembly RISC-V** (`asm/interlock.S`, convenção ABI ilp32), protocolo
UART bidirecional (envia leituras, recebe setpoints e limites de alarme),
LCD 16x2 via I2C, e controle de ventilador PWM com realimentação por
tacômetro. Aquecedor e umidificador são simulados por LEDs (vermelho e azul);
o ventilador é a única carga de potência real.
