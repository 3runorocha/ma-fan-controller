; ================ DEFINIÇÕES ================
.equ F_CPU = 16000000           ; Clock 16MHz
.equ DHT_PIN = PD2              ; DHT11 no pino D2 (PD2)
.equ DHT_DDR = DDRD
.equ DHT_PORT = PORTD
.equ DHT_PORTIN = PIND

.equ MODO_SIMULACAO = 1         ; 1 = simulação, 0 = sensor real

; Temperaturas simuladas (em graus Celsius)
.equ TEMP_SIM_1 = 25
.equ TEMP_SIM_2 = 28
.equ TEMP_SIM_3 = 32
.equ TEMP_SIM_4 = 35

; ================ SEGMENTO DE DADOS ================
.dseg
.org SRAM_START

temperatura_atual:  .byte 1     ; Temperatura atual em °C
umidade_atual:      .byte 1     ; Umidade atual em %
temp_sim_index:     .byte 1     ; Índice da temperatura simulada
contador_tempo:     .byte 2     ; Contador para troca de temperatura (word)
dht_data:           .byte 5     ; Buffer para 5 bytes do DHT11

; ================ SEGMENTO DE CÓDIGO ================
.cseg
.org 0x0000
    rjmp main                   ; Reset vector

; ================ PROGRAMA PRINCIPAL ================
main:
    ; Inicializa Stack Pointer
    ldi r16, LOW(RAMEND)
    out SPL, r16
    ldi r16, HIGH(RAMEND)
    out SPH, r16

    ; Inicializa variáveis
    rcall init_vars

    ; Configura PWM
    rcall init_pwm

    ; Configura DHT11
    rcall init_dht11

    ; Loop principal
loop_principal:
    .if MODO_SIMULACAO == 1
        rcall simula_temperatura
    .else
        rcall read_dht11_sensor
    .endif

    ; Calcula e ajusta PWM baseado na temperatura
    rcall ajusta_pwm_temp

    ; Delay de ~2 segundos (DHT11 requer mínimo 2s entre leituras)
    rcall delay_2s

    rjmp loop_principal

; ================ INICIALIZAÇÃO DE VARIÁVEIS ================
init_vars:
    ; Zera temperatura e índice
    ldi r16, 0
    sts temperatura_atual, r16
    sts temp_sim_index, r16

    ; Inicializa contador de tempo
    ldi r16, 0
    sts contador_tempo, r16
    sts contador_tempo+1, r16

    ret

; ================ INICIALIZAÇÃO PWM ================
; Configura Timer0 em Fast PWM mode, saída em OC0A (PD6)
init_pwm:
    ; Configura PD6 (OC0A) como saída
    sbi DDRD, PD6

    ; Configura Timer0 para Fast PWM mode
    ; WGM02:0 = 011 (Fast PWM, TOP = 0xFF)
    ; COM0A1:0 = 10 (Clear OC0A on Compare Match, set at BOTTOM - non-inverting)
    ldi r16, (1<<WGM01) | (1<<WGM00) | (1<<COM0A1)
    out TCCR0A, r16

    ; Prescaler = 64 (CS02:0 = 011)
    ; Frequência PWM = 16MHz / (64 * 256) ≈ 976 Hz
    ldi r16, (1<<CS01) | (1<<CS00)
    out TCCR0B, r16

    ; Duty cycle inicial = 0 (fan desligado)
    ldi r16, 0
    out OCR0A, r16

    ret

; ================ INICIALIZAÇÃO DHT11 ================
init_dht11:
    ; Configura pino DHT como saída (pull-up inicial)
    sbi DHT_DDR, DHT_PIN
    sbi DHT_PORT, DHT_PIN
    ret

; ================ SIMULAÇÃO DE TEMPERATURA ================
; Simula temperaturas variando a cada 5 ciclos (5 x 2s = 10s)
simula_temperatura:
    push r16
    push r17

    ; Incrementa contador
    lds r16, contador_tempo
    lds r17, contador_tempo+1
    adiw r16, 1
    sts contador_tempo, r16
    sts contador_tempo+1, r17

    ; Verifica se passou 5 ciclos (10 segundos)
    cpi r16, LOW(5)
    ldi r18, HIGH(5)
    cpc r17, r18
    brlo simula_temp_end

    ; Reseta contador
    ldi r16, 0
    sts contador_tempo, r16
    sts contador_tempo+1, r16

    ; Avança índice de temperatura
    lds r16, temp_sim_index
    inc r16
    cpi r16, 4              ; 4 temperaturas diferentes
    brlo simula_temp_store
    ldi r16, 0              ; Volta ao início

simula_temp_store:
    sts temp_sim_index, r16

    ; Seleciona temperatura baseada no índice
    cpi r16, 0
    breq simula_temp_0
    cpi r16, 1
    breq simula_temp_1
    cpi r16, 2
    breq simula_temp_2
    rjmp simula_temp_3

simula_temp_0:
    ldi r16, TEMP_SIM_1
    rjmp simula_temp_save

simula_temp_1:
    ldi r16, TEMP_SIM_2
    rjmp simula_temp_save

simula_temp_2:
    ldi r16, TEMP_SIM_3
    rjmp simula_temp_save

simula_temp_3:
    ldi r16, TEMP_SIM_4

simula_temp_save:
    sts temperatura_atual, r16

simula_temp_end:
    pop r17
    pop r16
    ret

; ================ LEITURA DHT11 ================
; Lê temperatura e umidade do sensor DHT11
read_dht11_sensor:
    push r16
    push r17
    push r18
    push r19
    push r20

    ; Envia sinal de START (LOW por 18ms)
    sbi DHT_DDR, DHT_PIN        ; Pino como saída
    cbi DHT_PORT, DHT_PIN       ; LOW
    rcall delay_18ms

    ; Pull-up por 20-40us
    sbi DHT_PORT, DHT_PIN       ; HIGH
    cbi DHT_DDR, DHT_PIN        ; Pino como entrada
    rcall delay_40us

    ; Aguarda resposta do DHT11 (80us LOW + 80us HIGH)
    rcall dht_wait_low
    rcall dht_wait_high
    rcall dht_wait_low

    ; Lê 40 bits de dados (5 bytes)
    ldi r20, 5                  ; 5 bytes
    ldi XL, LOW(dht_data)
    ldi XH, HIGH(dht_data)

read_dht_byte_loop:
    rcall dht_read_byte
    st X+, r16
    dec r20
    brne read_dht_byte_loop

    ; Valida checksum
    ; checksum = byte[0] + byte[1] + byte[2] + byte[3]
    ldi XL, LOW(dht_data)
    ldi XH, HIGH(dht_data)
    ld r16, X+
    ld r17, X+
    add r16, r17
    ld r17, X+
    add r16, r17
    ld r17, X+
    add r16, r17
    ld r17, X                   ; Checksum byte
    cp r16, r17
    brne read_dht_error

    ; Extrai dados (byte 2 = temperatura inteira)
    ldi XL, LOW(dht_data)
    ldi XH, HIGH(dht_data)
    adiw X, 2
    ld r16, X
    sts temperatura_atual, r16

    rjmp read_dht_end

read_dht_error:
    ; Em caso de erro, mantém última leitura válida
    nop

read_dht_end:
    pop r20
    pop r19
    pop r18
    pop r17
    pop r16
    ret

; Lê 1 byte (8 bits) do DHT11
dht_read_byte:
    push r17
    push r18
    ldi r17, 8              ; 8 bits
    ldi r16, 0              ; Acumulador

dht_read_bit_loop:
    rcall dht_wait_high

    ; Delay 30us para verificar duração do HIGH
    ldi r18, 80             ; ~30us @ 16MHz
dht_bit_delay:
    dec r18
    brne dht_bit_delay

    ; Se ainda HIGH, bit = 1, senão bit = 0
    lsl r16                 ; Shift left
    sbic DHT_PORTIN, DHT_PIN
    ori r16, 1

    rcall dht_wait_low

    dec r17
    brne dht_read_bit_loop

    pop r18
    pop r17
    ret

; Aguarda pino ir para LOW
dht_wait_low:
    sbic DHT_PORTIN, DHT_PIN
    rjmp dht_wait_low
    ret

; Aguarda pino ir para HIGH
dht_wait_high:
    sbis DHT_PORTIN, DHT_PIN
    rjmp dht_wait_high
    ret

; ================ AJUSTE PWM BASEADO NA TEMPERATURA ================
; Mapeia temperatura (20-40°C) para PWM (0-255)
ajusta_pwm_temp:
    push r16
    push r17
    push r18

    lds r16, temperatura_atual

    ; Se temp < 24°C, PWM = 0 (fan desligado)
    cpi r16, 24
    brlo pwm_min

    ; Se temp >= 36°C, PWM = 255 (fan máximo)
    cpi r16, 36
    brsh pwm_max

    ; Mapeia 24-35°C para 0-255
    ; PWM = (temp - 24) * 23
    subi r16, 24            ; temp - 24
    ldi r17, 23
    mul r16, r17            ; resultado em r1:r0
    mov r16, r0             ; Usa byte baixo (suficiente para 0-255)
    rjmp pwm_set

pwm_min:
    ldi r16, 0
    rjmp pwm_set

pwm_max:
    ldi r16, 255

pwm_set:
    out OCR0A, r16          ; Ajusta duty cycle

    pop r18
    pop r17
    pop r16
    ret

; ================ DELAYS ================
; Delay ~2 segundos
delay_2s:
    push r16
    push r17
    push r18

    ldi r18, 20             ; Loop externo
delay_2s_outer:
    ldi r17, 200            ; Loop médio
delay_2s_middle:
    ldi r16, 250            ; Loop interno
delay_2s_inner:
    dec r16
    brne delay_2s_inner
    dec r17
    brne delay_2s_middle
    dec r18
    brne delay_2s_outer

    pop r18
    pop r17
    pop r16
    ret

; Delay ~18ms
delay_18ms:
    push r16
    push r17

    ldi r17, 180
delay_18ms_outer:
    ldi r16, 250
delay_18ms_inner:
    dec r16
    brne delay_18ms_inner
    dec r17
    brne delay_18ms_outer

    pop r17
    pop r16
    ret

; Delay ~40us
delay_40us:
    push r16
    ldi r16, 160            ; ~40us @ 16MHz
delay_40us_loop:
    dec r16
    brne delay_40us_loop
    pop r16
    ret
