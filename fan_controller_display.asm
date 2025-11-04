; ================ DEFINIÇÕES ================
.equ F_CPU = 16000000           ; Clock 16MHz
.equ DHT_PIN = PD2              ; DHT11 no pino D2 (PD2)
.equ DHT_DDR = DDRD
.equ DHT_PORT = PORTD
.equ DHT_PORTIN = PIND

.equ LCD_ADDR = 0x27            ; Endereço I2C do LCD (0x27 ou 0x3F)
.equ LCD_BACKLIGHT = 0x08       ; Bit de backlight
.equ LCD_EN = 0x04              ; Bit Enable
.equ LCD_RW = 0x02              ; Bit Read/Write
.equ LCD_RS = 0x01              ; Bit Register Select

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
pwm_percent:        .byte 1     ; Percentual do PWM (0-100)

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
    
    ; Configura I2C e LCD
    rcall init_i2c
    rcall init_lcd
    
    ; Mensagem inicial no LCD
    rcall lcd_inicial

    ; Loop principal
loop_principal:
    .if MODO_SIMULACAO == 1
        rcall simula_temperatura
    .else
        rcall read_dht11_sensor
    .endif
    
    ; Calcula e ajusta PWM baseado na temperatura
    rcall ajusta_pwm_temp
    
    ; Atualiza display LCD
    rcall atualiza_display
    
    ; Delay de ~2 segundos
    rcall delay_2s
    
    rjmp loop_principal

; ================ INICIALIZAÇÃO DE VARIÁVEIS ================
init_vars:
    ldi r16, 0
    sts temperatura_atual, r16
    sts temp_sim_index, r16
    sts pwm_percent, r16
    sts contador_tempo, r16
    sts contador_tempo+1, r16
    ret

; ================ INICIALIZAÇÃO I2C ================
init_i2c:
    ; Configura SCL (PC5) e SDA (PC4) como saídas
    sbi DDRC, PC5               ; SCL
    sbi DDRC, PC4               ; SDA
    
    ; Pull-ups
    sbi PORTC, PC5
    sbi PORTC, PC4
    
    ; Configura TWI bit rate = 100kHz
    ; SCL freq = F_CPU / (16 + 2*TWBR*Prescaler)
    ; TWBR = ((F_CPU/SCL_freq) - 16) / 2 = ((16000000/100000) - 16) / 2 = 72
    ldi r16, 72
    sts TWBR, r16
    
    ; Prescaler = 1
    ldi r16, 0
    sts TWSR, r16
    
    ; Habilita TWI
    ldi r16, (1<<TWEN)
    sts TWCR, r16
    
    ret

; ================ INICIALIZAÇÃO PWM ================
init_pwm:
    sbi DDRD, PD6
    ldi r16, (1<<WGM01) | (1<<WGM00) | (1<<COM0A1)
    out TCCR0A, r16
    ldi r16, (1<<CS01) | (1<<CS00)
    out TCCR0B, r16
    ldi r16, 0
    out OCR0A, r16
    ret

; ================ INICIALIZAÇÃO DHT11 ================
init_dht11:
    sbi DHT_DDR, DHT_PIN
    sbi DHT_PORT, DHT_PIN
    ret

; ================ FUNÇÕES I2C ================
; Envia START condition
i2c_start:
    ldi r16, (1<<TWINT) | (1<<TWSTA) | (1<<TWEN)
    sts TWCR, r16
i2c_start_wait:
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp i2c_start_wait
    ret

; Envia STOP condition
i2c_stop:
    ldi r16, (1<<TWINT) | (1<<TWSTO) | (1<<TWEN)
    sts TWCR, r16
    ret

; Envia byte via I2C (r16 = byte)
i2c_write:
    sts TWDR, r16
    ldi r16, (1<<TWINT) | (1<<TWEN)
    sts TWCR, r16
i2c_write_wait:
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp i2c_write_wait
    ret

; ================ FUNÇÕES LCD I2C ================
; Envia nibble para LCD (r16 = nibble nos 4 bits superiores)
lcd_send_nibble:
    push r17
    ori r16, LCD_BACKLIGHT      ; Mantém backlight ligado
    mov r17, r16
    
    ; Envia com EN=1
    ori r17, LCD_EN
    rcall i2c_start
    ldi r16, (LCD_ADDR << 1)    ; Endereço + Write bit
    rcall i2c_write
    mov r16, r17
    rcall i2c_write
    rcall i2c_stop
    rcall delay_1us
    
    ; Envia com EN=0
    andi r17, ~LCD_EN
    rcall i2c_start
    ldi r16, (LCD_ADDR << 1)
    rcall i2c_write
    mov r16, r17
    rcall i2c_write
    rcall i2c_stop
    rcall delay_1us
    
    pop r17
    ret

; Envia comando para LCD (r16 = comando)
lcd_command:
    push r17
    mov r17, r16
    
    ; Envia nibble alto
    andi r16, 0xF0
    rcall lcd_send_nibble
    
    ; Envia nibble baixo
    mov r16, r17
    swap r16
    andi r16, 0xF0
    rcall lcd_send_nibble
    
    rcall delay_2ms
    pop r17
    ret

; Envia dado (caractere) para LCD (r16 = caractere ASCII)
lcd_data:
    push r17
    mov r17, r16
    
    ; Envia nibble alto com RS=1
    andi r16, 0xF0
    ori r16, LCD_RS
    rcall lcd_send_nibble
    
    ; Envia nibble baixo com RS=1
    mov r16, r17
    swap r16
    andi r16, 0xF0
    ori r16, LCD_RS
    rcall lcd_send_nibble
    
    rcall delay_1ms
    pop r17
    ret

; Inicializa LCD em modo 4-bit
init_lcd:
    rcall delay_50ms
    
    ; Sequência de inicialização 4-bit
    ldi r16, 0x30
    rcall lcd_send_nibble
    rcall delay_5ms
    
    ldi r16, 0x30
    rcall lcd_send_nibble
    rcall delay_1ms
    
    ldi r16, 0x30
    rcall lcd_send_nibble
    rcall delay_1ms
    
    ldi r16, 0x20               ; Modo 4-bit
    rcall lcd_send_nibble
    rcall delay_1ms
    
    ; Configuração do LCD
    ldi r16, 0x28               ; 4-bit, 2 linhas, 5x8
    rcall lcd_command
    
    ldi r16, 0x0C               ; Display ON, cursor OFF
    rcall lcd_command
    
    ldi r16, 0x06               ; Entry mode: incrementa, sem shift
    rcall lcd_command
    
    ldi r16, 0x01               ; Clear display
    rcall lcd_command
    rcall delay_2ms
    
    ret

; Posiciona cursor (r16 = linha [0-1], r17 = coluna [0-15])
lcd_set_cursor:
    push r18
    cpi r16, 0
    breq lcd_cursor_linha0
    ldi r18, 0xC0               ; Linha 1: 0xC0
    rjmp lcd_cursor_send
lcd_cursor_linha0:
    ldi r18, 0x80               ; Linha 0: 0x80
lcd_cursor_send:
    add r18, r17
    mov r16, r18
    rcall lcd_command
    pop r18
    ret

; Limpa LCD
lcd_clear:
    ldi r16, 0x01
    rcall lcd_command
    rcall delay_2ms
    ret

; Exibe string da memória de programa
; Z aponta para string (terminada em 0)
lcd_print_string:
    push r16
lcd_print_loop:
    lpm r16, Z+
    cpi r16, 0
    breq lcd_print_end
    rcall lcd_data
    rjmp lcd_print_loop
lcd_print_end:
    pop r16
    ret

; Exibe número de 2 dígitos (r16 = número 0-99)
lcd_print_number:
    push r16
    push r17
    push r18
    
    ; Divide por 10
    ldi r17, 10
    clr r18
lcd_div_loop:
    cp r16, r17
    brlo lcd_div_done
    sub r16, r17
    inc r18
    rjmp lcd_div_loop
lcd_div_done:
    ; r18 = dezenas, r16 = unidades
    
    ; Exibe dezena
    mov r19, r16
    mov r16, r18
    subi r16, -'0'
    rcall lcd_data
    
    ; Exibe unidade
    mov r16, r19
    subi r16, -'0'
    rcall lcd_data
    
    pop r18
    pop r17
    pop r16
    ret

; Exibe número de 3 dígitos (r16 = número 0-255)
lcd_print_number3:
    push r16
    push r17
    push r18
    push r19
    
    clr r18                     ; Centenas
    clr r19                     ; Dezenas
    
    ; Conta centenas
lcd_cent_loop:
    cpi r16, 100
    brlo lcd_dez_start
    subi r16, 100
    inc r18
    rjmp lcd_cent_loop
    
    ; Conta dezenas
lcd_dez_start:
    cpi r16, 10
    brlo lcd_uni_start
    subi r16, 10
    inc r19
    rjmp lcd_dez_start
    
lcd_uni_start:
    ; r18=centenas, r19=dezenas, r16=unidades
    push r16
    
    mov r16, r18
    subi r16, -'0'
    rcall lcd_data
    
    mov r16, r19
    subi r16, -'0'
    rcall lcd_data
    
    pop r16
    subi r16, -'0'
    rcall lcd_data
    
    pop r19
    pop r18
    pop r17
    pop r16
    ret

; Mensagem inicial
lcd_inicial:
    rcall lcd_clear
    ldi r16, 0
    ldi r17, 1
    rcall lcd_set_cursor
    ldi ZL, LOW(2*msg_init)
    ldi ZH, HIGH(2*msg_init)
    rcall lcd_print_string
    rcall delay_2s
    ret

; Atualiza display com temperatura e fan
atualiza_display:
    push r16
    push r17
    
    ; Linha 0: "Temp: XXC"
    ldi r16, 0
    ldi r17, 0
    rcall lcd_set_cursor
    ldi ZL, LOW(2*msg_temp)
    ldi ZH, HIGH(2*msg_temp)
    rcall lcd_print_string
    
    lds r16, temperatura_atual
    rcall lcd_print_number
    
    ldi r16, 'C'
    rcall lcd_data
    
    ; Linha 1: "Fan: XXX%"
    ldi r16, 1
    ldi r17, 0
    rcall lcd_set_cursor
    ldi ZL, LOW(2*msg_fan)
    ldi ZH, HIGH(2*msg_fan)
    rcall lcd_print_string
    
    lds r16, pwm_percent
    rcall lcd_print_number3
    
    ldi r16, '%'
    rcall lcd_data
    
    pop r17
    pop r16
    ret

; Strings na memória de programa
msg_init:   .db "Termostato", 0
msg_temp:   .db "Temp: ", 0
msg_fan:    .db "Fan: ", 0

; ================ SIMULAÇÃO DE TEMPERATURA ================
simula_temperatura:
    push r16
    push r17
    push r24
    push r25

    lds r24, contador_tempo
    lds r25, contador_tempo+1
    adiw r24, 1
    sts contador_tempo, r24
    sts contador_tempo+1, r25

    cpi r24, LOW(5)
    ldi r16, HIGH(5)
    cpc r25, r16
    brlo simula_temp_end

    ldi r24, 0
    ldi r25, 0
    sts contador_tempo, r24
    sts contador_tempo+1, r25

    lds r16, temp_sim_index
    inc r16
    cpi r16, 4
    brlo simula_temp_store
    ldi r16, 0

simula_temp_store:
    sts temp_sim_index, r16

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
    pop r25
    pop r24
    pop r17
    pop r16
    ret

; ================ LEITURA DHT11 ================
read_dht11_sensor:
    push r16
    push r17
    push r18
    push r19
    push r20

    sbi DHT_DDR, DHT_PIN
    cbi DHT_PORT, DHT_PIN
    rcall delay_18ms

    sbi DHT_PORT, DHT_PIN
    cbi DHT_DDR, DHT_PIN
    rcall delay_40us

    rcall dht_wait_low
    rcall dht_wait_high
    rcall dht_wait_low

    ldi r20, 5
    ldi XL, LOW(dht_data)
    ldi XH, HIGH(dht_data)

read_dht_byte_loop:
    rcall dht_read_byte
    st X+, r16
    dec r20
    brne read_dht_byte_loop

    ldi XL, LOW(dht_data)
    ldi XH, HIGH(dht_data)
    ld r16, X+
    ld r17, X+
    add r16, r17
    ld r17, X+
    add r16, r17
    ld r17, X+
    add r16, r17
    ld r17, X
    cp r16, r17
    brne read_dht_error

    ldi XL, LOW(dht_data)
    ldi XH, HIGH(dht_data)
    adiw X, 2
    ld r16, X
    sts temperatura_atual, r16

    rjmp read_dht_end

read_dht_error:
    nop

read_dht_end:
    pop r20
    pop r19
    pop r18
    pop r17
    pop r16
    ret

dht_read_byte:
    push r17
    push r18
    ldi r17, 8
    ldi r16, 0

dht_read_bit_loop:
    rcall dht_wait_high

    ldi r18, 80
dht_bit_delay:
    dec r18
    brne dht_bit_delay

    lsl r16
    sbic DHT_PORTIN, DHT_PIN
    ori r16, 1

    rcall dht_wait_low

    dec r17
    brne dht_read_bit_loop

    pop r18
    pop r17
    ret

dht_wait_low:
    sbic DHT_PORTIN, DHT_PIN
    rjmp dht_wait_low
    ret

dht_wait_high:
    sbis DHT_PORTIN, DHT_PIN
    rjmp dht_wait_high
    ret

; ================ AJUSTE PWM BASEADO NA TEMPERATURA ================
ajusta_pwm_temp:
    push r16
    push r17
    push r18
    push r19

    lds r16, temperatura_atual

    cpi r16, 24
    brlo pwm_min

    cpi r16, 36
    brsh pwm_max

    subi r16, 24
    ldi r17, 23
    mul r16, r17
    mov r16, r0
    
    ; Calcula percentual (PWM * 100 / 255)
    mov r18, r16
    ldi r19, 100
    mul r18, r19
    mov r18, r0
    mov r19, r1
    ldi r17, 255
    
    ; Divisão r19:r18 / r17 (aproximação)
    clr r16
pwm_div_loop:
    cp r18, r17
    cpc r19, r1
    brlo pwm_div_done
    sub r18, r17
    sbc r19, r1
    inc r16
    rjmp pwm_div_loop
pwm_div_done:
    sts pwm_percent, r16
    
    mov r16, r0
    rjmp pwm_set

pwm_min:
    ldi r16, 0
    sts pwm_percent, r16
    rjmp pwm_set

pwm_max:
    ldi r16, 255
    ldi r17, 100
    sts pwm_percent, r17

pwm_set:
    out OCR0A, r16

    pop r19
    pop r18
    pop r17
    pop r16
    ret

; ================ DELAYS ================
delay_2s:
    push r16
    push r17
    push r18
    ldi r18, 20
delay_2s_outer:
    ldi r17, 200
delay_2s_middle:
    ldi r16, 250
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

delay_50ms:
    push r16
    push r17
    ldi r17, 250
delay_50ms_outer:
    ldi r16, 200
delay_50ms_inner:
    dec r16
    brne delay_50ms_inner
    dec r17
    brne delay_50ms_outer
    pop r17
    pop r16
    ret

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

delay_5ms:
    push r16
    push r17
    ldi r17, 50
delay_5ms_outer:
    ldi r16, 250
delay_5ms_inner:
    dec r16
    brne delay_5ms_inner
    dec r17
    brne delay_5ms_outer
    pop r17
    pop r16
    ret

delay_2ms:
    push r16
    push r17
    ldi r17, 20
delay_2ms_outer:
    ldi r16, 250
delay_2ms_inner:
    dec r16
    brne delay_2ms_inner
    dec r17
    brne delay_2ms_outer
    pop r17
    pop r16
    ret

delay_1ms:
    push r16
    push r17
    ldi r17, 10
delay_1ms_outer:
    ldi r16, 250
delay_1ms_inner:
    dec r16
    brne delay_1ms_inner
    dec r17
    brne delay_1ms_outer
    pop r17
    pop r16
    ret

delay_40us:
    push r16
    ldi r16, 160
delay_40us_loop:
    dec r16
    brne delay_40us_loop
    pop r16
    ret

delay_1us:
    push r16
    ldi r16, 4
delay_1us_loop:
    dec r16
    brne delay_1us_loop
    pop r16
    ret