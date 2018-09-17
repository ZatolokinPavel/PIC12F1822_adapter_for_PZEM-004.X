/*
 * File:   main.c
 * Author: Затолокин Павел
 * 
 * PIC12F1822 / Int OSC @ 16MHz, 5V
 * Compiler: XC8 v2.00, MPLAB X IDE v5.05
 *
 * Created on 21 августа 2018 г., 23:24
 * 
 * Прошивка для контроллера PIC12F1822 - адаптер для подключения
 * датчика тока и напряжения 220В PZEM-004T к центральному
 * компьютеру Raspberry Pi по шине RS-232
 */


// Конфигурация контроллера (регистры CONFIG1 и CONFIG2)
#pragma config FCMEN    = ON    // Fail-Safe Clock Monitor
#pragma config IESO     = ON    // Internal External Switchover
#pragma config CLKOUTEN = OFF   // Clock Out to pin
#pragma config BOREN    = ON    // Сброс по снижению напряжения питания (BOR)
#pragma config CPD      = ON    // Защита памяти данных (EEPROM)
#pragma config CP       = ON    // Защита памяти программ
#pragma config MCLRE    = ON    // RA3/MCLR/VPP pin function is MCLR
#pragma config PWRTE    = ON    // Power-up Timer
#pragma config WDTE     = OFF   // Watchdog Timer
#pragma config FOSC     = INTOSC// Тактовый генератор (INTOSC - внутренний 20 МГц)

#pragma config LVP      = OFF   // Low-Voltage Programming
#pragma config BORV     = LO    // Brown-out Reset Voltage Selection
#pragma config STVREN   = ON    // Stack Overflow or Underflow will cause a Reset
#pragma config PLLEN    = ON    // фазовая автоподстройка частоты (ФАПЧ) (Phased-Locked Loop)
#pragma config WRT      = OFF   // Flash Memory Self-Write Protection

// Подключаем заголовочные файлы компилятора
#include <xc.h>             // стандартный для компилятора MPLAB XC8
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pic.h>            // общий для всех PIC
#include <pic12f1822.h>

#define _XTAL_FREQ 16000000 // указываем рабочую частоту для функций задержки
#define _UART_ADDRESS 002   // адрес этого устройства будет 2.
// Состояния приёма сообщений по шине RS-485
#define _UART_RC_ADDR 0     // ожидаем адрес получателя
#define _UART_RC_SIZE 1     // ожидаем размер сообщения
#define _UART_RC_MSG  2     // ожидаем очередной байт сообщения
#define _UART_RC_SUM  3     // ожидаем контрольную сумму
#define _UART_RC_ERR  4     // уже ничё не ожидаем, словили ошибку

// Объявление глобальных переменных
unsigned char i;            // Переменная «i» - уже более 25 лет на рынке счетчиков!
char temp = 0;              // временная переменная
char test_data[7] = {167,18,33,75,99,100,107};  // массив тестовых символов
char uart_TX_buffer[7];     // буфер для хранения передаваемых данных
char uart_next_TX = 0;      // номер следующего символа, который будет передан по UART
char uart_next_RC = 0;      // номер следующего символа, который будет принят по UART
char uart_RC_state = 0;     // флаг состояния приёма сообщений по UART
bool uart_RC_to_me = false; // этому ли устройству передают сообщение по RS-485?
char uart_RC_msg_size = 0;  // количество байт сообщения, которое нужно принять по UART
char uart_RC_message[1];    // собственно получаемое сообщение из шины RS-485

// Объявление всех функций, которые написаны после их вызова
void init_UART(void);
char read_char_UART(void);
void read_full_mess_UART(char);
void action_on_msg_UART(void);
void write_UART(char[7]);
void continue_write_UART(void);



// =============================================================================
// ============================ Заголовок закончен =============================
// =================== далее идёт собственно текст программы ===================
// =============================================================================



// Главная функция программы. С неё начинается выполнение программы.
void main(void) {
    OSCCONbits.IRCF = 0b1111;   // Internal Oscillator Frequency = 16 MHz HF
    OSCCONbits.SCS = 0b00;      // System Clock determined by FOSC<2:0> in Configuration Word 1.
    INTCONbits.GIE = 1;         // глобально разрешаем прерывания
    INTCONbits.PEIE = 1;        // разрешаем прерывания от периферии
    
    // Тестовые выходы
    TRISAbits.TRISA1 = 0;       // пин RA1 (ножка 6) на выход
    TRISAbits.TRISA2 = 0;       // пин RA2 (ножка 5) на выход
    ANSELAbits.ANSA1 = 0;       // пин RA1 как дискретный I/O (не аналоговый)
    ANSELAbits.ANSA2 = 0;       // пин RA2 как дискретный I/O (не аналоговый)
    PORTAbits.RA1 = 1;          // для начала зажигаем светодиод
    PORTAbits.RA2 = 1;
    // Для наглядности помигаем
    __delay_ms(500);
    RA2 = 0;
    __delay_ms(500);
    RA2 = 1;
    
    init_UART();                // конфигурируем порт RS-232
    
    while(1) {
    }
}


// Main Interrupt Service Routine (ISR)
void __interrupt() ISR(void) {
    if(PIR1bits.RCIF) {             // если прерывание от принятого байта по RS-232 (UART)
//        write_UART(test_data);
        temp = read_char_UART();    // читаем принятый байт
        read_full_mess_UART(temp);  // смотрим значение принятого байта и записываем полное сообщение
        action_on_msg_UART();
    }
    
//    if(PIR1bits.TXIF) {         // если это прерывание от передачи по UART
//        continue_write_UART();  // то загружаем следующий символ в очередь передачи
//    }
}


// Конфигурируем EUSART как асинхронный порт RS-232 в режиме slave
// SYNC=0, BRG16=0, BRGH=1: Baud Rate =  Fosc/(16*(SPBRG+1)) = 16000000/(16*(103+1)) = 9615.38 bit per second
void init_UART(void) {
    APFCONbits.RXDTSEL = 1;     // RX function is on RA5 (вместо RA1 по умолчанию)
    APFCONbits.TXCKSEL = 1;     // TX function is on RA4 (вместо RA0 по умолчанию)
    TRISAbits.TRISA4 = 0;       // TX Pin set as output
    TRISAbits.TRISA5 = 1;       // RX Pin set as input
    ANSELAbits.ANSA4 = 0;       // Digital I/O (регистр ANSELA)
    PIE1bits.TXIE = 0;          // пока передавать нечего, отключаем USART transmit interrupt
    PIE1bits.RCIE = 1;          // Enables the USART receive interrupt
    BAUDCONbits.SCKP  = 0;      // Transmit non-inverted data to the TX/CK pin
    BAUDCONbits.BRG16 = 0;      // 8-bit Baud Rate Generator is used
    BAUDCONbits.WUE   = 0;      // Wake-up Enable bit = Receiver is operating normally
    BAUDCONbits.ABDEN = 0;      // Auto-Baud Detect mode is disabled
    SPBRGH = 0;                 // determines the period of the free running baud rate timer
    SPBRGL = 103;               // determines the period of the free running baud rate timer
    // регистр TXSTA: transmit status and control register
    TXSTAbits.BRGH = 1;         // High Baud Rate Select bit
    TXSTAbits.TX9  = 0;         // 9-bit Transmit Enable bit
    TXSTAbits.SYNC = 0;         // EUSART Mode Select = Asynchronous mode
    TXSTAbits.TRMT = 1;         // на всякий случай Transmit Shift Register Status = empty
    TXSTAbits.TXEN = 1;         // Transmit Enable bit
    // регистр RCSTA: receive status and control register
    RCSTAbits.RX9  = 0;         // Selects 8-bit reception
    RCSTAbits.SPEN = 1;         // Serial port enabled (configures RX/DT and TX/CK pins as serial port pins)
    RCSTAbits.CREN = 1;         // Enables receiver
}


// Получение данных по шине RS-232
char read_char_UART(void) {
    if(RCSTAbits.OERR) {        // check for Receive Overrun Error 
        RCSTAbits.CREN = 0;     // if error -> restart UART чтобы дальше принимать данные
        RCSTAbits.CREN = 1;
    }
    RCSTAbits.FERR;             // надо будет прочитать и обработать ошибку фрейма
    return RCREG;               // receive the value and send it to main function
}

// Смотрим значение принятого байта - это адрес, размер или тело сообщения.
// И формируем полное сообщение.
void read_full_mess_UART(char byte) {
    switch(uart_RC_state) {
        case _UART_RC_ADDR:                 // если ожидали адрес получателя
            uart_RC_to_me = (byte == _UART_ADDRESS);    // определяем, этому ли устройству адресовано сообщение?
            uart_RC_state = _UART_RC_SIZE;              // следующий байт будет размер сообщения
            break;
        case _UART_RC_SIZE:                 // если ожидали размер сообщения
            uart_RC_msg_size = byte;
            if(uart_RC_msg_size != 1)       // если размер сообщения не такой как мы можем принять
                uart_RC_to_me = false;      // то считаем что это не нам
            uart_RC_state = _UART_RC_MSG;   // следующий байт - это уже данные
            break;
        case _UART_RC_MSG:                  // если ожидали очередной байт сообщения
            if(uart_RC_to_me)
                uart_RC_message[0] = byte;
            uart_RC_msg_size--;
            if(uart_RC_msg_size == 0)
                uart_RC_state = _UART_RC_SUM;
            break;
        case _UART_RC_SUM:                  // если ожидали контрольную сумму
            // TODO: нужно проверить контрольную сумму полученного сообщения
            uart_RC_state = _UART_RC_ADDR;
            break;
        case _UART_RC_ERR:      // если уже словили ошибку
            // TODO: надо продумать поведение
            break;
    }
}

void action_on_msg_UART(void) {
    if(uart_RC_state != _UART_RC_ADDR) return;      // сообщение ещё не принято
    if(!uart_RC_to_me) return;                      // сообщение не нам
    switch (uart_RC_message[0]) {
        case 0x34:
            RA1 = 0;
            RA2 = 1;
            break;
        case 0x78:
            RA1 = 1;
            RA2 = 0;
            break;
        case 0xff:
            RA1 = 1;
            RA2 = 1;
            break;
        default:
            RA1 = 0;
            RA2 = 0;
    }
}

// Передача массива из семи символов на шину RS-232
void write_UART(char array[7]) {
    if(uart_next_TX != 0) return;   // занято
    for(i=0; i < 7; i++)
        uart_TX_buffer[i] = array[i];
    TXREG = uart_TX_buffer[0];  // первый символ на передачу, и он сразу ушел в TSR
    TXREG = uart_TX_buffer[1];  // второй символ в очередь на передачу
    uart_next_TX = 2;           // номер следующего на передачу символа
    PIE1bits.TXIE = 1;          // включаем прерывание когда очередь из TXREG уйдёт на передачу
}

void continue_write_UART(void) {
    TXREG = uart_TX_buffer[uart_next_TX];   // ставим в очередь на передачу следующий символ
    uart_next_TX += 1;
    if(uart_next_TX == 7) {     // был передан последний символ
        uart_next_TX = 0;       // символов на передачу больше нет
        PIE1bits.TXIE = 0;      // закончили передачу, прерывание нам уже не нужно
    }
}
