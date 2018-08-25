/*
 * File:   main.c
 * Author: Затолокин Павел
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
#include <pic.h>            // общий для всех PIC
#include <pic12f1822.h>

#define _XTAL_FREQ 20000000 // указываем рабочую частоту для функций задержки

// Объявление глобальных переменных
unsigned char temp = 0;			// временная переменная

// Объявление всех функций, которые написаны после их вызова
void init_UART(void);
char read_UART(void);



// =============================================================================
// ============================ Заголовок закончен =============================
// =================== далее идёт собственно текст программы ===================
// =============================================================================



// Главная функция программы. С неё начинается выполнение программы.
void main(void) {
    INTCONbits.GIE = 1;         // глобально разрешаем прерывания
    INTCONbits.PEIE = 1;        // разрешаем прерывания от периферии
    TRISA   = 0x00;         // порт A весь на выход
    RA2     = 1;
    
    init_UART();            // конфигурируем порт RS-232
    
    while(1) {
//        RA2 = BAUDCONbits.RCIDL;
    }
}


// Main Interrupt Service Routine (ISR)
void __interrupt() ISR(void) {
//    if(RCIF) {              // если прерывание от принятого байта по RS-232 (UART)
        read_UART();        // читаем что же там пришло и реагируем
//    }
}


// Конфигурируем EUSART как асинхронный порт RS-232 в режиме slave
void init_UART(void) {
    TRISAbits.TRISA4 = 0;       // TX Pin set as output
    TRISAbits.TRISA5 = 1;       // RX Pin set as input
    ANSELAbits.ANSA4 = 0;       // Digital I/O (регистр ANSELA)
//    PIE1bits.TXIE = 1;          // Enables the USART transmit interrupt
    PIE1bits.RCIE = 1;          // Enables the USART receive interrupt
    BAUDCONbits.SCKP  = 0;      // Transmit non-inverted data to the TX/CK pin
    BAUDCONbits.BRG16 = 1;      // 16-bit Baud Rate Generator is used
    BAUDCONbits.WUE   = 0;      // Wake-up Enable bit = Receiver is operating normally
    BAUDCONbits.ABDEN = 0;      // Auto-Baud Detect mode is disabled
    SPBRGH = 0x02;              // determines the period of the free running baud rate timer (520)
    SPBRGL = 0x08;              // determines the period of the free running baud rate timer (520)
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
char read_UART(void) {
    RA2 = 0;            // test
    if(OERR) {          // check for Error 
        CREN = 0;       // If error -> Reset 
        CREN = 1;       // If error -> Reset 
    }
    while(!RCIF);       // hold the program till RX buffer is free
    return RCREG;       //receive the value and send it to main function
}
