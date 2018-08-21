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

void main(void) {
    TRISA   = 0x00;         // порт A весь на выход
    RA2     = 1;
    return;
}
