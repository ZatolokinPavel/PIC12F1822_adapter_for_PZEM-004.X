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
#pragma config FOSC     = INTOSC// Тактовый генератор (INTOSC - внутренний 16 МГц)

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
#define _WRITE        0     // производим запись (для I2C)
#define _READ         1     // производим чтение (для I2C)
// Состояния приёма сообщений по шине RS-485
#define _UART_RC_ADDR 0     // ожидаем адрес получателя
#define _UART_RC_SIZE 1     // ожидаем размер сообщения
#define _UART_RC_MSG  2     // ожидаем очередной байт сообщения
#define _UART_RC_SUM  3     // ожидаем контрольную сумму
#define _UART_RC_ERR  4     // уже ничё не ожидаем, словили ошибку
// Состояния передачи и приёма сообщений по шине I2C
#define _I2C_IDLE     0     // приём или передача не ведётся
#define _I2C_ADDRESS  1     // передача адреса (для мастера)
#define _I2C_WRITING  2     // идёт запись данных (для мастера)
#define _I2C_READING  3     // идёт чтение данных (для мастера)
#define _I2C_ACK      4     // подтверждение чтения байта данных (для мастера)
#define _I2C_STOPPING 5     // завершение передачи, будем отправлять условие STOP
#define _I2C_STOPPED  6     // передача завершена, ждём состояния STOP

// Объявление глобальных переменных
unsigned char i;            // Переменная «i» - уже более 25 лет на рынке счетчиков!
char temp = 0;              // временная переменная
char test_data[7] = {167,18,33,75,99,100,107};  // массив тестовых символов
bool uart_in_MASTER = false;// изначально ждём приёма сообщений
char uart_TX_buffer[7];     // буфер для хранения передаваемых данных по UART
char uart_TX_next = 0;      // номер следующего символа, который будет передан по UART
char uart_RC_state = 0;     // флаг состояния приёма сообщений по UART
bool uart_RC_to_me = false; // этому ли устройству передают сообщение по RS-485?
char uart_RC_msg_size = 0;  // количество байт сообщения, которое нужно принять по UART
char uart_RC_message[1];    // собственно получаемое сообщение из шины RS-485
char uart_RC_next = 0;      // номер следующего символа, который будет принят по UART
bool i2c_in_MASTER = false; // изначально ждём приёма сообщений
char i2c_RW;                // сейчас производим чтение или запись (актуально для мастера)
char i2c_state = _I2C_IDLE; // флаг состояния передачи/приёма сообщения по I2C
char i2c_address;           // адрес слейва I2C
char i2c_next = 0;          // номер следующего символа, который будет передан или принят по I2C
char i2c_TX_buffer[2];      // буфер для хранения передаваемых данных по I2C (адрес и один байт данных)
char i2c_RC_length;         // длина получаемого от слейва сообщения
char i2c_RC_message[8];     // полученное по I2C сообщение

// Объявление всех функций, которые написаны после их вызова
void init_UART(void);
void read_UART(void);
void action_on_msg_UART(void);
void write_UART(char[7]);
void write_char_UART(void);
void init_I2C(void);
void read_slave_I2C(void);
void write_master_I2C(char, char);
void write_char_I2C(void);
void read_master_I2C(char, char);
void read_char_I2C(void);



// =============================================================================
// ============================ Заголовок закончен =============================
// =================== далее идёт собственно текст программы ===================
// =============================================================================



// Главная функция программы. С неё начинается выполнение программы.
void main(void) {
    OSCCONbits.SPLLEN = 1;      // игнорируется так как config PLLEN = ON
    OSCCONbits.IRCF = 0b1111;   // Internal Oscillator Frequency = 16 MHz HF
    OSCCONbits.SCS = 0b00;      // System Clock determined by FOSC<2:0> in Configuration Word 1.
    INTCONbits.GIE = 1;         // глобально разрешаем прерывания
    INTCONbits.PEIE = 1;        // разрешаем прерывания от периферии
    
    // Тестовые выходы
    TRISAbits.TRISA0 = 0;       // пин RA0 (ножка 7) на выход
    TRISAbits.TRISA1 = 0;       // пин RA1 (ножка 6) на выход
    TRISAbits.TRISA2 = 0;       // пин RA2 (ножка 5) на выход
    ANSELAbits.ANSA0 = 0;       // пин RA0 как дискретный I/O (не аналоговый)
    ANSELAbits.ANSA1 = 0;       // пин RA1 как дискретный I/O (не аналоговый)
    ANSELAbits.ANSA2 = 0;       // пин RA2 как дискретный I/O (не аналоговый)
    PORTAbits.RA0 = 1;
    PORTAbits.RA1 = 1;          // для начала зажигаем светодиод
    PORTAbits.RA2 = 1;
    // Для наглядности помигаем
    __delay_ms(500);
    RA0 = 0;
    __delay_ms(500);
    RA0 = 1;
    __delay_ms(500);
    RA0 = 0;
    
    init_UART();                // конфигурируем порт RS-232 как slave
//    init_I2C();                 // конфигурируем I2C как slave
    
    while(1) {
    }
}


// Main Interrupt Service Routine (ISR)
void __interrupt() ISR(void) {
    if(PIR1bits.RCIF) {             // если прерывание от принятого байта по RS-232 (UART)
        read_UART();                // по одному байту записываем полное сообщение
        action_on_msg_UART();
    }
    
    if(PIR1bits.TXIF) {             // если это прерывание от передачи по UART
        write_char_UART();          // то загружаем следующий символ в очередь передачи
    }
    
    if(PIR1bits.SSP1IF && !i2c_in_MASTER) {     // если это прерывание от I2C в режиме SLAVE
        if(SSP1STATbits.R_nW) read_slave_I2C(); // по одному байту записываем полное сообщение
        if(!SSP1STATbits.R_nW) {}
    }
    
    if(PIR1bits.SSP1IF && i2c_in_MASTER) {      // если это прерывание от I2C в режиме MASTER
        PIR1bits.SSP1IF = 0;                    // нужно очистить флаг прерывания
        switch (i2c_RW) {
            case _WRITE: write_char_I2C(); break;   // загружаем следующий символ в очередь передачи
            case _READ: read_char_I2C(); break;
        }
    }
    
    if(PIR2bits.BCL1IF && i2c_in_MASTER) {      // проблема с шиной I2C
        PIR2bits.BCL1IF = 0;
    }
}


// Конфигурируем EUSART как асинхронный порт RS-232 в режиме slave
// SYNC=0, BRG16=0, BRGH=1: Baud Rate =  Fosc/(16*(SPBRG+1)) = 16000000/(16*(103+1)) = 9615.38 bit per second
void init_UART(void) {
    APFCONbits.RXDTSEL = 1;     // RX function is on RA5 (вместо RA1 по умолчанию)
    APFCONbits.TXCKSEL = 1;     // TX function is on RA4 (вместо RA0 по умолчанию)
    TRISAbits.TRISA4 = 0;       // TX pin set as output
    TRISAbits.TRISA5 = 1;       // RX pin set as input
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


// Передача массива из семи символов на шину RS-232
void write_UART(char array[7]) {
    if(uart_TX_next != 0) return;   // занято
    for(i=0; i < 7; i++)
        uart_TX_buffer[i] = array[i];
    uart_TX_next = 1;               // номер следующего на передачу символа
    PIE1bits.TXIE = 1;              // включаем прерывание что очередь из TXREG ушла на передачу
    TXREG = uart_TX_buffer[0];      // первый символ на передачу
}

void write_char_UART(void) {
    if(uart_TX_next == 0) return;   // ничего не передаётся
    TXREG = uart_TX_buffer[uart_TX_next];   // ставим в очередь на передачу следующий символ
    uart_TX_next += 1;
    if(uart_TX_next == 7) {         // был передан последний символ
        uart_TX_next = 0;           // символов на передачу больше нет
        PIE1bits.TXIE = 0;          // закончили передачу, прерывание нам уже не нужно
    }
}


// Получение байта данных по шине RS-232 и формирование полного сообщения.
void read_UART(void) {
    // Получение одного байта данных по шине RS-232
    if(RCSTAbits.OERR) {        // check for Receive Overrun Error 
        RCSTAbits.CREN = 0;     // if error -> restart UART чтобы дальше принимать данные
        RCSTAbits.CREN = 1;
    }
    RCSTAbits.FERR;             // надо будет прочитать и обработать ошибку фрейма
    char byte = RCREG;          // читаем принятый байт
    // смотрим что делать дальше с принятым байтом
    switch (uart_RC_state) {
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
        case _UART_RC_ERR:                  // если уже словили ошибку
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
            write_UART(test_data);
            break;
        case 0x78:
            RA1 = 1;
            RA2 = 0;
            write_UART(test_data);
            break;
        case 0xff:
            RA1 = 1;
            RA2 = 1;
            break;
        case 0x11:
            if(!i2c_in_MASTER) {
                i2c_in_MASTER = true;
                init_I2C();
            }
            write_master_I2C(0x68, 0x00);
            write_UART(test_data);
            break;
        case 0x12:
            read_master_I2C(0x68, 8);
            write_UART(test_data);
            break;
        default:
            RA0 = 0;
            RA1 = 0;
            RA2 = 0;
    }
}

void init_I2C(void) {
    TRISAbits.TRISA1 = 1;       // SCL pin set as input
    TRISAbits.TRISA2 = 1;       // SDA pin set as input
    // SSP1STAT
    SSP1STATbits.SMP = 1;       // Slew rate control disabled for standard speed mode (100 kHz and 1 MHz)
    SSP1STATbits.CKE = 0;       // Disable SMBus specific inputs
    // SSP1CON1
    SSP1CON1bits.WCOL = 0;      // No collision
    SSP1CON1bits.SSPOV = 0;     // No overflow
    SSP1CON1bits.SSPEN = 0;     // до конца настройки отключаем серийный порт
    SSP1CON1bits.CKP = 0;       // Holds clock low (clock stretch). For Slave only
    // SSP1CON2
    SSP1CON2bits.GCEN = 0;      // General call address disabled
    SSP1CON2bits.ACKDT = 0;     // Acknowledge Data bit: Acknowledge
    // SSP1CON3
    SSP1CON3bits.PCIE = 0;      // Stop detection interrupts are disabled
    SSP1CON3bits.SCIE = 0;      // Start detection interrupts are disabled
    SSP1CON3bits.BOEN = 0;      // SSP1BUF is only updated when SSP1OV is clear
    SSP1CON3bits.SDAHT = 0;     // линия связи простая, поэтому большую задержку не надо
    SSP1CON3bits.SBCDE = 0;     // Slave bus collision interrupts are disabled
    SSP1CON3bits.AHEN = 0;      // Address holding is disabled
    SSP1CON3bits.DHEN = 0;      // Data holding is disabled
    // SSP1CON1
    if(i2c_in_MASTER) {
        SSP1CON1bits.SSPM = 0b1000;     // I2C Master mode, clock = FOSC / (4 * (SSP1ADD+1))
        SSP1ADD = 39;                   // задаём частоту 100kHz
    } else {
        SSP1CON1bits.SSPM = 0b0110;     // I2C Slave mode, 7-bit address
        SSP1ADD = 0b10101000;           // slave address = 0x54
    }
    // clear the interrupt flags
    PIR1bits.SSP1IF = 0;
    PIR2bits.BCL1IF = 0;
    // enable the interrupts
    PIE1bits.SSP1IE = 1;        // Enables the I2C receive interrupt
    PIE2bits.BCL1IE = 1;        // Enables the MSSP Bus Collision Interrupt
    SSP1CON1bits.SSPEN = 1;     // Enables serial port (configures SDA and SCL pins as serial port pins)
}

// Получение байта данных по шине I2C (slave) и формирование полного сообщения.
void read_slave_I2C(void) {
    PIR1bits.SSP1IF = 0;        // нужно очистить флаг прерывания
    char byte = SSP1BUF;        // читаем принятый байт
    if(SSP1STATbits.D_nA) {
        i2c_RC_message[0] = byte;
    }
}

void write_master_I2C(char address, char data) {
    if(i2c_state != _I2C_IDLE) return;  // занято
    i2c_RW = _WRITE;
    i2c_address = address << 1 | 0;     // адрес слейва и направление передачи - запись
    i2c_TX_buffer[0] = data;            // данные для передачи
    write_char_I2C();                   // начинаем передачу
}

void write_char_I2C(void) {
    switch (i2c_state) {
        case _I2C_IDLE:
            i2c_next = 0;
            SSP1CON2bits.SEN = 1;       // запускаем передачу по i2c (состояние START)
            i2c_state = _I2C_ADDRESS;
            break;
        case _I2C_ADDRESS:
            SSP1BUF = i2c_address;      // передаём адрес слейва
            i2c_state = _I2C_WRITING;
            break;
        case _I2C_WRITING:
            SSP1BUF = i2c_TX_buffer[i2c_next];  // ставим в очередь на передачу следующий символ
            i2c_next += 1;
            if(i2c_next == 1)           // если был передан последний символ (а сейчас он только один)
                i2c_state = _I2C_STOPPING;
            break;
        case _I2C_STOPPING:
            SSP1CON2bits.PEN = 1;       // отправляем команду STOP
            i2c_state = _I2C_STOPPED;
            break;
        case _I2C_STOPPED:
            i2c_state = _I2C_IDLE;
            break;
    }
}

void read_master_I2C(char address, char length) {
    if(i2c_state != _I2C_IDLE) return;  // занято
    i2c_RW = _READ;
    i2c_address = address << 1 | 1;     // адрес слейва и направление передачи - чтение
    i2c_RC_length = length;             // сколько байт нужно получить от слейва
    read_char_I2C();                    // начинаем считывание
}

void read_char_I2C(void) {
    switch (i2c_state) {
        case _I2C_IDLE:
            i2c_next = 0;
            SSP1CON2bits.SEN = 1;       // запускаем передачу по i2c (состояние START)
            i2c_state = _I2C_ADDRESS;
            break;
        case _I2C_ADDRESS:
            SSP1BUF = i2c_address;      // передаём адрес слейва
            i2c_state = _I2C_READING;
            break;
        case _I2C_READING:
            SSP1CON2bits.RCEN = 1;      // начинаем чтение байта
            i2c_state = _I2C_ACK;
            break;
        case _I2C_ACK:
            i2c_RC_message[i2c_next] = SSP1BUF; // считываем принятый байт
            i2c_next += 1;
            if(i2c_next == i2c_RC_length) {     // если принято всё что нужно
                i2c_state = _I2C_STOPPING;
                SSP1CON2bits.ACKDT = 1;         // без подтверждения приёма
            } else {
                i2c_state = _I2C_READING;
                SSP1CON2bits.ACKDT = 0;         // будем подтверждать приём
            }
            SSP1CON2bits.ACKEN = 1;             // отсылаем подтверждение
            break;
        case _I2C_STOPPING:
            SSP1CON2bits.PEN = 1;               // отправляем команду STOP
            i2c_state = _I2C_STOPPED;
            break;
        case _I2C_STOPPED:
            i2c_state = _I2C_IDLE;
            break;
    }
}
