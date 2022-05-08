/* 
 * File:   main.c
 * Author: Sergio Boch 20887
 *
 * Created on May 6, 2022, 9:53 AM
 */

#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <stdint.h>  // Para poder usar los int de 8 bits

#define _XTAL_FREQ 1000000
uint8_t valor = 0; 
uint8_t mensaje;
uint8_t POT1;
void text(char msg[]);
void setup(void);

void __interrupt() isr (void){
    if(ADCON0bits.CHS == 0){            // Verificamos sea AN0 el canal seleccionado
            POT1 = ADRESH;
        PIR1bits.ADIF = 0;  }
    if(PIR1bits.RCIF){          // Verificación de datos recibidos
        mensaje= RCREG; // Mostramos valor recibido en el PORTD
    }
    return;
}

void main(void) {
   
    setup();
    while(1){
        while (valor==0){
        TXREG=0X0D;
        text("DESEA LEER EL POT1CIOMETRO? (SI/NO),");
        TXREG=0X0D;
        valor++;
        }
        if (mensaje=='N'){
            text("NO,");
            valor=0; 
            mensaje=0;}
        if (mensaje=='S'){
            text("SI,");
            PORTD=POT1; 
            valor=0; 
            mensaje=0;}
        if(ADCON0bits.GO == 0){             
       ADCON0bits.GO = 1;}
     }
}

void text(char msg[]){
    uint8_t a=0;
        while(msg[a]!=','){                 
            TXREG = msg[a];    
            a++;
            
            __delay_us(900);
            } 
        return;
}

void setup(void){
    ANSEL = 1;                // AN0 como entrada analógica
    ANSELH = 0x00;                 // I/O digitales
    TRISA = 1;                // AN0 como entrada
    PORTA = 0x00;
    TRISD= 0x00;
    PORTD= 0x00;
    TRISB= 0x00;
    PORTB= 0x00;
    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
    
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(60);   
    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de perifericos
    PIE1bits.RCIE = 1;          // Habilitamos Interrupciones de recepción
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;   
}

