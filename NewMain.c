/* 
 * File:   newmain.c
 * Author: Frankynov
 *
 * Created on 26 décembre 2013, 12:22
 *
 *
 *### LIBRAIRIES A INCLURE ###
 */
#include <p18cxxx.h>
#include <p18f46k22.h>
#include <delays.h>
#include <usart.h>
#include "xlcd.h"

/*
 ### DEFINITION DES ALIAS ###
 */
#define NUM     0
#define ANA     1
#define OUTPUT  0
#define INPUT   1
/* Boutons Ludo
#define UP      PORTEbits.RE1
#define DOWN    PORTCbits.RC0
#define RIGHT   PORTEbits.RE0
#define LEFT    PORTCbits.RC1
#define CENTER  PORTEbits.RE2
 * Boutons Kiki */
#define DOWN      PORTCbits.RC1
#define UP    PORTEbits.RE2
#define LEFT   PORTEbits.RE1
#define RIGHT    PORTCbits.RC0
#define CENTER  PORTEbits.RE0

#define PORT_LED PORTCbits.RC2
#define PORT_RELAIS PORTBbits.RB4
//Interruptions RFID, USB et Boutons
#define INT_RFID PIR3bits.RC2IF
#define INT_USB  PIR1bits.RCIF
#define INT_BOUTON   INTCONbits.INT0IF

/*
 ### FUSIBLES DU PIC ###
 */
#pragma config FOSC     = INTIO67
#pragma config PLLCFG   = OFF
#pragma config PRICLKEN = OFF
#pragma config FCMEN    = OFF
#pragma config PWRTEN   = ON
#pragma config BOREN    = OFF
#pragma config WDTEN    = OFF
#pragma config LVP      = OFF
#pragma config IESO     = OFF
#pragma config MCLRE    = EXTMCLR

/*
 ### PROTOTYPES DES FONCTIONS ###
 */
void InterruptionHaute(void);
void ecritureRFID(char, char, char, char, char);
void lectureRFID(char);
void LiczCRC2(unsigned char *, unsigned short *, unsigned char);
void send_usart_pk(char, char, char, char);
void check_root(char *);


/*
 ### DECLARATION DES VARIABLES GLOBALES ###
 */
unsigned char flag_rx_usb_completed = 0;
unsigned char flag_bouton_pressed = 0;
char caractere_usb_recu;
char trame_usb_recue_pk[5];
char trame_usb_nom_prenom[20];
char bufRecuRFID;

/*
 ### DECLARATION DES INTERRUPTIONS ###
 */
#pragma code highVector = 0x0008

void atInterruptHigh(void) {
    _asm GOTO InterruptionHaute _endasm
}
#pragma code

/*
 ### DECLARATION DES VARIABLES GLOBALES ###
 */
unsigned volatile char bufWriteRFID[15], bufReadRFID[6];
unsigned volatile char flag_lecture = 0, flag_ecriture = 0, toto = 0, pgm = 0, cpt = 0, isRoot = 0;
volatile char tabRecu[10];
volatile char tabFrancois[11], tabPassword[15];

void main(void) {
    OSCCONbits.IRCF = 0b111; // 16MHz
    /*
     *
     * REGISTRES ASSOCIES AUX PORTS
     *
     */
    // Définition des entrées-sorties analogiques-numériques
    ANSELA = NUM; //on place toutes les pins en numérique, XC8 se fout des pins qu'on ne peut pas définir
    ANSELB = NUM;
    ANSELC = NUM;
    ANSELD = NUM;
    ANSELE = NUM;
    TRISCbits.TRISC2 = OUTPUT; //LED
    TRISBbits.TRISB4 = OUTPUT; //RELAIS
    TRISBbits.TRISB5 = INPUT; //Dallas
    TRISEbits.TRISE1 = INPUT;
    TRISEbits.TRISE2 = INPUT;
    TRISCbits.TRISC0 = INPUT;
    TRISCbits.TRISC1 = INPUT;

    // USART pour RFID et USB
    // TX2 : AN26, RD6
    // RX2 : AN27, RD7
    // TX1 : AN18, RC6
    // RX1 : AN19, RC7
    TRISDbits.TRISD6 = INPUT; //  [p155 datasheet]
    TRISDbits.TRISD7 = INPUT; // [p155 datasheet]
    TRISCbits.TRISC6 = INPUT;
    TRISCbits.TRISC7 = INPUT;

    //CONFIGURATION USART USB//
    TXSTA1bits.TXEN = 1;
    TXSTA1bits.TX9 = 0;
    TXSTA1bits.BRGH = 0;
    TXSTA1bits.SYNC = 0;
    BAUDCON1bits.BRG16 = 1;
    SPBRG1 = 103;
    RCSTA1bits.SPEN = 1;
    RCSTA1bits.CREN = 1;
    RCSTA1bits.RX9 = 0;

    // CONFIGURATION USART RFID //
    TXSTA2bits.TXEN = 1;
    TXSTA2bits.TX9 = 0;
    TXSTA2bits.BRGH = 0;
    TXSTA2bits.SYNC = 0;
    BAUDCON2bits.BRG16 = 0;
    SPBRG2 = 25;
    RCSTA2bits.SPEN = 1;
    RCSTA2bits.CREN = 1;
    RCSTA2bits.RX9 = 0;

    /*
     *
     * REGISTRES ASSOCIES AUX INTERRUPTIONS
     *
     */

    // Registres globaux d'interruptions
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1; // = GIEL, interruptions basses
    INTCONbits.INT0IE = 1; // Activation sur INT0
    INTCON2bits.INTEDG0 = 0;
    RCONbits.IPEN = 1;
    // Registres d'interruption sur USART2 - RFID
    IPR3bits.RC2IP = 1;
    PIE3bits.RC2IE = 1;
    INT_RFID = 0;
    // Registres d'interruption sur USART1 - USB
    IPR1bits.RC1IP = 1;
    PIE1bits.RC1IE = 1;
    INT_USB = 0;
    INT_BOUTON = 0;

    // On commence !
    OpenXLCD(FOUR_BIT & LINES_5X7);
    PORT_RELAIS = 0;
    //AFFICHAGE AU DEMARRAGE
    while (BusyXLCD());
    WriteCmdXLCD(0x01); //Clear LCD
    while (BusyXLCD());
    SetDDRamAddr(0x00); //Première ligne Première colonne
    while (BusyXLCD());
    putrsXLCD("Starting ... ");
    while (BusyXLCD());
    SetDDRamAddr(0x40);
    while (BusyXLCD());
    putrsXLCD("TiOS 0.42 - 2014");
    while (BusyXLCD());
    Delay10KTCYx(200);
    Delay10KTCYx(200);
    Delay10KTCYx(200);
    Delay10KTCYx(200);
    while (BusyXLCD());
    WriteCmdXLCD(0x01); //Clear LCD
    while (BusyXLCD());
    SetDDRamAddr(0x00); //Première ligne Première colonne
    while (BusyXLCD());
    putrsXLCD("Bienvenue !");
    while (BusyXLCD());
    SetDDRamAddr(0x40);
    while (BusyXLCD());
    while (1) {

    }

}

void send_usart_pk(char a, char b, char c, char d) {
    while (TXSTA1bits.TRMT != 1);
    TXREG1 = a;
    while (TXSTA1bits.TRMT != 1);
    TXREG1 = b;
    while (TXSTA1bits.TRMT != 1);
    TXREG1 = c;
    while (TXSTA1bits.TRMT != 1);
    TXREG1 = d;
    while (TXSTA1bits.TRMT != 1);
    TXREG1 = 0;
}

#pragma interrupt InterruptionHaute

void InterruptionHaute(void) {

    if (INT_RFID == 1) {
        tabRecu[toto] = RCREG2;
        toto++;
        if (tabRecu[2] == 0x11) {
            // Ecriture
            PORTCbits.RC2 = 1;
            flag_ecriture = 1;
        }
        if (tabRecu[2] == 0x13) {
            // Lecture
            PORTCbits.RC2 = 0;
            flag_lecture = 1;
        }
        INT_RFID = 0;
    }
    // GESTION INTERRUPTION USB

    if (INT_USB == 1) {
        trame_usb_recue_pk[cpt] = RCREG1;
        //while (TXSTA1bits.TRMT != 1);
        //TXREG1 = trame_usb_recue_pk[cpt];
        while (BusyXLCD());
        WriteDataXLCD(trame_usb_recue_pk[cpt]);
        while (BusyXLCD());
        
        if (trame_usb_recue_pk[cpt] == '\0') {
            while (BusyXLCD());
            putrsXLCD("BZERO");
            while (BusyXLCD());
        }
        cpt++;
        if (cpt == 5) {
            check_root(trame_usb_recue_pk);
            cpt = 0;
        }
    }

    // Détection de pression de bouton
    if (INT_BOUTON == 1) {
        Delay10KTCYx(2);
        PORT_LED = 1;
        if (LEFT == 0) {
            flag_bouton_pressed = 1;
            send_usart_pk('P', 'K', '0', '2');
        }
        if (RIGHT == 0) {
            flag_bouton_pressed = 1;
            send_usart_pk('P', 'K', '0', '3');
        }
        if (CENTER == 0) {
            flag_bouton_pressed = 1;
            send_usart_pk('P', 'K', '0', '0');
        }
        if (UP == 0) {
            flag_bouton_pressed = 1;
            send_usart_pk('k', 'i', 'k', 'i');
        }
        PORT_LED = 0;
        INT_BOUTON = 0;
    }
}

void check_root(char *trame_usb_recue_pk) {
    if (trame_usb_recue_pk[0] == 'P' && trame_usb_recue_pk[1] == 'K' && trame_usb_recue_pk[2] == '0' && trame_usb_recue_pk[3] == '0') {
        isRoot = 1;
        while (BusyXLCD());
        SetDDRamAddr(0x00); //Première ligne Première colonne
        while (BusyXLCD());
        putrsXLCD("You R R00T !");
    }
}
