#include <p18cxxx.h>
#include <p18f46k22.h>
#include <delays.h>
#include <usart.h>
#include "xlcd.h"

#define NUM     0
#define ANA     1
#define OUTPUT  0
#define INPUT   1
#define UP PORTEbits.RE1
#define DOWN PORTCbits.RC0
#define RIGHT PORTEbits.RE0
#define LEFT PORTCbits.RC1
#define CENTER PORTEbits.RE2
#define INT_RFID PIR3bits.RC2IF
#define BT_FL   INTCONbits.INT0IF



//fusibles du pic à configurer obligatoirement!!!
#pragma config FOSC     = INTIO67// pour proteger le pic en ecriture et autre...
#pragma config PLLCFG   = OFF
#pragma config PRICLKEN = OFF
#pragma config FCMEN    = OFF
#pragma config PWRTEN   = ON
#pragma config BOREN    = OFF
#pragma config WDTEN    = OFF
#pragma config LVP      = OFF
#pragma config IESO     = OFF
#pragma config MCLRE    = EXTMCLR

//Prototypes
void InterruptionHaute(void);
void ecritureRFID(char, char, char, char, char);
void lectureRFID(char);
void removePass(void);
void LiczCRC2(unsigned char *, unsigned short *, unsigned char);

#pragma code highVector = 0x0008

void atInterruptHigh(void) {
    _asm GOTO InterruptionHaute _endasm
}
#pragma code

unsigned volatile char bufWriteRFID[15], bufReadRFID[6];
unsigned volatile char flag_lecture = 0, flag_ecriture = 0, toto = 0, INTER_RFID = 0, pgm=0;
volatile char tabRecu[10];
volatile char tabFrancois[11], tabPassword[15];

void main(void) {
    int z;
    OSCCONbits.IRCF = 0b111; // 16MHz
    // Registres globaux d'interruptions
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    RCONbits.IPEN = 1;
    // Registres d'interruption sur USART2
    IPR3bits.RC2IP = 1;
    PIE3bits.RC2IE = 1;
    INT_RFID = 0;

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

    // USART pour RFID
    // TX2 : AN26, RD6
    // RX2 : AN27, RD7
    TRISDbits.TRISD6 = INPUT; // Configures RD6 (TX2) in OUTPUT (write) [p155]
    TRISDbits.TRISD7 = INPUT; // Configures RD7 (RX2) in INPUT (read) [p155]

    TXSTA2bits.TXEN = 1; // Transmit enabled [p274][optional]
    RCSTA2bits.SPEN = 1; // Serial port enabled [p275][optional]
    RCSTA2bits.CREN = 1;
    TXSTA2bits.TX9 = 0;
    RCSTA2bits.RX9 = 0;

    TXSTA2bits.SYNC = 0; // Asynchronous mode [p274]
    TXSTA2bits.BRGH = 0; // Low speed [p274]
    BAUDCON2bits.BRG16 = 0; // 16-bit Baud Rate Generator is used [p276]
    /*
     *      We have to put n (103 = 0x67) in [SPBREGH1:SPBRG1] registers
     */
    SPBRG2 = 25;



    //LED OFF
    PORTCbits.RC2 = 0;

    // On commence !
    OpenXLCD(FOUR_BIT & LINES_5X7);
    //AFFICHAGE AU DEMARRAGE
    while (BusyXLCD());
    WriteCmdXLCD(0x01); //Clear LCD
    while (BusyXLCD());
    SetDDRamAddr(0x00); //Première ligne Première colonne
    while (BusyXLCD());

    for (z = 0; z < 10; z++) { //commencer de 0 -> 10 cela résulte en 11 incrémentation
        tabRecu[z] = 0;
    }
    for (z = 0; z < 6; z++) { //idem au dessus
        bufReadRFID[z] = 0;
    }
    for (z = 0; z < 15; z++) { //idem au dessuss
        bufWriteRFID[z] = 0;
    }

    while (1) {
        Delay10KTCYx(200);
        // LANCER LA LECTURE
        lectureRFID(0x02);
        Delay10KTCYx(200);
        tabRecu[7] = 0;
        tabRecu[8] = 0;
        tabRecu[9] = 0;

        if (flag_lecture == 1) {
            if (toto >= 9) {
                //LCD pour lecture
                while (BusyXLCD());
                SetDDRamAddr(0x00);
                while (BusyXLCD());
                putrsXLCD("Lecture...   ");
                while (BusyXLCD());
                SetDDRamAddr(0x40);
                while (BusyXLCD());
                putsXLCD(&tabRecu[3]);
                for (z = 0; z < 10; z++) {
                    tabRecu[z] = 0;
                }
                toto = 0;
                flag_lecture = 0;
            }
        }
        Delay10KTCYx(200);

    ecritureRFID('T', 'o', 't', 'o', 0x02);
    Delay10KTCYx(200);
    if (flag_ecriture == 1) {
        if (toto >= 5) {
            if (tabRecu[3] == 0xff) {
                while (BusyXLCD());
                SetDDRamAddr(0x00);
                while (BusyXLCD());
                putrsXLCD("Ecriture OK!   ");
                while (BusyXLCD());
                for (z = 0; z < 10; z++) {
                    tabRecu[z] = 0;
                }
                toto = 0;

            } else {
                while (BusyXLCD());
                SetDDRamAddr(0x00);
                while (BusyXLCD());
                putrsXLCD("                ");
                while (BusyXLCD());
                SetDDRamAddr(0x00);
                while (BusyXLCD());
                putrsXLCD("Ecriture FAIL !");
                for (z = 0; z < 10; z++) {
                    tabRecu[z] = 0;
                }
                toto = 0;
            }
        }
        flag_ecriture = 0;
    }
    }
    
}



// Fonctions du LCD

void DelayFor18TCY(void) //fonction à définir car elles ne sont pas comprises dans les librairie du xlcd
{
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
}

void DelayPORXLCD(void) {
    Delay1KTCYx(60);
    return;
}

void DelayXLCD(void) {
    Delay1KTCYx(20);
    return;
}

void ecritureRFID(char dt1, char dt2, char dt3, char dt4, char sect) {
    unsigned char send = 0;
    unsigned tmp;

    // Config tab
    bufWriteRFID[0] = 0xff; // Tous les modules RFID
    bufWriteRFID[1] = 0x0a; // Nbr total d'instructions
    bufWriteRFID[2] = 0x10; // Lecture/ecriture (c'est ecriture ici)
    bufWriteRFID[3] = dt1; // Envoie 1
    bufWriteRFID[4] = dt2; // Envoie 2
    bufWriteRFID[5] = dt3; // Envoie 3
    bufWriteRFID[6] = dt4; // Envoie 4
    bufWriteRFID[7] = sect; // Choix du secteur 2

    // On place le CRCH et CRCL dans 9 et 10, on les inverse

    LiczCRC2(bufWriteRFID, (unsigned short *) &bufWriteRFID[8], 8);
    tmp = bufWriteRFID[8];
    bufWriteRFID[8] = bufWriteRFID[9];
    bufWriteRFID[9] = tmp;

    TXREG2 = bufWriteRFID[send];
    send++;
    while (send <= 9) {
        while (TXSTA2bits.TRMT2 != 1);
        TXREG2 = bufWriteRFID[send];
        send++;
    }

    // 1) Adresse de la première case du tableau pour commencer le calcul
    // 2) Premier emplacement de libre pour stocker CRCH et CRCL
    // 3) On fait le calcul sur les 8 premières cellules
}

void lectureRFID(char dataSector) {
    unsigned int i;
    /*      Description of the SectorRead command frame
     *
     *      bufRFID[0] = Module Address (0xff to target all the cards)
     *      bufRFID[1] = Frame length       (0x06 in case of SectorRead command frame)
     *      bufRFID[2] = Command            (0x12 in case of SectorRead command frame)
     *      bufRFID[3] = Data Sector
     *      bufRFID[4] = CRCH
     *      bufRFID[5] = CRCL
     */
    bufReadRFID[0] = 0xff;
    bufReadRFID[1] = 0x06;
    bufReadRFID[2] = 0x12;
    bufReadRFID[3] = dataSector;
    LiczCRC2(bufReadRFID, (unsigned short *) &bufReadRFID[4], 4);

    i = bufReadRFID[4];
    bufReadRFID[4] = bufReadRFID[5];
    bufReadRFID[5] = i;


    // Sends the all frame
    for (i = 0; i < 6; i++) {
        while (TXSTA2bits.TRMT2 != 1);
        TXREG2 = bufReadRFID[i];
    }
}

void LiczCRC2(unsigned char *ZAdr, unsigned short *DoAdr, unsigned char Ile) {
    int i, NrBajtu;
    unsigned short C;
    *DoAdr = 0;
    for (NrBajtu = 1; NrBajtu <= Ile; NrBajtu++, ZAdr++) {
        C = ((*DoAdr >> 8)^*ZAdr) << 8;
        for (i = 0; i < 8; i++)
            if (C & 0x8000) C = (C << 1)^0x1021;
            else C = C << 1;
        *DoAdr = C^(*DoAdr << 8);
    }
}

#pragma interrupt InterruptionHaute

void InterruptionHaute(void) {
    if (INT_RFID == 1) {
        tabRecu[toto] = RCREG2;
        toto++;
        if (tabRecu[2] == 0x11)
        {
            // Ecriture
            PORTCbits.RC2=1;
            flag_ecriture = 1;
        }
        if (tabRecu[2] == 0x13) {
            // Lecture
            PORTCbits.RC2=0;
            flag_lecture = 1;
        }
        INT_RFID = 0;
    }

}
