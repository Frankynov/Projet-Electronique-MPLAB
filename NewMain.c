/*
 * File:   newmain.c
 * 
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
#include "ftoa.h"

/*
 ### DEFINITION DES ALIAS ###
 */
#define NUM     0
#define ANA     1
#define OUTPUT  0
#define INPUT   1
/* Boutons Ludo*/
#define UP      PORTEbits.RE1
#define DOWN    PORTCbits.RC0
#define RIGHT   PORTEbits.RE0
#define LEFT    PORTCbits.RC1
#define CENTER  PORTEbits.RE2
/* Boutons Quentin
#define DOWN      PORTCbits.RC1
#define UP    PORTEbits.RE2
#define LEFT   PORTEbits.RE1
#define RIGHT    PORTCbits.RC0
#define CENTER  PORTEbits.RE0
 */
#define PORT_LED PORTCbits.RC2
#define PORT_RELAIS PORTBbits.RB4
//Interruptions RFID, USB et Boutons
#define INT_USART_RFID PIR3bits.RC2IF
#define INT_USART_USB  PIR1bits.RCIF
#define INT_BOUTON   INTCONbits.INT0IF
#define INT_TIMER1  PIR1bits.TMR1IF

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
void InterruptionBasse(void);
void send_usart_pk(char, char, char, char);
void reset_tableau_USB(void);
void reset_tableau_RFID(void);
void analyser_trame_recue(void);
void ecriture_PK_RFID(void);
void LiczCRC2(unsigned char *, unsigned short *, unsigned char);
void afficherLumiere(void);

/*
 ### DECLARATION DES VARIABLES GLOBALES ###
 */
volatile unsigned char flag_bouton_pressed = 0, flag_debut_trame_USB = 0, cpt_tab_USB = 0;
volatile unsigned char flag_fin_trame_USB = 0, flag_prenom_recu_USB = 0, flag_new_PK_recue_USB = 0;
volatile unsigned char flag_fin_trame_RFID_envoyee = 0, flag_fin_trame_RFID_recue = 0;
volatile char caractere_usb_recu;
volatile char tabPrenomRecuUSB[20];
volatile char tabRecuUSB[20];
volatile char tabKeyRecuUSB[4];
volatile char bufferLumiere [20];
volatile unsigned char bufWriteRFID[15], bufReadRFID[6];
volatile char bufRecuRFID;

/*
 ### DECLARATION DES INTERRUPTIONS ###
 */
#pragma code highVector = 0x0008

void atInterruptHigh(void) {
    _asm GOTO InterruptionHaute _endasm
}
#pragma code

#pragma code lowVector = 0x0018

void atInterruptLow(void) {
    _asm GOTO InterruptionBasse _endasm
}
#pragma code

/*
 ### DECLARATION DES VARIABLES GLOBALES ###
 */
unsigned volatile char bufWriteRFID[15], bufReadRFID[6];
unsigned volatile char flag_lecture_RFID = 0, flag_ecriture_RFID = 0, cpt_tab_RFID = 0, pgm = 0, isRoot = 0;
volatile char tabRecuRFID[10];

void main(void) {
    OSCCONbits.IRCF = 0b111; // 16MHz
    /*
     *
     * REGISTRES ASSOCIES AUX PORTS
     *
     */
    // Définition des entrées-sorties analogiques-numériques
    ANSELA = NUM; //on place toutes les pins en numérique
    ANSELB = NUM;
    ANSELC = NUM;
    ANSELD = NUM;
    ANSELE = NUM;
    ANSELDbits.ANSD4 = ANA; // Analogique pour la photodiode

    TRISCbits.TRISC2 = OUTPUT; //LED
    TRISBbits.TRISB4 = OUTPUT; //RELAIS
    TRISBbits.TRISB5 = INPUT; //Dallas
    TRISEbits.TRISE1 = INPUT; // Boutons
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

    // CONFIGURATION PHOTODIODE CAN //
    // Configuration du CAN
    ADCON0bits.ADON = 1; // Active le module de CAN
    ADCON0bits.CHS = 24; // On lit la Diode en analogique (AN24)
    ADCON2bits.ADFM = 1; // Justifier à droite
    ADCON1bits.PVCFG = 0b00; // Valeur tension haute = VDD 5V
    ADCON1bits.NVCFG = 0b00; // Valeur tension basse = masse 0V
    ADCON2bits.ADCS = 0b010;
    ADCON2bits.ACQT = 0b010; // Sécurité

    /*
     *
     * REGISTRES ASSOCIES AUX INTERRUPTIONS / TIMERS
     *
     */

    // Registres globaux d'interruptions + boutons
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1; // = GIEL, interruptions basses
    INTCONbits.INT0IE = 1; // Activation sur INT0
    INTCON2bits.INTEDG0 = 0;
    INT_BOUTON = 0;
    RCONbits.IPEN = 1;
    // Registres d'interruption sur USART2 - RFID
    IPR3bits.RC2IP = 1;
    PIE3bits.RC2IE = 1;
    INT_USART_RFID = 0;
    // Registres d'interruption sur USART1 - USB
    IPR1bits.RC1IP = 1;
    PIE1bits.RC1IE = 1;
    INT_USART_USB = 0;

    // Registres de Timer
    T1CONbits.TMR1ON = 1; //On active le timer1
    T1CONbits.TMR1CS = 0b00; // Clock système interne/4 (FOSC/4), 4 MHz
    T1CONbits.T1CKPS = 0b11; // configurer le prescale value (ici 8)
    T1GCONbits.TMR1GE = 0; //Cfr porte logique, pas obligatoire en cours théorie

    // Registres de Timer/Interruption
    PIE1bits.TMR1IE = 1; // Active les interruptions sur le timer 1
    IPR1bits.TMR1IP = 0; // Mettre la priorité h/l sur TMR1 overflow interrupt
    INT_TIMER1 = 0; // On est sur que l'interruption va se faire
    TMR1H = 0x3C;
    TMR1L = 0xAF;


    /*
     *
     * DEMARRAGE DU PROGRAMME
     *
     */
    OpenXLCD(FOUR_BIT & LINES_5X7);
    reset_tableau_USB();
    reset_tableau_RFID();
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
    // Quelques délais pour simuler le démarrage de la carte
    Delay10KTCYx(200);
    Delay10KTCYx(200);
    Delay10KTCYx(200);
    Delay10KTCYx(200);
    while (BusyXLCD());
    WriteCmdXLCD(0x01); //Clear LCD
    while (BusyXLCD());
    SetDDRamAddr(0x00); //Première ligne Première colonne
    while (BusyXLCD());
    putrsXLCD("Show badge &");
    while (BusyXLCD());
    SetDDRamAddr(0x40);
    while (BusyXLCD());
    putrsXLCD("Press left Key ");
    while (BusyXLCD());



    while (1) {
        if (flag_fin_trame_USB == 1) {
            analyser_trame_recue();
            flag_fin_trame_USB = 0;
        }
        if (flag_new_PK_recue_USB == 1) {
            while (BusyXLCD());
            WriteCmdXLCD(0x01);
            while (BusyXLCD());
            SetDDRamAddr(0x00);
            while (BusyXLCD());
            putrsXLCD("Show badge &");
            while (BusyXLCD());
            SetDDRamAddr(0x40);
            while (BusyXLCD());
            putrsXLCD("Press Right Key");
            while (BusyXLCD());
        }
        if (flag_ecriture_RFID == 1) {
            if (cpt_tab_RFID >= 5) {
                if (tabRecuRFID[3] == 0xff) {
                    while (BusyXLCD());
                    WriteCmdXLCD(0x01);
                    while (BusyXLCD());
                    SetDDRamAddr(0x00);
                    while (BusyXLCD());
                    putrsXLCD("Ecriture WIN ! ");
                    while (BusyXLCD());
                } else {
                    while (BusyXLCD());
                    WriteCmdXLCD(0x01);
                    while (BusyXLCD());
                    SetDDRamAddr(0x00);
                    while (BusyXLCD());
                    putrsXLCD("Ecriture FAIL !");
                    while (BusyXLCD());
                }
                flag_ecriture_RFID = 0;
                while (BusyXLCD());
                SetDDRamAddr(0x00);
                reset_tableau_RFID();
            }

        }
        if (flag_lecture_RFID == 1) {
            if (cpt_tab_RFID >= 5) {
                if (tabRecuRFID[7] == 0xff) {
                    while (BusyXLCD());
                    WriteCmdXLCD(0x01);
                    while (BusyXLCD());
                    SetDDRamAddr(0x00);
                    while (BusyXLCD());
                    putrsXLCD("RFID OK !       ");
                    while (BusyXLCD());
                    Delay10KTCYx(200);
                    Delay10KTCYx(200);
                    send_usart_pk(tabRecuRFID[3], tabRecuRFID[4], tabRecuRFID[5], tabRecuRFID[6]);
                }
                flag_lecture_RFID = 0;
            }

        }
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
    // GESTION INTERRUPTION RFID

    if (INT_USART_RFID == 1) {
        tabRecuRFID[cpt_tab_RFID] = RCREG2;
        cpt_tab_RFID++;
        if (tabRecuRFID[2] == 0x11) {
            // Ecriture
            flag_ecriture_RFID = 1;
        }
        if (tabRecuRFID[2] == 0x13) {
            // Lecture
            flag_lecture_RFID = 1;
        }
        INT_USART_RFID = 0;
    }
    // GESTION INTERRUPTION USB

    if (INT_USART_USB == 1) {
        caractere_usb_recu = RCREG1;

        if (caractere_usb_recu != '\0') {
            tabRecuUSB[cpt_tab_USB] = caractere_usb_recu;
            cpt_tab_USB++;
        } else
            flag_fin_trame_USB = 1;
        INT_USART_USB = 0;
    }


    // Détection de pression de bouton
    if (INT_BOUTON == 1) {
        // Rebond
        Delay10KTCYx(2);
        PORT_LED = 1;
        if (LEFT == 0) {
            flag_bouton_pressed = 1;
            //TODO : implémenter appel lecture RFID
            send_usart_pk('P', 'K', '0', '2');
        }
        if (RIGHT == 0) {
            flag_bouton_pressed = 1;
            if (flag_new_PK_recue_USB == 1) {
                flag_new_PK_recue_USB = 0;
                ecriture_PK_RFID();
            } else
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
        if (DOWN == 0) {
            flag_bouton_pressed = 1;
        }

        PORT_LED = 0;
        INT_BOUTON = 0;
    }
}

#pragma interrupt InterruptionBasse

void InterruptionBasse(void) {
    if (INT_TIMER1 == 1) {
        afficherLumiere();
        // A refaire à chaque fois, sinon il reprend à 0
        TMR1H = 0x3C;
        TMR1L = 0xAF;
        INT_TIMER1 = 0;
    }
}

void analyser_trame_recue(void) {
    unsigned char i = 2;
    // Cette fonction permet d'analyser le contenu du tableau reçu via USART USB
    // Cas d'un tableau de prénom
    if (tabRecuUSB[0] == '#' && tabRecuUSB[1] == 'P') {
        flag_prenom_recu_USB = 1;
        while (BusyXLCD());
        WriteCmdXLCD(0x01);
        while (BusyXLCD());
        putrsXLCD("Bienvenue,");
        while (BusyXLCD());
        SetDDRamAddr(0x40);
        while (BusyXLCD());

        while (i < cpt_tab_USB) {
            while (BusyXLCD());
            WriteDataXLCD(tabRecuUSB[i]);
            while (BusyXLCD());
            i++;
        }
        while (BusyXLCD());
        SetDDRamAddr(0x00);
        cpt_tab_USB = 0;
        reset_tableau_USB();
    }
    // Cas d'un tableau de nouvelle PK
    if (tabRecuUSB[0] == '#' && tabRecuUSB[1] == 'K') {
        flag_new_PK_recue_USB = 1;
        while (BusyXLCD());
        SetDDRamAddr(0x00);
        // DEBUG Affichage de la PK recue
        /*
        while (i < cpt_tab_USB) {
            while (BusyXLCD());
            WriteDataXLCD(tabRecuUSB[i]);
            while (BusyXLCD());
            i++;
        }
         */
        cpt_tab_USB = 0;
    }
}

void reset_tableau_USB(void) {
    for (cpt_tab_USB = 0; cpt_tab_USB < 20; cpt_tab_USB++) {
        tabRecuUSB[cpt_tab_USB] = 0;
    }
    cpt_tab_USB = 0;
}

void reset_tableau_RFID(void) {
    unsigned char j;
    for (j = 0; j < 14; j++) {
        tabRecuRFID[j] = 0;
        cpt_tab_RFID = 0;
    }
}

void ecriture_PK_RFID(void) {
    unsigned char send = 0;
    unsigned char tmp;
    // Config tab
    bufWriteRFID[0] = 0xff; // Tous les modules RFID
    bufWriteRFID[1] = 0x0a; // Nbr total d'instructions
    bufWriteRFID[2] = 0x10; // Lecture/ecriture (c'est ecriture ici)
    bufWriteRFID[3] = tabRecuUSB[2]; // Envoie P
    bufWriteRFID[4] = tabRecuUSB[3]; // Envoie K
    bufWriteRFID[5] = tabRecuUSB[4]; // Envoie N°
    bufWriteRFID[6] = tabRecuUSB[5]; // Envoie N°
    bufWriteRFID[7] = 0x02; // Choix du secteur 2

    // On place le CRCH et CRCL dans 8 et 9, on les inverse
    // 1) Adresse de la première case du tableau pour commencer le calcul
    // 2) Premier emplacement de libre pour stocker CRCH et CRCL (casting)
    // 3) On fait le calcul sur les 8 premières cellules (0 -> 7)

    LiczCRC2(bufWriteRFID, (unsigned short *) &bufWriteRFID[8], 8);
    tmp = bufWriteRFID[8];
    bufWriteRFID[8] = bufWriteRFID[9];
    bufWriteRFID[9] = tmp;

    // Envoi de la trame
    TXREG2 = bufWriteRFID[send];
    send++;
    while (send <= 9) {
        while (TXSTA2bits.TRMT2 != 1);
        TXREG2 = bufWriteRFID[send];
        send++;
    }
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

void afficherLumiere(void) {
    double valeurLueDiode = 0;
    ADCON0bits.GO_DONE = 1;
    while (BusyXLCD());
    SetDDRamAddr(0x0c);
    while (BusyXLCD());
    putrsXLCD("    ");

    while (ADCON0bits.GO_DONE != 0); //La conversion s'effectue dans cette boucle
    valeurLueDiode = (ADRESH * 256 + ADRESL);
    valeurLueDiode /= 204.6; // Pour obtenir une valeur en Volts (1023 = 5V, 0 = 0V)
    // Conversion du nombre en caractère alphanumérique, précision 2 chiffres et mode non scientifique
    valeurLueDiode *= 20; // affichage en %

    ftoa(valeurLueDiode, bufferLumiere, 0, 'f');
    // Affichage du résultat à l'écran
    while (BusyXLCD());
    SetDDRamAddr(0x0d);
    while (BusyXLCD());
    putsXLCD(bufferLumiere);
    // On replace le pointeur à la dernière case, 2ème ligne
    while (BusyXLCD());
    SetDDRamAddr(0x4f);
    while (BusyXLCD());
}
