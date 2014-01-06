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
void lectureRFID(char);
void LiczCRC2(unsigned char *, unsigned short *, unsigned char);
void afficherLumiere(void);
void deconnexion(void);

/*
 ### DECLARATION DES VARIABLES GLOBALES ###
 * Boutons
 * USART 1 USB
 * CAN
 */
volatile unsigned char flag_bouton_lecture = 0, flag_bouton_ecriture = 0, flag_bouton_central = 0, flag_bouton_deconnexion = 0;
volatile unsigned char flag_fin_trame_USB = 0, cpt_tab_USB = 0, flag_prenom_recu_USB = 0, flag_new_PK_recue_USB = 0;
volatile char caractere_usb_recu;
volatile char tabRecuUSB[20];
volatile char bufferLumiere [20];

/*
 ### DECLARATION DES VARIABLES GLOBALES RFID###
 */
volatile unsigned char bufWriteRFID[15], bufReadRFID[6];
unsigned volatile char flag_lecture_RFID = 0, flag_ecriture_RFID = 0, iRec = 0;
volatile char tabReponseRFID[15];
volatile char tabLectureRFID[5];
volatile unsigned char bufWriteRFID[15], bufReadRFID[6];

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
    tabLectureRFID[0] = 0;
    tabLectureRFID[1] = 0;
    tabLectureRFID[2] = 0;
    tabLectureRFID[3] = 0;
    tabLectureRFID[4] = 0;
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
    // Affichage du premier écran pour connecter un utilisateur
    deconnexion();


    /*
     *
     * BOUCLE PRINCIPALE DU PROGRAMME
     * Chargée de vérifier en permanence
     * les différents FLAGS et déclencher
     * les fonctions appropriées
     *
     */

    while (1) {
        if (flag_bouton_lecture == 1) {
            SetDDRamAddr(0x00);
            while (BusyXLCD());
            putrsXLCD("Reading...  ");
            while (BusyXLCD());
            Delay10KTCYx(120);
            lectureRFID(0x02);
            flag_bouton_lecture = 0;
        }
        if (flag_bouton_ecriture == 1) {
            SetDDRamAddr(0x00);
            while (BusyXLCD());
            putrsXLCD("Writing...  ");
            while (BusyXLCD());
            ecriture_PK_RFID();
            flag_bouton_ecriture = 0;
        }
        if (flag_bouton_deconnexion == 1) {
            deconnexion();
            flag_bouton_deconnexion = 0;
        }
        // TODO DEBUG : Permet d'écrire un PK admin !!
        if (flag_bouton_central == 1) {
            SetDDRamAddr(0x00);
            while (BusyXLCD());
            putrsXLCD("Writing ROOT tag");
            while (BusyXLCD());
            tabRecuUSB[2] = 'P';
            tabRecuUSB[3] = 'K';
            tabRecuUSB[4] = '0';
            tabRecuUSB[5] = '0';
            ecriture_PK_RFID();
            flag_bouton_central = 0;
        }
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
            flag_new_PK_recue_USB = 0;

        }

        if (flag_lecture_RFID == 1) {
            if (iRec >= 9) {
                if (tabReponseRFID[7] == 0xff) {
                    while (BusyXLCD());
                    WriteCmdXLCD(0x01); //Clear LCD
                    while (BusyXLCD());
                    SetDDRamAddr(0x00);
                    while (BusyXLCD());
                    putrsXLCD("Reading OK!  ");
                    Delay10KTCYx(200);
                    Delay10KTCYx(200);
                    send_usart_pk(tabReponseRFID[3], tabReponseRFID[4], tabReponseRFID[5], tabReponseRFID[6]);
                    reset_tableau_RFID();
                } else {
                    while (BusyXLCD());
                    WriteCmdXLCD(0x01); //Clear LCD
                    while (BusyXLCD());
                    SetDDRamAddr(0x00);
                    while (BusyXLCD());
                    putrsXLCD("Reading err.");
                    while (BusyXLCD());
                    SetDDRamAddr(0x40);
                    while (BusyXLCD());
                    putrsXLCD("Try again ?");
                    reset_tableau_RFID();
                }
            }
            flag_lecture_RFID = 0;
        }

        if (flag_ecriture_RFID == 1) {
            if (iRec >= 5) {
                if (tabReponseRFID[3] == 0xff) {
                    while (BusyXLCD());
                    WriteCmdXLCD(0x01);
                    while (BusyXLCD());
                    SetDDRamAddr(0x00);
                    while (BusyXLCD());
                    putrsXLCD("Success !");
                    while (BusyXLCD());
                    // Simulation du redémarrage de la carte pour un nouvel utilisateur
                    deconnexion();
                } else {
                    while (BusyXLCD());
                    WriteCmdXLCD(0x01);
                    while (BusyXLCD());
                    SetDDRamAddr(0x00);
                    while (BusyXLCD());
                    putrsXLCD("FAIL !   ");
                    while (BusyXLCD());
                    SetDDRamAddr(0x40);
                    while (BusyXLCD());
                    putrsXLCD("Try again ?");
                }
                flag_ecriture_RFID = 0;
                while (BusyXLCD());
                SetDDRamAddr(0x00);
                reset_tableau_RFID();
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
        tabReponseRFID[iRec] = RCREG2;
        iRec++;
        if (tabReponseRFID[2] == 0x11) {
            // Ecriture
            flag_ecriture_RFID = 1;
        }
        if (tabReponseRFID[2] == 0x13) {
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
            flag_bouton_lecture = 1;
        }
        if (RIGHT == 0) {
            flag_bouton_ecriture = 1;
        }
        if (CENTER == 0) {
            flag_bouton_central = 1;
            send_usart_pk('P', 'K', '0', '0');
        }
        if (UP == 0) {
            flag_bouton_deconnexion = 1;
        }
        if (DOWN == 0) {
            //flag_bouton_lecture = 1;
            // TODO DEBUG : Si pas de module RFID
            send_usart_pk('P', 'K', '0', '2');
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
    // Cas d'un tableau de prénom --> On l'affiche sur le LCD
    if (tabRecuUSB[0] == '#' && tabRecuUSB[1] == 'P') {
        flag_prenom_recu_USB = 1;
        while (BusyXLCD());
        WriteCmdXLCD(0x01);
        while (BusyXLCD());
        putrsXLCD("Welcome,");
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
    // -> Activation du flag
    // -> Ecriture de la PK via RFID à partir de la boucle While
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
        tabReponseRFID[j] = 0;
        iRec = 0;
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
    unsigned char send = 0, tmp;
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
    // Inversion des cases 4 et 5
    tmp = bufReadRFID[4];
    bufReadRFID[4] = bufReadRFID[5];
    bufReadRFID[5] = tmp;

    TXREG2 = bufReadRFID[send];
    send++;

    // Envoi complet de la trame
    while (send <= 5) {
        while (TXSTA2bits.TRMT != 1);
        TXREG2 = bufReadRFID[send];
        send++;
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

void deconnexion(void) {
    while (BusyXLCD());
    WriteCmdXLCD(0x01); //Clear LCD
    while (BusyXLCD());
    SetDDRamAddr(0x00);
    while (BusyXLCD());
    putrsXLCD("Loading...  ");
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
}
