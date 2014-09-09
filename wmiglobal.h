// globale definition

// abgeleitet von wm3000ud V1.02
// welche schon das neue calculate model unterstützt
// V1.01 18.01.12
// es wurden eine query und ein command eingeführt zum setzen, rücksetzen und abfragen des sensor schutz mosfet ( sense:protection) weil die hardware sich nicht schützen kann...... lach :-)
// V1.02 14.02.12
// es gibt jetzt detailiertere fehlermeldungen wenn im xml justage import file fehler auftreten auf der schnittstelle
// beim lesen der justagedaten aus dem flash werden auf der hinteren stelle der versionsnummern für lca und ctrl änderungen zugelassen ohne dass es zu einer meldung nicht justiert oder so führt.
// V1.03 29.11.12
// kommando zur umschaltung absolut / differenzmessung eingebaut

// version vorne hochgezählt zur unterscheidung alte/neue cpu
// V2.04 einbau einer synchronisation auf atmel toggle bit. hierzu wird das zFPGA1reg device verwendet. auf register mit
// der adresse 0xfff wird in bit 0 die auswertung des atmel toggle bit gesetzt. es wird beim start des pcb servers 10sek.
// gewartet ob der atmel ins leben kommt. danach wird durchgestartet.

// V2.05 es wird beim start nachgeschaut ob die datei atmelFlashfilePath existiert,  wenn ja muss der
// atmel neu programmiert werden. hierzu wird zunächst ein reset durchgeführt, der bootloader innerhalb
// 100ms angesprochen, dadurch startet dieser nicht in die applikation durch und anschliessend das
// intel hexfile gelesen und in den atmel geschrieben. danach wir die apllikation gestartet und gewartet
// dass der atmel läuft. wenn ja, dann wird das file gelöscht.

#ifndef WMGOBAL_H
#define WMGLOBAL_H

#define FPGADeviceNode "/dev/zFPGA1reg"
#define CheckSumOffset 56
#define LeiterkartenName "wm3000i"
#define ServerBasisName "wm3000id"
#define ServerVersion "V2.05"
#define InpBufSize 4096
#define atmelFlashfilePath "/opt/zera/bin/atmel.hex"
#define atmelResetBit 16

// wenn WMDEBUG -> kein fork() 
//#define WMDEBUG 1

#endif
