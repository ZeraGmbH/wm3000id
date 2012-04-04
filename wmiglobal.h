// globale definition

// abgeleitet von wm3000ud V1.02
// welche schon das neue calculate model unterstützt
// V1.01 18.01.12
// es wurden eine query und ein command eingeführt zum setzen, rücksetzen und abfragen des sensor schutz mosfet ( sense:protection) weil die hardware sich nicht schützen kann...... lach :-)
// V1.02 14.02.12
// es gibt jetzt detailiertere fehlermeldungen wenn im xml justage import file fehler auftreten auf der schnittstelle
// beim lesen der justagedaten aus dem flash werden auf der hinteren stelle der versionsnummern für lca und ctrl änderungen zugelassen ohne dass es zu einer meldung nicht justiert oder so führt.


#ifndef WMGOBAL_H
#define WMGLOBAL_H

#define CheckSumOffset 56
#define LeiterkartenName "wm3000i"
#define ServerBasisName "wm3000id"
#define ServerVersion "V1.02"
#define InpBufSize 4096

// wenn WMDEBUG -> kein fork() 
//#define WMDEBUG 1

#endif
