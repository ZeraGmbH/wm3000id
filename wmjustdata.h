// header datei wmjustdata.h

#ifndef WMJUSTDATA_H
#define WMJUSTDATA_H

#include <qdatastream.h>
#include <q3textstream.h>
#include "justdata.h"


enum jDataStatus { wrongVERS = 2, wrongSNR = 4};

const int GainCorrOrder = 3;
const int PhaseCorrOrder  = 3;
const int OffsetCorrOrder = 0;
	

class cWMJustData { // alle korrekturdaten für einen bereich + status 
public:
    cWMJustData();
    ~cWMJustData();
    
    cJustData* m_pGainCorrection;
    cJustData* m_pPhaseCorrection; 
    cJustData* m_pOffsetCorrection;
    
    void Serialize(QDataStream&); // zum schreiben aller justagedaten in flashspeicher
    void Deserialize(QDataStream&); // zum lesen aller justagedaten aus flashspeicher
    
    int getStatus(); // gibt den gesamt status der justage zurück 1 = justiert 0 nicht justiert
    void setDefault();
};


#endif

