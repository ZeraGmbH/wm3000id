#ifndef WMJUSTDATABASE_H
#define WMJUSTDATABASE_H

#include <qdatastream.h>
#include "justdata.h"


enum jDataStatus { wrongVERS = 2, wrongSNR = 4};

class cWMJustDataBase { // virtuelle basisklasse justagedaten wm3000
public:
    cWMJustDataBase(){}
    ~cWMJustDataBase(){}

    cJustDataBase* m_pGainCorrection;
    cJustDataBase* m_pPhaseCorrection;
    cJustDataBase* m_pOffsetCorrection;

    virtual void Serialize(QDataStream&) = 0; // zum schreiben aller justagedaten in flashspeicher
    virtual void Deserialize(QDataStream&) = 0; // zum lesen aller justagedaten aus flashspeicher

    virtual QString SerializeStatus() = 0;
    virtual void DeserializeStatus(QString) = 0;

    virtual void setStatus(int stat) = 0;
    virtual int getStatus() = 0; // gibt den gesamt status der justage zurück 1 = justiert 0 nicht justiert

};




#endif // WMJUSTDATABASE_H
