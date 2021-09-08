// header datei wmjustdata.h

#ifndef WMJUSTDATA_H
#define WMJUSTDATA_H


#include <qdatastream.h>
#include "wmjustdatabase.h"
#include "justdata.h"


class cWMJustData:public cWMJustDataBase { // alle korrekturdaten für einen bereich + status
public:
    cWMJustData();
    ~cWMJustData();
    
    void Serialize(QDataStream&); // zum schreiben aller justagedaten in flashspeicher
    void Deserialize(QDataStream&); // zum lesen aller justagedaten aus flashspeicher

    virtual QString SerializeStatus(); // für den xml export
    virtual void DeserializeStatus(QString);

    virtual int getStatus(); // gibt den status der justage zurück 80 (64+16) = justiert
    virtual void setStatus(int stat);

private:
    int m_nStatus;
};


#endif

