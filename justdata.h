// header datei justdata.h
// klassen deklaration rund ums justieren

#ifndef JUSTDATA_H
#define JUSTDATA_H

#include <qdatastream.h>

#include "justdatabase.h"

class cJustData: public cJustDataBase { // klasse für justage koeffizienten und stützstellen altes format
public: 
    cJustData(int order,double init); 
    ~cJustData();
    virtual void Serialize(QDataStream&); // zum schreiben der justagedaten in flashspeicher
    virtual void Deserialize(QDataStream&); // reicht eine routine für koeffizienten und nodes
    virtual void setStatus(int);
    virtual int getStatus();

    virtual QString SerializeStatus();
    virtual void DeserializeStatus(QString);
};

	
#endif
