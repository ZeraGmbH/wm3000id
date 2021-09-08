#ifndef JUSTDATAV208_H
#define JUSTDATAV208_H

#include <qdatastream.h>

#include "justdatabase.h"

class cJustDataV208:public cJustDataBase { // klasse für justage koeffizienten und stützstellen altes format
public:
    cJustDataV208(int order,double init);
    ~cJustDataV208();
    virtual void Serialize(QDataStream&); // zum schreiben der justagedaten in flashspeicher
    virtual void Deserialize(QDataStream&); // reicht eine routine für koeffizienten und nodes
    virtual void setStatus(int);
    virtual int getStatus();

    virtual QString SerializeStatus();
    virtual void DeserializeStatus(QString);


private:
    int m_nStatus; // der status wird nur bei den neuere justage werte ab V2.08 verwendet
};

#endif // JUSTDATAV208_H
