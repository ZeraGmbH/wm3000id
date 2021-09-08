#ifndef WMJUSTDATAV208_H
#define WMJUSTDATAV208_H

#include <qdatastream.h>
#include "wmjustdatabase.h"
#include "justdatav208.h"


class cWMJustDataV208:public cWMJustDataBase { // alle korrekturdaten für einen bereich + status
public:
    cWMJustDataV208();
    ~cWMJustDataV208();

    void Serialize(QDataStream&); // zum schreiben aller justagedaten in flashspeicher
    void Deserialize(QDataStream&); // zum lesen aller justagedaten aus flashspeicher

    virtual QString SerializeStatus(); // für den xml export
    virtual void DeserializeStatus(QString);

    virtual void setStatus(int);
    virtual int getStatus(); // gibt den gesamt status der justage zurück 1 = justiert 0 nicht justiert

};
#endif // WMJUSTDATAV208_H
