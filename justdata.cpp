// implementierung justdata.cpp
// rund ums justieren

#include <math.h>
#include "justdata.h"
#include "gaussmatrix.h"



// ab hier justdata klasse ...hält die koeffizienten und stützstellen

cJustData::cJustData(int order,double init)
    :cJustDataBase(order, init)
{
}


cJustData::~cJustData()
{
}


void cJustData::Serialize(QDataStream& qds) // zum schreiben der justagedaten in flashspeicher
{
    int i;
    for (i = 0; i < m_nOrder+1; i++)
        qds << m_pCoefficient[i];
    for (i = 0; i < m_nOrder+1; i++)
        m_pJustNode[i].Serialize(qds);
}


void cJustData::Deserialize(QDataStream& qds) // lesen der justagedaten aus flashspeicher
{
    int i;
    for (i = 0; i < m_nOrder+1; i++)
        qds >> m_pCoefficient[i];
    for (i = 0; i < m_nOrder+1; i++)
        m_pJustNode[i].Deserialize(qds);
}

void cJustData::setStatus(int) // es gibt hier keinen status
{
}


int cJustData::getStatus()
{
    return 0;
}


QString cJustData::SerializeStatus()
{
    return QString("0"); // wir haben hier gar keinen status
}


void cJustData::DeserializeStatus(QString) // wie schon gesagt ...
{
}
