#include "justdatav208.h"

cJustDataV208::cJustDataV208(int order,double init)
    :cJustDataBase(order, init)
{
    m_nStatus = 0;
}


cJustDataV208::~cJustDataV208()
{
}


void cJustDataV208::Serialize(QDataStream& qds) // zum schreiben der justagedaten in flashspeicher
{
    int i;
    qds << m_nStatus;
    for (i = 0; i < m_nOrder+1; i++)
        qds << m_pCoefficient[i];
    for (i = 0; i < m_nOrder+1; i++)
        m_pJustNode[i].Serialize(qds);
}


void cJustDataV208::Deserialize(QDataStream& qds) // lesen der justagedaten aus flashspeicher
{
    int i;
    qds >> m_nStatus;
    for (i = 0; i < m_nOrder+1; i++)
        qds >> m_pCoefficient[i];
    for (i = 0; i < m_nOrder+1; i++)
        m_pJustNode[i].Deserialize(qds);
}


void cJustDataV208::setStatus(int stat)
{
    if (stat == 0)
        m_nStatus = stat;
    else
        m_nStatus = 1;
}


int cJustDataV208::getStatus()
{
    return m_nStatus;
}


QString cJustDataV208::SerializeStatus() // für xml file
{
    return QString("%1").arg(m_nStatus);
}


void cJustDataV208::DeserializeStatus(QString s)
{
    m_nStatus = s.toInt();
}



