// implemention cWMJustData

#include "zeraglobal.h"
#include "justdata.h"
#include "wmjustdata.h"


cWMJustData::cWMJustData()
{
    m_pGainCorrection = new cJustData(1, 1.0); // 1.ordnung
    m_pPhaseCorrection = new cJustData(3, 0.0); // 3.ordnung
    m_pOffsetCorrection =  new cJustData(0, 0.0); // 0.te ordnung
    setStatus(0);
}


cWMJustData::~cWMJustData()
{
    delete m_pGainCorrection; 
    delete m_pPhaseCorrection;
    delete m_pOffsetCorrection;
}
	    

void cWMJustData::Serialize(QDataStream& qds)  // zum schreiben aller justagedaten in flashspeicher
{
    m_pGainCorrection->Serialize(qds); 
    m_pPhaseCorrection->Serialize(qds);
    m_pOffsetCorrection->Serialize(qds);
    qds << m_nStatus; // die alten justage daten haben nur einen status
}
 

void cWMJustData::Deserialize(QDataStream& qds) // zum lesen aller justagedaten aus flashspeicher
{
    m_pGainCorrection->Deserialize(qds); 
    m_pPhaseCorrection->Deserialize(qds);
    m_pOffsetCorrection->Deserialize(qds);
    qds >> m_nStatus; // die alten justage daten haben nur einen status
}


QString cWMJustData::SerializeStatus()
{
    return QString("%1").arg(m_nStatus);
}


void cWMJustData::DeserializeStatus(QString s)
{
    m_nStatus = s.toInt();
}


int cWMJustData::getStatus()
{
    if ((m_nStatus & (RangeGainJustified + RangePhaseJustified)) == (RangeGainJustified + RangePhaseJustified))
        return 1;
    else
        return 0;
}


void cWMJustData::setStatus(int stat)
{
    m_nStatus = stat;
}
