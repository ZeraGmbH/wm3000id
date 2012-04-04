// implemention cWMJustData

#include <qdatastream.h>
#include <q3textstream.h>
#include "wmjustdata.h"


cWMJustData::cWMJustData()
{
    m_pPhaseCorrection = new cJustData(PhaseCorrOrder, 1.0);
    m_pGainCorrection = new cJustData(GainCorrOrder, 1.0); 
    m_pOffsetCorrection =  new cJustData(OffsetCorrOrder, 0.0);
    m_nStatus = 0; // nix justiert... nix kaputt
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
    qds << m_nStatus;
}
 
void cWMJustData::Deserialize(QDataStream& qds) // zum lesen aller justagedaten aus flashspeicher
{
    m_pGainCorrection->Deserialize(qds); 
    m_pPhaseCorrection->Deserialize(qds);
    m_pOffsetCorrection->Deserialize(qds);
    qds >> m_nStatus;
}


QString cWMJustData::SerializeStatus()
{
    return QString("%1;").arg(m_nStatus);
}


void cWMJustData::DeserializeStatus(const QString& s)
{
    m_nStatus = s.section(';',0,0).toInt();
}


void cWMJustData::setStatus(int stat)
{
    m_nStatus = stat;
}
 

int cWMJustData::getStatus()
{
    return m_nStatus;
}
