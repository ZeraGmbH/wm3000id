// implemention cWMJustData

#include <qdatastream.h>
#include <q3textstream.h>
#include "wmjustdata.h"


cWMJustData::cWMJustData()
{
    setDefault();
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
}
 

void cWMJustData::Deserialize(QDataStream& qds) // zum lesen aller justagedaten aus flashspeicher
{
    m_pGainCorrection->Deserialize(qds); 
    m_pPhaseCorrection->Deserialize(qds);
    m_pOffsetCorrection->Deserialize(qds);
}


int cWMJustData::getStatus()
{
    return (m_pGainCorrection->getStatus() & m_pPhaseCorrection->getStatus() & m_pOffsetCorrection->getStatus());
}


void cWMJustData::setDefault()
{
    m_pPhaseCorrection = new cJustData(PhaseCorrOrder, 0.0);
    m_pGainCorrection = new cJustData(GainCorrOrder, 1.0);
    m_pOffsetCorrection =  new cJustData(OffsetCorrOrder, 0.0);
}
