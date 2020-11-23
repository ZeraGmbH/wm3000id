#include "justdatav208.h"
#include "wmjustdatav208.h"


cWMJustDataV208::cWMJustDataV208()
{
    m_pGainCorrection = new cJustDataV208(3, 1.0); // 3.ordnung
    m_pPhaseCorrection = new cJustDataV208(3, 0.0); // 3.ordnung
    m_pOffsetCorrection =  new cJustDataV208(0, 0.0); // 0.te ordnung
}


cWMJustDataV208::~cWMJustDataV208()
{
    delete m_pGainCorrection;
    delete m_pPhaseCorrection;
    delete m_pOffsetCorrection;
}


void cWMJustDataV208::Serialize(QDataStream& qds)  // zum schreiben aller justagedaten in flashspeicher
{
    m_pGainCorrection->Serialize(qds);
    m_pPhaseCorrection->Serialize(qds);
    m_pOffsetCorrection->Serialize(qds);
}


void cWMJustDataV208::Deserialize(QDataStream& qds) // zum lesen aller justagedaten aus flashspeicher
{
    m_pGainCorrection->Deserialize(qds);
    m_pPhaseCorrection->Deserialize(qds);
    m_pOffsetCorrection->Deserialize(qds);
}

QString cWMJustDataV208::SerializeStatus()
{
    return QString(""); // wir haben hier keinen status
}


void cWMJustDataV208::DeserializeStatus(QString)
{
}


void cWMJustDataV208::setStatus(int)
{
}


int cWMJustDataV208::getStatus()
{
    return (m_pGainCorrection->getStatus() & m_pPhaseCorrection->getStatus() & m_pOffsetCorrection->getStatus());
}

