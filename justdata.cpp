// implementierung justdata.cpp
// rund ums justieren

#include <math.h>
#include "justdata.h"
#include "gaussmatrix.h"


cJustNode::cJustNode(double corr, double arg)
    :m_fCorrection(corr),m_fArgument(arg)
{
}


void cJustNode::Serialize(QDataStream& qds)
{
    qds << m_fCorrection << m_fArgument;
}


void cJustNode::Deserialize(QDataStream& qds)
{
    qds >> m_fCorrection >> m_fArgument;
}


QString cJustNode::Serialize()
{
    QString s;
    s = QString("%1,%2;").arg(m_fCorrection,0,'f',6)
	                       .arg(m_fArgument,0,'f',6);
    return s;
}

 
void cJustNode::Deserialize(const QString& s)
{
    m_fCorrection = s.section( ',',0,0).toDouble();
    m_fArgument = s.section( ',',1,1).toDouble();
}
    

cJustNode& cJustNode::operator = (const cJustNode& jn)
{
    this->m_fCorrection = jn.m_fCorrection;
    this->m_fArgument = jn.m_fArgument;
    return (*this);
}


// ab hier justdata klasse ...hält die koeffizienten und stützstellen

cJustData::cJustData(int order,double init)
    :m_nOrder(order)
{
    m_pCoefficient = new double[order+1];  
    m_pJustNode = new cJustNode[order+1]; 
    setNode(0 , cJustNode(init,0.0)); // setzt den 1. node und die folgenden 
    cmpCoefficients();
}


cJustData::~cJustData()
{
    delete [] m_pCoefficient;
    delete [] m_pJustNode;
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
	
	
QString cJustData::SerializeCoefficients() // fürs xml file halten wir das getrennt
{
    int i;
    QString s = "";
    for (i = 0; i < m_nOrder+1; i++)
	s += QString("%1;").arg(m_pCoefficient[i],0,'f',12);
    return s;
}


QString cJustData::SerializeNodes()
{
    int i;
    QString s = "";
    for (i = 0; i < m_nOrder+1; i++)
	s += m_pJustNode[i].Serialize();
    return s;
}    


void cJustData::DeserializeCoefficients(const QString& s) 
{	
    int i;
    for (i = 0; i < m_nOrder+1; i++)
	m_pCoefficient[i] = s.section(';',i,i).toDouble();
}


void cJustData::DeserializeNodes(const QString& s)
{
    int i;
    QString t;
    for (i = 0; i < m_nOrder+1; i++)
	m_pJustNode[i].Deserialize(s.section(';',i,i));
}


bool cJustData::setNode(int index, cJustNode jn) // !!! setzen der stützstellen ist reihenfolge abhängig !!!
{
    if (index <= m_nOrder)
    {
	int i;
	for (i = index; i < m_nOrder+1; i++)
	    m_pJustNode[i] = jn;
	return true;
    }
    return false;
}


cJustNode* cJustData::getNode(int index) // lassen sich auch rücklesen
{
    return &m_pJustNode[index];
}
 

bool cJustData::setCoefficient(int index, double value)
{
    if (index <= m_nOrder)
    {
	int i;
	m_pCoefficient[index] = value;
	if (index < m_nOrder)
	    for (i = index+1; i < m_nOrder+1; i++)
		m_pCoefficient[i] = 0.0;
	return true;
    }
    return false;
}


double cJustData::getCoefficient(int index)
{
    return m_pCoefficient[index];
}


bool cJustData::cmpCoefficients() // berechnet aus den stützstellen die koeffizienten
{
    const double epsilon = 1e-7;
    int realOrd, i; // reale ordnung feststellen
    
    realOrd = 0;
    if (m_nOrder > 0) // nur wenn wir höhere ordnung zulassen untersuchen welche ordnung es ist
    {
	for (i = 0;i < m_nOrder; i++) {
	    if (fabs(m_pJustNode[i].m_fArgument - m_pJustNode[i+1].m_fArgument) < epsilon)
		break;
	    realOrd++;
	}
    }
    
    // matrix befüllen
    cGaussMatrix *Matrix;
    Matrix = new cGaussMatrix(realOrd+1);
    cGaussNode gn;
    for (i = 0; i < realOrd+1; i++)
    {
	gn.m_fNode = m_pJustNode[i].m_fCorrection;
	gn.m_fArg = m_pJustNode[i].m_fArgument;
	Matrix->setMatrix(i, gn);
    }
        
    // matrix berechnen
    Matrix->cmpKoeff();
    
    // und auslesen
    for (i = 0; i < realOrd+1; i++)
	setCoefficient(i, Matrix->getKoeff(realOrd-i));
    // bzw. 0 setzen der nicht berechneten koeffizienten
    for (i = i; i < m_nOrder+1; i++)
	setCoefficient(i, 0.0);
    delete Matrix;
    
    return true;
}

    
double cJustData::getCorrection(double arg) // berechnet den korrekturwert
{
    double Arg = 1.0;
    double Corr = 0.0;
    for (int i = 0; i < m_nOrder+1; i++) { // korrektur funktion ist  n. ordnung
	Corr += m_pCoefficient[i] * Arg;
	Arg *= arg;
    }
    
    return Corr;
}    
    


