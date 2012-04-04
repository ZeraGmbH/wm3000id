// gaussmatrix implementierung

#include "gaussmatrix.h"

cGaussMatrix::cGaussMatrix(int n) {
    m_nn = n;
    m_py = new double[n];
    m_pkoeff = new double[n];
    m_pmatrix = new double[n*n];
}


cGaussMatrix::~cGaussMatrix() {
    delete[] m_py;
    delete[] m_pkoeff;
    delete[] m_pmatrix;
}    


void cGaussMatrix::setMatrix(int z, cGaussNode node) {
    m_py[z] = node.m_fNode; // yz
    int i, aoffs;
    double x = node.m_fArg; // startwert
    double ko = 1.0;
    aoffs = (z+1) * m_nn -1; // adress offset
    for (i = 0; i < m_nn; i++) { // yz = az*x^3 +bz*x^2 +cz*x^1 +dz*x^0
	m_pmatrix[aoffs - i] = ko;
	ko *= x; // potenzieren     
    }
    // eine zeile in der matrix initialisiert
}


void cGaussMatrix::cmpKoeff() { // auflösen der matrix durch elimination und koeffizienten bestimmen
    int i,z,s; // zeile, spalte
    double fak;
    for (i = 1; i < m_nn; i++) {
	for (z = i; z < m_nn; z++)
	{
	    if (m_pmatrix[z*m_nn + i - 1] != 0.0)
		fak = -m_pmatrix[(i-1)*m_nn + i - 1] / m_pmatrix[z * m_nn + i - 1];
	    else
		fak = 0.0;
	    m_py[z] = fak * m_py[z] + m_py[i-1];
	    for (s = i-1; s < m_nn; s++) 
		m_pmatrix[z*m_nn + s] = fak * m_pmatrix[z*m_nn  + s] + m_pmatrix[(i-1)*m_nn +s]; 
	}  
    } // gauss'sche eliminiation erledigt
    
     for (i = 0; i < m_nn; i++) { // alle koeffizienten bestimmen
	z = m_nn -1 - i; // wir starten in der letzten zeile 
	m_pkoeff[z] = m_py[z];
	if (i > 0) { // wir hangeln uns hoch
	    for (s = z+1; s < m_nn; s++) 
		m_pkoeff[z] -= m_pmatrix[z*m_nn + s] * m_pkoeff[s];
	}
	if (m_pmatrix[z*m_nn + z] != 0.0) 
	    m_pkoeff[z] /= m_pmatrix[z*m_nn + z];
	else
	   m_pkoeff[z] = 0.0;
    }
}
    
     
double cGaussMatrix::getKoeff(int n) {
    return m_pkoeff[n];
}
