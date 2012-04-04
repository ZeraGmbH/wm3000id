// headerfile für gausmatrix klasse
// dient zur numerischen lösung von gleichungssystemen mit
// variabler anzahl von unbekannten 
//  y=ax^3 + bx^2 + cx +d ist eine gleichung 3. ordnung
// 

#ifndef GMATRIX_H
#define GMATRIX_H

struct cGaussNode { // eine stützstelle
    double m_fNode; // stützwert
    double m_fArg; // argument
};
    

class cGaussMatrix {
public:
    cGaussMatrix(int); // legt eine Matrix n'ter ordnung an
    ~cGaussMatrix(); 
    void setMatrix(int, cGaussNode);
    void cmpKoeff();
    double getKoeff(int); 
private:
    int m_nn; // anzahl unbekannte n
    double *m_py; // n werte
    double *m_pkoeff; // n koeffizienten
    double *m_pmatrix; // n*n werte (quadratische matrix)
};

#endif

