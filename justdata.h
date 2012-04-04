// header datei justdata.h
// klassen deklaration rund ums justieren

#ifndef JUSTDATA_H
#define JUSTDATA_H

#include <qdatastream.h>
#include <q3textstream.h>


class cJustNode { // stützspunkt kann sich serialisieren und besteht aus stützwert (correction) und argument
public:
    cJustNode(double corr, double arg);
    cJustNode(){};
    ~cJustNode(){};
    void Serialize(QDataStream&);
    void Deserialize(QDataStream&);
    QString Serialize();
    void Deserialize(const QString&);
    cJustNode& operator = (const cJustNode&);
    
    double m_fCorrection;
    double m_fArgument;
};


// ein justagedaten objekt hat eine max. mögliche ordnung.
// diese muss nicht zwangsläufig ausgenutzt werden.
// wird z.B. nur die 1. stützstelle eingetragen ist die tatsächliche ordnung 0
// so wird ein justagedaten objekt auch default initialisiert

class cJustData { // basisklasse für justage koeffizienten und stützstellen
public: 
    cJustData(int order,double init); 
    ~cJustData();
    void Serialize(QDataStream&); // zum schreiben der justagedaten in flashspeicher
    void Deserialize(QDataStream&); // reicht eine routine für koeffizienten und nodes 
    QString SerializeCoefficients(); // fürs xml file halten wir das getrennt
    QString SerializeNodes();
    void DeserializeCoefficients(const QString&);
    void DeserializeNodes(const QString&);
    bool setNode(int index, cJustNode jn); // !!! setzen der stützstellen ist reihenfolge abhängig !!!
    cJustNode* getNode(int index); // lassen sich auch rücklesen
    bool setCoefficient(int index, double); // !!! setzen der koeffizienten ist reihenfolge abhängig !!!
    double getCoefficient(int index);
    bool cmpCoefficients(); // berechnet aus den stützstellen die koeffizienten
    double getCorrection(double arg); // berechnet den korrekturwert  c= ax^order +bx^order-1 ...
private:
    double* m_pCoefficient; // es werden dyn. arrays[ordnung+1]  erzeugt
    cJustNode* m_pJustNode; // dito
    int m_nOrder; // wir merken uns die ordnung
};

	
#endif
