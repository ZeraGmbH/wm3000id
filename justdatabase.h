#ifndef JUSTDATABASE_H
#define JUSTDATABASE_H

#include <qdatastream.h>
#include <q3textstream.h>


class cJustNode { // stützspunkt kann sich serialisieren und besteht aus stützwert (correction) und argument
public:
    cJustNode(double corr, double arg);
    cJustNode(){}
    ~cJustNode(){}
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


class cJustDataBase { // virtuelle basisklasse für justage koeffizienten und stützstellen
public:
    cJustDataBase(int order,double init);
    ~cJustDataBase();
    virtual void Serialize(QDataStream&) = 0; // zum schreiben der justagedaten in flashspeicher
    virtual void Deserialize(QDataStream&) = 0; // reicht eine routine für koeffizienten und nodes
    virtual void setStatus(int) = 0;
    virtual int getStatus() = 0;

    virtual QString SerializeStatus() = 0; // fürs xml file halten wir alle exports getrennt
    virtual void DeserializeStatus(QString) = 0;

    QString SerializeCoefficients();
    QString SerializeNodes();

    void DeserializeCoefficients(const QString&);
    void DeserializeNodes(const QString&);

    bool setNode(int index, cJustNode jn); // !!! setzen der stützstellen ist reihenfolge abhängig !!!
    cJustNode* getNode(int index); // lassen sich auch rücklesen
    bool setCoefficient(int index, double); // !!! setzen der koeffizienten ist reihenfolge abhängig !!!
    double getCoefficient(int index);
    bool cmpCoefficients(); // berechnet aus den stützstellen die koeffizienten
    double getCorrection(double arg); // berechnet den korrekturwert  c= ax^order +bx^order-1 ...


protected:
    double* m_pCoefficient; // es werden dyn. arrays[ordnung+1]  erzeugt
    cJustNode* m_pJustNode; // dito
    int m_nOrder; // wir merken uns die ordnung
};



#endif // JUSTDATABASE_H
