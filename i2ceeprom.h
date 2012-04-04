// header datei für i2c eeprom implementationen

#ifndef I2CEEPROM_H
#define I2CEEPROM_H

class QString;

class cI2CEEProm { // virtuelle basisklasse für i2c eeproms
public:
    cI2CEEProm(QString,int,short);
    cI2CEEProm(); 
    virtual ~cI2CEEProm(){};
    virtual int WriteData(char*,int,int)=0; // schreibt die daten ab char*  , mit der länge (int) ab adresse (int) ins EEProm; rückgabewert gibt die tatsächlich geschriebenen bytes an
    virtual int ReadData(char*,int,int)=0; // wie write, nur halt zum lesen
    virtual int size()=0;
    short I2CAdress;
    QString DevNode;
    int DebugLevel;
};

#define Write24LC256 0
#define Read24LC256 1

class c24LC256: public cI2CEEProm {
public:
    c24LC256(QString,int,short);
    virtual int WriteData(char*,int,int);
    virtual int ReadData(char*,int,int);
    virtual int size();
};

#endif
