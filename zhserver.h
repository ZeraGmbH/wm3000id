// headerdatei zera hardware server 
// basis klasse für einen solchen
// rein virtuell -> definiert was ein hw-server
// immer benötigt

// cNodeZHServer ist eine erweiterung zum navigieren in scpi (ähnlichen)
// befehlslisten 

#ifndef ZHSERVER_H
#define ZHSERVER_H

#include <netinet/in.h>
#include <q3ptrlist.h>
#include <qstringlist.h>
#include <qstring.h>
#include "scpi.h"
#include "wmscpi.h"
#include "parse.h"
#include "cmdinterpret.h"

class cNodeZHServer: public cNode {
public:
    cNodeZHServer(QStringList*,int,cNode*,cNode*,SCPICmdType,SCPICmdType); 
    virtual ~cNodeZHServer(){};
    // konstruktor, psNodeNames,psNode2Set, nNodedef, pNextNode, pNewLevelNode, Cmd, Query
    virtual cNode* TestNode(cCmdInterpreter*,char**); // zeiger, zeiger auf zeiger auf inputzeile, testet den knoten
    void SetNodeNameList(QStringList*); // zum späteren umschreiben der liste der knotennamen
private:
    QStringList* sNodeNames; // liste der möglichen nodes (es handelt sich z.b. um kanal namen)
};


class cZHClient {
public:
    cZHClient(){};
    cZHClient(int,struct sockaddr_in*);
    ~cZHClient(); //  allokierten speicher ggf. freigeben
    int sock; // socket für den die verbindung besteht
    struct sockaddr_in addr; // address informationen
    void SetOutput(const char*); // setzt den output, d.h. legt auch daten dafür an
    char* GetOutput(); // gibt den output zurück
    bool OutpAvail(); // true wenn mind. 1 zeichen im ausgabepuffer
    void ClearInput(); // löscht den input buffer
    void AddInput(char*); // addiert einen teil string zum buffer
    char* GetInput(); // gibt zeiger auf input
private: 
    QString sOutput;    
    QString sInput;
};

typedef Q3PtrList<cZHClient> ClientList;

class cZHServer {
    
public:
    cZHServer();
    cZHServer(cCmdInterpreter*);
    virtual ~cZHServer(){};
    virtual int Execute(); // server ausführen
    QString& GetSoftwareVersion();
    virtual int SetServerNr(const char*); // setzen der device nr -> neuen server namen
    virtual void AddClient(int, sockaddr_in*); // fügt einen client hinzu
    virtual void DelClient(int); // entfernt einen client
protected:
    ClientList clientlist; // liste der clients
    QString sServerName;
    static int ActSock; // der aktive socket im Execute 
    static QString sSoftwareVersion; // version des hw-servers (programm name + version )
    cCmdInterpreter* pCmdInterpreter; // der benutzte kommando interpreter
};    
    
#endif // ifndef ZHSERVER_H
