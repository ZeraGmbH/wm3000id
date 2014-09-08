// header datei wm3000id.h
// globale definitionen


#ifndef WM3000ID_H
#define WM3000ID_H

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qmap.h>
#include <q3valuelist.h>
#include <qdatetime.h>
#include <qdom.h>


#include "wmjustdata.h"
#include "zhserver.h"
#include "wmscpi.h"

// alles default werte, können beim start überschrieben werden
#define I2CDeviceNode "/dev/i2c-3"
#define I2CMasterAdress 0x20
#define I2CSlaveAdress 0x21

// diese werte sind fix
#define I2CEEPromAdress 0x50
#define MaxClients 10 

/*
#define Ch0RangeAdrPIC 0
#define Ch1RangeAdrPIC 1 
#define EEPromEnableAdrPIC 5
#define EEPromEnable 0x04 
#define BusyMask 0x80
#define OverloadMask 0x40
*/

#define MaxDebugLevel 7
#define DEBUG1 (DebugLevel & 1) // alle fehlermeldungen loggen
#define DEBUG2 (DebugLevel & 2) // alle i2c aktivitäten loggen
#define DEBUG3 (DebugLevel & 4) // alle client an-,abmeldungen

#define ch0_n 14 /* 14 bereiche für kanal 0 */
#define ch1_n 24 /* 24 bereiche für kanal 1 */ 

enum hw_cmdcode {	hwGetSerialNr = 0x0001,	hwGetDevName = 0x0002,
			hwGetCtrlVersion = 0x0003,	hwGetLCAVersion = 0x0004,
			hwGetPCBVersion = 0x0005,hwSetSerialNr = 0x0006,
			hwSetPCBVersion = 0x0007,hwStartBootloader = 0x0008,
			
			hwGetInfStat  = 0x0100,	
			
			hwGetCritStat = 0x0200,
			hwResetCritStat = 0x0201,	hwSetIntMask = 0x0202,
			hwGetIntMask = 0x0203,	
			
			hwSetSeqMask = 0x1000,	hwGetSeqMask = 0x1001,
			hwSetMode = 0x1004,	hwGetMode = 0x1005,
			hwSetFrequency = 0x1006,	hwGetFrequency = 0x1007,
			hwSetSRate = 0x1008,	hwGetSRate = 0x1009,
			hwGetPCBTemp = 0x100A,	hwGetFlashWriteAccess=0x100B,
			hwSetPPSSync = 0x100C,	hwGetPPSSync = 0x100D,
			hwSetSenseProt = 0x100E,  hwGetSenseProt = 0x100F,
            hwSetSenseAbsDiff = 0x1010,  hwGetSenseAbsDiff = 0x1011,

			hwSetRange = 0x1100,	hwGetRange = 0x1101};

			
			
struct hw_cmd {
    hw_cmdcode cmdcode;
    char device;
    const char* par;
    short plen;
    short cmdlen;
    quint8* cmddata;
    short RM;
};


enum bl_cmdcode {  blReadInfo = 0x00,		blStartProgram = 0x01,
		   blReadFlashBlock = 0x10,		blReadEEPromBlock = 0x11,
		   blReadAdressPointer = 0x12,	blWriteFlashBlock = 0x20,
		   blWriteEEPromBlock = 0x21,	blWriteAddressPointer = 0x22};

struct bl_cmd {
    bl_cmdcode cmdcode;
    char* par;
    short plen;
    short cmdlen;
    quint8* cmddata;
    short RM;
};


enum enConfFlags { blAutoIncr = 1, blReadCommandsAvail = 2};

struct blInfo {
    ushort ConfigurationFlags;
    ushort MemPageSize;
    uchar AdressPointerSize;
};

void initLists();

// enum {UnVisible = 1, UnSelectable = 2};

struct sRange {
        const char* RName;		// Range  Name
        const char* RValue;		// Range Value 100%
        const char* RRejection;		// Range Rejection 100%
        char  RSelCode;                 // Range Selection Code (only for internal use)
        char RType;                     // Volt, Ampere.....
        char RSpec;                     // phsys, log, virt
        cWMJustData* pJustData;         // Zeiger auf Justierdaten
};


typedef QMap<QString,sRange*> tChannelRangeArrayMap; // zeigt für jeden kanal auf ein array v. sRange
typedef QMap<QString,QStringList*> tChannelListMap; // zeigt für jeden kanal auf eine stringliste 
typedef Q3ValueList<int> tSockList;
typedef QMap<QString,tSockList*> tChannelSockListMap;
 

class cWM3000iServer: public cZHServer , public cbIFace {
public:
    cWM3000iServer();
    virtual ~cWM3000iServer(){};
    virtual void DelClient(int); // entfernt einen client
    virtual int Execute(); // server ausführen überladen v. cZHServer
    virtual void AddClient(int, sockaddr_in*); // fügt einen client hinzu
    
    virtual const char* SCPICmd( SCPICmdType, char*);
    virtual const char* SCPIQuery( SCPICmdType, char*);
    
    int SetI2CDevNode(char*);
    int SetI2CSlaveAdr(char*);
    int SetI2CMasterAdr(char*);
    int sDebugLevel(char*);
    
    static QStringList MeasChannelList; // liste aller messkanäle
    
private:
    // die routinen für das mmemory modell
    
    const char* mFile2Justdata( char*);	     	     
    const char* mJustData2File( char*);	     	     
    const char* mEEProm2JustData( char*);	     	     
    const char* mJustData2EEProm( char*);
    const char* mGetEEpromEnable();
    const char* mGetEEPrChksum();

    // die routinen für das system modell
    
    const char* mSetPPSSync(); // indirekt fürs system modell 
    const char* mGetText(hw_cmdcode); // dito

    const char* mSetSenseAbsDiff(char*);
    const char* mSetSyncPeriod(char*);
    const char* mGetSyncPeriod();
    const char* mSetSyncSource(char*);
    const char* mGetSyncSource();
    const char* mSetPSamples(char*);	
    const char* mGetPSamples();	
    const char* mSetSampleMode(char*);		
    const char* mGetSampleMode();
    const char* mSetSampleFrequency(char*);		
    const char* mGetSampleFrequency();
    const char* mSetSerialNumber(char*);
    const char* mGetSerialNumber();
    const char* mControlerMemUpdate(bl_cmdcode, char*);
    const char* mControlerFlashUpdate(char*);
    const char* mControlerEEpromUpdate(char*);
    const char* mControlerStartProgram(char*);
    const char* mControlerStartBootloader(char*);
    const char* mSetPCBVersion(char*);
    const char* mGetPCBVersion();
    const char* mGetCTRLVersion();
    const char* mGetLCAVersion();
    const char* mGetI2CMasterAdress();
    const char* mGetI2CSlaveAdress();				  
    const char* mGetI2CDeviceNode();
    const char* mSetDebugLevel( char*);
    const char* mGetDebugLevel();
    const char* mGetDeviceVersion();
    const char* mGetServerVersion();  
  
    // die routinen für das status modell
    
    const char* mGetAdjustmentStatus();
    const char* mGetDeviceStatus();	     
       
    // die routinen für das calculate model
    
   const char* mOutCValueCatalog();
   const char* mSetCalculateModel();
   const char* mGetRValue();
   const char* mGetRejection();
   const char* mGetCValue(char*); // abfrage des korrekturwertes (ev. mit parameter)
   const char* mSetStatus( char*);
   const char* mGetStatus();
   const char* mSetCValueCCoefficient( char* );
   const char* mGetCValueCCoefficient();
   const char* mGetCValueCCoefficientName();
   const char* mSetCValueCNode( char* );
   const char* mGetCValueCNode();
   const char* mGetCValueCNodeName();
   const char* mCmpCCoefficient(char*);
   
   // die routinen für das sense model
   
    const char* mOutRangeCatalog();
    const char* mGetRange();
    const char* mSetRange( char*);
    const char* mChannelClose( char*);
    const char* mChannelOpen( char*);
    const char* mOutChannelCatalog();
    const char* mGetStatOVL();
    const char* mResetStatOVL(char*);
    const char* mGetProtection();
    const char* mSetProtection(char*);
    
    bool Test4HWPresent();
    bool EEPromAccessEnable();
    bool isAtmelRunning();
    bool wait4AtmelRunning();
    bool programAtmelFlash();
    void AddChannelClient(QString&); // fügt hinzu/löscht einen client der open/close
    void DelChannelClient(QString&); //  auf einem kanal durchgeführt hat ( parameter ist der kanal) 
    
    sRange* SearchRange(QString&,QString&); // holt einen zeiger auf sRange abhängig v. kanal,range
    bool GetAdjInfo(QDomNode);

    // i2c hilfsroutinen
    
    void GenCommand(hw_cmd*); // generiert das kommando
    int I2CWriteCommand(hw_cmd*); // rückgabewert -1  wenn fehler sonst länge output
    int I2CReadOutput(quint8*,int); // rückgabewert -1  wenn fehler sonst anzahl gelesener zeichen
    
    void GenBootloaderCommand(bl_cmd*);
    int I2CBootloaderCommand(bl_cmd*);
    char* GenAdressPointerParameter(uchar adresspointerSize, ulong adr);
    
    bool ReadJustData();
    Q_UINT16 m_nChksumFlash;
    
    QStringList CValueList; // fürs dekodieren liste aller bereitgestellten korrekturwerte 
    QStringList CCoeffientList; // dito für die koeffizienten
    QStringList CNodeList; // und für die nodes
    tSockList SockList0,SockList1; // 2 socketlisten
    QStringList Ch0RangeList,Ch1RangeList; // 2 bereichlisten (eine/kanal)
    
    tChannelRangeArrayMap ChannelRangeArrayMap; // alle sRange* pro kanal
    tChannelListMap ChannelRangeListMap; // pro kanal eine liste mit bereich namen 
    tChannelListMap ChannelCValueListMap; // pro kanal eine liste mit korrekturwert "beschreibern"
    tChannelListMap ChannelCCoeffientListMap; // pro kanal eine liste mit korrekturkoeffizienten namen
    tChannelListMap ChannelCNodeListMap; // pro kanal eine liste mit stützstellen namen
    tChannelSockListMap ChannelSockListMap; // pro kanal eine liste der clients (sockets)  die ein open ausgeführt haben
    QString sI2CDevNode; 
    int I2CSlaveAdr;
    int I2CMasterAdr;
    int DebugLevel;
    int SyncSource; // 0 =extern 1=intern
    int SyncPeriod; // periode in ms 100...10000 
    int SMode; // 0..3 default 0
    int PSamples; // 80 od. 256 default 80 , führt zu Anpassung der adc clk
    QString sSerialNumber;
    QString sDeviceVersion; // version der hardware
    QDateTime DateTime; // datum,uhrzeit
    
    QString m_sFPGADeviceNode; // für den zugriff zur hardware (fpga register)

    QString Answer;
    int m_nJDataStat;
    double SampleFrequency;
};
#endif
