// kommando knoten  für wm3000i hardware server
// es werden alle kommando knoten für zhserver nodes,
// scpi nodes, commands und , queries für den 
// bearbeitenden kommando interpreter definiert

#include <qstringlist.h>

#include "scpi.h" // für scpi knoten
#include "wmscpi.h"
#include "zhserver.h" // für zhserver knoten
#include "wm3000id.h" // der eigentliche server


extern QStringList cWM3000iServer::MeasChannelList; // liste aller messkanäle

// die vollständige kommando liste

cNodeSCPI* Sense;
                   cNodeSCPI* SenseChannel;
                                      cNodeSCPI* SenseChannelCatalog;
                   cNodeZHServer* SenseCName;
	                        cNodeSCPI* SenseCNameOpen;
	                        cNodeSCPI* SenseCNameClose;
    		          cNodeSCPI* SenseCNameRange;
			               cNodeSCPI* SenseCNameRangeCatalog;
	     cNodeSCPI* SenseStatus;		       
	                        cNodeSCPI* SenseStatusOverload; 
				 cNodeSCPI* SenseStatusOverloadReset;
	     cNodeSCPI* SenseProtect;			 
				 
cNodeSCPI* Calculate;
                    cNodeZHServer* CalculateCName;
			    cNodeSCPI* CalculateCNameCValue;
	                                                    cNodeSCPI* CalculateCNameCValueCatalog;
			    cNodeZHServer* CalculateCNameCValue2;
			                        cNodeSCPI* CalculateCNameCValue2CCoefficient;
			                        cNodeSCPI* CalculateCNameCValue2CNode;		    
			    cNodeZHServer* CalculateCNameRName;							          cNodeSCPI* CalculateCNameRNameRValue;
				          cNodeSCPI* CalculateCNameRNameRejection;      
				          cNodeZHServer* CalculateCNameRNameCValue;
				          cNodeSCPI* CalculateCNameRNameStatus;
				          cNodeZHServer* CalculateCNameRNameCCoefficient; // korrekturkoeffizienten
				          cNodeZHServer* CalculateCNameRNameCNode; // stützstellen
	      cNodeSCPI* CalculateCompute;		  
			    cNodeZHServer* CalculateComputeCValue;				  
					  
					  
					  
cNodeSCPI* Status;
	     cNodeSCPI* StatusDevice;	 
	     cNodeSCPI* StatusAdjustment;
	     
cNodeSCPI* System;
                   cNodeSCPI* SystemVersion;
		           cNodeSCPI* SystemVersionServer;
		           cNodeSCPI* SystemVersionDevice;	   
		           cNodeSCPI* SystemVersionPCB;
		           cNodeSCPI* SystemVersionCTRL;	   	   
		           cNodeSCPI* SystemVersionLCA;	   
	     cNodeSCPI* SystemDebug;	   
                   cNodeSCPI* SystemCommunication;
		          cNodeSCPI* SystemCommunicationDeviceNode;
		          cNodeSCPI* SystemCommunicationI2CMasterAdress;                             	        	          cNodeSCPI* SystemCommunicationI2CSlaveAdress;	
	     cNodeSCPI* SystemSerialNr;
	     cNodeSCPI* SystemSampling;
	                        cNodeSCPI* SystemSamplingFrequency; 
		          cNodeSCPI* SystemSamplingMode;		
		          cNodeSCPI* SystemSamplingPSamples;

         cNodeSCPI* SystemSense;
                    cNodeSCPI* SystemSenseMode;

                   cNodeSCPI* SystemSynchronisation;
	                        cNodeSCPI* SystemSynchronisationSource; 
		          cNodeSCPI* SystemSynchronisationPeriod;
	     cNodeSCPI* SystemUpdate;
	                        cNodeSCPI* SystemUpdateControler;
				cNodeSCPI* SystemUpdateControlerBootloader;
				cNodeSCPI* SystemUpdateControlerProgram;
				cNodeSCPI* SystemUpdateControlerFlash;
				cNodeSCPI* SystemUpdateControlerEEprom;
			  
			  
cNodeSCPI* MMemory;
                   cNodeSCPI* MMemoryRead;
                   cNodeSCPI* MMemoryWrite; 
	     cNodeSCPI* MMemoryImport;	   
	     cNodeSCPI* MMemoryExport;
	     cNodeSCPI* MMemoryEnable;
	     cNodeSCPI* MMemoryChksum;
	     
// cNodeScpi (QString,tNodeSpec,cNode*,cNode*,SCPICmdType,SCPICmdType); 
// konstruktor, sNodeName, nNodedef, pNextNode, pNewLevelNode, Cmd, Query				
// konstruktor, psNodeNames,psNode2Set, nNodedef, pNextNode, pNewLevelNode, Cmd, Query
//cNodeZHServer::cNodeZHServer(QStringList* sl,QString* s,tNodeSpec ns,cNode* n1,cNode* n2,SCPICmdType,SCPICmdType)
	       
cNode* InitCmdTree() {
    
    // implementiertes mmemory modell

    MMemoryChksum=new cNodeSCPI("CHKSUM",isQuery,NULL,NULL,nixCmd,GetEEPromChksum);
    MMemoryEnable=new cNodeSCPI("ENABLE",isQuery,MMemoryChksum,NULL,nixCmd,GetEEPromEnable);
    MMemoryExport=new cNodeSCPI("EXPORT",isCommand,MMemoryEnable,NULL,JustData2File,nixCmd);	     	     
    MMemoryImport=new cNodeSCPI("IMPORT",isCommand,MMemoryExport,NULL,File2Justdata,nixCmd);	     	     
    MMemoryWrite=new cNodeSCPI("WRITE",isCommand,MMemoryImport,NULL,JustData2EEProm,nixCmd);	     
    MMemoryRead=new cNodeSCPI("READ",isCommand,MMemoryWrite,NULL,EEProm2JustData,nixCmd);	     	     
    MMemory=new cNodeSCPI("MMEMORY",isNode,NULL,MMemoryRead,nixCmd,nixCmd);	     
    
    // implementiertes system modell
    SystemSenseMode=new cNodeSCPI("MODE",isCommand,NULL,NULL,SetSenseAbsDiff,nixCmd);
    SystemSense=new cNodeSCPI("SENSE",isNode,NULL,SystemSenseMode,nixCmd,nixCmd);

    SystemSynchronisationPeriod=new cNodeSCPI("PERIOD",isQuery | isCommand,NULL,NULL,SetSyncPeriod,GetSyncPeriod);
    SystemSynchronisationSource=new cNodeSCPI("SOURCE",isQuery | isCommand,SystemSynchronisationPeriod,NULL,SetSyncSource,GetSyncSource);
    SystemSynchronisation=new cNodeSCPI("SYNCHRONISATION",isNode,SystemSense,SystemSynchronisationSource,nixCmd,nixCmd);
    SystemSamplingPSamples=new cNodeSCPI("PSAMPLES",isQuery | isCommand,NULL,NULL,SetPSamples,GetPSamples);
    SystemSamplingMode=new cNodeSCPI("MODE",isQuery | isCommand,SystemSamplingPSamples,NULL,SetSampleMode,GetSampleMode);
    SystemSamplingFrequency=new cNodeSCPI("FREQUENCY",isQuery | isCommand,SystemSamplingMode,NULL,SetSampleFrequency,GetSampleFrequency);
    SystemSampling=new cNodeSCPI("SAMPLING",isNode,SystemSynchronisation,SystemSamplingFrequency,nixCmd,nixCmd);
    SystemUpdateControlerEEprom=new cNodeSCPI("EEPROM",isCommand,NULL,NULL,ControlerEEpromUpdate,nixCmd);
    SystemUpdateControlerFlash=new cNodeSCPI("FLASH",isCommand,SystemUpdateControlerEEprom,NULL,ControlerFlashUpdate,nixCmd);
    SystemUpdateControlerProgram=new cNodeSCPI("PROGRAM",isCommand,SystemUpdateControlerFlash,NULL,ControlerStartProgram,nixCmd);
    SystemUpdateControlerBootloader=new cNodeSCPI("BOOTLOADER",isCommand,SystemUpdateControlerProgram,NULL,ControlerStartBootloader,nixCmd);
    SystemUpdateControler=new cNodeSCPI("CONTROLER",isNode,NULL,SystemUpdateControlerBootloader,nixCmd,nixCmd);
    SystemUpdate=new cNodeSCPI("UPDATE",isNode,SystemSampling,SystemUpdateControler,nixCmd,nixCmd);
    
    SystemSerialNr=new cNodeSCPI("SERNR",isQuery | isCommand,SystemUpdate,NULL,SetSerialNumber,GetSerialNumber);
    SystemCommunicationI2CMasterAdress=new cNodeSCPI("I2CMASTER",isQuery,NULL,NULL,nixCmd,GetI2CMasterAdress);
    SystemCommunicationI2CSlaveAdress=new cNodeSCPI("I2CSLAVE",isQuery,SystemCommunicationI2CMasterAdress,NULL,nixCmd,GetI2CSlaveAdress);				  
    SystemCommunicationDeviceNode=new cNodeSCPI("DEVNODE",isQuery,SystemCommunicationI2CSlaveAdress,NULL,nixCmd,GetI2CDeviceNode);
    SystemCommunication=new cNodeSCPI("COMMUNICATION",isNode,SystemSerialNr,SystemCommunicationDeviceNode,nixCmd,nixCmd);
    SystemDebug=new cNodeSCPI("DEBUG",isQuery | isCommand,SystemCommunication,NULL,SetDebugLevel,GetDebugLevel);
    SystemVersionLCA=new cNodeSCPI("LCA",isQuery,NULL,NULL,nixCmd,GetLCAVersion);
    SystemVersionCTRL=new cNodeSCPI("CTRL",isQuery,SystemVersionLCA,NULL,nixCmd,GetCTRLVersion);
    SystemVersionPCB=new cNodeSCPI("PCB",isQuery | isCommand,SystemVersionCTRL,NULL,SetPCBVersion,GetPCBVersion);
    SystemVersionDevice=new cNodeSCPI("DEVICE",isQuery,SystemVersionPCB,NULL,nixCmd,GetDeviceVersion);
    SystemVersionServer=new cNodeSCPI("SERVER",isQuery,SystemVersionDevice,NULL,nixCmd,GetServerVersion);  
    SystemVersion=new cNodeSCPI("VERSION",isNode,SystemDebug,SystemVersionServer,nixCmd,nixCmd);	         
    System=new cNodeSCPI("SYSTEM",isNode,MMemory,SystemVersion,nixCmd,nixCmd);	     
    
    // implementiertes status modell
	     
    StatusAdjustment=new cNodeSCPI("ADJUSTMENT",isQuery,NULL,NULL,nixCmd,GetAdjustmentStatus);
    StatusDevice=new cNodeSCPI("DEVICE",isQuery,StatusAdjustment,NULL,nixCmd,GetDeviceStatus);
    Status=new cNodeSCPI("STATUS",isNode,System,StatusDevice,nixCmd,nixCmd);	     
    
    // implementiertes calculate model
    
    CalculateComputeCValue=new cNodeZHServer(NULL,isCommand,NULL,NULL,CmpCCoefficient,nixCmd);
    // die QStringList in CalculateComputeCValue wird später in abhängigkeit von CalculateCName gesetzt
    CalculateCompute=new cNodeSCPI("COMPUTE",isNode,NULL,CalculateComputeCValue,nixCmd,nixCmd);
    CalculateCNameRNameCNode=new cNodeZHServer(NULL,isQuery | isCommand,NULL,NULL,SetCValueCNode,GetCValueCNode);
    // die QStringList in CalculateCNameRNameCNode wird später in abhängigkeit von CalculateCName gesetzt
    CalculateCNameRNameCCoefficient=new cNodeZHServer(NULL,isQuery | isCommand,CalculateCNameRNameCNode,NULL,SetCValueCCoefficient,GetCValueCCoefficient);
    // die QStringList in CalculateCNameRNameCCoefficient wird später in abhängigkeit von CalculateCName gesetzt
    CalculateCNameRNameStatus=new cNodeSCPI("STATUS",isQuery | isCommand,CalculateCNameRNameCCoefficient,NULL,SetStatus,GetStatus);
    CalculateCNameRNameCValue=new cNodeZHServer(NULL,isQuery | isQuerywPar,CalculateCNameRNameStatus,NULL,nixCmd,GetCValue);
    // die QStringList in CalculateCNameRNameCValue wird später in abhängigkeit von CalculateCName gesetzt
    CalculateCNameRNameRejection=new cNodeSCPI("REJECTION",isQuery,CalculateCNameRNameCValue,NULL,nixCmd,GetRejection);
    CalculateCNameRNameRValue=new cNodeSCPI("RVALUE",isQuery,CalculateCNameRNameRejection,NULL,nixCmd,GetRValue);
    CalculateCNameRName=new cNodeZHServer(NULL,isNode,CalculateCompute,CalculateCNameRNameRValue,nixCmd,nixCmd);
    // die QStringList in CalculateCNameRName wird später in abhängigkeit von CalculateCName gesetzt
    CalculateCNameCValue2CNode=new cNodeSCPI("CNODE",isQuery,NULL,NULL,nixCmd,GetCValueCNodeName);
    CalculateCNameCValue2CCoefficient=new cNodeSCPI("CCOEFFICIENT",isQuery,CalculateCNameCValue2CNode,NULL,nixCmd,GetCValueCCoefficientName);
    CalculateCNameCValue2 = new cNodeZHServer(NULL,isNode,CalculateCNameRName,CalculateCNameCValue2CCoefficient,nixCmd,nixCmd);
    // die QStringList in CalculateCNameCValue2 wird später in abhängigkeit von CalculateCName gesetzt
    CalculateCNameCValueCatalog=new cNodeSCPI("CATALOG",isQuery,NULL,NULL,nixCmd,OutCValueCatalog);
    CalculateCNameCValue=new cNodeSCPI("CVALUE",isNode,CalculateCNameCValue2,CalculateCNameCValueCatalog,nixCmd,nixCmd);
    CalculateCName=new cNodeZHServer(&cWM3000iServer::MeasChannelList,isNode,NULL,CalculateCNameCValue,nixCmd,SetCalculateModel);
    // wenn CalculateCName als gültiger knoten dekodiert wurde -> muss CalculateCNameRName, CalculateCNameCValue2, CalculateCNameRNameCValue,CalculateCNameRNameCCoeffient  und CalculateCNameRNameCNode entsprechend angepasst werden    
    Calculate=new cNodeSCPI("CALCULATE",isNode,Status,CalculateCName,nixCmd,nixCmd);
   
    // implementiertes sense model    
    
    SenseProtect=new cNodeSCPI("PROTECTION",isCommand | isQuery,NULL,NULL,SetProtection,GetProtection);
    SenseStatusOverloadReset=new cNodeSCPI("RESET",isCommand,NULL,NULL,ResetStatOVL,nixCmd);				 
    SenseStatusOverload=new cNodeSCPI("OVERLOAD",isNode | isQuery,NULL,SenseStatusOverloadReset,nixCmd,GetStatOVL);
    SenseStatus=new cNodeSCPI("STATUS",isNode,SenseProtect,SenseStatusOverload,nixCmd,nixCmd);        SenseCNameRangeCatalog=new cNodeSCPI("CATALOG",isQuery,NULL,NULL,nixCmd,OutRangeCatalog);
    SenseCNameRange=new cNodeSCPI("RANGE",isNode | isCommand | isQuery,NULL,SenseCNameRangeCatalog,SetRange,GetRange);
    SenseCNameClose=new cNodeSCPI("CLOSE",isCommand,SenseCNameRange,NULL,ChannelClose,nixCmd);
    SenseCNameOpen=new cNodeSCPI("OPEN",isCommand,SenseCNameClose,NULL,ChannelOpen,nixCmd);
    SenseCName=new cNodeZHServer(&cWM3000iServer::MeasChannelList,isNode,SenseStatus,SenseCNameOpen,nixCmd,nixCmd);
    SenseChannelCatalog=new cNodeSCPI("CATALOG",isQuery,NULL,NULL,nixCmd,OutChannelCatalog);
    SenseChannel=new cNodeSCPI("CHANNEL",isNode,SenseCName,SenseChannelCatalog,nixCmd,nixCmd);
    Sense=new cNodeSCPI("SENSE",isNode,Calculate,SenseChannel,nixCmd,nixCmd);
    return (Sense);
}
