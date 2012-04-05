/////////////////////////////////////////////////////////////////////////////
// cIntelHexFileIO: Class handling Intel Hex Files

#include <qstring.h>
#include <q3memarray.h>
#include <q3valuelist.h>

struct THexFileMemRegion
{
    // data
    quint32 nStartAddress;
    QByteArray ByteArrContent;
    // methods
    THexFileMemRegion();
    THexFileMemRegion( int StartAddress, const QByteArray& byteArray);
    THexFileMemRegion& operator = (const THexFileMemRegion& obj);
    bool operator == (const THexFileMemRegion& obj) const;
    quint32 GetMaxAddress();
};
	

struct THexFileMessage 
{ 
    // data 
    ulong nLineNo; 
    QString sMsgText;				// either string 
    // methods 
    THexFileMessage(){}; 
    THexFileMessage(ulong _nLineNo, const QString& _sMsgText); 
};


typedef Q3ValueList<THexFileMemRegion> THexFileMemRegionList;
typedef Q3ValueList<THexFileMessage> THexFileMessageList;
	
	
class cIntelHexFileIO
{
public:
    cIntelHexFileIO();
    virtual ~cIntelHexFileIO(); 
    
    // File I/O
    bool ReadHexFile(const QString& fileName);
	
    
    // Memory Read iterated: In case more than one block meet address range: Stuff empty regions between     
    // with unwritten byte value. End is signalled by an empty array. If no block is found for the adress,  
    // the address is incremented to the next block found in steps of dwBlockLen to ensure to write one 
    // flash block only once. By adding dwStartAddressModuloBlockLen+dwOffsetToModulo the effective
    // start address of the memory block in byteArray is calculated.
    void GetMemoryBlock(const quint32& nBlockLen, quint32& nStartAddressModuloBlockLen, QByteArray& byteArray, quint32& nOffsetToModulo);

    
private:
    bool isHexText(const char c);
    uchar Hex2Bin(const char* c);
    uchar ASCII2Hex(char c);
    THexFileMemRegionList m_ListMemRegions;
    THexFileMessageList m_sListErrWarn;
    QString m_strFileName;
    uchar m_byteErasedByteValue;
};

