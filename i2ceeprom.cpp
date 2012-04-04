// i2c eeprom implementation

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <qstring.h>
#include "i2cutils.h"
#include "i2ceeprom.h"
#include <QtGlobal>

cI2CEEProm::cI2CEEProm(QString dn, int dl, short adr) {
    DevNode = dn;
    DebugLevel = dl;
    I2CAdress = adr;
}


c24LC256::c24LC256(QString dn, int dl, short adr)
    : cI2CEEProm(dn,dl,adr) {
}


int c24LC256::WriteData(char* data,int n,int adr) {
    quint8 outpBuf[66]; // 2 adr byte, max 64 byte daten
    struct i2c_msg Msgs = {addr: I2CAdress, flags: I2C_M_RD, len: 5, buf:  outpBuf }; // 1 message
    struct i2c_rdwr_ioctl_data EEPromData = { msgs: &(Msgs), nmsgs: 1 };
    int toWrite=n;
    
    if ( I2CTransfer(DevNode,I2CAdress,DebugLevel,&EEPromData) )
	return 0; // konnte nichts schreiben 
    
    Msgs.flags = 0; // umschalten auf schreiben 
    char* mydata = data;
    while (toWrite) {
	if (adr > 32767 ) break; // dann sind wir fertig 
	outpBuf[0] = (adr >> 8) & 0xff; outpBuf[1] = adr & 0xff;
	int pl = 64 - (adr & 0x3f); // was noch in eine memory seite passt
	int l = (toWrite > pl) ? pl : toWrite; // was jetzt geschrieben werden soll 
	memcpy((void*)&outpBuf[2],(void*)mydata,l);
	mydata+=l;
	Msgs.len = l+2; // das ist die länge für den i2c treiber
	int r;
	while (( r = I2CTransfer(DevNode,I2CAdress,DebugLevel,&EEPromData)) == 2); // device node ok , aber eeprom ist busy
	if (r) break; // probleme mit device node
	adr+=l; // nächste adresse
	toWrite-=l; // was noch zu senden ist
    }
    return(n-toWrite);
}


int c24LC256::ReadData(char* data,int n,int adr) {
    quint8 outpBuf[2];
    quint8 inpBuf[n];
    struct i2c_msg Msgs[2] = { {addr :I2CAdress, flags: 0,len: 2,buf: &(outpBuf[0])}, // 2 messages (tagged format )
			  {addr :I2CAdress, flags: (I2C_M_RD+I2C_M_NOSTART), len: n, buf: &(inpBuf[0])} };    
     
    struct i2c_rdwr_ioctl_data EEPromData = {  msgs: &(Msgs[0]), nmsgs: 2 };
 
    outpBuf[0]=(adr >> 8) & 0xff; outpBuf[1]=adr & 0xff;
    
    if ( I2CTransfer(DevNode,I2CAdress,DebugLevel,&EEPromData) == 0 ) { // alles ok und gelesen
	memcpy((void*)data,(void*)&inpBuf[0],n);
	return(n);
    }
    else return(0);
}

int c24LC256::size() {
    return 32768;
}

