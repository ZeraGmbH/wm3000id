// implementierung i2cutils

#include <syslog.h>
#include <fcntl.h>
#include <unistd.h>
#include "i2cutils.h"

int I2CTransfer(QString devn,int i2cadr, int dl, i2c_rdwr_ioctl_data* iod) {
    int fd;
    if ( (fd = open(devn.latin1(),O_RDWR)) < 0 ) {
	if (dl & 1)  syslog(LOG_ERR,"error opening i2c device: %s\n",devn.latin1());
	return(1); // error connection	    
    }
    if ( ioctl(fd,I2C_RETRIES,0) < 0 ) {
    	if (dl & 1)  syslog(LOG_ERR,"error setting retries of i2c device: %s\n",devn.latin1());
	return(1); // error connection	    
    }
    if ( ioctl(fd,I2C_TIMEOUT,500) < 0 ) {
    	if (dl & 1)  syslog(LOG_ERR,"error setting retries of i2c device: %s\n",devn.latin1());
	return(1); // error connection	    
    }
    if ( ioctl(fd,I2C_RDWR,iod) < 0 ) {
	if (dl & 1)  syslog(LOG_ERR,"error read/write i2c slave at adress: %d\n",i2cadr);
	close(fd);
	return(2); // error device
    }
    if (dl & 2)  syslog(LOG_INFO,"read/write i2c slave at adress: %d done\n",i2cadr);
    close(fd);
    return(0); // acknowledge 	    
}


