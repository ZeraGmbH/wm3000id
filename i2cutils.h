// header datei für i2cutils
// start 12.01.08 Peter Lohmer

#include <qstring.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#ifndef I2CUTILS_H
#define I2CTILS_H

int I2CTransfer( QString, int, int, i2c_rdwr_ioctl_data*);

#endif // I2CUTILS_H
