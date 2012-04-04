// header datei für crcutils
// start 12.01.08 Peter Lohmer

#include <QtGlobal>

#ifndef CRCUTILS_H
#define CRCTILS_H

char CalcByteCRC(char,char);
char CalcBlockCRC(quint8*,int);

#endif // CRCUTILS_H
