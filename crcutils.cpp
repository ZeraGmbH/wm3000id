// implementierung crcutils

char CalcByteCRC(char crc, char data) {
    crc = crc ^ data;
    for (char i = 0; i < 8; i++) {
	if (crc & 0x01)
	    crc = (crc >> 1) ^ 0x8c;
	else
	    crc >>= 1;
    }
    return crc;
}


char CalcBlockCRC(char* pBuff, int iBuffLen) {
    char crc = 0;
    for (int i = 0; i < iBuffLen; i++)
	crc = CalcByteCRC(crc, pBuff[i]);
    return crc;
}
		
