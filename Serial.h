#ifndef SERIAL_H
#define SERIAL_H
#include <windows.h>  

int Serial_open(LPCWSTR COMx, int BaudRate);
int Serial_read(void *OutBuf, int size);
int Serial_write(const void *Buf, int size);
void Serial_close(void);


#endif


