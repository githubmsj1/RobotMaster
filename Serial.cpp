#include "Serial.h"  
#include <TCHAR.H>  
#include <windows.h>  


HANDLE hCom;

/**
open serial
@param COMx: eg:_T("COM1")
@param BaudRate:
return 0 success ,return Negative is haed err
*/
int Serial_open(LPCWSTR COMx, int BaudRate)
{
	DCB dcb = { 0 };
	hCom = CreateFile(COMx,
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		0,//FILE_FLAG_OVERLAPPED,   //同步方式 或 重叠方式  
		0
		);

	if (hCom == INVALID_HANDLE_VALUE)
	{
		DWORD dwError = GetLastError();
		return -1;
	}

	dcb.DCBlength = sizeof(DCB);

	if (!GetCommState(hCom, &dcb))
	{
		DWORD dwError = GetLastError();
		return -1;
	}

	dcb.BaudRate = BaudRate;    //波特率  
	dcb.ByteSize = 8;           //位数  
	dcb.Parity = NOPARITY;      //奇偶检验  
	dcb.StopBits = ONESTOPBIT;   //停止位数  

	if (!SetCommState(hCom, &dcb))
	{
		DWORD dwError = GetLastError();
		return -1;
	}
	if (!PurgeComm(hCom, PURGE_RXCLEAR))    return -1;

	SetupComm(hCom, 1024, 1024);
	return 0;
}

/**
serial read
@param Buf:data buf
@param size:
@return The len of read
*/
int Serial_read(void *OutBuf, int size)
{
	DWORD cnt = 0;
	ReadFile(hCom, OutBuf, size, &cnt, 0);
	return cnt;
}

/**
serial write
@param Buf:data buf
@param size:bytes of Buf
@return The len of writen
*/
int Serial_write(const void *Buf, int size)
{
	DWORD dw;
	WriteFile(hCom, Buf, size, &dw, NULL);
	return dw;
}

/**
serial close
*/
void Serial_close(void)
{
	CloseHandle(hCom);
}

