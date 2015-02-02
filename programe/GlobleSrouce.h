#ifndef _GLOBLESROUCE_H_
#define _GLOBLESROUCE_H_

#include "time.h" // time
#include "stdio.h"

#include <io.h> //_access
#include"string.h"
#include "direct.h" // mkdir
#include "time.h" // time

#include "conio.h"//键盘

#include <Winsock2.h>
#pragma comment(lib, "ws2_32.lib")

//时钟================================
class cClock
{
public:
	LARGE_INTEGER litmp;
	LONGLONG timeSysStart,timeSysNow;
	double dfFreq,dfMinus; 

	cClock();
	double now();
	void reset();
};

//记录文件===========================
/*
class DataFile
{
public:
	char fileName[100];
	FILE *fp88;
	bool bFileCreate88 = false;
	int fileNameIndex88=0;
	bool m_bStartSave=false;
	char DirName[100];

};*/

//shell===========================
class cShell
{
public:
	char key;

	char getCommand();
};





class cFile
{
	char	fileName[100];
	FILE	*fp;
	bool	bFileCreate  ;
	int		fileNameIndex;
	char	DirName[100];
public:
	cFile();

	bool m_bStartSave;
	bool FileInit();
	void SaveTraceData(double x,double y,double z,double t,double vx,double vy,double vz,int flag);
};

#endif