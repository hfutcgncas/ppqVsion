#include"GlobleSrouce.h"

//globle ver--------------------
cFile  dataFile;
cClock SystemClock;
//-----------------------------------------



cClock::cClock()
{
	QueryPerformanceFrequency(&litmp);
	dfFreq = (double)litmp.QuadPart;// 获得计数器的时钟频率
	QueryPerformanceCounter(&litmp);
	timeSysStart = litmp.QuadPart;
}

void cClock::reset()
{
	QueryPerformanceCounter(&litmp);
	timeSysStart = litmp.QuadPart;
}

double cClock::now()//单位为ms
{
	double nowRead;
	QueryPerformanceCounter(&litmp);
	timeSysNow = litmp.QuadPart; 
	timeSysNow -= timeSysStart;
	if(timeSysNow<0)timeSysNow += 0x7FFFFFFF;  //LONGLONG 为64位整形，故如此进位
	nowRead    = timeSysNow/dfFreq*1000.0;
	if(nowRead>1000000)
	{
		nowRead -= 1000000;
		this->reset();
	}
	return nowRead;
}

//==============================================

char cShell::getCommand()
{
	key = '0';
	if (_kbhit())
	{//有按键
		key = getch();
		switch(key)
		{
		case 'q':
			dataFile.SaveTraceData(0,0,0,0,0,0,0,1);
			printf("\n退出程序.\n");
			break;
		case 's':
			//保存文件,按一次S，保存一次，并清空数据区
			if (!dataFile.m_bStartSave)
			{
				dataFile.m_bStartSave = true;
				printf("Begin to Save File.\n");
			}
			else
			{
				dataFile.m_bStartSave = false;
				dataFile.SaveTraceData(0,0,0,0,0,0,0,1);//关闭文件
				printf("Stop Save File.\n");
			}
			break;
		default:
			printf("\n命令错误.\n");
		}
	}
	return key;

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cFile::cFile()
{
	m_bStartSave  = false ;
	bFileCreate   = false ;
	fileNameIndex = 0     ;
}

//===============================================================
void cFile::SaveTraceData(double x,double y,double z,double t,double vx,double vy,double vz,int flag)
{//保存数据
	//flag = 88:保存88的数据 98:保存98的数据,其他：关闭文件
	if (flag==1)
	{//关闭
		if (bFileCreate)
		{
			fclose(fp);
			bFileCreate = false;
			char data[100];
			sprintf(data,"%d",++fileNameIndex);
			WritePrivateProfileString("SaveFilePara","FileIndex",data, "ballData\\sys.ini");
		}
		return;
	}
	if (flag==88)
	{
		if (bFileCreate==false)
		{
			//读取ini文件
			char iniData[100];
			GetPrivateProfileString("SaveFilePara","FileIndex","0",iniData,100,"ballData\\sys.ini");
			fileNameIndex = atoi(iniData);		
			sprintf(fileName,"%s\\ballData%d.txt",DirName,fileNameIndex);
			if((fp=fopen(fileName,"at"))==NULL)
			{//创建
				printf("\nCreate File Failed!\n");
				return;
			}
			bFileCreate = true;
		}
		else
		{//写入
			fprintf(fp,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", x,y,z,t,vx,vy,vz);
		}
	}
}
//======================================
bool cFile::FileInit()
{
	struct tm *localTime;
	time_t ti;
	char strTime[100];
	strTime[0]='0';
	ti=time(NULL); 
	localTime=localtime(&ti);	//获取日期 
	
	if(_access("ballData",0)==-1)
	{//文件夹不存在
		mkdir("ballData");
	}
	
	sprintf(DirName,"ballData\\球轨迹数据_%04d_%02d_%02d",
		localTime->tm_year+1900,
		localTime->tm_mon+1,
		localTime->tm_mday);
	if (_access(DirName,0)==-1)
	{
		mkdir(DirName); 
		DeleteFile("ballData\\sys.ini");
		//新建文件夹
	}
	return 1;
}