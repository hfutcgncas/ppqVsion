#include"GlobleSrouce.h"

//globle ver--------------------
cFile  dataFile;
cClock SystemClock;
//-----------------------------------------



cClock::cClock()
{
	QueryPerformanceFrequency(&litmp);
	dfFreq = (double)litmp.QuadPart;// ��ü�������ʱ��Ƶ��
	QueryPerformanceCounter(&litmp);
	timeSysStart = litmp.QuadPart;
}

void cClock::reset()
{
	QueryPerformanceCounter(&litmp);
	timeSysStart = litmp.QuadPart;
}

double cClock::now()//��λΪms
{
	double nowRead;
	QueryPerformanceCounter(&litmp);
	timeSysNow = litmp.QuadPart; 
	timeSysNow -= timeSysStart;
	if(timeSysNow<0)timeSysNow += 0x7FFFFFFF;  //LONGLONG Ϊ64λ���Σ�����˽�λ
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
	{//�а���
		key = getch();
		switch(key)
		{
		case 'q':
			dataFile.SaveTraceData(0,0,0,0,0,0,0,1);
			printf("\n�˳�����.\n");
			break;
		case 's':
			//�����ļ�,��һ��S������һ�Σ������������
			if (!dataFile.m_bStartSave)
			{
				dataFile.m_bStartSave = true;
				printf("Begin to Save File.\n");
			}
			else
			{
				dataFile.m_bStartSave = false;
				dataFile.SaveTraceData(0,0,0,0,0,0,0,1);//�ر��ļ�
				printf("Stop Save File.\n");
			}
			break;
		default:
			printf("\n�������.\n");
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
{//��������
	//flag = 88:����88������ 98:����98������,�������ر��ļ�
	if (flag==1)
	{//�ر�
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
			//��ȡini�ļ�
			char iniData[100];
			GetPrivateProfileString("SaveFilePara","FileIndex","0",iniData,100,"ballData\\sys.ini");
			fileNameIndex = atoi(iniData);		
			sprintf(fileName,"%s\\ballData%d.txt",DirName,fileNameIndex);
			if((fp=fopen(fileName,"at"))==NULL)
			{//����
				printf("\nCreate File Failed!\n");
				return;
			}
			bFileCreate = true;
		}
		else
		{//д��
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
	localTime=localtime(&ti);	//��ȡ���� 
	
	if(_access("ballData",0)==-1)
	{//�ļ��в�����
		mkdir("ballData");
	}
	
	sprintf(DirName,"ballData\\��켣����_%04d_%02d_%02d",
		localTime->tm_year+1900,
		localTime->tm_mon+1,
		localTime->tm_mday);
	if (_access(DirName,0)==-1)
	{
		mkdir(DirName); 
		DeleteFile("ballData\\sys.ini");
		//�½��ļ���
	}
	return 1;
}