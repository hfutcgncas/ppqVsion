#include"GlobleSrouce.h"
#include"math.h"
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

//=================================================================================================


bool cBallModel::predict_OneStep(BallPoint In, BallPoint* pOut, double dt)
{
	//�������е�λΪ m ��m/s ��s
	double V;
	double a[3];
	double C1 = 0.1500, C2 = 0.0060, g = 9.802;

	V = sqrt(In.Vx * In.Vx + In.Vy * In.Vy + In.Vz * In.Vz);
	a[0] = -C1*V*In.Vx + C2*In.Wy*In.Vz - C2*In.Wz*In.Vy;
	a[1] = -C1*V*In.Vy + C2*In.Wz*In.Vx - C2*In.Wx*In.Vz;
	a[2] = -C1*V*In.Vz + C2*In.Wx*In.Vy - C2*In.Wy*In.Vx - g;


	pOut->x = In.x + In.Vx * dt;
	pOut->y = In.y + In.Vy * dt;
	pOut->z = In.z + In.Vz * dt;

	pOut->Vx = In.Vx + a[0] * dt;
	pOut->Vy = In.Vy + a[1] * dt;
	pOut->Vz = In.Vz + a[2] * dt;

	pOut->Wx = In.Wx;
	pOut->Wy = In.Wy;
	pOut->Wz = In.Wz;

	pOut->t = In.t + dt;

	return true;
}
bool cBallModel::predict_Rebound(BallPoint In, BallPoint* pOut)
{
	double r = 0.02;
	double u = 0.25, et = 0.93;  // ��Ҫ�ҵ����ʵ�Ħ��ϵ��0.25������ϵ��
	double vbt1 = In.x - r*In.Wy;
	double vbt2 = In.y + r*In.Wx;
	double vs = 1 - 2.5*u*(1 + et)*signFun(In.Vz) / sqrt(vbt1*vbt1 + vbt2*vbt2);
	double aa = u*(1 + et)*signFun(In.Vz) / sqrt(vbt1*vbt1 + vbt2*vbt2);
	if (vs <= 0)
	{
		aa = 2 / 5.0;
	}

	pOut->Vx = (1 - aa)*(In.Vx) + aa*r*In.Wy;
	pOut->Vy = (1 - aa)*(In.Vy) - aa*r*In.Wx;
	pOut->Vz = -et*(In.z);

	pOut->Wx = -3.0*aa / 2.0 / r*(In.Vy) + (1 - 3 * aa / 2)*In.Wx;
	pOut->Wy = 3.0*aa / 2.0 / r*(In.Vx) + (1 - 3 * aa / 2)*In.Wy;
	pOut->Wz = In.Wz;

	pOut->x = In.x;
	pOut->y = In.y;
	pOut->z = In.z;

	pOut->t = In.t;
	return true;
}
int cBallModel::predict(BallPoint startPoint, BallPoint* pendPoint, double stopPlane)
//return 1 : ����
//return -1 : ����
//return 2 : δ����
//return 3 : ��ʱ
{
	int count = 0;
	double dt = 0.0005;
	BallPoint temp1 = startPoint;
	BallPoint temp2;


	if (startPoint.Vx < 0 )
	{
		return -1;//����Ԥ��
	}
	//����ǰ
	while (temp1.z > 0.05)
	{
		predict_OneStep(temp1, pendPoint, dt);
		temp1 = *pendPoint;
		count++;

		if (temp1.x > stopPlane) return 2;  //δ����
		if( count > 0.5 / dt) return 3; //��ʱ
	}
	//����
	predict_Rebound(temp1, pendPoint);
	//������
	while (temp1.x < stopPlane)
	{
		predict_OneStep(temp1, pendPoint, dt);
		temp1 = *pendPoint;
		count++;

		if (count > 0.5 / dt)
			return 3; //��ʱ
	}

	return 1; //����
}
