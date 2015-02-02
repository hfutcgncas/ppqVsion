#include "stdio.h"
#include <iostream>
#include "camer.h"
#include "Cam3D.h"

#include <io.h> //_access
#include"string.h"
#include "direct.h" // mkdir
#include "time.h" // time

#include "conio.h"

#include <vector>
#include "cRobotArm.h"
#include "GlobleSrouce.h"

#include "time.h" // time
#include <Winsock2.h>



#pragma comment(lib, "ws2_32.lib")

using namespace std;

#define		CamA_IP		"192.168.64.88"
#define		CamA_PORT		2002
#define		CamB_IP		"192.168.64.89"
#define		CamB_PORT		2002



//globle ver--------------------
extern cClock SystemClock;
extern cFile  dataFile;
//------------------------------

bool CheckBallStatueChange(sBallStatue BallStatue,sBallStatue BallStatue_last);
bool CheckTraceChange(double x_out,double y_out,double z_out,double vx_out,double vy_out,double vz_out,double t_out);



//-----------------------------------------
static cHis3DTemp ballPosHis,ballDoubtPosHis,ballPredictHis;
static sBallStatue BallStatue,BallStatue_Last,BallStatue_doubt_Last;
static int DoultCount;

static int PreCount;



//==========================
//==========================
void main()
{
	double t_o,t_c;
	cRobotArm	robotArm;
	cCam3D      robotVision;
	cShell      shell;
	double ballHit[7];

	int Predicted = 0;
	
	ballPosHis.clear();
//��ʼ�����ʱ----------------------
//--------------------------------------------
	dataFile.FileInit();									//���ݴ洢�ĳ�ʼ��
	robotVision.Init(CamA_IP,CamA_PORT,CamB_IP,CamB_PORT);	//���������ĳ�ʼ��
	robotArm.ComInit();										//���˶�������ͨ�ŵĳ�ʼ��
	InitMatrax();//////////

	printline(30);
    //-----------------------------------
	t_o = SystemClock.now();
	while(shell.getCommand()!='q') //��ѭ��
	{
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	/*	while (SystemClock.now() - t_o < 5)
		{
		}*/
		//t_o = SystemClock.now();  //ʱ�Ӷ��ڸ�λ
		double t_out,x_out,y_out,z_out,vx_out,vy_out,vz_out;
		t_out = SystemClock.now()/1000.0;
		if (t_out > 1000)
		{
			robotVision.Sync();
			continue;
		}	
		char rt = robotVision.RefreshSrcData();
		if( rt!= 0  && SystemClock.now() - t_o > 5 ) //
		{
			t_o = SystemClock.now();
			robotVision.ReConstruction(rt,&t_out,&x_out,&y_out,&z_out,&vx_out,&vy_out,&vz_out);//��ά�ؽ�

			if (rt == 2)
			{
				x_out += 700;
			}

			if ( dataFile.m_bStartSave )   //�洢ƹ����λ��
			{
				dataFile.SaveTraceData( x_out  , y_out , z_out  , t_out ,vx_out,vy_out,rt,88);
			} 

			//if (CheckTraceChange(x_out,y_out,z_out,vx_out,vy_out,vz_out,t_out))//�������¹켣
			//{
			//	cout<<"Prepare to Predict"<<endl;
			//	Predicted = 0;	
			//	ballPredictHis.clear();
			//}
			//if ( Predicted == 0 )
			//{
			//	ballPredictHis.push_back(x_out,y_out,z_out,vx_out,vy_out,vz_out,t_out);
			//	if ( ballPredictHis.size() >= 5 )
			//	{
			//		
			//		Predict_traceV2(ballPredictHis,ballHit);
			//		Predicted = 1;

			//		robotArm.HitBall(ballHit[0],ballHit[1],ballHit[2],ballHit[3],ballHit[4],ballHit[5],ballHit[6]*1000);

			//		for (int i = 0;i<7;i++)
			//		{
			//			cout<<ballHit[i]<<" ";
			//		}
			//		cout<<endl;
			//		printline(30);
			//		ballPredictHis.clear();
			//	}	
			//}
	    }
		//---------------------------------------------
	}
	
	return;

}
//-------------------------------------------------------------------
//
//
//
//
//
//
//-------------------------------------------------------------------
//==================================
bool CheckBallStatueChange(sBallStatue BallStatue,sBallStatue BallStatue_last)
{
	if(BallStatue.direction != BallStatue_last.direction)return 1;//����ı�}
	if(BallStatue.direction == 1 && (BallStatue.x_last < BallStatue_last.x_last))return 1;//������켣λ�ò���
	if(BallStatue.direction == 0 && (BallStatue.x_last > BallStatue_last.x_last))return 1;//������켣λ�ò���
	if(   BallStatue.t_last - BallStatue_last.t_last > 1 
		||BallStatue.t_last - BallStatue_last.t_last < 0 )return 1;//ʱ��������1s
	return 0;
}
//==================================
bool CheckTraceChange(double x_out,double y_out,double z_out,double vx_out,double vy_out,double vz_out,double t_out)
{
	BallStatue.x_last = x_out;  //BallStatue��ʾ��ǰ״̬
	BallStatue.t_last = t_out;
	BallStatue.direction = (vx_out>0);

	if(!CheckBallStatueChange(BallStatue,BallStatue_Last)) //��ƹ����켣δ�䣬����BallStatue_Last
	{
		ballDoubtPosHis.clear();

		BallStatue_Last.x_last = x_out;
		BallStatue_Last.t_last = t_out;
		BallStatue_Last.direction = (vx_out>0);


		if (dataFile.m_bStartSave && vx_out > 0 )   //�洢ƹ����λ��
		{
			dataFile.SaveTraceData( x_out  , y_out , z_out  , t_out ,vx_out,vy_out,vz_out,88);
		}  

		if( ballPosHis.size()< 5  )  //����ʷ��¼����С��5 ��׷����ʷ����
		{
			ballPosHis.push_back(x_out,y_out,z_out,vx_out,vy_out,vz_out,t_out);
		}
		else if(  BallStatue_Last.predicted == 0) //�������㹻�����л���׼��Ԥ���״̬
		{
			BallStatue_Last.predicted = 1;
			cout<<"New trace "<<PreCount++<<endl;

			if( BallStatue_Last.direction == 1 )
			{
				return true;
			}
		}
	}
	else //���б仯�����¼�����仯�������������仯��4��ʱ������ballPosHis�����ballDoubtPosHis��������BallStatue_Last
	{
		if( ballDoubtPosHis.size() <= 4 )
		{
			if( !CheckBallStatueChange(BallStatue,BallStatue_doubt_Last) ) //����ǰ�켣����ɹ켣״̬һ�£���׷�ӿ��ɹ켣����
			{
				ballDoubtPosHis.push_back(x_out,y_out,z_out,vx_out,vy_out,vz_out,t_out);
				BallStatue_doubt_Last.x_last = x_out;
				BallStatue_doubt_Last.t_last = t_out;
				BallStatue_doubt_Last.direction = (vx_out>0);
			}
			else //����ǰ�켣����ɹ켣״̬��һ�£������֮ǰ�Ŀ��ɹ켣�洢��
			{
				ballDoubtPosHis.clear();
				ballDoubtPosHis.push_back(x_out,y_out,z_out,vx_out,vy_out,vz_out,t_out);
				BallStatue_doubt_Last.x_last = x_out;
				BallStatue_doubt_Last.t_last = t_out;
				BallStatue_doubt_Last.direction = (vx_out>0);
			}
		}
		else//������5�����쳣ʱ����Ϊ�켣�ı�
		{
			ballPosHis.copy(ballDoubtPosHis);
			BallStatue_Last.predicted = 0;
			ballDoubtPosHis.clear();

			BallStatue_Last.x_last = x_out;
			BallStatue_Last.t_last = t_out;
			BallStatue_Last.direction = (vx_out>0);

			if (dataFile.m_bStartSave)
			{
				dataFile.SaveTraceData(0,0,0,0,0,0,0,1);//�رյ�ǰ�ļ����Դ����µĹ켣�ļ�
				cout<<"NewFile"<<endl;

			//	dataFile.SaveTraceData(1,2,3,4,5,6,7,88);
			/*	for (int M =0;M<ballPosHis.size();M++) //�����ļ��м�¼�ı�ǰ���ɹ켣�ڵļ�¼
				{
					dataFile.SaveTraceData(ballPosHis.tempX[M],ballPosHis.tempY[M],ballPosHis.tempZ[M],ballPosHis.tempTime[M],ballPosHis.tempVx[M],ballPosHis.tempVy[M],ballPosHis.tempVz[M],88);
				}*/
			}
		}
	}
	return 0;
}



