#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <time.h>
#include <Winsock2.h> // for gettimeofday

#include "SHough.h"
#include "SPHough.h"

using std::endl;
using std::cin;
using std::cout;
using std::ifstream;

#define BUF_SIZE 1024
//compare the speed of SHT and SPHT
// dist 

#ifdef _WIN32
int gettimeofday(struct timeval *tp, void *tzp)
{
    FILETIME time;
    __int64 tmp;

    if ( NULL == tp) return -1;

    GetSystemTimeAsFileTime(&time);

    tmp = time.dwHighDateTime;
    tmp <<= 32;
    tmp |= time.dwLowDateTime;
    tmp /= 10; // it was in 100 nanosecond periods
    tp->tv_sec = tmp / 1000000 - 11644473600L; // Windows Epoch begins at 12:00 AM 01.01.1601
    tp->tv_usec = tmp % 1000000;
    return 0;
}
#endif

int main()
{
	ifstream inf("BumperThine1.dpt");
	std::vector<Vec3> allpt, pts1, pts2;
	int i,j;
	bool bTmp;
	float x,y,z;
	char str[BUF_SIZE];
	Vec3 pt3;	
	while(!inf.eof() ){
	 	inf.getline(str, BUF_SIZE,'['); 
		inf.getline(str, BUF_SIZE,']');
		sscanf(str, "%f, %f, %f", &x,&y,&z);
		pt3.x=x; pt3.y=y; pt3.z=z;
//		cout<<"x="<<x<<" y="<<y<<" z="<<z<<endl;
		allpt.push_back(pt3);
	}

	Vec3 RobotPos={0,0,0};
	BumperOri Ori=left;


//	time_t tSHT1, tSHT2, tSPHT1, tSPHT2;
	struct timeval tSHT1, tSHT2, tSPHT1, tSPHT2;
	int Iteration=1;

//	SHough MySHough(RobotPos, Ori);
//	SPHough MySpHough(RobotPos, Ori);


//	myhough.PushScan(allpt);
	for (i=0;i<100;i++)
		pts1.push_back(allpt[i]);
	for( ;i<allpt.size();i++)
		pts2.push_back(allpt[i]);
	//test the speed of Standard Hough

	int ConfOut;
	Vec3 Bumperpt1, Bumperpt2; // bumper is from Bumperpt1 to Bumperpt2; 

	//time(&tSPHT1);
	gettimeofday(&tSPHT1,NULL);
	for (i=0;i<Iteration;i++){
		SPHough* pMySphough = new SPHough(RobotPos, Ori);

		pMySphough->SetUpdataThreshold(50);
		pMySphough->PushScan(pts1);
		bTmp=pMySphough->Update(pts2,-Pi, 0);
		pMySphough->GetBestFit(Bumperpt1, Bumperpt2, ConfOut );
		delete pMySphough;
	}
	//time(&tSPHT2);
	gettimeofday(&tSPHT2,NULL);

	//time(&tSHT1);
	gettimeofday(&tSHT1,NULL);
	for (i=0;i<Iteration;i++){
		SHough* pMyShough = new SHough(RobotPos, Ori);

		pMyShough->SetUpdataThreshold(50);
		pMyShough->PushScan(pts1);
		bTmp=pMyShough->Update(pts2,-Pi, 0);
		pMyShough->GetBestFit(Bumperpt1, Bumperpt2, ConfOut );
		delete pMyShough;
	}
	//time(&tSHT2);
	gettimeofday(&tSHT2,NULL);

//	cout<<"time SPHT="<<tSPHT2-tSPHT1<<endl;
//	cout<<"time SHT="<<tSHT2-tSHT1<<endl;
	cout<<"time SPHT="<<tSPHT2.tv_sec-tSPHT1.tv_sec+(tSPHT2.tv_usec-tSPHT1.tv_usec)*1e-6<<endl;
	cout<<"time SHT="<<tSHT2.tv_sec-tSHT1.tv_sec+(tSHT2.tv_usec-tSHT1.tv_usec)*1e-6<<endl;
	//cout<<"RhoOut="<<RhoOut<<"  ThetaOut="<<ThetaOut<<" ConfOut="<<ConfOut<<endl;
/*	cout<<"Pt1=("<<Bumperpt1.x<<","<<Bumperpt1.y<<")"<<endl;
	cout<<"Pt2=("<<Bumperpt2.x<<","<<Bumperpt2.y<<")"<<endl;
	cout<<" ConfOut="<<ConfOut<<endl;
*/
	return 0;
}

