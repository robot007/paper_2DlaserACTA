#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <vector>
#include <algorithm>

#include "SHough.h"
#include "SPHough.h"

using std::endl;
using std::cin;
using std::cout;
using std::ifstream;

#define BUF_SIZE 1024


int main()
{
	ifstream inf("bumper33.dpt");
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

	SHough myhough(RobotPos, Ori);
//	SPHough myhough(RobotPos, Ori);

//	myhough.PushScan(allpt);

	for (i=0;i<100;i++)
		pts1.push_back(allpt[i]);
	for( ;i<allpt.size();i++)
		pts2.push_back(allpt[i]);
	myhough.SetUpdataThreshold(50);
	myhough.PushScan(pts1);
	bTmp=myhough.Update(pts2,-Pi, 0);
	cout<<bTmp<<endl;

//	float RhoOut, ThetaOut;
	int ConfOut;
	Vec3 Bumperpt1, Bumperpt2; // bumper is from Bumperpt1 to Bumperpt2; 
	myhough.GetBestFit(Bumperpt1, Bumperpt2, ConfOut );

	//cout<<"RhoOut="<<RhoOut<<"  ThetaOut="<<ThetaOut<<" ConfOut="<<ConfOut<<endl;
	cout<<"Pt1=("<<Bumperpt1.x<<","<<Bumperpt1.y<<")"<<endl;
	cout<<"Pt2=("<<Bumperpt2.x<<","<<Bumperpt2.y<<")"<<endl;
	cout<<" ConfOut="<<ConfOut<<endl;

	return 0;
}

