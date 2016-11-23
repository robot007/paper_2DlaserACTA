#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <vector>
#include <algorithm>

#include "Hough.h"

using std::endl;
using std::cin;
using std::cout;
using std::ifstream;

#define BUF_SIZE 1024


int main()
{
//	ifstream inf("bumper33.dpt");
//	std::vector<Pt3D> allpt;
	float x,y,z;
	char str[BUF_SIZE];
	Pt3D pt3;	
/*	while(!inf.eof() ){
	 	inf.getline(str, BUF_SIZE,'['); 
		inf.getline(str, BUF_SIZE,']');
		sscanf(str, "%f, %f, %f", &x,&y,&z);
		pt3.x=x; pt3.y=y; pt3.z=z;
//		cout<<"x="<<x<<" y="<<y<<" z="<<z<<endl;
		allpt.push_back(pt3);
	}
	*/
	Hough myhough(SHT);
	Pt3D RobotPos;
	RobotPos.x=0; 
	RobotPos.y=0;
	RobotPos.z=0; 
//	myhough.PushScan(allpt, RobotPos);
	
	float RhoOut, ThetaOut;
	int ConfOut;
//	myhough.GetBestFit(RhoOut, ThetaOut, ConfOut);

	cout<<"RhoOut="<<RhoOut<<"  ThetaOut="<<ThetaOut<<" ConfOut="<<ConfOut<<endl;
//	inf.close();
	return 0;
}

