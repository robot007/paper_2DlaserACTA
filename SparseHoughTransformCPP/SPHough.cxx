#include <string.h>
#include <math.h>
#include <iostream.h>
#include <fstream>

#include "SPHough.h"

using std::ofstream;
using std::endl;

SPHough::SPHough(Vec3 RobotPos , BumperOri BumOriInput)
{
	Vec3 T ={0,0,0};
	m_ThetaAtom=9*DEG2RAD; // angular resolution 9 deg. old 18
	m_RhoAtom  =0.06;// distance resolution 6cm. old 0.5m; 
	m_RhoMin   =0.5; // min distance 0.5m
	m_RhoMax   =6.0; // max 6m 
	m_CornerAng=3*Pi; //started from invalid value
	m_PtNum    =0;
	m_AccumSpaceIs2D=false;
	//size of matrix
	m_RhoNum = (int) ceil( (m_RhoMax - m_RhoMin)/ m_RhoAtom);
	m_ThetaNum = (int) ceil( 2*Pi / m_ThetaAtom ); 


//	memcpy(&m_RobotInitPos , &T, sizeof(Vec3) );
	m_RobotInitPos = T;
	m_BumperOri = BumOriInput;

	of1.open("zhen.txt");
	linenum=0;
	
}

SPHough::~SPHough()
{
	if(!m_AccumSpaceIs2D)
		m_AccumSpace.clear();
	m_PtStorage.clear();


	of1.close();
}

void SPHough::PutPtToAccumspace(Vec3 Pt)
//different from SHough
{
	int Len=m_PtStorage.size();
	int i,Idx;
	Line NewLine;
	Vec3 pttmp;

	if(Len>2){
		for (i=0; i<Len; i+=5){
			Vec3 Pt1, Pt2;
			Pt1 = m_PtStorage[0];
			Pt2 = m_PtStorage[1];

			NewLine=FitLine(m_PtStorage[i], Pt);
			if(NewLine.counter==0)//invalide line
				continue; //consider the next point

			if( (Idx=IsExistLineInAccumSpace(NewLine)) < 0 ) {
//				of1<<"invalid Idx="<<Idx<<endl;
//				of1<<"new line ="<<NewLine.NTheta<<","<<NewLine.NRho<<
//					","<<NewLine.counter<<endl;
				//invalid Index, it is a new line
				m_AccumSpace.push_back(NewLine);

				linenum++;
			}
			else{//valid index. It is already existed
				//increase the counter 
//				of1<<"valid Idx="<<Idx<<endl;
				m_AccumSpace[Idx].counter++;

//				of1<<"Idx="<<Idx<<" cont="<<m_AccumSpace[Idx].counter<<endl;
				linenum++;
			}
		}
	}
	else if (Len==2){
		//init the AccumSpace
		NewLine=FitLine(m_PtStorage[0], m_PtStorage[1]);
		if (NewLine.counter!=0)
			m_AccumSpace.push_back(NewLine);
	}
	else { //do nothing
	}
}

Line SPHough::FitLine(Vec3 Pt1, Vec3 Pt2)
{
//Matlab code 
/*
            if abs(dat(cnt1,1)-dat(cnt2,1))<1e-6
                alpha=pi/2; % do not distinguish +-pi/2, since there is no direction of the line
            else 
                alpha=atan( (dat(cnt1,2)-dat(cnt2,2))/(dat(cnt1,1)-dat(cnt2,1)) );
            end
            rho0=dat(cnt1,1)*cos(alpha-pi/2) + dat(cnt1,2)*sin(alpha-pi/2);
            Theta=alpha-sign(rho0)*pi/2;
            CellTheta=floor(Theta/ResTheta)*ResTheta;
            Rho=abs(rho0); % dat(cnt2,1)*cos(Theta)+dat(cnt2,2)*sin(Theta);
            CellRho=floor(Rho/ResRho)*ResRho;
*/
	float Theta , Alpha, RhoTmp;
	Line RetLine;
	
	if (fabs( Pt1.x-Pt2.x )<1e-6 )// Pt1.x==Pt2.x
		Alpha = (float)Pi/2; // do not distinguish +-pi/2, since there is no direction of the line
	else 
		Alpha = atan( (Pt1.y-Pt2.y)/(Pt1.x-Pt2.x) );
	RhoTmp = Pt1.x*cos(Alpha-Pi/2) + Pt1.y*sin(Alpha-Pi/2);

//	of1<<"RhoTmp="<<RhoTmp<<endl;

	//now, change RhoTmp to position definate
	Theta= Alpha-fSign(RhoTmp)*Pi/2;
	RhoTmp=fabs(RhoTmp);
	
//	of1<<"Theta "<< Theta <<endl;	
	
	RetLine.NTheta = ceil( (Theta+Pi)/m_ThetaAtom - 0.5); //maybe outof range 
	// Theta \in [-Pi, Pi). Grid the Theta in cell by m_ThetaAtom
	// round the result
	if(RhoTmp>=m_RhoMin && RhoTmp<=m_RhoMax){
		RetLine.NRho = (unsigned short) ceil( (RhoTmp-m_RhoMin) / m_RhoAtom - 0.5); 
		RetLine.counter = 1; //when pushing to AccumSpace, at the first time, counter=1;
		return RetLine;
	}
	else{
		RetLine.NRho=0;
		RetLine.NTheta=0;
		RetLine.counter=0;
		return RetLine;
	}
}

int SPHough::IsExistLineInAccumSpace(Line InLine)
// if exist, return the index, which is no less than 0
// if not exist, return -1
{
	int Len,i;
	Len=m_AccumSpace.size();
	for(i=0; i<Len; i++){
		if( m_AccumSpace[i].NTheta==InLine.NTheta && 
			m_AccumSpace[i].NRho==InLine.NRho ){
			//exist in the AccumSpace
			return i;
		}
	}
	return -1;
}

void SPHough::GetBestFit(Vec3 & BumperPt1, Vec3 & BumperPt2, int &confidence)
{
	int Len, i, MaxCnt;
	Vec3 BumperMin, BumperMax, vTmp, MinX, MaxX;
	float Theta, RhoTmp, Rho, x, y;
	bool MinMaxInvalid=true;
	Line MaxLn; 

	Len = m_AccumSpace.size();
	//find the best fit line
	MaxLn.counter=0;
	MaxLn.NRho=0; 
	MaxLn.NTheta=0;

	of1<<"Get Best Fit "<<Len<<endl;

	for (i=0; i<Len; i++){
		if(MaxLn.counter < m_AccumSpace[i].counter ) {
			//MaxCnt=MAX(MaxCnt, m_AccumSpace[i].counter)
			//memcpy(&MaxLn, &m_AccumSpace[i], sizeof(Line) );
//			of1<<"m_AccumSpace[i].counter="<<m_AccumSpace[i].counter<<endl;
			MaxLn = m_AccumSpace[i];
		}
		of1<<"NRho="<<m_AccumSpace[i].NRho<<" NTheta="<<m_AccumSpace[i].NTheta<<
			" counter="<<m_AccumSpace[i].counter<<endl;
	}
	int LNum, LptNum;
	LptNum=m_PtStorage.size();
	LNum=(LptNum-1)*LptNum/2;
	confidence = MaxLn.counter*100/LNum;

	of1<<"Len "<<Len<<endl;
	of1<<"conf="<<confidence<<" LNum="<<LNum<<" counter"<<MaxLn.counter<<endl;
	of1<<"linenum="<<linenum<<endl;
	of1<<MinMaxInvalid<<endl;

	Rho=MaxLn.NRho*m_RhoAtom+m_RhoMin;
	Theta=MaxLn.NTheta*m_ThetaAtom-Pi;

	of1<<"NRho="<<MaxLn.NRho<<" Rho="<<Rho<<" Theta="<<Theta<<endl;

	//find the bumper points now
	if (confidence>0 && Len > 2 ) {
		//have enough points and fund the best fit line
		//  then find the two ending point

		//among all the points fall on this line, find the two
		//		with min/max X value
		for (i=0; i< Len ; i++){
			//try if Rho=x*cos(Theta)+y*sin(Theta) is valid
			vTmp=m_PtStorage[i];
			x=vTmp.x; y=vTmp.y;
			RhoTmp=x*cos(Theta) + y*sin(Theta);
			if ( fabs(RhoTmp-Rho)<=m_RhoAtom ){
				// vTmp is on the best fit line
				if (MinMaxInvalid == true){
					//init
					MinX=vTmp;
					MaxX=MinX;

					of1<<MinX.x<<endl;
					of1<<MinX.y<<endl;
					of1<<MinX.z<<endl;
//					of1<<MaxX<<endl;

					MinMaxInvalid=false; 
				}
				else{
					//update value
					BumperMin=MinXPt(MinX, vTmp);
					BumperMax=MaxXPt(MaxX, vTmp);
				}
			}
		}
	}
	else{
		//invalid value: 
		vTmp.x=0; vTmp.y=0; vTmp.z=0;
		BumperMin=vTmp;
		BumperMax=vTmp;
	}
	BumperPt1=BumperMin;
	BumperPt2=BumperMax;
	return;
}
