#include <iostream>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <fstream>
#include "SHough.h"

using std::cout;
using std::endl;
using std::ofstream;


SHough::SHough(Vec3 RobotPos, BumperOri BumOriInput)
: m_AccumSpaceIs2D(true)
{
	int i,j;
	m_ThetaAtom=9*DEG2RAD; // angular resolution. old 18 deg
	m_RhoAtom  =0.06; //resolution 6 cm   //old 0.5; 
	m_RhoMin   =0.5;// 0.5 m 
	m_RhoMax   =6.0;// to 6 m
	m_CornerAng=3*Pi; //started from invalid value
	m_PtNum    =0;

	//size of matrix
	m_RhoNum = (int) ceil( (m_RhoMax - m_RhoMin)/ m_RhoAtom);
	m_ThetaNum = (int) ceil( 2*Pi / m_ThetaAtom ); 
	// If Rho>0 m_SHT_Theta ranged from -Pi to 0, since the bumper in on the left hand side
	// Here Rho might <= 0, which indicate the bumper is either at right of align with y axis
	m_AccumSpace = new unsigned short*[m_ThetaNum];

	for(i=0; i<m_ThetaNum; i++){
		m_AccumSpace[i] = new unsigned short[m_RhoNum];
		for (j=0; j<m_RhoNum; j++)
			m_AccumSpace[i][j] = 0;
	}
	//overwrite the parameters changed by SHough() 
	memcpy(&m_RobotInitPos , &RobotPos, sizeof(Vec3) );
	m_BumperOri = BumOriInput;

//	of1.open("zhens.txt");
}

SHough::SHough()
: m_AccumSpaceIs2D(true)
// can't recursivly call constructor
{
	m_ThetaAtom=18*DEG2RAD;
	m_RhoAtom  =0.5; 
	m_RhoMin   =0.5;
	m_RhoMax   =6.0;
	m_CornerAng=3*Pi; //started from invalid value
	m_PtNum    =0;
	//size of matrix
	m_RhoNum = (int) ceil( (m_RhoMax - m_RhoMin)/ m_RhoAtom);
	m_ThetaNum = (int) ceil( 2*Pi / m_ThetaAtom ); 
	Vec3 RobotPos;
	RobotPos.x=0; RobotPos.y=0; RobotPos.z=0;
	memcpy(&m_RobotInitPos , &RobotPos, sizeof(Vec3) );
	m_BumperOri = left;
}

SHough::~SHough()
{
	int i;
	if( m_AccumSpaceIs2D){
		for(i=0 ; i<m_ThetaNum; i++){
			delete [] m_AccumSpace[i];
		}
		delete [] m_AccumSpace;
	}
	else{//m_AccumSpace is either 2D matrix 
		// or vector
//		m_AccumSpace.clear();
	}
	m_PtStorage.clear();
}

//void SHough::GetBestFit(float &Rho, float &Theta, int &confidence, 
//						Vec3 & BumperPt1, Vec3 & BumperPt2,)
void SHough::GetBestFit(Vec3 & BumperPt1, Vec3 & BumperPt2, int &confidence)
{
	// BumperPt1 is the point fall on the best fit line, and 
	//		has the largest X value 
	// BumperPt2 is the point with the minium X value
	//		among all the points that fall on the best fit line.
	//		(Rho,Theta) is the best fit line in the accumulate space 
	// confidence is the percent of points that falls on the 
	//		best fit line. 
	int i, j, MaxIdxI, MaxIdxJ;
	float Rho, Theta;
	unsigned short MaxCount=0;

	int Len;
	Vec3 vTmp, BumperMin, BumperMax; 
		//two points on the end of the fitted bumper
	float RhoTmp, x, y;
	Vec3 MaxX, MinX;
	bool MinMaxInvalid=true;

	MaxCount = m_AccumSpace[0][0];
	Len = m_PtStorage.size();
	
	for(i=0;i<m_ThetaNum; i++){
		for(j=0;j<m_RhoNum; j++){
			if(m_AccumSpace[i][j] > MaxCount){
				MaxIdxI = i;
				MaxIdxJ = j;
				MaxCount = m_AccumSpace[i][j];
			}
		}
	}
	Theta = MaxIdxI * m_ThetaAtom - Pi;
	Rho = MaxIdxJ * m_RhoAtom + m_RhoMin;					
	confidence = MaxCount*100/m_PtNum; 
	
//	cout<<"Len="<<Len<<endl;
//	cout<<"Rho="<<Rho<<" NRho="<<MaxIdxJ<<endl;

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
					MinMaxInvalid=false; //
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

void SHough::PushScan(std::vector<Vec3> InputPts)
{
	int i, Len;
	int PtAdded=0;	
	

	Len = InputPts.size();
	switch( m_BumperOri ){
	case left:
	case right:
		for (i=0; i<Len; i++){
			PutPtToAccumspace(InputPts[i]);
			m_PtStorage.push_back(InputPts[i]);
		}

		PtAdded += Len;
		break;
	case lefthalf:
		if ( fabs((float)m_CornerAng)>Pi){
		//if needed, find the corner. This will be run only once
			FindCorner(m_CornerAng, InputPts);
		}
		for (i=0 ; i<Len ; i++){
			if ( atan( (InputPts[i].y-m_RobotInitPos.y)/(InputPts[i].x-m_RobotInitPos.x) )
				< m_CornerAng ){
				PutPtToAccumspace(InputPts[i]);
				m_PtStorage.push_back(InputPts[i]);
				PtAdded ++;
			}
		}
		
		break;
	case righthalf:
		if ( fabs((float)m_CornerAng)>Pi){
				//if needed, find the corner. This will be run only once
				FindCorner(m_CornerAng, InputPts);
		}
		for (i=0 ; i<Len ; i++){
			if ( atan( (InputPts[i].y-m_RobotInitPos.y)/(InputPts[i].x-m_RobotInitPos.x) )
				> m_CornerAng ){
				PutPtToAccumspace(InputPts[i]);
				m_PtStorage.push_back(InputPts[i]);
				PtAdded ++;
			}
		}
		break;
	}
	m_PtNum += PtAdded;
}

void SHough::FindCorner(double &CornAng,  std::vector<Vec3> InputPts)
{
	double MinDist2, Dist2;
	int MinIdx,i,Len;

	
		//invalid m_CornerAng, need to compute again
		// if the bumper is on the left, m_CornerAng is between [-Pi/2 0]
		// if the bumper is on the right, m_CornerAng in between [0 Pi/2]
		MinDist2=(InputPts[0].x-m_RobotInitPos.x)*(InputPts[0].x-m_RobotInitPos.x) + 
			(InputPts[0].y-m_RobotInitPos.y)*(InputPts[0].y-m_RobotInitPos.y);
		MinIdx = 0;
		Len = InputPts.size();
		for ( i = 0 ; i<Len ; i++){
			Dist2 = (InputPts[i].x-m_RobotInitPos.x)*(InputPts[i].x-m_RobotInitPos.x) + 
			(InputPts[i].y-m_RobotInitPos.y)*(InputPts[i].y-m_RobotInitPos.y);

			if (Dist2<MinDist2){
				MinDist2 = Dist2;
				MinIdx   = i;
			}
		}
		CornAng = atan( (InputPts[MinIdx].y-m_RobotInitPos.y)/(InputPts[MinIdx].x-m_RobotInitPos.x) );
}


void SHough::PutPtToAccumspace(Vec3 Pt)
//to 2D AccumSpace
{
	double ThetaIterator, RhoTmp;
	int i, RhoIdx;

	for(ThetaIterator = -Pi, i=0 ; i < m_ThetaNum ; 
		ThetaIterator += m_ThetaAtom, i++){
		RhoTmp = Pt.x * cos(ThetaIterator)+Pt.y * sin(ThetaIterator);
		RhoIdx = (int) ceil( (RhoTmp-m_RhoMin) / m_RhoAtom - 0.5); 
		//use ceil to simulate round
		if (RhoIdx >=0 && RhoIdx<m_RhoNum){
			m_AccumSpace[i][RhoIdx]++;
		}
	}
}

Vec3 SHough::MaxXPt(Vec3 Pt1, Vec3 Pt2) 
{
	if (Pt1.x >= Pt2.x)
		return Pt1;
	else 
		return Pt2;
}
Vec3 SHough::MinXPt(Vec3 Pt1, Vec3 Pt2) 
{	
	if (Pt1.x < Pt2.x)
		return Pt1;
	else 
		return Pt2;
}

bool SHough::Update(std::vector<Vec3> NewPts, double MinAng, double MaxAng)
// MinAng and MaxAng must be in [-Pi, Pi)
{
	int i, Len;
	int PtAdded=0;	
	std::vector<Vec3> HalfPts ;

	Len = NewPts.size();
	switch( m_BumperOri ){
	case left:
	case right:
		if(UpdateAccumspace(NewPts,MinAng, MaxAng))
			return true;
		else 
			return false;
		break;
	case lefthalf:
		if ( fabs((float)m_CornerAng)>Pi){
		//if needed, find the corner. This will be run only once
			FindCorner(m_CornerAng, NewPts);
		}
		for (i=0 ; i<Len ; i++){
			if ( atan( (NewPts[i].y-m_RobotInitPos.y)/(NewPts[i].x-m_RobotInitPos.x) )
				< m_CornerAng ){
				HalfPts.push_back(NewPts[i]);
			}
		}
		if( UpdateAccumspace(HalfPts,MinAng, MaxAng))
			return true;
		else 
			return false;
		break;
	case righthalf:
		if ( fabs((float)m_CornerAng)>Pi){
				//if needed, find the corner. This will be run only once
				FindCorner(m_CornerAng, NewPts);
		}
		for (i=0 ; i<Len ; i++){
			if ( atan( (NewPts[i].y-m_RobotInitPos.y)/(NewPts[i].x-m_RobotInitPos.x) )
				> m_CornerAng ){
				HalfPts.push_back(NewPts[i]);
			}
		}
		if( UpdateAccumspace(HalfPts,MinAng, MaxAng) )
			return true;
		else 
			return false;
		break;
	}
}

void SHough::SetUpdataThreshold(int Threshold)
{
	m_UpdateThreshold=Threshold;
}

bool SHough::UpdateAccumspace(std::vector<Vec3> NewPts, double MinAng, double MaxAng)
{
	int MinAngIdx, MaxAngIdx, Len,i, j, RhoIdx, MaxCount;
	double RhoTmp,Theta;

	MinAngIdx=(MinAng-(-Pi))/m_ThetaAtom;
	MaxAngIdx=(MaxAng-(-Pi))/m_ThetaAtom;
	
	Len=NewPts.size();
	MaxCount=0;
	for(i=0;i<Len;i++){
		for(j=MinAngIdx; j<=MaxAngIdx; j++){//Only update a small region
			Theta=j*m_ThetaAtom-Pi;
			RhoTmp = NewPts[i].x * cos(Theta)+NewPts[i].y * sin(Theta);
			RhoIdx = (int) ceil( (RhoTmp-m_RhoMin) / m_RhoAtom - 0.5); 
			//use ceil to simulate round
			if (RhoIdx >=0 && RhoIdx<m_RhoNum){
				m_AccumSpace[j][RhoIdx]++;
				MaxCount=MaxCount>m_AccumSpace[j][RhoIdx]?MaxCount:m_AccumSpace[j][RhoIdx];
				//MaxCount=MAX(MaxCount,m_AccumSpace[j][RhoIdx])
				//live update, we suppose most of points will pass the MaxCount point
			}
		}
		m_PtStorage.push_back(NewPts[i]);
		m_PtNum++;
	}
//	cout<<"sz="<<m_PtStorage.size()<<endl;

	if( (MaxCount*100/m_PtStorage.size()) < m_UpdateThreshold )
		return false; // update is invalid
	else
		return true; // update is valid
}