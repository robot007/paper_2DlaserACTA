#include <iostream>
#include <math.h>
#include <string.h>
#include "Hough.h"
using std::endl;
using std::cin;
using std::cout;


Hough::Hough(char type)
	:m_HoughType(type)
	// for SHT
	,m_SHT_RhoMin(0.5)
	,m_SHT_RhoAtom(0.1)
	,m_SHT_RhoMax(5)
	,m_SHT_ThetaAtom(18*DEG2RAD)	
	,m_SHT_AccumSpace(NULL)
	,m_SHT_PtNum(0)
	//for LHT
	,m_LHT_LogCos(NULL)
	,m_LHT_AccumSpace(NULL)	
	,m_LHT_RhoMin(0.5)
	,m_LHT_RhoAtom(0.1)
	,m_LHT_RhoMax(5)
	,m_LHT_ThetaAtom(18*DEG2RAD)
	//SPHT
	,m_SPHT_RhoMin(0.5) 
	,m_SPHT_RhoAtom(0.1) //Rho resolution 0.1 m
	,m_SPHT_ThetaAtom(18.0*DEG2RAD) //Theta resolution 18Deg
	,m_SPHT_AccumSpace(NULL)	
{
	int i,j;
	
	m_CornerAng=3*Pi; //is m_CornerAng not \in [-Pi/2 0], it is invalid
	switch(m_HoughType){
	case LHT:
		m_LHT_RhoNum = (int) ceil( (m_LHT_RhoMax - m_LHT_RhoMin)/ m_LHT_RhoAtom);
		m_LHT_ThetaNum = (int) ceil( Pi / m_LHT_ThetaAtom ); 
		// If Rho>0 m_LHT_Theta ranged from -Pi to 0, since the bumper in on the left hand side
		// Here Rho might <= 0, which indicate the bumper is either at right of align with y axis
		m_LHT_AccumSpace = (unsigned short**) new unsigned short[m_LHT_ThetaNum];
		for(i=0; i<m_LHT_ThetaNum; i++){
			m_LHT_AccumSpace[i] = new unsigned short[m_LHT_RhoNum];
			for (j=0; j<m_LHT_RhoNum; j++)
				m_LHT_AccumSpace[i][j] = 0;
		}
		
		m_LHT_LogCos = new int[m_LHT_ThetaNum];
		for(i=0; i<m_LHT_ThetaNum; i++)
			m_LHT_LogCos[i] = log(cos(i*m_LHT_ThetaAtom));
		break;
	case SHT:
		m_SHT_RhoNum = (int) ceil( (m_SHT_RhoMax - m_SHT_RhoMin)/ m_SHT_RhoAtom);
		m_SHT_ThetaNum = (int) ceil( Pi / m_SHT_ThetaAtom ); 
		// If Rho>0 m_SHT_Theta ranged from -Pi to 0, since the bumper in on the left hand side
		// Here Rho might <= 0, which indicate the bumper is either at right of align with y axis
		m_SHT_AccumSpace = (unsigned short**) new unsigned short[m_SHT_ThetaNum];
		for(i=0; i<m_SHT_ThetaNum; i++){
			m_SHT_AccumSpace[i] = new unsigned short[m_SHT_RhoNum];
			for (j=0; j<m_SHT_RhoNum; j++)
				m_SHT_AccumSpace[i][j] = 0;
		}
		break;
	case SPHT:
	default:
		
		// SPHT_HashNum = (int) ceil(SPHT_HashRhoLength / SPHT_RhoAtom); 
		//the size of the Hash table
	
		break;
	}
}

Hough::~Hough()
{
	int i;
	if (m_SHT_AccumSpace != NULL ){
//		for (i=0; i<m_SHT_ThetaNum; i++)
//			delete [] m_SHT_AccumSpace[i];
//		delete [] m_SHT_AccumSpace;
	}
/*	if (m_LHT_AccumSpace != NULL ){
		for (i=0; i<m_LHT_ThetaNum; i++)
			delete [] m_LHT_AccumSpace[i];
		delete[] m_LHT_AccumSpace;
	}
	if (m_LHT_LogCos != NULL ){
		delete[] m_LHT_LogCos;
	}
	*/
}

void Hough::PushScan(std::vector<Pt3D> InputPts, Pt3D RobotPos)
{
	
	//ALL
	std::vector<Pt3D> MyIterator;
	int i,Len,j;
	//LHT
	
	//SHT
	double ThetaIterator, RhoTmp;
	int RhoIdx;
	//SPHT
	double MinDist2, Dist2, MinIdx; //to save computation, compare distance square
	double Ang;
	
	//implemtation
	switch(m_HoughType){
	case LHT:
		
		break;
	case SHT:
		if ( fabs((float)m_CornerAng)>Pi){
			//invalid m_CornerAng, need to compute again
			// if the bumper is on the left, m_CornerAng is between [-Pi/2 0]
			
			//MinDist=sqrt(InputPts.begin()->x^2 + InputPts.begin()->y^2 );
			MinDist2=InputPts.begin()->x * InputPts.begin()->x + InputPts.begin()->y * InputPts.begin()->y ;
			MinIdx = 0;
			Len = InputPts.size();
//			for (MyIterator = InputPts.begin(); MyIterator != InputPts.end(); 
//				MyIterator++, i++){
//				Dist2 = MyIterator->x*MyIterator->x + MyIterator->y*MyIterator->y; 
//				if (Dist2<MinDist2){
//					MinDist2 = Dist2;
//					MinIdx   = i;
//				}
//			}
			for (i=0 ; i<Len ; i++){
				Dist2 = InputPts[i].x*InputPts[i].x + InputPts[i].y*InputPts[i].y;
				if ( Dist2 < MinDist2 ) {
					MinDist2  = Dist2;
					MinIdx	  = i;
				}
			}
			m_CornerAng = atan2(InputPts[MinIdx].y-RobotPos.y, 
				InputPts[MinIdx].x-RobotPos.x);
		}
		//bumper is on the left and align to the x axis
		// so find the angles smaller than m_CornerAng		
		Len = InputPts.size();
		for (j=0 ; j<Len ; j++){
			for(ThetaIterator = -Pi, i=0 ; i < m_SHT_ThetaNum ; ThetaIterator += m_SHT_ThetaAtom, i++){
				RhoTmp = InputPts[j].x * cos(ThetaIterator) +
						 InputPts[j].y * sin(ThetaIterator);
				RhoIdx = (int) ceil(RhoTmp / m_SHT_RhoAtom - 0.5); //use ceil to simulate round

				

				if(RhoIdx >=0 && RhoIdx < m_SHT_RhoMax){
					cout<<"i="<<i<<"RhoIdx="<<RhoIdx<<endl;
//					m_SHT_AccumSpace[i][RhoIdx]++; //AccumSpace(theta)(rho)
				}
			}
		}
		m_SHT_PtNum += InputPts.size();
		break;
	case SPHT:
	default:
		Len = InputPts.size();
/*		if ( abs(m_CornerAng)>Pi){
			//invalid m_CornerAng, need to compute again
			// if the bumper is on the left, m_CornerAng is between [-Pi/2 0]
			
			//MinDist=sqrt(InputPts.begin()->x^2 + InputPts.begin()->y^2 );
//			MinDist2=InputPts.begin()->x*InputPts.begin()->x + InputPts.begin()->y*InputPts.begin()->y ;
			MinDist2=InputPts[0].x * InputPts[0].x + InputPts[0].y * InputPts[0].y;
			MinIdx = 0;
//			for (MyIterator = InputPts.begin(); MyIterator != InputPts.end(); 
//				MyIterator++, i++){
//				Dist2 = MyIterator->x*MyIterator->x + MyIterator->y*MyIterator->y; 
//				if (Dist2<MinDist2){
//					MinDist2 = Dist2;
//					MinIdx   = i;
//				}
//			}
			for (i=0 ; i<Len ; i++){
				Dist2 = InputPts[i].x*InputPts[i].x + InputPts[i].y*InputPts[i].y;
				if (Dist2<MinDist2){
					MinDist2 = Dist2;
					MinIdx   = i;
				}
			}

			m_CornerAng = atan2(InputPts[MinIdx].y-RobotPos.y, 
				InputPts[MinIdx].x-RobotPos.x);
		}
*/
		//bumper is on the left and align to the x axis
		// so find the angles smaller than m_CornerAng		
//		for (MyIterator = InputPts.begin(); MyIterator != InputPts.end(); 
//			MyIterator++){
//			Ang = atan2( MyIterator->y-RobotPos.y, MyIterator->x-RobotPos.x);
//			if( Ang < m_CornerAng){
//				PutPtToSPHTAccumSpace(*MyIterator);
//				m_SPHT_Pts.push_back(*MyIterator);
//			}
//		}
		for (i=0 ; i<Len ; i++){
			Ang = atan2( InputPts[i].y-RobotPos.y, InputPts[i].x-RobotPos.x);
			
//			if( Ang < m_CornerAng){
				m_SPHT_Pts.push_back(InputPts[i]);
				PutPtToSPHTAccumSpace(InputPts[i]);
//			}
		}

		break;
	}
}

void Hough::PutPtToSPHTAccumSpace(Pt3D Point)
{
	//m_SPHT_AccumSpace
	int i,Len;
	int idx;
	SPHT_Line Line;
	double rho,theta, alpha ;
	Len = m_SPHT_Pts.size();
	for (i=0;i<Len-1;i++){
//	from Matlab	
//            if abs(dat(cnt1,1)-dat(cnt2,1))<1e-6
//                alpha=pi/2; % do not distinguish +-pi/2, since there is no direction of the line
//            else 
//                alpha=atan( (dat(cnt1,2)-dat(cnt2,2))/(dat(cnt1,1)-dat(cnt2,1)) );
//            end
//            rho0=dat(cnt1,1)*cos(alpha-pi/2) + dat(cnt1,2)*sin(alpha-pi/2);
//            Theta=alpha-sign(rho0)*pi/2;
//            CellTheta=floor(Theta/ResTheta)*ResTheta;
//            Rho=abs(rho0); % dat(cnt2,1)*cos(Theta)+dat(cnt2,2)*sin(Theta);
//            CellRho=floor(Rho/ResRho)*ResRho;		
		if ( fabs(Point.x - m_SPHT_Pts[i].x) < 1e-4){
			cout<<m_SPHT_Pts[i].x<<"  "<<m_SPHT_Pts[i].y<<endl;
			cout<<"Point.x y="<<Point.x<<"  "<<Point.y<<endl;
			alpha=Pi/2.0;
		}
		else {
			cout<<m_SPHT_Pts[i].y<<"  "<<m_SPHT_Pts[i].x<<endl;

			alpha=atan( (m_SPHT_Pts[i].y - Point.y)/(m_SPHT_Pts[i].x - Point.x) );
		}
		
		rho = Point.x * cos(alpha - Pi/2.0) + Point.y * sin(alpha - Pi/2.0);
		float tmp;
		tmp = sign(rho);
		tmp = tmp*Pi/2.0;
		

		theta = alpha - sign(rho)* Pi/2.0;
		rho = fabs((float)rho);
		Line.NTheta = (unsigned short) floor( (theta + Pi) / m_SPHT_ThetaAtom + 0.5); //round , 
		
		
		tmp = Line.NTheta * m_SPHT_ThetaAtom -Pi;

		// $theta \in (-Pi, Pi]$ so need to shift it
		Line.NRho   = (unsigned short) floor( (rho-m_SPHT_RhoMin) / m_SPHT_RhoAtom +0.5); //round 
	
		
		Line.counter= 0;
		if ((idx = IsExistInSPHTAccumSpace(Line))== -1 ){
			//not exist in the AccumSpace
			m_SPHT_AccumSpace.push_back(Line);
		}
		else {
			//alread exist, so update the counter
			m_SPHT_AccumSpace[idx].counter++;
		}
			
	}
}

int Hough::IsExistInSPHTAccumSpace(SPHT_Line MyLine)
{
	int len,idx;
	len = m_SPHT_AccumSpace.size();
	for (idx=0; idx < len; idx++){
		if (m_SPHT_AccumSpace[idx].NRho == MyLine.NRho && 
			m_SPHT_AccumSpace[idx].NTheta == MyLine.NTheta){
			m_SPHT_AccumSpace[idx].counter++;
			return idx;
			//exit from the for loop
		}
	}
	return -1; //can't find in m_SPHT_AccumSpace
}

void Hough::GetBestFit(float &Rho, float &Theta, int &confidence)
//don't call this, if the accumulation space is null
{
	//SPHT
	int MaxCount=0, MaxIdx=0;
	int i,j,len;
	//SHT
	int MaxIdxI = 0, MaxIdxJ = 0;
	//code 
	switch(m_HoughType){
	case LHT:
		break;
	case SHT:
		MaxCount = m_SHT_AccumSpace[0][0];
		for(i=0;i<m_SHT_ThetaNum; i++){
			for(j=0;j<m_SHT_RhoNum; j++){
				if(m_SHT_AccumSpace[i][j] > MaxCount){
					MaxIdxI = i;
					MaxIdxJ = j;
					MaxCount = m_SHT_AccumSpace[i][j];
				}
			}
		}
		Theta = MaxIdxI * m_SHT_ThetaAtom -Pi;
		Rho = MaxIdxJ * m_SHT_RhoAtom + m_SHT_RhoMin;					
		confidence = MaxCount*100/m_SHT_PtNum; 
		cout << Theta << endl;
		cout << Rho << endl;
		cout << confidence << endl;
		break;
	case SPHT:
	default:
		len = m_SPHT_AccumSpace.size();
		MaxCount = m_SPHT_AccumSpace[0].counter;

		cout<<"MaxCount="<<MaxCount<<endl;

		MaxIdx   = 0;
		for (i=1; i< len; i++){
			if(m_SPHT_AccumSpace[i].counter > MaxCount){
				MaxCount = m_SPHT_AccumSpace[i].counter;
				MaxIdx   = i;
			}
		}	 
		Rho   = m_SPHT_AccumSpace[MaxIdx].NRho   * m_SPHT_RhoAtom + m_SPHT_RhoMin;
		Theta = m_SPHT_AccumSpace[MaxIdx].NTheta * m_SPHT_ThetaAtom - Pi; //shift back to $(-Pi, Pi]$
		float AllPossibility; // = $C_{len}^2 = len(len-1)/2$
		AllPossibility = len*1.0*(len-1)/2.0;
		cout<<"len="<<len<<" all="<<AllPossibility<<endl;
		confidence = (int) (MaxCount * 100 / AllPossibility); //percentage over all
		break;
	}

}