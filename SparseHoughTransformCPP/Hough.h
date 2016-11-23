#ifndef  SPHT_HSDFA12398LKNDFSOIU09U341KJHDGOIUP
  #define SPHT_HSDFA12398LKNDFSOIU09U341KJHDGOIUP

#include <vector>


#define  START_ANGLE	-90
#define  FINAL_ANGLE	0

#define SPHT 0
#define LHT  1
#define SHT  2
#define Pi (3.14159265358979)
#define DEG2RAD (3.14159265358979/180.0)

typedef struct myPt3D{
	float x,y,z;
}Pt3D;

class Hough
{
public:
	Hough(char type=SPHT);
	~Hough();
	typedef struct _SPHT_Line{
		unsigned short NTheta, NRho;
		unsigned short counter;
	}SPHT_Line;

	void PushScan(std::vector<Pt3D> InputPts, Pt3D RobotPos);
	void PutPtToSPHTAccumSpace(Pt3D Point);
	int  IsExistInSPHTAccumSpace(SPHT_Line InLine);
	void GetBestFit(float &Rho, float &Theta, int &confidence);
	

private:
	//TrimSidePts(std::vector<Pt3D> & VPts, float &CornerAng=3*Pi); //if CornerAng==3*Pi, need to 
	//compute the CornerAng again.
	//CornerAng must be inside the 4th quadrum 		
	char m_HoughType;
	
	//for SHT
	double m_SHT_RhoMin, m_SHT_RhoAtom, m_SHT_RhoMax, m_SHT_ThetaAtom;
	unsigned short ** m_SHT_AccumSpace; 
	int m_SHT_RhoNum, m_SHT_ThetaNum;
	int m_SHT_PtNum;
	
	//for LHT
	int *m_LHT_LogCos;
	int m_LHT_RhoNum, m_LHT_ThetaNum;
	unsigned short ** m_LHT_AccumSpace;
	double m_LHT_RhoMin, m_LHT_RhoAtom, m_LHT_RhoMax, m_LHT_ThetaAtom;
	
	//SPHT
//	std::vector<std::vector<SPHT_Line>> m_SPHT_AccumSpace;
//	const float SPHT_HashRhoLength=1.0;
// 	use Hash table in future 
	double m_SPHT_RhoMin;
	double m_SPHT_RhoAtom, m_SPHT_ThetaAtom, m_SPHT_HashNum;
	std::vector<SPHT_Line> m_SPHT_AccumSpace;
	std::vector<Pt3D> m_SPHT_Pts;
	double m_CornerAng;
	
	double sign(double input) {if(input==0) return 0;
							  else if(input>0) return 1.0;
								   else return -1.0; }};
#endif //SPHT_HSDFA12398LKNDFSOIU09U341KJHDGOIUP
