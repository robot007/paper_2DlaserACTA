#ifndef SPHOUGH_H15498jkgd89jklgsu9ojfdknl8opu
	#define SPHOUGH_H15498jkgd89jklgsu9ojfdknl8opu
#include <vector>
#include <fstream>

#include "SHough.h"

using std::ofstream;

typedef struct _Line{
		unsigned short NTheta, NRho, counter;//
}Line;
	
class SPHough: public SHough
{
public:
	SPHough(Vec3 RobotPos , BumperOri BumOriInput=left);
	~SPHough();

	void PutPtToAccumspace(Vec3 Pt); //different from SHough
//	void PushScan(std::vector<Vec3> InputPts); //inherented from SHough
	void GetBestFit(Vec3 & BumperPt1, Vec3 & BumperPt2, int &confidence); 
		//different from SHough
//	void FindCorner(double &CornAng, std::vector<Vec3> InputPts); //from SHough

	ofstream of1;

protected:
//	bool m_AccumSpaceIs2D;
	int m_RhoNum, m_ThetaNum;
	double m_RhoMin, m_RhoAtom, m_RhoMax, m_ThetaAtom, m_CornerAng;
	int m_PtNum;
//	std::vector<Vec3> m_PtStorage; // inherented from SHough
//store the pushed in points, so that we can find the Bumper Points
//	
	int IsExistLineInAccumSpace(Line InLine);
// if exist, return the index, which is no less than 0
// if not exist, return -1
	int fSign(float f){ if(f>0){return 1;}else{if(f<0) {return -1;}else{return 0;} } } 
	
	Line FitLine(Vec3 Pt1, Vec3 Pt2);
	std::vector<Line> m_AccumSpace;

	int linenum;
};

#endif