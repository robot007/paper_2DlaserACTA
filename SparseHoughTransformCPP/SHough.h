#ifndef SHOUGH_Hasiowqtjkndfiouou845ja
	#define SHOUGH_Hasiowqtjkndfiouou845ja
#include <vector>
#include <fstream>
using std::ofstream;

#define Pi (3.14159265358979)
#define DEG2RAD (3.14159265358979/180.0)

typedef struct myVec3D{
	double x,y,z;
}Vec3;
typedef	enum MyBumperOri{
		left,
		lefthalf,
		right,
		righthalf
}BumperOri;


class SHough
{
public:

	SHough(Vec3 RobotPos , BumperOri BumOriInput=lefthalf);
	SHough();
	~SHough();

	virtual void PutPtToAccumspace(Vec3 Pt);
	//void GetBestFit(float &Rho, float &Theta, int &confidence);
	void GetBestFit(Vec3 & BumperPt1, Vec3 & BumperPt2,int &confidence );
	// BumperPt1 is the point fall on the best fit line, and 
	//	has the largest X value 
	// BumperPt2 is the point with the minium X value, among 
	//	all the points that fall on the best fit line.
	//(Rho,Theta) is the best fit line in the accumulate space 
	// confidence is the percent of points that falls on the 
	//	best fit line.
	void PushScan(std::vector<Vec3> InputPts);
	// Just push the LATEST point set into this object. Don't push the 
	// points have already been pushed inside.
	void FindCorner(double &CornAng, std::vector<Vec3> InputPts);
	// Internal methord. Find the nearest point as the corner

	void SetUpdataThreshold(int Threshold);
	// if the Threshold percent points are not on the best fit bumper line, then the update is invalid
	bool Update(std::vector<Vec3> NewPts, double MinAng, double MaxAng);
	//Update the accumulate space only in a small region. From (MinAng, MaxAng)
	// MinAng and MaxAng must be in [-Pi, Pi)
	//return true if the confidence is higher than m_UpdateThreshold
	// return false if not.
	bool UpdateAccumspace(std::vector<Vec3> NewPts, double MinAng, double MaxAng);
protected:
	bool m_AccumSpaceIs2D;
	unsigned short ** m_AccumSpace;
	Vec3 m_RobotInitPos;	
	int m_RhoNum, m_ThetaNum;
	double m_RhoMin, m_RhoAtom, m_RhoMax, m_ThetaAtom, m_CornerAng;
	int m_PtNum,m_UpdateThreshold;
	BumperOri m_BumperOri;
	std::vector<Vec3> m_PtStorage; //store the pushed in points, so that we can find the Bumper Points
	Vec3 MaxXPt(Vec3 Pt1, Vec3 Pt2) ;//{ Pt1.x>=Pt2.x ? return Pt1 : return Pt2 ; }; 
		// return the point (from Pt1 and Pt2) with the maximum X 
	Vec3 MinXPt(Vec3 Pt1, Vec3 Pt2) ; //{ Pt1.x<Pt2.x  ? return(Pt1) : return(Pt2);}; 
		// return the point (from Pt1 and Pt2) with the minimum X 
	ofstream of1;
};

#endif