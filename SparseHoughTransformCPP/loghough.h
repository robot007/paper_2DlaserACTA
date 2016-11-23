#ifdef LOGHOUGH_H_sadl98754KJHliuafq2jkl2364iyohyt87hk
#define GHOUGH_H_sadl98754KJHliuafq2jkl2364iyohyt87hk
class CLogHough{
public:
	CLogHough();
	~CLogHough();
	InitLogHough();//reset Accumulation Space,
	//note: the Log-Hough algorithm is resolution varient
	GetBestLine(double &d, double &alpha);
	GetResolution(double & dDel, double & alphaDel);
	PutPoint(double x, double y);//need to handle 2DPoint later
	
private:
	char ** LHAccSpace;
	char * RefLine;
	const 
	const double phi=


	
	
}

#endif // GHOUGH_H_sadl98754KJHliuafq2jkl2364iyohyt87hk


