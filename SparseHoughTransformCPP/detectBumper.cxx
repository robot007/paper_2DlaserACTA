//	detectBumper.cxx -- Defines the "detectBumper" class

#include "detectBumper.h"

DetectBumper :: DetectBumper(LaserComm& laser_comm) : laser_comm(lc)
{
}

//	NOTE: Make Sure in the "main.cxx" that this thread starts after
//	the "laserComm" thread starts.
Thread :: Status DetectBumper :: body()
{
	//	The Laser resolution by default is HALF-DEGREE (361 Points)
	//	If we want to change the resolution to ONE-DEGREE (181 Points),
	//	Call this function.
	//	lc.setNumPoints(181);

	while (!checkKill())
	{
		//	Get the input information for "detectBumper" agent
		laser_data = lc.getLaserPointsInCone(START_ANGLE, FINAL_ANGLE);
		fitBumperLines();
	}

	return end();

}

void DetectBumper :: fitBumperLines()
{
}
