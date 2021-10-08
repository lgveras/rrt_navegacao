#pragma once
#include "NavigationEnvironment.h"
#include "G2PlanarPH.h"

class CurveCollisionChecker{
public:
	CurveCollisionChecker(NavigationEnvironment* ne);
	~CurveCollisionChecker();
	bool isCurveCollisionFree(G2PlanarPH* curve);
private:
	NavigationEnvironment* ne;
};