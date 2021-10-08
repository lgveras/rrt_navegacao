#pragma once

#include "CPointPH.h"
#include "G2PlanarPH.h"

class Utils{
public:
	Utils();
	~Utils();
	static CPointPH* cutEdge(CPointPH points[3], double L);
	static CPointPH cutPoint(CPointPH point1, CPointPH point2, double theta, double L);
	static G2PlanarPH* smoothEdge(CPointPH points[], double L);
};