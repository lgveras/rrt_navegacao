#include "Utils.h"
#include "MathUtils.h"

Utils::Utils(){
}


Utils::~Utils(){
}

G2PlanarPH* Utils::smoothEdge(CPointPH points[], double L) {
	return new G2PlanarPH(points, L);
}

CPointPH Utils::cutPoint(CPointPH point1, CPointPH point2, double theta, double L) {
	if (point1.xPos <= point2.xPos) {
		return  CPointPH(cos(theta)*L + point1.xPos, sin(theta)*L + point1.yPos);
	} else {
		return CPointPH(point1.xPos - cos(theta)*L, point1.yPos + sin(theta)*L);
	}
}

CPointPH* Utils::cutEdge(CPointPH points[3], double L) {
	CPointPH shaperPoints[3];
	double h1, theta1, h2, theta2;

	h1 = MathUtils::euclidianDistance(points[0].xPos, points[0].yPos, points[1].xPos, points[1].yPos);
	theta1 = asin((points[0].yPos - points[1].yPos) / h1);

	h2 = MathUtils::euclidianDistance(points[1].xPos, points[1].yPos, points[2].xPos, points[2].yPos);
	theta2 = asin((points[2].yPos - points[1].yPos) / h2);

	shaperPoints[0] = cutPoint(points[1], points[0], theta1, L);
	shaperPoints[1] = points[1];
	shaperPoints[2] = cutPoint(points[1], points[2], theta2, L);

	return shaperPoints;
}