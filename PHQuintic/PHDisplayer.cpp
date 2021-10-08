#include "PHDisplayer.h"
#include <iostream>

using namespace std;

PHDisplayer::PHDisplayer(G2PlanarPH curve[], CPointPH points[][3], CPointPH shaperPoints[][3]){

	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 3; j++) {
			this->points[i][j] = points[i][j];
			this->shaperPoints[i][j] = shaperPoints[i][j];
		}
		this->curve[i] = curve[i];
	}
}


PHDisplayer::~PHDisplayer(){
}

void PHDisplayer::display(Graphics* graphic) {
	double deltaT = 0.01;
	this->graphics = graphic;
	float color[3] = { 0,0,1 };
	float color2[3] = { 1,1,0 };
	float color3[3] = { 1,0,0 };
	float color4[3] = { 1, 0, 1 };

	for (int i = 0; i < 8; i++) {
		this->graphics->drawLine(points[i][0].xPos, points[i][0].yPos, points[i][1].xPos, points[i][1].yPos, color2, 3);
		this->graphics->drawPoint(points[i][0].xPos, points[i][0].yPos, color4, 10);
		this->graphics->drawLine(points[i][1].xPos, points[i][1].yPos, points[i][2].xPos, points[i][2].yPos, color2, 3);

		this->graphics->drawLine(shaperPoints[i][0].xPos, shaperPoints[i][0].yPos, shaperPoints[i][1].xPos, shaperPoints[i][1].yPos, color3, 3);
		this->graphics->drawLine(shaperPoints[i][1].xPos, shaperPoints[i][1].yPos, shaperPoints[i][2].xPos, shaperPoints[i][2].yPos, color3, 3);

		CPointPH ptTemp;

		for (double t = 0; t <= 1; t += deltaT) {
			ptTemp = CPointPH(real(G2beval(Dgr, this->curve[i].spline[1].p, t)), imag(G2beval(Dgr, this->curve[i].spline[1].p, t)));
			double a1 = ptTemp.xPos;
			double a2 = ptTemp.yPos;
			this->graphics->drawPoint(a1, a2, color, 3);
		}	

		cout << "Curvature by index: " << this->curve[i].G2getExtremumCurvature() << endl;
		cout << "Extremum curvature: " << G2getCurvature(this->curve[i], 0.5) << endl;
	}

	cout << " ------------------- End of curvatures --------------- " << endl;

}