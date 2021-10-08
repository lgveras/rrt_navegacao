#include "Graphics.h"
#include "CPointPH.h"
#include "PlanarPH.h"
#include "PHDisplayer.h"
#include <iostream>
#include "MathUtils.h"

using namespace std;

G2PlanarPH smoothEdge(CPointPH points[3], double L){	
	//double h = MathUtils::euclidianDistance(points[0].xPos, points[0].yPos, points[1].xPos, points[1].yPos);
	//double phi = acos(abs(points[1].xPos - points[0].xPos) / h);
	G2PlanarPH curve(points, L);
	return curve;
}

CPointPH cutPoint(CPointPH point1, CPointPH point2, double theta, double L) {
	if (point1.xPos <= point2.xPos) {
		return  CPointPH(cos(theta)*L + point1.xPos, sin(theta)*L + point1.yPos);
	}
	else {
		return CPointPH(point1.xPos - cos(theta)*L, point1.yPos + sin(theta)*L);
	}
}

CPointPH* cutEdge(CPointPH points[3], double L) {
	CPointPH shaperPoints[3];
	double h1, theta1, h2, theta2;

	h1 = MathUtils::euclidianDistance(points[0].xPos, points[0].yPos, points[1].xPos, points[1].yPos);
	theta1 = asin((points[0].yPos - points[1].yPos) / h1);

	h2 = MathUtils::euclidianDistance(points[1].xPos, points[1].yPos, points[2].xPos, points[2].yPos);
	theta2 = asin((points[2].yPos - points[1].yPos) / h2);

	/*if (points[0].xPos <= points[1].xPos) {
		shaperPoints[0] = CPointPH(cos(theta1)*(h1 - L) + points[0].xPos, sin(theta1)*(h1 - L) + points[0].yPos);
		shaperPoints[1] = points[1];
	} else {
		if (points[0].yPos >= points[1].yPos)
			shaperPoints[0] = CPointPH(cos(theta1)*L + points[1].xPos, sin(theta1)*L + points[1].yPos);
		else
			shaperPoints[0] = CPointPH(cos(theta1)*L + points[1].xPos, points[1].yPos - sin(theta1)*L);
		shaperPoints[1] = points[1];
	}*/

	shaperPoints[0] = cutPoint(points[1], points[0], theta1, L);
	shaperPoints[1] = points[1];
	shaperPoints[2] = cutPoint(points[1], points[2], theta2, L);

	/*if (points[1].xPos <= points[0].xPos) {
		shaperPoints[0] = CPointPH(cos(theta1)*L + points[1].xPos, sin(theta1)*L + points[1].yPos);
	}
	else {
		shaperPoints[0] = CPointPH(points[1].xPos - cos(theta1)*L, points[1].yPos + sin(theta1)*L);
	}

	if (points[1].xPos <= points[2].xPos) {
		shaperPoints[2] = CPointPH(cos(theta2)*L + points[1].xPos, sin(theta2)*L + points[1].yPos);
	} else {
		shaperPoints[2] = CPointPH(points[1].xPos - cos(theta2)*L, points[1].yPos + sin(theta2)*L);
	}*/

	return shaperPoints;
}

int main(int argv, char** argc) {
	//double maxCapa = 0.004;
	double maxCapa = 0.015;

	//double L = 5;
	double L, d;

	Graphics graphics;
	graphics.init(argv, argc, new CPointPH(0,0), new CPointPH(5000, 5000), "curve");

	//graphics.init(argv, argc, new CPointPH(0,0), new CPointPH(500, 500), "curve");


	CPointPH points[8][3];
	CPointPH* result;
	CPointPH cuttedPoints[8][3];
	G2PlanarPH curves[8];
	double offset = 30;
	int i;
	double scale = 50;

	L = 150;
	double originX = 100;
	double originY = 250;

	double x = originX*cos(Pi / 6) + originY*sin(Pi / 6);
	double y = originX*-sin(Pi / 6) + originY*cos(Pi / 6);

	/*CPointPH p_i = CPointPH(x, y);
	CPointPH p_c = CPointPH(x + L, y);

	double theta = 0;

	for (i = 0; i < 8; i++) {	
		CPointPH p_o = CPointPH(x + L*(1 + cos(theta)), y + L*sin(theta));
		theta += Pi / 4;

		cuttedPoints[i][0] = p_i;
		cuttedPoints[i][1] = p_c;
		cuttedPoints[i][2] = p_o;

		curves[i] = smoothEdge(cuttedPoints[i], L);
	}*/

	i = 0;
	points[i][0] = CPointPH(scale*5, scale*80);
	points[i][1] = CPointPH(scale*20, scale*82);
	points[i][2] = CPointPH(scale*25, scale*90);
	L = G2getMaxL(points[i], maxCapa);
	d = G2getOffSetObstacleDistance(points[i], L);
	cout << "Dados para a Curva " << i << " - L: " << L << "; d: " <<d << endl;
	result = cutEdge(points[i], L);
	cuttedPoints[i][0] = result[0];
	cuttedPoints[i][1] = result[1];
	cuttedPoints[i][2] = result[2];
	curves[i] = smoothEdge(cuttedPoints[i], L);

	i = 1;
	points[i][0] = CPointPH(scale* (5 + offset), scale* 80);
	points[i][1] = CPointPH(scale* (20 + offset), scale* 82);
	points[i][2] = CPointPH(scale*(10 + offset), scale*90);
	L = G2getMaxL(points[i], maxCapa);
	d = G2getOffSetObstacleDistance(points[i], L);
	cout << "Dados para a Curva " << i << " - L: " << L << "; d: " <<d << endl;
	result = cutEdge(points[i], L);
	cuttedPoints[i][0] = result[0];
	cuttedPoints[i][1] = result[1];
	cuttedPoints[i][2] = result[2];
	curves[i] = smoothEdge(cuttedPoints[i], L);

	i = 2;
	points[i][0] = CPointPH(scale *(5 + offset*1.8), scale * 80);
	points[i][1] = CPointPH(scale *(20 + offset*1.8), scale * 82);
	points[i][2] = CPointPH(scale *(10 + offset*1.8), scale * 70);
	L = G2getMaxL(points[i], maxCapa);
	d = G2getOffSetObstacleDistance(points[i], L);
	cout << "Dados para a Curva " << i << " - L: " << L << "; d: " <<d << endl;
	result = cutEdge(points[i], L);
	cuttedPoints[i][0] = result[0];
	cuttedPoints[i][1] = result[1];
	cuttedPoints[i][2] = result[2];
	curves[i] = smoothEdge(cuttedPoints[i], L);

	/*i = 3;
	points[i][0] = CPointPH(scale *(5 + offset*2.5), scale * 80);
	points[i][1] = CPointPH(scale *(20 + offset*2.5), scale * 82);
	points[i][2] = CPointPH(scale *(25 + offset*2.5), scale * 70);
	L = G2getMaxL(points[i], maxCapa);
	d = G2getOffSetObstacleDistance(points[i], L);
	cout << "Dados para a Curva " << i << " - L: " << L << "; d: " <<d << endl;
	result = cutEdge(points[i], L);
	cuttedPoints[i][0] = result[0];
	cuttedPoints[i][1] = result[1];
	cuttedPoints[i][2] = result[2];
	curves[i] = smoothEdge(cuttedPoints[i], L);*/

	i = 3;
	points[i][0] = CPointPH(scale *(25 + offset*2.5), scale * 70 );
	points[i][1] = CPointPH(scale *(20 + offset*2.5), scale * 82);
	points[i][2] = CPointPH(scale *(5 + offset*2.5), scale * 80);
	L = G2getMaxL(points[i], maxCapa);
	d = G2getOffSetObstacleDistance(points[i], L);
	cout << "Dados para a Curva " << i << " - L: " << L << "; d: " << d << endl;
	result = cutEdge(points[i], L);
	cuttedPoints[i][0] = result[0];
	cuttedPoints[i][1] = result[1];
	cuttedPoints[i][2] = result[2];
	curves[i] = smoothEdge(cuttedPoints[i], L);

	i = 4;
	points[i][0] = CPointPH(scale * 15, scale * 40);
	points[i][1] = CPointPH(scale * 5, scale * 60);
	points[i][2] = CPointPH(scale * 15, scale * 70);
	L = G2getMaxL(points[i], maxCapa);
	d = G2getOffSetObstacleDistance(points[i], L);
	cout << "Dados para a Curva " << i << " - L: " << L << "; d: " <<d << endl;
	result = cutEdge(points[i], L);
	cuttedPoints[i][0] = result[0];
	cuttedPoints[i][1] = result[1];
	cuttedPoints[i][2] = result[2];
	curves[i] = smoothEdge(cuttedPoints[i], L);

	//i = 5;
	//points[i][0] = CPointPH(scale * (20 + offset), scale * 62);
	//points[i][1] = CPointPH(scale * (5 + offset), scale * 60);
	//points[i][2] = CPointPH(scale * (0 + offset), scale * 40);
	//L = G2getMaxL(points[i], maxCapa);
	//d = G2getOffSetObstacleDistance(points[i], L);
	//cout << "Dados para a Curva " << i << " - L: " << L << "; d: " <<d << endl;
	//result = cutEdge(points[i], L);
	//cuttedPoints[i][0] = result[0];
	//cuttedPoints[i][1] = result[1];
	//cuttedPoints[i][2] = result[2];
	//curves[i] = smoothEdge(cuttedPoints[i], L);
	i = 5;
	points[i][0] = CPointPH(scale * (0 + offset), scale * 40);
	points[i][1] = CPointPH(scale * (5 + offset), scale * 60);
	points[i][2] = CPointPH(scale * (20 + offset), scale * 62);
	L = G2getMaxL(points[i], maxCapa);
	d = G2getOffSetObstacleDistance(points[i], L);
	cout << "Dados para a Curva " << i << " - L: " << L << "; d: " <<d << endl;
	result = cutEdge(points[i], L);
	cuttedPoints[i][0] = result[0];
	cuttedPoints[i][1] = result[1];
	cuttedPoints[i][2] = result[2];
	curves[i] = smoothEdge(cuttedPoints[i], L);

	i = 6;
	points[i][0] = CPointPH(scale * (20 + offset)*1.8, scale * 62);
	points[i][1] = CPointPH(scale * (5 + offset)*1.8, scale * 60);
	points[i][2] = CPointPH(scale * (0 + offset)*1.8, scale * 68);
	L = G2getMaxL(points[i], maxCapa);
	d = G2getOffSetObstacleDistance(points[i], L);
	cout << "Dados para a Curva " << i << " - L: " << L << "; d: " <<d << endl;
	result = cutEdge(points[i], L);
	cuttedPoints[i][0] = result[0];
	cuttedPoints[i][1] = result[1];
	cuttedPoints[i][2] = result[2];
	curves[i] = smoothEdge(cuttedPoints[i], L);

	i = 7;
	points[i][0] = CPointPH(scale * (20 + offset*2.5), scale * 62);
	points[i][1] = CPointPH(scale * (5 + offset*2.5), scale * 60);
	points[i][2] = CPointPH(scale * (10 + offset*2.5), scale * 68);
	L = G2getMaxL(points[i], maxCapa);
	d = G2getOffSetObstacleDistance(points[i], L);
	cout << "Dados para a Curva " << i << " - L: " << L << "; d: " <<d << endl;
	result = cutEdge(points[i], L);
	cuttedPoints[i][0] = result[0];
	cuttedPoints[i][1] = result[1];
	cuttedPoints[i][2] = result[2];
	curves[i] = smoothEdge(cuttedPoints[i], L);

	PHDisplayer displayer(curves, points, cuttedPoints);
	
	graphics.registerDisplayable(&displayer);
	graphics.startGraphics();
}