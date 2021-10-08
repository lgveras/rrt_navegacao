/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MathUtils.cpp
 * Author: LuizGustavo
 * 
 * Created on 8 de Agosto de 2016, 16:25
 */

#include "MathUtils.h"
#include <math.h>
#include "Node.h"
#include "Waypoint.h"

MathUtils::MathUtils() {
}

MathUtils::MathUtils(const MathUtils& orig) {
}

MathUtils::~MathUtils() {
}

double MathUtils::euclidianDistanceNode(Node* p1, Node* p2){
    return (sqrt( pow(p2->getNodeWaypoint()->getLongitude() - p1->getNodeWaypoint()->getLongitude(), 2) 
            + pow(p2->getNodeWaypoint()->getLatitude() - p1->getNodeWaypoint()->getLatitude(), 2)));
}

double MathUtils::euclidianDistancePoint(Point_2* p1, Point_2* p2){
	/*return (sqrt(pow(p2->x().exact().to_double() - p1->x().exact().to_double(), 2)
		+ pow(p2->y().exact().to_double() - p1->y().exact().to_double(), 2)));*/
	return (sqrt(pow(p2->x() - p1->x(), 2)
		+ pow(p2->y() - p1->y(), 2)));
}

double MathUtils::euclidianDistance(float x1, float y1, float x2, float y2){
	return (sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)));
}

double MathUtils::houghDistancePoint(Point_2 * origin, Point_2* end, Point_2 * p) {
	double d = euclidianDistancePoint(origin, p);
	double a = (p->y() - origin->y()) / (p->x() - origin->x());
	double theta = atan(a) * (180/3.14);
	//return sqrt(d*d + theta*theta);
	double alfa = theta - 45;
	double diagonal = euclidianDistancePoint(origin, end);
	return sqrt((alfa*alfa / (90.0*90.0)) + ((d*d) / (diagonal*diagonal)));
	//return d;
}

double MathUtils::transformed(Point_2 * p, double deltaX, double deltaY) {
	return p->x()*deltaX + p->y()*deltaY;
}

void MathUtils::change(float* value1, float* value2 ){
    float *temp = value1;
    value1 = value2;
    value2 = temp;
}

bool MathUtils::compareIntArrays(int * pt1Init, int * pt1End, int * pt2Init){
	while (pt1Init <= pt1End) {
		if (*pt1Init != *pt2Init) {
			return false;
		}

		pt1Init++;
		pt2Init++;
	}
	return true;
}
