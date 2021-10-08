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

MathUtils::MathUtils() {
}

MathUtils::MathUtils(const MathUtils& orig) {
}

MathUtils::~MathUtils() {
}

double MathUtils::euclidianDistance(float x1, float y1, float x2, float y2){
	return (sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)));
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
