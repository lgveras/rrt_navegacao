/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MathUtils.h
 * Author: LuizGustavo
 *
 * Created on 8 de Agosto de 2016, 16:25
 */

#ifndef MATHUTILS_H
#define MATHUTILS_H

#include  <random>
#include  <iterator>

class MathUtils {
public:
    MathUtils();
    MathUtils(const MathUtils& orig);
    virtual ~MathUtils();
	static double euclidianDistance(float x1, float y1, float x2, float y2);
    static void change(float* value1, float* value2 );
	template<class Element, typename Iter>
	static Element vectorRandomSelection(Iter start, Iter end){
		std::random_device rd;
		std::mt19937 generator(rd());
		//generator.seed(seed);
		std::template uniform_int_distribution<> distribution(0, std::distance(start, end) - 1);
		std::template advance(start, distribution(generator));
		return *start;
	};
	static bool compareIntArrays(int * pt1Init, int* pt1End, int *pt2Init);
private:
};

#endif /* MATHUTILS_H */


