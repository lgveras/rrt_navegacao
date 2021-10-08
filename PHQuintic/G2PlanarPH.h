#pragma once
#include <complex>
#include <math.h>
#include "CPointPH.h"
using namespace std;

/*
Author: Bohan Dong, University of California, Davis(2014)
This code is intended only for experimental and research purposes. 
No commercial use may be made without the author's express permision. 
*/

struct G2PHquintic
{
	complex<double> p[6] ;		//Bezier control points of PH quintic
	complex<double> w[3] ;		//Bernstein coefficients of w(t) polynomial
	double sigma[5] ;			//parametric speed Bernstein coefficients
	double s[6] ;				//arc length Bernstein coefficients
};

class G2PlanarPH
{
protected:
	void Spline(bool closed);
	void Hermite(complex<double> p0, complex<double> p1, complex<double> p4, complex<double> p5);
	void G2Hermite(complex<double> p0);
	double L, theta, phi, lambda;
public:
	G2PHquintic spline[Max];
	complex<double> q[Max+1];
	int num, iter;

	G2PlanarPH();
	G2PlanarPH(const CPointPH m_pt[], int index);
	G2PlanarPH(const CPointPH m_pt[]);
	G2PlanarPH(const CPointPH p[], double L);

	virtual ~G2PlanarPH();

	friend complex<double> G2beval(int n, const complex<double> b[], double t);
	friend double G2beval(int n, const double b[], double t);
	friend void G2tridiag_open(int n, const complex<double> a[],
									 const complex<double> b[], 
									 const complex<double> c[], 
									 const complex<double> d[], 
									 complex<double>x[]);
	friend void G2tridiag_closed(int n, const complex<double> a[],
									  const complex<double> b[], 
									  const complex<double> c[], 
									  const complex<double> d[], 
									  complex<double>x[]);
	friend complex<double> G2getCtrlPt(const G2PlanarPH &pph, int i, int j);
	friend int G2getIter(const G2PlanarPH &pph);
	friend double G2getParaSpeed(const G2PlanarPH &pph, int i, double t);
	friend double G2getArcLength(const G2PlanarPH &pph, int i, double t);
	double G2getEnergy();
	friend double G2getEnergy(const G2PlanarPH &pph, int i);
	friend double G2getMaxL(const CPointPH p[], double capa);
	friend double G2getOffSetObstacleDistance(const CPointPH p[], double L);
	friend double G2calcTheta(const CPointPH p[]);
	friend double G2getCurvature(const G2PlanarPH &pph, double t);
	double G2getExtremumCurvature();
	double G2getTotalArcLength();

	friend complex<double> * G2getOffset(const G2PlanarPH &pph, int i, double d);
};