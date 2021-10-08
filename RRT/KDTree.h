#include "ANN/ANN.h"
#include "CGALDefinitions.h"

#pragma once
class KDTree{
public:
	KDTree(int dim, int qtdPts);
	void insert(Point_2& queryPoint);
	ANNidx searchIndex(Point_2& queryPoint, int neighQtd, double errorSearch);
	Point_2 search(Point_2& queryPoint, int neighQtd, double errorSearch);
	Point_2 searchAndRemove(Point_2& queryPoint, int neighQtd, double errorSearch);
	Point_2 getByIndex(ANNidx idx);
	void removeByIndex(ANNidx idx);
	void checkByIndex(ANNidx idx);
	bool isCheckedByIndex(ANNidx idx);
	void build();
	int getSize();
	~KDTree();
private:
	ANNpointArray dataPoints; 
	bool* dataPointsCheck;
	int const dim; //dimension
	//int k = 0;
	int qtdPts; //maximum number of data points
	ANNdistArray dists; 
	ANNkd_tree* kdTree;
	int eps; //error bound
	int countPts = 0;
};