#include "KDTree.h"
#include <iostream>

using namespace std;

KDTree::KDTree(int dim, int qtdPts):dim(dim), qtdPts(qtdPts){
	this->dataPoints = annAllocPts(qtdPts, dim); // allocate data points
	this->dataPointsCheck = new bool[qtdPts];
	for (int i = 0; i < qtdPts; i++) {
		this->dataPointsCheck[i] = false;
	}
}

KDTree::~KDTree(){
	delete this->kdTree;
}

void KDTree::insert(Point_2& queryPoint){
	dataPoints[countPts][0] = queryPoint.x(); dataPoints[countPts][1] = queryPoint.y();
	this->countPts++; 
}

void KDTree::build() {
	if (kdTree == nullptr) {
		this->kdTree = new ANNkd_tree( // build search structure
			this->dataPoints, // the data points
			this->qtdPts, // number of points
			this->dim); // dimension of space
	}
}

//ANNidx KDTree::searchIndex(Point_2& queryPoint, int neighQtd, double errorSearch) {
//	double eps = errorSearch;
//	//int  k = neighQtd == 0 ? 1 : neighQtd;
//	int k = neighQtd;
//	//int k = this->qtdPts;
//	//cout << "k = " << k << endl;
//	ANNidx* nnIdx = new ANNidx[k]; // allocate near neigh indices
//	ANNdist* dists = new ANNdist[k]; // allocate near neighbor dists
//	ANNpoint queryPt = annAllocPt(this->dim); // allocate query point
//	queryPt[0] = queryPoint.x(); queryPt[1] = queryPoint.y();
//
//	if (this->kdTree != nullptr) {
//		this->kdTree->annkSearch( // search
//			queryPt, // query point
//			k, // number of near neighbors
//			nnIdx, // nearest neighbors (returned)
//			dists, // distance (returned)
//			eps); // error bound
//	} else {
//		cout << "KD-tree not builded!" << endl;
//	}
//
//	/*cout << "------------------------------" << endl;
//	for (int i = 0; i < k; i++) {
//		cout << "Query (" << queryPt[0] << ", " << queryPt[1] << ")" << endl;
//		cout << "(" << this->dataPoints[nnIdx[i]][0] << ", " << this->dataPoints[nnIdx[i]][1] << ")  Idx: " << nnIdx[i] << " \t Dist: " << dists[i] << endl;
//	}
//	cout << "------------------------------" << endl;*/
//
//	delete queryPt;
//
//	/*if (neighQtd == 0) {
//		if(queryPoint.x() == dataPoints[nnIdx[k-1]][0] && queryPoint.y() == dataPoints[nnIdx[k-1]][1])
//			return nnIdx[k];
//		else
//			return nnIdx[k - 1];
//	}*/
//	ANNidx t = nnIdx[neighQtd];
//	int step = 1;
//	while (this->dataPointsCheck[t] == true) {
//		t = nnIdx[neighQtd + step];
//		step++;
//	}
//
//	return t;
//}

ANNidx KDTree::searchIndex(Point_2& queryPoint, int neighQtd, double errorSearch) {
	double eps = errorSearch;
	int k = neighQtd;
	ANNidx* nnIdx = new ANNidx[k]; // allocate near neigh indices
	ANNdist* dists = new ANNdist[k]; // allocate near neighbor dists
	ANNpoint queryPt = annAllocPt(this->dim); // allocate query point
	queryPt[0] = queryPoint.x(); queryPt[1] = queryPoint.y();

	if (this->kdTree != nullptr) {
		this->kdTree->annkSearch( // search
			queryPt, // query point
			k, // number of near neighbors
			nnIdx, // nearest neighbors (returned)
			dists, // distance (returned)
			eps); // error bound
	} else {
		cout << "KD-tree not builded!" << endl;
	}

	/*cout << "------------------------------" << endl;
	for (int i = 0; i < k; i++) {
		cout << "Query (" << queryPt[0] << ", " << queryPt[1] << ")" << endl;
		cout << "(" << this->dataPoints[nnIdx[i]][0] << ", " << this->dataPoints[nnIdx[i]][1] << ")  Idx: " << nnIdx[i] << " \t Dist: " << dists[i] << endl;
	}
	cout << "------------------------------" << endl;*/

	delete queryPt;
	return nnIdx[k-1];
}

Point_2 KDTree::search(Point_2& queryPoint, int neighQtd, double errorSearch) {
	ANNidx idx = searchIndex(queryPoint, neighQtd, errorSearch);
	Point_2 result(this->dataPoints[idx][0], dataPoints[idx][1]);
	return result;
}

Point_2 KDTree::searchAndRemove(Point_2& queryPoint, int neighQtd, double errorSearch) {
	ANNidx idx = searchIndex(queryPoint, neighQtd, errorSearch);
	Point_2 result(this->dataPoints[idx][0], this->dataPoints[idx][1]);
	this->removeByIndex(idx);
	
	return result;
}

Point_2 KDTree::getByIndex(ANNidx idx) {
	Point_2 result(this->dataPoints[idx][0], this->dataPoints[idx][1]);
	return result;
}

void KDTree::removeByIndex(ANNidx idx) {
	this->dataPoints[idx][0] = std::numeric_limits<double>::max(); this->dataPoints[idx][1] = std::numeric_limits<double>::max();
	//if (countPts > 0) {
	//	this->dataPoints[idx][0] = this->dataPoints[this->countPts - 1][0];
	//	this->dataPoints[idx][1] = this->dataPoints[this->countPts - 1][1];
	//}
	this->countPts--;
}

void KDTree::checkByIndex(ANNidx idx) {
	this->dataPointsCheck[idx] = true;
	this->countPts--;
}

bool KDTree::isCheckedByIndex(ANNidx idx) {
	return this->dataPointsCheck[idx];
}

int KDTree::getSize() {
	return this->countPts;
}