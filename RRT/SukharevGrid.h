#pragma once
#include "Node.h"
class SukharevGrid{
public:
	SukharevGrid();
	SukharevGrid(Node* initialCoordinate, Node* finalCoordinate, double vectorX, double vectorY);
	~SukharevGrid();
	int build(int k);
	int buildLattice(int k);
	int getCentroidIndex(Node* node);
	Node* getNoVisitedCentroidByIndex(int index);
	Node* getCentroidByIndex(int index);
	Node* getNearestCentroid(Node* node);
	double getStepCentroidPositionLat();
	double getStepCentroidPositionLong();
	int getGridSize();
	double getDispersion();
private:
	int qtdCellsByAxis;
	double  vectorX, vectorY;
	double longitudeInterval;
	double latitudeInterval;
	double initialVerticeCellLong;
	double initialVerticeCellLat;
	double stepCentroidPositionLong;
	double stepCentroidPositionLat;
	double finalVerticeCellLong;
	double finalVerticeCellLat;
	vector<Node*> centroids;
	double dispersion;
};

