#include "SukharevGrid.h"
#include "MathUtils.h"

SukharevGrid::SukharevGrid(){
}

SukharevGrid::SukharevGrid(Node * initialCoordinate, Node * finalCoordinate, double dirX, double dirY) :vectorX(dirX), vectorY(dirY){
	this->longitudeInterval = finalCoordinate->getNodeWaypoint()->getLongitude() -
							   initialCoordinate->getNodeWaypoint()->getLongitude();
	this->latitudeInterval = finalCoordinate->getNodeWaypoint()->getLatitude() -
							  initialCoordinate->getNodeWaypoint()->getLatitude();

	this->initialVerticeCellLong = initialCoordinate->getNodeWaypoint()->getLongitude();
	this->initialVerticeCellLat = initialCoordinate->getNodeWaypoint()->getLatitude();

	this->finalVerticeCellLong = finalCoordinate->getNodeWaypoint()->getLongitude();
	this->finalVerticeCellLat = finalCoordinate->getNodeWaypoint()->getLatitude();
}

SukharevGrid::~SukharevGrid(){
}

int SukharevGrid::build(int k){
	if (this->centroids.size() > 0) {
		this->centroids.clear();
	}

	this->qtdCellsByAxis = floor(sqrt(k));
	this->stepCentroidPositionLong = longitudeInterval / (2 * qtdCellsByAxis);
	this->stepCentroidPositionLat = latitudeInterval / (2 * qtdCellsByAxis);

	bool outOfGridBound = false;

	double currentVerticeCellLat = this->initialVerticeCellLat;
	double currentVerticeCellLong = this->finalVerticeCellLong;

	//latitude de centróide para célula
	double latCentroide = this->initialVerticeCellLat + stepCentroidPositionLat;
	//Longitude de centróide para célula
	double longCentroide = this->finalVerticeCellLong - stepCentroidPositionLong;

	//Constroi a grade no sentindo superior para inferior, como uma matriz
	while (outOfGridBound != true) {
		Node* newSukharevPoint = new Node{ latCentroide, longCentroide};
		this->centroids.push_back(newSukharevPoint);	
		//avanço no eixo horizontal
		latCentroide = (latCentroide + 2 * stepCentroidPositionLat) + (vectorX);
		if (this->finalVerticeCellLat <= latCentroide){
			latCentroide = this->initialVerticeCellLat + stepCentroidPositionLat;
			//avanço no eixo vertical
			longCentroide = longCentroide - 2 * stepCentroidPositionLong + (vectorY);
			if (this->initialVerticeCellLong >= longCentroide) {
				outOfGridBound = true;
			}
		}
	}

	//Calculation of the dispersion to set P of sukharev centroids
	double minResult, maxResult, minDistance, maxDistance;
	minDistance = numeric_limits<double>::max();
	maxDistance = numeric_limits<double>::min();

	vector<Node*>::iterator currentNodeIt;
	for (currentNodeIt = this->centroids.begin(); currentNodeIt < this->centroids.end(); currentNodeIt++) {
		vector<Node*>::iterator distanceNodeIt;
		for (distanceNodeIt = this->centroids.begin(); distanceNodeIt < this->centroids.end(); distanceNodeIt++) {
			if (*currentNodeIt != *distanceNodeIt) {
				minResult = MathUtils::euclidianDistanceNode(*currentNodeIt, *distanceNodeIt);
				if (minResult < minDistance) {
					minDistance = minResult;
				}
			}
		}

		if (minDistance > maxDistance) {
			maxDistance = minDistance;
		}
	}

	this->dispersion = maxDistance;
	//this->dispersion = this->stepCentroidPositionLong *2;
	return 1;
}

int SukharevGrid::buildLattice(int k) {
	if (this->centroids.size() > 0) {
		this->centroids.clear();
	}

	double N = k;
	pair<double, double> generatorVector(1, 19);

	double intpart;
	for (int i = 1; i <= N; i++) {
		double p = (i / N);
		double x = modf(p * generatorVector.first, &intpart);
		double y = modf(p * generatorVector.second, &intpart);
		Node* newCentroid = new Node{ x * this->finalVerticeCellLat, y * this->finalVerticeCellLat };
		this->centroids.push_back(newCentroid);
	}

	//Calculation of the dispersion to set P of lattice centroids
	double minResult, maxResult, minDistance, maxDistance;
	minDistance = numeric_limits<double>::max();
	maxDistance = numeric_limits<double>::min();

	vector<Node*>::iterator currentNodeIt;
	for (currentNodeIt = this->centroids.begin(); currentNodeIt < this->centroids.end(); currentNodeIt++) {
		vector<Node*>::iterator distanceNodeIt;
		for (distanceNodeIt = this->centroids.begin(); distanceNodeIt < this->centroids.end(); distanceNodeIt++) {
			if (*currentNodeIt != *distanceNodeIt) {
			minResult = MathUtils::euclidianDistanceNode(*currentNodeIt, *distanceNodeIt);
			if (minResult < minDistance) {
				minDistance = minResult;
			}
			}
		}

		if (minDistance > maxDistance) {
			maxDistance = minDistance;
		}
	}

	this->dispersion = maxDistance;
	return 1;
}

double SukharevGrid::getDispersion() {
	return this->dispersion;
}

int SukharevGrid::getCentroidIndex(Node * node){

	int column = ceil(node->getX() / ((this->finalVerticeCellLat - this->initialVerticeCellLat) / this->qtdCellsByAxis)) - 1;
	if (node->getX() == 0) {
		column = 0;
	}
	int row = this->qtdCellsByAxis - ceil(node->getY() / ((this->finalVerticeCellLong - this->initialVerticeCellLong) / this->qtdCellsByAxis));
	if (node->getY()==0) {
		row = this->qtdCellsByAxis - 1;
	}
	int result = row*this->qtdCellsByAxis + column;

	return result;
}

Node* SukharevGrid::getNearestCentroid(Node* node) {
	Node* nearest = NULL;
	double distance, limitDistance;

	limitDistance = numeric_limits<double>::max();

	vector<Node*>::iterator currentNodeIt;
	for (currentNodeIt = this->centroids.begin(); currentNodeIt < this->centroids.end(); currentNodeIt++) {
		distance = MathUtils::euclidianDistanceNode(*currentNodeIt, node);
		if (distance < limitDistance) {
			limitDistance = distance;
			nearest = *currentNodeIt;
		}
	}

	if (!nearest->isVisited())
		return nearest;
	else
		return nullptr;
}

Node* SukharevGrid::getNoVisitedCentroidByIndex(int index) {
	if (index >= this->centroids.size()) {
		return nullptr;
	}

	if (this->centroids[index]->isVisited()) {
		return nullptr;
	}

	return this->centroids[index];
}

Node* SukharevGrid::getCentroidByIndex(int index) {
	if (index >= this->centroids.size()) {
		return nullptr;
	}

	return this->centroids[index];
}

double SukharevGrid::getStepCentroidPositionLat(){
	return 	this->stepCentroidPositionLong;
}

double SukharevGrid::getStepCentroidPositionLong(){
	return this->stepCentroidPositionLong;
}

int SukharevGrid::getGridSize(){
	return qtdCellsByAxis*qtdCellsByAxis;
}