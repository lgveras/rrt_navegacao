#include "Cell.h"

using namespace std;

Cell::Cell(int _index, vector<Point_2> vertices, int i, int j, int binaryGridValue):rowIndex(i), columnIndex(j), index(_index){
	this->shape =  new Polygon_2{vertices.begin(), vertices.end()};
	this->binaryGridValue = binaryGridValue;
}

Cell::~Cell(){
}

Polygon_2* Cell::getShape(){
	return this->shape;
}

int Cell::getBinaryGridValue() {
	if (this) {
		return this->binaryGridValue;
	}
	return 0;
}

const int Cell::getColumnIndex(){
	return this->columnIndex;
}

const int Cell::getRowIndex(){
	return this->rowIndex;
}

void Cell::setAddedToObstacle(bool added) {
	this->inObstacle = added;
}

bool Cell::isAddedToObstacle() {
	return this->inObstacle;
}

int Cell::getIndex() {
	return this->index;
}

Point_2 Cell::getCentroid() {
	/*double middleX = this->shape->vertex(0).x().approx().inf() + 
			  (this->shape->vertex(1).x().approx().inf() - this->shape->vertex(0).x().approx().inf())/2 ;
	double middleY = this->shape->vertex(0).y().approx().inf() +
			  (this->shape->vertex(3).y().approx().inf() - this->shape->vertex(0).y().approx().inf()) / 2;*/
	double middleX = this->shape->vertex(0).x() +
		(this->shape->vertex(1).x() - this->shape->vertex(0).x()) / 2;
	double middleY = this->shape->vertex(0).y() +
		(this->shape->vertex(3).y() - this->shape->vertex(0).y()) / 2; 
	return Point_2{middleX, middleY};
}

bool Cell::isVerifiedToPolygonConstruction(){
	return this->verifiedToPolygonConstruction;
}

void Cell::setVerifiedToPolygonConstruction(bool value){
	this->verifiedToPolygonConstruction = value;
}
