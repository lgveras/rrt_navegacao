#pragma once

//#include <CGAL/Polygon_2.h>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <vector>
#include "CGALDefinitions.h"

//typedef CGAL::Exact_predicates_exact_constructions_kernel K;
//typedef CGAL::Polygon_2<K> Polygon_2;
//typedef K::Point_2  Point_2;

using namespace std;

class Cell{
public:
	Cell(int _index, vector<Point_2> vertices, int i, int j, int binaryGridValue);
	~Cell();
	Polygon_2* getShape();
	int getBinaryGridValue();
	const int getColumnIndex();
	const int getRowIndex();
	void setAddedToObstacle(bool added);
	bool isAddedToObstacle();
	int getIndex();
	Point_2 getCentroid();
	bool isVerifiedToPolygonConstruction();
	void setVerifiedToPolygonConstruction(bool value);
private:
	Polygon_2* shape;
	int binaryGridValue;
	bool inObstacle = false;
	bool verifiedToPolygonConstruction = false;
	const int columnIndex;
	const int rowIndex;
	int index;
};