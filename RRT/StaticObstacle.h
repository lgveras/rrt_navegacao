#pragma once
//#include <CGAL/Polygon_2.h>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/Simple_cartesian.h>
#include <vector>
#include "Cell.h"
#include "Node.h"
#include "CGALDefinitions.h"

//typedef CGAL::Exact_predicates_exact_constructions_kernel CGALKernel;
//typedef CGAL::Simple_cartesian<double> CGALKernel;
//typedef CGAL::Polygon_2<CGALKernel> Polygon_2;
//typedef CGALKernel::Point_2 Point_2;
//typedef CGALKernel::Segment_2 Segment_2;

using namespace std;

class StaticObstacle{
public:
	StaticObstacle(vector<Cell*> obstacleCells, Polygon_2 obstacleShape, Node* startLimit, Node* endLimit);
	StaticObstacle(vector<Cell*> obstacleCells, Polygon_2 obstacleShape);
	~StaticObstacle();
	Polygon_2* getShape();
	Polygon_2* getOffSet();
	vector<Cell*> getObstacleCells();
	bool intersection(Segment_2 segment);
	bool intersectionNode(Node* node);
	//bool intersectInside(Segment_2 segment);
	vector<Point_2> getConvexVertices();
	vector<Point_2> getOffSetConvexVertices();
	void generateOffSet(double d);
	void setNavigationEnvironmentLimits(Node* startLimit, Node* endLimit);
private:
	Polygon_2 offSet;
	Polygon_2 shape;
	vector<Cell*> obstacleCells;
	vector<Point_2> convexVertices;
	Node* environmentLimits[2];
};