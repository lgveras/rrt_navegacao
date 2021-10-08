/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   NavigationEnvironment.h
 * Author: LuizGustavo
 *
 * Created on 8 de Agosto de 2016, 14:20
 */

#ifndef NAVIGATIONENVIRONMENT_H
#define NAVIGATIONENVIRONMENT_H

#include "Node.h"
#include "boost/random.hpp"
#include "Cell.h"
#include "BinaryGrid.h"
#include "StaticObstacle.h"
#include "SukharevGrid.h"
#include "GridEnum.h"

typedef boost::mt19937 randomGenerator;

using namespace std;

class NavigationEnvironment {
public:
    NavigationEnvironment();
	NavigationEnvironment(BinaryGrid binaryGrid, bool withOffSet, double initCoordLong, double initCoordLat, double finalCoordLong, double finalCoordLat, int seed);
	NavigationEnvironment(vector<StaticObstacle*> obstacles, double initCoordLong, double initCoordLat, double finalCoordLong, double finalCoordLat, int seed);
	//NavigationEnvironment(BinaryGrid binaryGrid, double extensionLatitude, double extensionLongitude, int seed);
    NavigationEnvironment(float initialLatitude, float initialLongitude,
						  float finalLatitude, float finalLongitude, int seed);
    virtual ~NavigationEnvironment();
    Node* generateNode();
    Node* getInitialCoordinate();
    Node* getFinalCoordinate();
    void putNodeInsideLimits(Node* node);
    void buildGrid(int k, GridEnum gridType);
	vector<Cell*> getGrid();
	vector<StaticObstacle*> getStaticObstacles();
	Cell* getCellAtCoordinateIndex(int x, int y);
	Cell* getCellByXYCoordinate(double x, double y);
	Cell* getCellByIndex(int index);
	vector<Cell*> getNeigborhoodCell(Cell* cell);
	vector<Cell*> getNeigborhoodObstacleCell(Cell* cell);
	int getQtdRows();
	int getQtdColumns();
	SukharevGrid getDispersionGrid();
	int getSeedToRandomGenerator();
	int getValueFromBinaryGrid(int i, int j);
	double getFreeSpaceArea();
	GridEnum getGridType();
private:
    //TODO: Mudar essas váriaveis para um tipo mais adequado para representar
    //uma coordenada geográfica, não usar um nó.
	BinaryGrid binaryGrid;
    Node* initialCoordinate;
    Node* finalCoordinate;
	randomGenerator generator;
	boost::random::uniform_real_distribution<> genX;
	boost::random::uniform_real_distribution<> genY;
	vector<Cell*> cells;
	int cellRows;
	int cellColumns;
    SukharevGrid sukharevGrid;
	void configureRandomGenerators();
	vector<StaticObstacle*> staticObstacles;
	void initializeCoordinates(double initialLongitude, double initialLatitude,
							   double finalLongitude, double finalLatitude);
	void constructStaticObstacles(bool withOffSet);
	void searchObstacleCells(vector<Cell*>& cellsObstacle, Cell* currentCell);
	void constructObstaclePolygon(Polygon_2& obstacleVertices, Cell* currentCell, int direction);
	void constructObstaclePolygon(Polygon_2& obstacleVertices, Cell* currentCell);
	int calcDirectionAdjacentCells(Cell* currentCell, Cell* nextCell);
	int seed;
	GridEnum gridType = GridEnum::NONE_GRID;
};

#endif /* NAVIGATIONENVIRONMENT_H */