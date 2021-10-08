/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   NavigationEnvironment.cpp
 * Author: LuizGustavo
 * 
 * Created on 8 de Agosto de 2016, 14:20
 */
#include <stdlib.h>
#include <iostream>
#include "NavigationEnvironment.h"
#include "Node.h"
#include "Waypoint.h"
#include "MathUtils.h"
#include "BinaryGrid.h"
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <time.h>
#include "CGALDefinitions.h"

////TODO::Definir arquivo type setter para os tipos do CGAL
//typedef CGAL::Exact_predicates_exact_constructions_kernel K;
///*TODO: Para não ficar um monte de definição ao longo do código,
//definir essas paradas aqui em um só lugar e ir chamando no código.
//depois pode ficar dificil de mudar*/
//typedef K::Point_2  Point_2;

using namespace std;

int countCorner = 0;

int mask1[4] = { 0,0,0,1 };
int mask2[4] = { 0,1,0,0 };
int mask3[4] = { 1,0,0,0 };
int mask4[4] = { 0,0,1,0 };

int maskSpecialCaseA[6] = { 1,1,0,0,1,1 };
int maskSpecialCaseB[6] = { 0,0,0,1,1,0 };
int maskSpecialCaseBExtended[9] = { 0,0,0,1,1,0,0,1,0 };
int maskSpecialCaseC[6] = { 0,0,0,0,1,0 };
int maskSpecialCaseD[6] = { 0,1,0,0,1,1 };
int maskSpecialCaseE[6] = { 0,1,1,1,1,1 };
int maskSpecialCaseF[6] = { 0,1,1,0,1,1 };
int maskSpecialCaseG[6] = { 1,1,1,0,1,1 };
int maskSpecialCaseH[6] = { 1,1,1,1,1,1 };
int maskSpecialCaseI[6] = { 1,1,0,0,1,0 };


int mask1WithCornerA[4] = { 0,1,1,1 };
int mask1WithCornerB[4] = { 1,0,1,1 };
int mask1WithCornerC[4] = { 1,1,0,1 };

int mask2WithCornerA[4] = { 0,1,1,1 };
int mask2WithCornerB[4] = { 1,1,0,1 };
int mask2WithCornerC[4] = { 1,1,1,0 };

int mask3WithCornerA[4] = { 1,0,1,1 };
int mask3WithCornerB[4] = { 1,1,0,1 };
int mask3WithCornerC[4] = { 1,1,1,0 };

int mask4WithCornerA[4] = { 0,1,1,1 };
int mask4WithCornerB[4] = { 1,0,1,1 };
int mask4WithCornerC[4] = { 1,1,1,0 };

int maskSize = 3;

NavigationEnvironment::NavigationEnvironment() {
}

NavigationEnvironment::NavigationEnvironment(BinaryGrid binaryGrid, bool withOffSet, double initCoordLong, double initCoordLat, double finalCoordLong, double finalCoordLat, int seed):seed(seed){
	double extensionLatitude = finalCoordLat - initCoordLat;
	double extensionLongitude = finalCoordLong - initCoordLong;

	double sideCellLatitude = extensionLatitude / binaryGrid.getColumnQuantity();
	double sideCellLongitude = extensionLongitude / binaryGrid.getRowQuantity();
	double currentLat = 0;
	double currentLong = extensionLongitude;
	int indexCell = 0;
	//CONSTRUÇÃO DA GRADE DO AMBIENTE DE NAVEGAÇÃO COM BASE NO GRID BINÁRIO
	//A grade é construída de cima para baixo, a partir do canto superior esquerdo, tendo
	//como inicio a posição (0, extensãoLongitude).
	//Cada célula é construída em sentido anti-horário, a partir do vértice do canto superior esquerdo
	//da célula com índice 0.
	cout << "Construindo grade do ambiente de navegação ..." << endl;
	for (int i = 0; i < binaryGrid.getRowQuantity(); i++) {
		for (int j = 0; j < binaryGrid.getColumnQuantity(); j++) {
			//A célula deve ser construída em sentido anti-horário senão ocorre um erro na
			//função JOIN do CGAL.
			vector<Point_2> vertices = {Point_2(currentLat, currentLong - sideCellLongitude),
										Point_2(currentLat + sideCellLatitude, currentLong - sideCellLongitude),
										Point_2(currentLat + sideCellLatitude, currentLong),
										Point_2(currentLat, currentLong)};

			cells.push_back(new Cell{ indexCell, vertices, i, j, binaryGrid.getValueInMatriz(i, j)});
			currentLat += sideCellLatitude;
			indexCell++;
			//cout << binaryGrid.getValueInMatriz(i, j) << " ";
		}
		//cout << endl;
		currentLat = 0;
		currentLong -= sideCellLongitude;
	}
	this->cellRows = binaryGrid.getColumnQuantity();
	this->cellColumns = binaryGrid.getRowQuantity();
	cout << "Grade do ambiente de navegação construída" << endl;

	this->initializeCoordinates(initCoordLong, initCoordLat, finalCoordLong, finalCoordLat);
	this->constructStaticObstacles(withOffSet);
	this->configureRandomGenerators();
}

NavigationEnvironment::NavigationEnvironment(vector<StaticObstacle*> obstacles, double initCoordLong, double initCoordLat, double finalCoordLong, double finalCoordLat, int seed) :seed(seed) {
	double extensionLatitude = finalCoordLat - initCoordLat;
	double extensionLongitude = finalCoordLong - initCoordLong;

	this->initializeCoordinates(initCoordLong, initCoordLat, finalCoordLong, finalCoordLat);
	this->staticObstacles = obstacles;
	this->configureRandomGenerators();
}

//TODO:Adaptar esse construtor para construir células
NavigationEnvironment::NavigationEnvironment(float initialLatitude, float initialLongitude,
	float finalLatitude, float finalLongitude, int seed):seed(seed) {
	this->initializeCoordinates(initialLatitude, initialLongitude, finalLatitude, finalLongitude);
	this->configureRandomGenerators();
}

NavigationEnvironment::~NavigationEnvironment() {
}

int NavigationEnvironment::getValueFromBinaryGrid(int i, int j) {
	return this->binaryGrid.getValueInMatriz(i, j);
}

double NavigationEnvironment::getFreeSpaceArea(){
	return 800*800;
}

GridEnum NavigationEnvironment::getGridType(){
	return this->gridType;
}

//Constroi obstáculos estáticos a partir dos valores obtidos da grade binária
void NavigationEnvironment::constructStaticObstacles(bool withOffSet) {
	for (int i = 0; i < this->cellRows; i++) {
		for (int j = 0; j < this->cellColumns; j++) {
			Cell* currentCell = this->getCellAtCoordinateIndex(i, j);
			if (currentCell->getBinaryGridValue() == 1 && !(currentCell->isAddedToObstacle())) {
				vector<Cell*> obstacleCells;
				searchObstacleCells(obstacleCells, currentCell);

				Polygon_2 obstacleShape;
				constructObstaclePolygon(obstacleShape, currentCell);

				//Remove duplicated vertices from Polygon
				int count1 = 0;
				//vector<Point_2> toErase;
				for (Polygon_2::Vertex_iterator it1 = obstacleShape.vertices_begin(); it1 < obstacleShape.vertices_end(); it1++) {
					int count2 = 0;
					for (Polygon_2::Vertex_iterator it2 = obstacleShape.vertices_begin(); it2 < obstacleShape.vertices_end(); it2++) {
						/*if ((*it1).x().approx().inf() == (*it2).x().approx().inf() &&
							(*it1).y().approx().inf() == (*it2).y().approx().inf()) {*/
						if ((*it1).x() == (*it2).x() &&
							(*it1).y() == (*it2).y()) {
							if (count1 != count2) {
								Polygon_2::Vertex_iterator toDelete = it2;
								it2--;
								obstacleShape.erase(toDelete);
							}
						}
						count2++;
					}
					count1++;
				}
				StaticObstacle* newObstacle = new StaticObstacle{ obstacleCells, obstacleShape, this->initialCoordinate, this->finalCoordinate };
				//newObstacle->setNavigationEnvironmentLimits(this->initialCoordinate, this->finalCoordinate);
				this->staticObstacles.push_back(newObstacle);
				//this->staticObstacles.push_back(new StaticObstacle{ obstacleCells, obstacleShape });
			}
		}
	}

	if(withOffSet){
		int d = 50;
		for (int i = 0; i < this->staticObstacles.size(); i++) {
			this->staticObstacles[i]->generateOffSet(d);
		}
	}
}

void NavigationEnvironment::constructObstaclePolygon(Polygon_2& obstacleVertices, Cell* currentCell) {
	int initialDirection = -1;
	constructObstaclePolygon(obstacleVertices, currentCell, initialDirection);
}

void NavigationEnvironment::constructObstaclePolygon(Polygon_2& obstacleVertices, Cell* currentCell, int direction) {
	if (currentCell != nullptr && currentCell->getBinaryGridValue() == 1 && !(currentCell->isVerifiedToPolygonConstruction())) {
		currentCell->setVerifiedToPolygonConstruction(true);

		int i = currentCell->getRowIndex(), j = currentCell->getColumnIndex();
		Cell* nextCell = nullptr;
		bool toVerify = true;
		int indexVertex;
		//Point_2 p = currentCell->getShape()->vertex(3);

		//#############################################
		//***************Special Cases*****************
		//#############################################
		/* Direction legend: -1:Null direction	 1: Up	  2: Left		3: Down		4:Right*/
		int toApplyMaskSNeighborSpecialCase[9] = { getCellAtCoordinateIndex(i - 1, j - 1) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j - 1)->getBinaryGridValue(),
												getCellAtCoordinateIndex(i - 1, j) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j)->getBinaryGridValue(),
												getCellAtCoordinateIndex(i - 1, j + 1) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j + 1)->getBinaryGridValue(),
												getCellAtCoordinateIndex(i , j - 1) == nullptr ? 0 : getCellAtCoordinateIndex(i, j - 1)->getBinaryGridValue(),
												getCellAtCoordinateIndex(i ,j)->getBinaryGridValue(),
												getCellAtCoordinateIndex(i, j + 1) == nullptr ? 0 : getCellAtCoordinateIndex(i, j + 1)->getBinaryGridValue(),												
												getCellAtCoordinateIndex(i+1, j - 1) == nullptr ? 0 : getCellAtCoordinateIndex(i + 1, j - 1)->getBinaryGridValue(),
												getCellAtCoordinateIndex(i+1, j) == nullptr ? 0 : getCellAtCoordinateIndex(i + 1, j)->getBinaryGridValue(),
												getCellAtCoordinateIndex(i+1, j + 1) == nullptr ? 0 : getCellAtCoordinateIndex(i + 1, j + 1)->getBinaryGridValue() };

		int toApplyMaskSpecialCase[6] = { getCellAtCoordinateIndex(i - 1, j - 1) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j - 1)->getBinaryGridValue(),
										getCellAtCoordinateIndex(i - 1, j) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j)->getBinaryGridValue(),
										getCellAtCoordinateIndex(i - 1, j + 1) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j + 1)->getBinaryGridValue(),
										getCellAtCoordinateIndex(i , j - 1) == nullptr ? 0 : getCellAtCoordinateIndex(i, j - 1)->getBinaryGridValue(),
										getCellAtCoordinateIndex(i ,j)->getBinaryGridValue(),
										getCellAtCoordinateIndex(i, j + 1) == nullptr ? 0 : getCellAtCoordinateIndex(i, j + 1)->getBinaryGridValue() };

		int toApplyMaskMinorSpecialCase[4] = { getCellAtCoordinateIndex(i - 1, j) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j)->getBinaryGridValue(),
											getCellAtCoordinateIndex(i - 1, j + 1) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j + 1)->getBinaryGridValue(),
											getCellAtCoordinateIndex(i, j)->getBinaryGridValue(),
											getCellAtCoordinateIndex(i, j + 1) == nullptr ? 0 : getCellAtCoordinateIndex(i, j + 1)->getBinaryGridValue() };

		if (direction == 2){
			if (MathUtils::compareIntArrays(maskSpecialCaseA, maskSpecialCaseA + maskSize + 2, toApplyMaskSpecialCase)) {
				constructObstaclePolygon(obstacleVertices, getCellAtCoordinateIndex(i - 1, j), 1);
			} else if (MathUtils::compareIntArrays(maskSpecialCaseD, maskSpecialCaseD + maskSize + 2, toApplyMaskSpecialCase)) {
				constructObstaclePolygon(obstacleVertices, getCellAtCoordinateIndex(i - 1, j), 1);
			}else if (MathUtils::compareIntArrays(maskSpecialCaseE, maskSpecialCaseE + maskSize + 2, toApplyMaskSpecialCase) ||
					  MathUtils::compareIntArrays(maskSpecialCaseF, maskSpecialCaseF + maskSize + 2, toApplyMaskSpecialCase) || 
					  MathUtils::compareIntArrays(maskSpecialCaseG, maskSpecialCaseG + maskSize + 2, toApplyMaskSpecialCase) ||
					  MathUtils::compareIntArrays(maskSpecialCaseH, maskSpecialCaseH + maskSize + 2, toApplyMaskSpecialCase)) {
				currentCell->setVerifiedToPolygonConstruction(false);
				return;
			}
		}

		if (direction == 1) {
			if (MathUtils::compareIntArrays(maskSpecialCaseB, maskSpecialCaseB + maskSize + 2, toApplyMaskSpecialCase)) {
				if (MathUtils::compareIntArrays(maskSpecialCaseBExtended, maskSpecialCaseBExtended + maskSize + 5, toApplyMaskSNeighborSpecialCase)){
					obstacleVertices.push_back(currentCell->getShape()->vertex(2));
				}else {
					obstacleVertices.push_back(currentCell->getShape()->vertex(2));
					constructObstaclePolygon(obstacleVertices, getCellAtCoordinateIndex(i, j - 1), 2);
				}
			} else if (MathUtils::compareIntArrays(maskSpecialCaseC, maskSpecialCaseC + maskSize + 2, toApplyMaskSpecialCase)) {
				obstacleVertices.push_back(currentCell->getShape()->vertex(2));
			} else if (MathUtils::compareIntArrays(mask4WithCornerB, mask4WithCornerB + maskSize, toApplyMaskMinorSpecialCase)) {
				constructObstaclePolygon(obstacleVertices, getCellAtCoordinateIndex(i, j + 1), 4);
				obstacleVertices.push_back(currentCell->getShape()->vertex(2));
				constructObstaclePolygon(obstacleVertices, getCellAtCoordinateIndex(i - 1, j), 1);
			} else if (MathUtils::compareIntArrays(maskSpecialCaseI, maskSpecialCaseI + maskSize + 2, toApplyMaskSpecialCase)) {
				constructObstaclePolygon(obstacleVertices, getCellAtCoordinateIndex(i - 1, j), 1);
			}
		}

		//#############################################
		//*******MASK 1: Left Upper Vertice************
		//#############################################

		nextCell = getCellAtCoordinateIndex(i, j - 1);
		if (nextCell != nullptr && nextCell->isVerifiedToPolygonConstruction()) {
			toVerify = false;
		}

		if(toVerify){
			int toApplyMask1[4] = { getCellAtCoordinateIndex(i - 1, j - 1) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j - 1)->getBinaryGridValue(),
				getCellAtCoordinateIndex(i - 1, j) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j)->getBinaryGridValue(),
				getCellAtCoordinateIndex(i , j - 1) == nullptr ? 0 : getCellAtCoordinateIndex(i, j - 1)->getBinaryGridValue(),
				getCellAtCoordinateIndex(i ,j)->getBinaryGridValue() };

			indexVertex = 3;
			if (MathUtils::compareIntArrays(mask1, mask1 + maskSize, toApplyMask1)) {
				//Calc the coordination of the cell and get the upper most left node.
				obstacleVertices.push_back(currentCell->getShape()->vertex(indexVertex));
			} else {			
					if (MathUtils::compareIntArrays(mask1WithCornerA, mask1WithCornerA + maskSize, toApplyMask1) ||
						MathUtils::compareIntArrays(mask1WithCornerB, mask1WithCornerB + maskSize, toApplyMask1) ||
						MathUtils::compareIntArrays(mask1WithCornerC, mask1WithCornerC + maskSize, toApplyMask1)) {					
						obstacleVertices.push_back(currentCell->getShape()->vertex(indexVertex));					
					}
			}

			if (nextCell != nullptr && nextCell->getBinaryGridValue() == 1) {
				constructObstaclePolygon(obstacleVertices, nextCell, calcDirectionAdjacentCells(currentCell, nextCell));
			}
		}
		
		toVerify = true;

		//#############################################
		//*******MASK 2: Left Lower Vertice************
		//#############################################

		nextCell = getCellAtCoordinateIndex(i + 1, j);
		if (nextCell != nullptr && nextCell->isVerifiedToPolygonConstruction()) {
			toVerify = false;
		}

		if (toVerify){
			int toApplyMask2[4] = { getCellAtCoordinateIndex(i, j - 1) == nullptr ? 0 : getCellAtCoordinateIndex(i, j - 1)->getBinaryGridValue(),
									getCellAtCoordinateIndex(i, j)->getBinaryGridValue(),
									getCellAtCoordinateIndex(i + 1, j - 1) == nullptr ? 0 : getCellAtCoordinateIndex(i + 1, j - 1)->getBinaryGridValue(),
									getCellAtCoordinateIndex(i + 1 , j) == nullptr ? 0 : getCellAtCoordinateIndex(i + 1, j)->getBinaryGridValue() };

			indexVertex = 0;
			if (MathUtils::compareIntArrays(mask2, mask2 + maskSize, toApplyMask2)) {
				//Calc the coordination of the cell and get the upper most left node.
				obstacleVertices.push_back(currentCell->getShape()->vertex(indexVertex));
			} else {
				if (MathUtils::compareIntArrays(mask2WithCornerA, mask2WithCornerA + maskSize, toApplyMask2) ||
					MathUtils::compareIntArrays(mask2WithCornerB, mask2WithCornerB + maskSize, toApplyMask2) ||
					MathUtils::compareIntArrays(mask2WithCornerC, mask2WithCornerC + maskSize, toApplyMask2)) {				
					obstacleVertices.push_back(currentCell->getShape()->vertex(indexVertex));				
				}
			}

		
			if (nextCell != nullptr && nextCell->getBinaryGridValue() == 1) {
				constructObstaclePolygon(obstacleVertices, nextCell, calcDirectionAdjacentCells(currentCell, nextCell));
			}
		}

		toVerify = true;

		//#############################################
		//*******MASK 3:Right Lower Vertice************
		//#############################################

		nextCell = getCellAtCoordinateIndex(i, j + 1);
		if (nextCell != nullptr && nextCell->isVerifiedToPolygonConstruction()) {
			toVerify = false;
		}

		if (toVerify){
			int toApplyMask3[4] = { getCellAtCoordinateIndex(i, j)->getBinaryGridValue(),
									getCellAtCoordinateIndex(i, j + 1) == nullptr ? 0 : getCellAtCoordinateIndex(i, j + 1)->getBinaryGridValue(),
									getCellAtCoordinateIndex(i + 1, j) == nullptr ? 0 : getCellAtCoordinateIndex(i + 1, j)->getBinaryGridValue(),
									getCellAtCoordinateIndex(i + 1 , j + 1) == nullptr ? 0 : getCellAtCoordinateIndex(i + 1 , j + 1)->getBinaryGridValue() };

			indexVertex = 1;
			if (MathUtils::compareIntArrays(mask3, mask3 + maskSize, toApplyMask3)) {
				//Calc the coordination of the cell and get the upper most left node.
				obstacleVertices.push_back(currentCell->getShape()->vertex(indexVertex));
			} else {
				if (MathUtils::compareIntArrays(mask3WithCornerA, mask3WithCornerA + maskSize, toApplyMask3) || 
					MathUtils::compareIntArrays(mask3WithCornerB, mask3WithCornerB + maskSize, toApplyMask3) || 
					MathUtils::compareIntArrays(mask3WithCornerC, mask3WithCornerC + maskSize, toApplyMask3)) {
				
					obstacleVertices.push_back(currentCell->getShape()->vertex(indexVertex));
					
				}
			}

			if (nextCell != nullptr && nextCell->getBinaryGridValue() == 1) {
				constructObstaclePolygon(obstacleVertices, nextCell, calcDirectionAdjacentCells(currentCell, nextCell));
			}
		}


		toVerify = true;

		//#############################################
		//*******MASK 4:Right Upper Vertice************
		//#############################################

		nextCell = getCellAtCoordinateIndex(i - 1, j);
		if (nextCell != nullptr && nextCell->isVerifiedToPolygonConstruction()) {
			toVerify = false;
		}

		if (toVerify){

			int toApplyMask4[4] = { getCellAtCoordinateIndex(i - 1, j) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j)->getBinaryGridValue(),
									getCellAtCoordinateIndex(i - 1, j + 1) == nullptr ? 0 : getCellAtCoordinateIndex(i - 1, j + 1)->getBinaryGridValue(),
									getCellAtCoordinateIndex(i, j)->getBinaryGridValue(),
									getCellAtCoordinateIndex(i, j + 1) == nullptr ? 0 : getCellAtCoordinateIndex(i, j + 1)->getBinaryGridValue() };

			indexVertex = 2;
			if (MathUtils::compareIntArrays(mask4, mask4 + maskSize, toApplyMask4)) {
				//Calc the coordination of the cell and get the upper most left node.
				obstacleVertices.push_back(currentCell->getShape()->vertex(indexVertex));
			} else {
				if (MathUtils::compareIntArrays(mask4WithCornerA, mask4WithCornerA + maskSize, toApplyMask4) || 
					MathUtils::compareIntArrays(mask4WithCornerB, mask4WithCornerB + maskSize, toApplyMask4) ||
					MathUtils::compareIntArrays(mask4WithCornerC, mask4WithCornerC + maskSize, toApplyMask4)) {
				
					obstacleVertices.push_back(currentCell->getShape()->vertex(indexVertex));
				}
			}

			if (nextCell != nullptr && nextCell->getBinaryGridValue() == 1) {
				constructObstaclePolygon(obstacleVertices, nextCell, calcDirectionAdjacentCells(currentCell, nextCell));
			}
		}
	}
	return;
}

int NavigationEnvironment::calcDirectionAdjacentCells(Cell* currentCell, Cell* nextCell) {
	Point_2 centroidCurrentCell = currentCell->getCentroid();
	Point_2 centroidNextCell = nextCell->getCentroid();
	/* Direction legend: -1:No direction 1: Up	2: Left	3: Down	4:Right*/
	/*if (centroidCurrentCell.x().approx().inf() < centroidNextCell.x().approx().inf()) {
		return 4;
	} else if(centroidCurrentCell.x().approx().inf() > centroidNextCell.x().approx().inf()){
		return 2;
	} else if (centroidCurrentCell.y().approx().inf() < centroidNextCell.y().approx().inf()) {
		return 1;
	} else if (centroidCurrentCell.y().approx().inf() > centroidNextCell.y().approx().inf()) {
		return 3;
	}*/
	if (centroidCurrentCell.x() < centroidNextCell.x()) {
		return 4;
	}
	else if (centroidCurrentCell.x() > centroidNextCell.x()) {
		return 2;
	}
	else if (centroidCurrentCell.y() < centroidNextCell.y()) {
		return 1;
	}
	else if (centroidCurrentCell.y() > centroidNextCell.y()) {
		return 3;
	}
}

void NavigationEnvironment::searchObstacleCells(vector<Cell*>& cellsObstacle, Cell* currentCell) {
	if (currentCell != NULL && currentCell->getBinaryGridValue() == 1 && currentCell->isAddedToObstacle() != true) {
		cellsObstacle.push_back(currentCell);
		currentCell->setAddedToObstacle(true);
		int cellX = currentCell->getColumnIndex();
		int cellY = currentCell->getRowIndex();

		//Primeira coluna da vizinhança de Moore
		Cell* nextCell = getCellAtCoordinateIndex(cellY - 1, cellX - 1);
		searchObstacleCells(cellsObstacle, nextCell);
		nextCell = getCellAtCoordinateIndex(cellY, cellX - 1);
		searchObstacleCells(cellsObstacle, nextCell);
		nextCell = getCellAtCoordinateIndex(cellY + 1, cellX - 1);
		searchObstacleCells(cellsObstacle, nextCell);

		//Segunda coluna da vizinhança de Moore
		nextCell = getCellAtCoordinateIndex(cellY - 1, cellX);
		searchObstacleCells(cellsObstacle, nextCell);
		nextCell = getCellAtCoordinateIndex(cellY, cellX);
		searchObstacleCells(cellsObstacle, nextCell);
		nextCell = getCellAtCoordinateIndex(cellY + 1, cellX);
		searchObstacleCells(cellsObstacle, nextCell);

		//Terceira coluna da vizinhança de Moore
		nextCell = getCellAtCoordinateIndex(cellY - 1, cellX + 1);
		searchObstacleCells(cellsObstacle, nextCell);
		nextCell = getCellAtCoordinateIndex(cellY, cellX + 1);
		searchObstacleCells(cellsObstacle, nextCell);
		nextCell = getCellAtCoordinateIndex(cellY + 1, cellX + 1);
		searchObstacleCells(cellsObstacle, nextCell);
	}
	return;
}

Cell* NavigationEnvironment::getCellAtCoordinateIndex(int row, int column){
	if (row < 0 || column < 0 || row >= this->cellRows || column >= this->cellColumns) {
		return nullptr;
	}

	return this->cells[row*this->cellColumns + column];
}

Cell * NavigationEnvironment::getCellByXYCoordinate(double x, double y){
	if (x < this->initialCoordinate->getX() || y < this->initialCoordinate->getY() || 
		x > this->finalCoordinate->getX() || y > this->finalCoordinate->getY()) {
		return NULL;
	}

	double intervalMaxLong = this->finalCoordinate->getX() - this->initialCoordinate->getX();
	double stepLong = intervalMaxLong / this->cellColumns;
	double intervalLong = x - this->initialCoordinate->getX();
	int column = floor(intervalLong / stepLong);
	//int column = floor((fabs(x) + fabs(this->initialCoordinate->getX())) / ((this->finalCoordinate->getX() - this->initialCoordinate->getX()) / this->cellColumns));
	if (x == (this->finalCoordinate->getX() - this->initialCoordinate->getX())) {
		column--;
	}

	double intervalMaxLat = this->finalCoordinate->getY() - this->initialCoordinate->getY();
	double stepLat = intervalMaxLat / this->cellRows;
	double intervalLat = y - this->initialCoordinate->getY();
	int row = floor(intervalLat / stepLat);

	//int row = this->cellColumns - floor((fabs(y) + fabs(this->initialCoordinate->getY()))/((this->finalCoordinate->getY() - this->initialCoordinate->getY())/this->cellRows)) - 1;
	if (y == (this->finalCoordinate->getY() - this->initialCoordinate->getY())) {
		row = 0;
	}	

	return this->cells[row*this->cellColumns + column];
}

void NavigationEnvironment::initializeCoordinates(double initialLongitude, double initialLatitude,
												  double finalLongitude, double finalLatitude) {
	////TODO: especificar direito todas as possibilidades de coordenadas.
	//if ((initialLatitude < 0) || (finalLatitude < 0) || (initialLongitude < 0) || (initialLongitude < 0)) {
	//	this->initialCoordinate = new Node{ 0, 0 };
	//	this->finalCoordinate = new Node{ 1000, 1000 };
	//} else {
	//	if (initialLatitude > finalLatitude) {
	//		//TODO:Mudar depois a classe do método change
	//		MathUtils::change(&initialLatitude, &finalLatitude);
	//	}

	//	if (initialLongitude > finalLongitude) {
	//		MathUtils::change(&initialLongitude, &finalLongitude);
	//	}
	//	this->initialCoordinate = new Node{ initialLatitude, initialLongitude };
	//	this->finalCoordinate = new Node{ finalLatitude, finalLongitude };
	//}

	this->initialCoordinate = new Node{ initialLongitude, initialLatitude};
	this->finalCoordinate = new Node{ finalLongitude, finalLatitude};
}

void NavigationEnvironment::configureRandomGenerators() {
	//this->generator = randomGenerator(static_cast<unsigned int>(time(0)));
	this->generator.seed(this->seed);
	if (this->initialCoordinate->getX() >= 0 && this->finalCoordinate->getX() >= 0 &&
		this->initialCoordinate->getY() >= 0 && this->finalCoordinate->getY() >= 0) {
		this->genX = boost::random::uniform_real_distribution<>(this->initialCoordinate->getX(), this->finalCoordinate->getX());
		this->genY = boost::random::uniform_real_distribution<>(this->initialCoordinate->getY(), this->finalCoordinate->getY());
	} else {
		this->genX = boost::random::uniform_real_distribution<>();
		this->genY = boost::random::uniform_real_distribution<>();
	}
}

Node* NavigationEnvironment::generateNode() {
	// ALGORITMO RRT - LINHA 4: q_rand recebe RAND_CONFIG(C)
	Node* qRand;
	//std::uniform_real_distribution<> distributionX(this->initialCoordinate->getX(), this->finalCoordinate->getX());
	//std::uniform_real_distribution<> distributionY(this->initialCoordinate->getY(), this->finalCoordinate->getY());
	//this->genX = boost::random::uniform_real_distribution<>(this->initialCoordinate->getX(), this->finalCoordinate->getX());
	//this->genY = boost::random::uniform_real_distribution<>(this->initialCoordinate->getY(), this->finalCoordinate->getY());

	double limite_superior_latitude = this->finalCoordinate->getX();  
	double limite_superior_longitude = this->finalCoordinate->getY(); 
	double limite_inferior_latitude = this->initialCoordinate->getX();
	double limite_inferior_longitude = this->initialCoordinate->getY();

	/*double randX = this->genY(this->generator);
	double randY = this->genY(this->generator);
*/
	double intervalo = (rand() % (int)(100 * (limite_superior_latitude - limite_inferior_latitude) + 1)) / 100.0;
	double latitude = limite_inferior_latitude + intervalo;

	intervalo = (rand() % (int)(100 * (limite_superior_longitude - limite_inferior_longitude) + 1)) / 100.0;
	double longitude = limite_inferior_longitude + intervalo;

	if (limite_superior_latitude < 0 || limite_superior_longitude < 0 || limite_inferior_latitude < 0 || limite_inferior_longitude < 0)
		qRand = new Node{ latitude, longitude };
	else
		qRand = new Node{ this->genX(this->generator), this->genY(this->generator) };

	return qRand;
}

vector<Cell*> NavigationEnvironment::getGrid(){
	return this->cells;
}

vector<StaticObstacle*> NavigationEnvironment::getStaticObstacles(){
	return this->staticObstacles;
}

Node* NavigationEnvironment::getInitialCoordinate(){
    return initialCoordinate;
}

Node* NavigationEnvironment::getFinalCoordinate(){
    return finalCoordinate;
}

void NavigationEnvironment::putNodeInsideLimits(Node* node){
       if(node->getNodeWaypoint()->getLatitude() < this->initialCoordinate->getNodeWaypoint()->getLatitude()){
              node->getNodeWaypoint()->setLatitude(this->initialCoordinate->getNodeWaypoint()->getLatitude());
       }
       
       if(node->getNodeWaypoint()->getLatitude() > this->finalCoordinate->getNodeWaypoint()->getLatitude()){
              node->getNodeWaypoint()->setLatitude(this->finalCoordinate->getNodeWaypoint()->getLatitude());
       }
       
       if(node->getNodeWaypoint()->getLongitude() < this->initialCoordinate->getNodeWaypoint()->getLongitude()){
              node->getNodeWaypoint()->setLongitude(this->initialCoordinate->getNodeWaypoint()->getLongitude());
       }
       
        if(node->getNodeWaypoint()->getLongitude() > this->finalCoordinate->getNodeWaypoint()->getLongitude()){
              node->getNodeWaypoint()->setLongitude(this->finalCoordinate->getNodeWaypoint()->getLongitude());
       }
}

void NavigationEnvironment::buildGrid(int k, GridEnum gridType){
	this->sukharevGrid = SukharevGrid(this->initialCoordinate, this->finalCoordinate, 0, 0);
	if (gridType == GridEnum::SUKHAREV_GRID) {
		this->sukharevGrid.build(k);
		this->gridType = GridEnum::SUKHAREV_GRID;
	} else if (gridType == GridEnum::LATTICE_GRID) {
		this->sukharevGrid.buildLattice(k);
		this->gridType = GridEnum::LATTICE_GRID;
	}
}

SukharevGrid NavigationEnvironment::getDispersionGrid(){
	return this->sukharevGrid;
}

int NavigationEnvironment::getSeedToRandomGenerator(){
	return this->seed;
}

Cell* NavigationEnvironment::getCellByIndex(int index){
	return this->cells.at(index);
}

vector<Cell*> NavigationEnvironment::getNeigborhoodCell(Cell* cell) {
	vector<Cell*> neighborhood;

	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++) {
			Cell* cellToAdd = this->getCellAtCoordinateIndex(cell->getRowIndex() + i, cell->getColumnIndex() + j);
			if (cellToAdd != NULL) {
				neighborhood.push_back(cellToAdd);
			}
			cellToAdd->~Cell();
		}
	}

	return neighborhood;
}

vector<Cell*> NavigationEnvironment::getNeigborhoodObstacleCell(Cell* cell) {
	vector<Cell*> neighborhood;

	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++) {			
			Cell* cellToAdd = this->getCellAtCoordinateIndex(cell->getRowIndex() + i, cell->getColumnIndex() + j);
			if (cellToAdd != NULL && cellToAdd->getBinaryGridValue() == 1) {
				neighborhood.push_back(cellToAdd);
			}
			cellToAdd->~Cell();
		}
	}

	return neighborhood;
}

int NavigationEnvironment::getQtdRows() {
	return this->cellRows;
}

int NavigationEnvironment::getQtdColumns(){
	return this->cellColumns;
}