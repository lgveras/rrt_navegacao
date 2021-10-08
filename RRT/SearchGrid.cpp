#include "SearchGrid.h"
#include <iostream>
#include "MathUtils.h"

SearchGrid::SearchGrid(double initialLatitude, double initialLongitude,
					   double finalLatitude, double finalLongitude, int qtdColumns, int qtdRows):
					   initialLatitude(initialLatitude), initialLongitude(initialLongitude),
					   finalLatitude(finalLatitude), finalLongitude(finalLongitude), qtdColumns(qtdColumns), qtdRows(qtdRows){
	//TODO:Checar se as coordenadas est�o coerentes.
	this->build();
}

SearchGrid::~SearchGrid(){
}

void SearchGrid::configure(double initialLatitude, double initialLongitude,
	double finalLatitude, double finalLongitude, int qtdColumns, int qtdRows){
	this->initialLatitude = initialLatitude;
	this->initialLongitude = initialLongitude;
	this->finalLatitude = finalLatitude;
	this->finalLongitude = finalLongitude;
	this->qtdColumns = qtdColumns;
	this->qtdRows = qtdRows;
}

void SearchGrid::build() {
	if (cells.size() == 0) {
		int count = qtdColumns*qtdRows;
		while (count != 0) {
			this->cells.push_back(new CellGrid());
			count--;
		}
	} else {
		std::cout << "Grade de busca j� constru�da!\n";
	}
}

void SearchGrid::insertNode(Node* node) {	
	int index = this->getIndexCell(node);
	this->getCellByIndex(index)->insert(node);
}

Node* SearchGrid::searchNearestTo(Node* node){
	int index = getIndexCell(node);
	bool founded = false;
	Node* nearestNode;
	/*TODO:Existe um erro aqui. Mesmo que exista um ponto na mesma c�lula que o n� de bucsa, n�o siginifica que esse
	� o n� mais pr�ximo. Resolver esse problema fazendo uma busca na vizinhan�a
	UPDATE: Erro corrigido,  ao inv�s de ir aumentando a vizinhan�a de busca, o algoritmo busca no NE inteiro. Melhorar depois.*/
	nearestNode = getCellByIndex(index)->searchNearest(node);
	if (!nearestNode) {
		double distance;
		double lesserDistance = numeric_limits<double>::max();
		Node* currentNode;
		for (int i = 0; i < this->cells.size(); i++) {
			currentNode = this->cells[i]->searchNearest(node);
			if (currentNode) {
				distance = MathUtils::euclidianDistanceNode(currentNode, node);
				if (distance < lesserDistance) {
					lesserDistance = distance;
					nearestNode = currentNode;
				}
			}
		}
	}
	return nearestNode;
}

CellGrid* SearchGrid::getCellByIndex(int index) {
	return this->cells[index];
}

int SearchGrid::getIndexCell(Node* node) {

	double incrementLat = fabs(this->finalLatitude - this->initialLatitude) / this->qtdColumns;
	int column = floor(fabs(node->getX() - this->initialLatitude) / incrementLat);

	double incrementLong = fabs(this->finalLongitude - this->initialLongitude) / this->qtdRows;
	int row = floor(fabs(node->getY() - this->initialLongitude) / incrementLong);

	//int column = floor(fabs(node->getX()) / (fabs(this->finalLatitude - this->initialLatitude) / this->qtdColumns));


	//if (fabs(node->getX()) == fabs(this->finalLatitude - this->initialLatitude)) {
	//	column--;
	//}

	//int row = this->qtdRows - floor(fabs(node->getY()) / (fabs(this->finalLongitude - this->initialLongitude) / this->qtdRows))-1;
	//
	//if (fabs(node->getY()) == fabs(this->finalLongitude - this->initialLongitude)) {
	//	row = 0;
	//}

	return row*this->qtdColumns + column;
}

vector<CellGrid*> SearchGrid::getCells() {
	return this->cells;
}