#pragma once

#include <vector>
#include "CellGrid.h"

class SearchGrid{
public:
	SearchGrid(double initialLatitude, double initialLongitude,
			   double finalLatitude, double finalLongitude, int qtdColumns, int qtdRows);
	~SearchGrid();
	void configure(double initialLatitude, double initialLongitude, double finalLatitude, double finalLongitude, int qtdColumns, int qtdRows);
	void build();
	void insertNode(Node* node);
	Node* searchNearestTo( Node* node);
	vector<CellGrid*> getCells();
	CellGrid* getCellByIndex(int index);
private:
	vector<CellGrid*> cells;
	int qtdColumns;
	int qtdRows;
	double initialLatitude, finalLatitude, initialLongitude, finalLongitude;
	int getIndexCell(Node * node);
};

