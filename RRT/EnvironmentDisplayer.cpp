#include "EnvironmentDisplayer.h"

EnvironmentDisplayer::EnvironmentDisplayer(NavigationEnvironment& ne){
	this->navigationSpace = &ne;
}

EnvironmentDisplayer::~EnvironmentDisplayer(){
}

void EnvironmentDisplayer::displayGrid() {
	vector<Cell*>::iterator gridIt;
	vector<Cell*> grid = this->navigationSpace->getGrid();
	float colorGrid[3] = { 1,1,1 };
	for (gridIt = grid.begin(); gridIt < grid.end(); gridIt++) {
		Cell* cell = *gridIt;
		this->graphics->drawUnfilledPolygon((*cell->getShape()), colorGrid);
	}
}

void EnvironmentDisplayer::displaySukharevCentroids() {
	Node* current = this->navigationSpace->getDispersionGrid().getCentroidByIndex(0);
	int index = 1;
	float color[3] = { 0.5 , 0.2 ,0 };
	while (current) {
		this->graphics->drawPoint(current->getX(), current->getY(), color, 8);
		current = this->navigationSpace->getDispersionGrid().getCentroidByIndex(index);
		index++;
	}
}

//TODO:Isolar displayStaticObjects em um método a parte
void EnvironmentDisplayer::displayStaticObjects() {
	vector<StaticObstacle*>::iterator soIt;
	vector<StaticObstacle*> obstacles = this->navigationSpace->getStaticObstacles();
	float colorOffset[3] = { 0 , 1 ,0 };
	float colorShape[3] = { 0 , 0 ,0 };

	for (soIt = obstacles.begin(); soIt < obstacles.end(); soIt++) {
		StaticObstacle* currentObstacle = *soIt;
		vector<Cell*> obstacles = currentObstacle->getObstacleCells();

		if(obstacles.size() <= 0){
			//this->graphics->drawPolygon(*(currentObstacle->getShape()));
			this->graphics->drawUnfilledPolygon(*(currentObstacle->getShape()),colorShape);
		} else {
			for (int i = 0; i < obstacles.size(); i++) {
				this->graphics->drawPolygon(*obstacles[i]->getShape());
			}
		}

		if (currentObstacle->getOffSet()) {
			this->graphics->drawUnfilledPolygon(*(currentObstacle->getOffSet()), colorOffset);
		}		
	}
}

void EnvironmentDisplayer::display(Graphics* graphic) {
	this->graphics = graphic;
	//this->displayGrid();
	this->displayStaticObjects();
	this->displaySukharevCentroids();
}