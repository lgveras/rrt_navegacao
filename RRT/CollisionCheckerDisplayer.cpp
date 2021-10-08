#include "CollisionCheckerDisplayer.h"

CollisionCheckerDisplayer::CollisionCheckerDisplayer(BialkowskiCollisionChecker checker){
	this->checker = checker;
}

CollisionCheckerDisplayer::~CollisionCheckerDisplayer(){
}

void CollisionCheckerDisplayer::display(Graphics* graphic) {
	this->graphics = graphic;
	this->displayCollisionFree();
	this->displayCollisionObs();
}

void CollisionCheckerDisplayer::displayCollisionFree() {
	map<Node*, double, NodePointerCompare> freeSetPoints = this->checker.getCollisionFreeSet();
	for (map<Node*, double>::iterator it = freeSetPoints.begin(); it != freeSetPoints.end(); ++it) {
		float color[] = { 0.0, 0.0, 1.0 };
		this->graphics->drawCircle(it->first->getX(), it->first->getY(), it->second, color, 1);
	}

	//for (int i = 0; i < freeSetPoints.size(); i++) {
	//	//Point_2 point = { freeSetPoints[i]->getSample()->getX(), freeSetPoints[i]->getSample()->getY()}
	//	this->graphics->drawCircle(freeSetPoints[i]->getSample()->getX(), freeSetPoints[i]->getSample()->getY(),
	//		freeSetPoints[i]->getDistance(), 0.0, 0.0, 1.0);
	//}
}

void CollisionCheckerDisplayer::displayCollisionObs() {
	map<Node*, double, NodePointerCompare> obsSetPoints = this->checker.getCollisionObsSet();
	for (map<Node*, double>::iterator it = obsSetPoints.begin(); it != obsSetPoints.end(); ++it) {
		float color[] = { 1.0, 0.0, 0.0 };
		this->graphics->drawCircle(it->first->getX(), it->first->getY(),
			it->second, color, 1);
	}
	//for (int i = 0; i < obsSetPoints.size(); i++) {
	//	//Point_2 point = { freeSetPoints[i]->getSample()->getX(), freeSetPoints[i]->getSample()->getY()}
	//	this->graphics->drawCircle(obsSetPoints[i]->getSample()->getX(), obsSetPoints[i]->getSample()->getY(),
	//		obsSetPoints[i]->getDistance(), 1.0, 0.0, 0.0);
	//}
}
