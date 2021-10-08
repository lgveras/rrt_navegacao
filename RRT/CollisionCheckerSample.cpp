#include "CollisionCheckerSample.h"

CollisionCheckerSample::CollisionCheckerSample(Node * sample, double distance):
	sample(sample),distance(distance){
}

CollisionCheckerSample::~CollisionCheckerSample(){
}

double CollisionCheckerSample::getDistance(){
	return this->distance;
}

Node * CollisionCheckerSample::getSample(){
	Node* nodeNotConstant = new Node{ this->sample->getX(), this->sample->getY()};
	return nodeNotConstant;
}