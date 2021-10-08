#pragma once
#include "Node.h"

class CollisionCheckerSample{
public:
	CollisionCheckerSample(Node* sample, double distance);
	~CollisionCheckerSample();
	double getDistance();
	Node* getSample();
private:
	const double distance = 0;
	const Node* sample;
};

