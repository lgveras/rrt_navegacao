#pragma once

#include "Node.h";

class IRRT{
public:
	virtual ~IRRT();
	virtual void build(Node* qInit, Node *qGoal, double distanceToGenerateRoute) = 0;
	Node* getRoot();
private:
	virtual Node* createNewNode(Node* qNear, Node*qRand) = 0;

};