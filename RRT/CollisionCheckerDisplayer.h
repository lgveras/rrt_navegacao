#pragma once

#include "Graphics.h"
#include "Displayable.h"
#include "BialkowskiCollisionChecker.h"

class CollisionCheckerDisplayer: public Displayable{
public:
	CollisionCheckerDisplayer(BialkowskiCollisionChecker checker);
	~CollisionCheckerDisplayer();
	void display(Graphics* graphic);
private:
	Graphics* graphics;
	BialkowskiCollisionChecker checker;
	void displayCollisionFree();
	void displayCollisionObs();
};

