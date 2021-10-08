#pragma once

#include "NavigationEnvironment.h"
#include "Graphics.h"
#include "Displayable.h"

class EnvironmentDisplayer : public Displayable{
public:
	EnvironmentDisplayer(NavigationEnvironment& ne);
	~EnvironmentDisplayer();
	void display(Graphics* graphic);
private:
	Graphics* graphics;
	NavigationEnvironment* navigationSpace;
	void displayGrid();
	void displaySukharevCentroids();
	void displayStaticObjects();
};