#pragma once

#include "Graphics.h"
#include "Displayable.h"
#include "G2PlanarPH.h"

class PHDisplayer :	public Displayable{
	G2PlanarPH curve[8];
	Graphics* graphics;
	CPointPH points[8][3];
	CPointPH shaperPoints[8][3];
public:
	PHDisplayer(G2PlanarPH curve[], CPointPH points[][3], CPointPH shaperPoints[][3]);
	~PHDisplayer();
	void display(Graphics* graphic);
};

