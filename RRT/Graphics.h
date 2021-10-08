/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Graphics.h
 * Author: LuizGustavo
 *
 * Created on 30 de Agosto de 2016, 10:06
 */

#ifndef GRAPHICS_H
#define GRAPHICS_H

//#include <CGAL/Polygon_2.h>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include "Waypoint.h"
#include "Displayable.h"
#include <vector>
#include "CGALDefinitions.h"

//typedef CGAL::Exact_predicates_exact_constructions_kernel K;
//typedef CGAL::Polygon_2<K> Polygon_2;
//typedef K::Point_2  Point_2;
//typedef Polygon_2::Vertex_iterator VertexIterator;

using namespace std;

class Displayable;

class Graphics {
public:
    static const double PI;
    static const float OPEN_GL_INITIAL_X;
    static const float OPEN_GL_DISTANCE_X;
    static const float OPEN_GL_INITIAL_Y;
    static const float OPEN_GL_DISTANCE_Y; 

    Graphics();
	Graphics(bool interruptLoop);
    Graphics(const Graphics& orig);
	Graphics(const Graphics& orig, bool interruptLoop);
    virtual ~Graphics();
	void init(int argc, char** argv, Waypoint* neInitCoordination, Waypoint* neEndCoordination, string fileNameBMP);
    void drawLine(float x1, float y1, float x2, float y2, float color[3], int width);
    void drawPoint(float x, float y, float color[3], int width);
    void drawCircle(double x, double y, double radius, float color[3], int width);
	void drawUnfilledCircle(double x, double y, double radius, float color[3], int width);
    void drawArc(double cx, double cy, double r, double start_angle, 
                 double arc_angle, int num_segments);
	void drawUnfilledPolygon(Polygon_2 polygon, float color[3]);
	void drawPolygon(Polygon_2 polygon);
	void registerDisplayable(Displayable* displayable);
	void startGraphics();
	void drawNumber(int number, double x, double y, double size);
	void saveToFile(const char *fileName);
	string getFileNameBMP();
	void showGraphicsAgain();
private:
	Waypoint neInitialCoordinate;
	Waypoint neFinalCoordinate;
	vector<Displayable*> listDisplayables;
	float convertToOpenGLCoordinateX(float x);
	float convertToOpenGLCoordinateY(float y);
	float convertToOpenGLDistance(float distance);
	void drawPolygonVertices(Polygon_2 polygon);
	void loopGraphics(void(*displayer)());
	string fileNameBMP;
	bool interruptLoop;
	static Graphics* instance;
	static void showDisplayables();
	static int plotted;
};

#endif /* GRAPHICS_H */