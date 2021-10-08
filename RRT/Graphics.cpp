/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Graphics.cpp
 * Author: LuizGustavo
 * 
 * Created on 30 de Agosto de 2016, 10:06
 */

#include "Graphics.h"
#include "RRTDisplayer.h"
#include "Displayable.h"
#include <glut.h>
#include <freeglut.h>
#include <GL/gl.h>
#include <math.h>
#include <windows.h>
#include <WinGDI.h>
#include <pngwriter.h>

const double Graphics::PI = 3.141592653589793238463;
const float Graphics::OPEN_GL_INITIAL_X = -1;
const float Graphics::OPEN_GL_DISTANCE_X= 2;
const float Graphics::OPEN_GL_INITIAL_Y = -1;
const float Graphics::OPEN_GL_DISTANCE_Y = 2;  

Graphics* Graphics::instance = NULL;
int Graphics::plotted=1;

//TODO:Remodelar essa classe. Analisar...
Graphics::Graphics() {
	this->interruptLoop = true;
}

Graphics::Graphics(bool interruptLoop) :interruptLoop(interruptLoop) {
}

Graphics::Graphics(const Graphics& orig) {
	this->interruptLoop = true;
}

Graphics::Graphics(const Graphics & orig, bool interruptLoop):interruptLoop(interruptLoop){
}

Graphics::~Graphics() {
}

void Graphics::init(int argc, char** argv, Waypoint* neInitCoordination, Waypoint* neEndCoordination, string fileNameBMP){
	this->neInitialCoordinate = *neInitCoordination;
	this->neFinalCoordinate = *neEndCoordination;
	this->fileNameBMP = fileNameBMP;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(1000, 1000);
	glutInitWindowPosition(100, 100);
    glutCreateWindow("INPE: Ambiente de navegação - RRT");
	instance = this;
}

void Graphics::drawLine(float x1, float y1, float x2, float y2, float color[3], int width) {
	x1 = convertToOpenGLCoordinateX(x1);
	y1 = convertToOpenGLCoordinateY(y1);
	x2 = convertToOpenGLCoordinateX(x2);
	y2 = convertToOpenGLCoordinateY(y2);
	glColor3f(color[0], color[1], color[2]);
    glLineWidth(width);
    glBegin (GL_LINES);
        glVertex2f(x1, y1);
        glVertex2f(x2, y2);
    glEnd();
    glFlush();
}

void Graphics::drawPoint(float x, float y, float color[3], int width){
	x = convertToOpenGLCoordinateX(x);
	y = convertToOpenGLCoordinateY(y);
	glColor3f(color[0], color[1], color[2]);
    glPointSize(width);
    glBegin(GL_POINTS);
        glVertex2d(x, y);
    glEnd();  
    glFlush();
}

void Graphics::drawCircle(double x, double y, double radius, float color[3], int width){
	x = convertToOpenGLCoordinateX(x);
	y = convertToOpenGLCoordinateY(y);
	radius = convertToOpenGLDistance(radius);
	glColor3f(color[0], color[1], color[2]);
	glLineWidth(width);
    glBegin(GL_POLYGON);
      for(float i=0;i<100;i+=0.01)      {
         float cosine = radius*cos(i*2*PI);
         float sine = radius*sin(i*2*PI);
         glVertex2f(x+cosine,y+sine);
      }
    glEnd();
    glFlush();
}

void Graphics::drawUnfilledCircle(double x, double y, double radius, float color[3], int width) {
	x = convertToOpenGLCoordinateX(x);
	y = convertToOpenGLCoordinateY(y);
	radius = convertToOpenGLDistance(radius);
	glColor3f(color[0], color[1], color[2]);
	glLineWidth(width);
	glBegin(GL_LINE_LOOP);
	for (float i = 0; i<100; i += 0.01) {
		float cosine = radius*cos(i * 2 * PI);
		float sine = radius*sin(i * 2 * PI);
		glVertex2f(x + cosine, y + sine);
	}
	glEnd();
	glFlush();
}

void Graphics::loopGraphics(void (*displayer)()){
	glutDisplayFunc(displayer);    
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    glutMainLoop();
	//glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
}

void Graphics::showGraphicsAgain() {
	glutMainLoop();
}

void Graphics::startGraphics(){
	this->loopGraphics(this->showDisplayables);
}

//Desenha um polígono sem preenchimento
void Graphics::drawUnfilledPolygon(Polygon_2 polygon, float color[3]){
	//glColor3f(0.2, 0.3, 0.5);
	glLineWidth(5);
	glColor3f(color[0], color[1], color[2]);
	glBegin(GL_LINE_LOOP);
		drawPolygonVertices(polygon);
	glEnd();
	glFlush();
}

//Desenha um polígono com preenchimento
void Graphics::drawPolygon(Polygon_2 polygon) {
	glColor3f(0.2, 0.3, 0.5);
	glBegin(GL_POLYGON);
		drawPolygonVertices(polygon);
	glEnd();
	glFlush();
}

//Desenha um polígono com preenchimento
void Graphics::drawNumber(int number, double x, double y, double size) {
	glColor3f(1, 0, 0);
	//x = convertToOpenGLCoordinateX(x);
	y = convertToOpenGLCoordinateY(y);
	double space = 13;
	int j = 0;
	int k = 0;

	while (number > 9) {
		k = number % 10;
		glRasterPos2f(convertToOpenGLCoordinateX(x - (j*space)), y);
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, 48 + k);
		j++;
		number /= 10;
	}

	//glRasterPos2f((x - (j*space_char)), y);
	glRasterPos2f(convertToOpenGLCoordinateX(x - (j*space)), y);
	glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, 48 + number);
}

void Graphics::saveToFile(const char *fileName){
	/*FILE *file;

	GLint viewPort[4];

	glGetIntegerv(GL_VIEWPORT, viewPort);
	glReadBuffer(GL_FRONT);
	size_t size = 4 * viewPort[2] * viewPort[3];
	GLbyte *pixels = (GLbyte*)malloc(4 * viewPort[2]* viewPort[3]);
	glReadPixels(0, 0, viewPort[2], viewPort[3], GL_BGR_EXT, GL_UNSIGNED_BYTE, pixels);

	pngwriter png(viewPort[2], viewPort[3], 0, fileName);*/

	/*size_t x = 1;
	size_t y = 1;
	double R, G, B;
	for (size_t i = 0; i < size; i++) {
		switch (i % 3) {
		case 2:B = (double)pixels[i];
			break;
		case 1:G = (double)pixels[i];
			break;
		case 0:R = (double)pixels[i];
			png.plot(x, y, R, G, B);
			if (x == viewPort[2]) {
				x = 1;
				y++;
			}
			else { x++; }
			break;
		}
	}*/

	//png.close();

	FILE *file;
	unsigned long imageSize;
	GLbyte *data = NULL;
	GLint viewPort[4];
	GLenum lastBuffer;
	BITMAPFILEHEADER bmfh;
	BITMAPINFOHEADER bmih;

	bmfh.bfType = 'MB';
	bmfh.bfReserved1 = 0;
	bmfh.bfReserved2 = 0;
	bmfh.bfOffBits = 54;
	glGetIntegerv(GL_VIEWPORT, viewPort);
	imageSize = ((viewPort[2] + ((4 - (viewPort[2] % 4)) % 4))*viewPort[3] * 3) + 2;
	bmfh.bfSize = imageSize + sizeof(bmfh) + sizeof(bmih);
	data = (GLbyte*)malloc(imageSize);
	glPixelStorei(GL_PACK_ALIGNMENT, 4);
	glPixelStorei(GL_PACK_ROW_LENGTH, 0);
	glPixelStorei(GL_PACK_SKIP_ROWS, 0);
	glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
	glPixelStorei(GL_PACK_SWAP_BYTES, 1);
	glGetIntegerv(GL_READ_BUFFER, (GLint*)&lastBuffer);
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, viewPort[2], viewPort[3], GL_BGR_EXT, GL_UNSIGNED_BYTE, data);
	data[imageSize - 1] = 0;
	data[imageSize - 2] = 0;
	glReadBuffer(lastBuffer);
	
	bmih.biSize = 40;
	bmih.biWidth = viewPort[2];
	bmih.biHeight = viewPort[3];
	bmih.biPlanes = 1;
	bmih.biBitCount = 24;
	bmih.biCompression = 0;
	bmih.biSizeImage = imageSize;
	bmih.biXPelsPerMeter = 45089;
	bmih.biYPelsPerMeter = 45089;
	bmih.biClrUsed = 0;
	bmih.biClrImportant = 0;

	file = fopen(fileName, "wb");
	fwrite(&bmfh, sizeof(bmfh), 1, file);
	fwrite(&bmih, sizeof(bmih), 1, file);
	fwrite(data, imageSize, 1, file);
	free(data);
	fclose(file);
}

void Graphics::registerDisplayable(Displayable* displayable){
	this->listDisplayables.push_back(displayable);
}

void Graphics::drawPolygonVertices(Polygon_2 polygon) {
	VertexIterator verticesIt;

	//for (verticesIt = polygon.vertices_begin(); verticesIt < polygon.vertices_end(); verticesIt++) {
	for (verticesIt = polygon.vertices_begin(); verticesIt <  polygon.vertices_end(); verticesIt++) {
		Point_2 vertice = *verticesIt;
		/*double x = convertToOpenGLCoordinateX(vertice.x().approx().inf());
		double y = convertToOpenGLCoordinateY(vertice.y().approx().inf());*/
		double x = convertToOpenGLCoordinateX(vertice.x());
		double y = convertToOpenGLCoordinateY(vertice.y()); 
		glVertex2f(x, y);
		//cout << "#1: x = " << vertice.x().approx().inf() << "	y = " << " " << vertice.y().approx().inf() << endl;
	}
	//EdgeIterator edgeIt;
	//for (edgeIt = polygon.edges_begin(); edgeIt < polygon.edges_end(); edgeIt++) {
	//	Segment_2 edge = *edgeIt;
	//	/*double x = convertToOpenGLCoordinateX(vertice.x().approx().inf());
	//	double y = convertToOpenGLCoordinateY(vertice.y().approx().inf());*/
	//	double x = convertToOpenGLCoordinateX(edge.start().x());
	//	double y = convertToOpenGLCoordinateY(edge.start().y());
	//	glVertex2f(x, y);
	//	x = convertToOpenGLCoordinateX(edge.end().x());
	//	y = convertToOpenGLCoordinateY(edge.end().y());
	//	glVertex2f(x, y);
	//	//cout << "#1: x = " << vertice.x().approx().inf() << "	y = " << " " << vertice.y().approx().inf() << endl;
	//}
}

void Graphics::showDisplayables() {
	//if (plotted) {
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	for (int i = 0; i < instance->listDisplayables.size(); i++) {
		instance->listDisplayables[i]->display(instance);
	}
	glutSwapBuffers();
	//plotted = 0;
	if (instance->getFileNameBMP().size()!=0) {
		instance->saveToFile(instance->getFileNameBMP().c_str());
	}

	if (instance->interruptLoop) {
		glutLeaveMainLoop();
	}
	//}
}

string Graphics::getFileNameBMP() {
	return this->fileNameBMP;
}

void Graphics::drawArc(double cx, double cy, double r, 
                       double start_angle, double arc_angle, int num_segments){ 
    double theta;
    double tangetial_factor;
    double radial_factor;
    double x;
    double y;
    
    //theta is now calculated from the arc angle instead, the - 1 bit comes 
    //from the fact that the arc is open
    theta = arc_angle / double(num_segments - 1);   

    tangetial_factor = tan(theta);

    radial_factor = cos(theta);
    
    //We now start at the start angle
    x = r * cos(start_angle);
    y = r * sin(start_angle);

    //since the arc is not a closed curve, this is a strip now
    glBegin(GL_LINE_STRIP);
    for(int ii = 0; ii < num_segments; ii++){ 
            glVertex2f(x + cx, y + cy);

            float tx = -y; 
            float ty = x; 

            x += tx * tangetial_factor; 
            y += ty * tangetial_factor; 

            x *= radial_factor; 
            y *= radial_factor; 
    } 
    glEnd(); 
}

float Graphics::convertToOpenGLCoordinateX(float x) {
	float intervalX = (this->neFinalCoordinate.getLatitude() - this->neInitialCoordinate.getLatitude());
	float percentageX = (x - this->neInitialCoordinate.getLatitude()) / intervalX;
	return OPEN_GL_INITIAL_X + (OPEN_GL_DISTANCE_X*percentageX);
}

float Graphics::convertToOpenGLCoordinateY(float y) {
	float intervalY = (this->neFinalCoordinate.getLongitude() - this->neInitialCoordinate.getLongitude());
	float percentageY = (y - this->neInitialCoordinate.getLongitude())/ intervalY;
	return OPEN_GL_INITIAL_Y + (OPEN_GL_DISTANCE_Y*percentageY);
}

float Graphics::convertToOpenGLDistance(float distance) {
	float distanceNe = MathUtils::euclidianDistance(neInitialCoordinate.getLatitude(), neInitialCoordinate.getLongitude(), 
													neFinalCoordinate.getLatitude(), neFinalCoordinate.getLongitude());
	float distanceOpenGL = MathUtils::euclidianDistance(OPEN_GL_INITIAL_X, OPEN_GL_INITIAL_Y, OPEN_GL_INITIAL_X + OPEN_GL_DISTANCE_X, OPEN_GL_INITIAL_Y + OPEN_GL_DISTANCE_Y);
	return (distanceOpenGL*distance) / distanceNe;
}