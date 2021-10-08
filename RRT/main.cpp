/**
 * Arquivo principal geração da RRT.
 */
    
/**
* Doutorado em Computação Aplicada - INPE
* Autor: Luiz Gustavo Diniz de Oliveira Véras (luiizgustavo@gmail.com)
* Filiação: Instituto Nacional de Pesquisas Espaciais - INPE
*			Instituto de Estudos Avançados - IEAv
*			Instituto Federal de Educação, Ciência e Tecnologia de São Paulo
* Created on 2 de Agosto de 2016, 15:25
*/

#include <cstdlib>

#include "RRTDisplayer.h"
#include "CollisionCheckerDisplayer.h"
#include "NavigationEnvironment.h"
#include "VisibilityGraph.h"
#include "EnvironmentDisplayer.h"
#include "RRTStar.h"
#include "RRTStarSukharevVertices.h"
#include "RRTStarSmart.h"
#include "RRTCurve.h"
#include "RRTGrid.h"
#include "InformedRRTStar.h"
#include "BITStar.h"
#include "BinaryGrid.h"
#include "ResultOutputer.h"
#include <string>
#include <CGAL/Simple_cartesian.h>
#include "StaticObstacle.h"
#include "CPointPH.h"
#include "GridEnum.h"
#include <G2PlanarPH.cpp>

using namespace std;

// LABIRINTO
//q_init->latitude = -22.99;
//q_init->longitude = coordenada_inicial.wpt_dLong + (coordenada_final.wpt_dLong - coordenada_inicial.wpt_dLong) / 2.0;//50.0;
//q_destino->latitude = -22.15;
//q_destino->longitude = -45.25;//-45.05;

//coordenada inicial do ambiente de navegação - 22.994583 - 45.99875
//coordenada final do ambiente de navegação - 22.000417 - 45.002917

//Entrada para as comparações com o código do Felipe
//"C:/RRTSimulations/navEnvs/matriz_binaria_1201x1201_elevacoes.txt" 3 "matriz_binaria_1201x1201_elevacoes" 2500 500 450 970 970 15
//"C:/RRTSimulations/navEnvs/matriz_binaria_1201x1201_elevacoes.txt" 3 "matriz_binaria_1201x1201_elevacoes" 1000 25 25 975 975 1

//Configurations INPUTs: "C:/RRTSimulations/navEnvs/binary_grid_40x40_spiral.txt" 2 "spiral" 1000 500 500 975 975 1
//Configurations INPUTs: "C:/RRTSimulations/navEnvs/binary_grid_20x20_zig_zag.txt" 2 "zig_zag" 1000 500 50 500 950 1
//Configurations INPUTs: "C:/RRTSimulations/navEnvs/binary_grid_140x140_cluttered2500Obstacles.txt" 2 "cluttered2500Obstacles" 7000 25 25 975 975 1
//Configurations INPUTs: "C:/RRTSimulations/navEnvs/binary_grid_40x40_cluttered50Obstacles.txt" 2 "cluttered50Obstacles" 1000 25 25 975 975 1
//"C:/RRTSimulations/navEnvs/binary_grid_60x60_cluttered100Obstacles.txt" 1 "cluttered100Obstacles" 3 25 25 975 975 1
//Configurations INPUTs: "C:/RRTSimulations/navEnvs/binary_grid_40x40_cluttered5Obstacles.txt" 2 "cluttered5Obstacles" 1000 25 25 975 975 1
//"C:/RRTSimulations/navEnvs/binary_grid_280x280_cluttered10000Obstacles.txt" 2 "cluttered10000Obstacles" 30000 25 25 970 970 1
//"C:/RRTSimulations/navEnvs/binary_grid_40x40_phcurve.txt" 7 "phcurve" 10000 50 50 3500 3500 15
void saveDataBySeed(string fileBySeedName, float* dataInit, float* dataEnd) {
	cout << "Generating data for statistics..." << endl;
	string columns[] = { "seed", "final_cost", "planning_time", "nodes", "collisions" };
	char fileSeparator = ';';
	int sizeOfData = 5 - 1;
	ResultOutputer* costsBySeedOutputer = new ResultOutputer{fileBySeedName, columns, columns + sizeOfData, fileSeparator};

	//Catch data from simulation and input to file
	costsBySeedOutputer->pushData( dataInit, dataEnd);
	cout << "Data generated!!" << endl;
}

vector<StaticObstacle*> createObstaclesRRTVisionExperiment() {
	//coordendas qinit = 120, 30; qgoal 750, 850
	vector<StaticObstacle*> obstacles;

	Polygon_2 shapeObs1;
	shapeObs1.push_back(Point_2(138.77746, 1000 - 959.48406));
	shapeObs1.push_back(Point_2(286.35098, 1000 - 975.5565));
	shapeObs1.push_back(Point_2(409.08535, 1000 - 905.42221));
	shapeObs1.push_back(Point_2(330.18469, 1000 - 765.15363));
	shapeObs1.push_back(Point_2(162.15543, 1000 - 789.99288));
	(shapeObs1.orientation() == CGAL::RIGHT_TURN) ? shapeObs1.reverse_orientation() : (void)true ;

	Polygon_2 shapeObs2;
	shapeObs2.push_back(Point_2(32.115412,	304.98061));
	shapeObs2.push_back(Point_2(56.954518,	258.22442));
	shapeObs2.push_back(Point_2(134.39409,  281.60253));
	shapeObs2.push_back(Point_2(124.16622,	325.43645));
	shapeObs2.push_back(Point_2(113.93835,	373.65376));
	shapeObs2.push_back(Point_2(74.487994,	347.35341));
	shapeObs2.push_back(Point_2(23.348661,	379.4983));
	(shapeObs2.orientation() == CGAL::RIGHT_TURN) ? shapeObs2.reverse_orientation() : (void)true;

	Polygon_2 shapeObs3;
	shapeObs3.push_back(Point_2(309.72895, 414.56541));
	shapeObs3.push_back(Point_2(413.46873, 290.36927));
	shapeObs3.push_back(Point_2(442.69124, 291.83042));
	shapeObs3.push_back(Point_2(426.61886, 418.94877));
	shapeObs3.push_back(Point_2(555.19776, 410.18201));
	shapeObs3.push_back(Point_2(616.56497, 342.96996));
	shapeObs3.push_back(Point_2(552.27551, 306.44172));
	shapeObs3.push_back(Point_2(473.37484, 348.81451));
	shapeObs3.push_back(Point_2(492.36945, 294.75266));
	shapeObs3.push_back(Point_2(578.57574, 287.44703));
	shapeObs3.push_back(Point_2(654.55418, 325.43643));
	shapeObs3.push_back(Point_2(520.13078, 511.00001));
	shapeObs3.push_back(Point_2(394.47413, 489.08306));
	shapeObs3.push_back(Point_2(375.47951, 429.17671));
	(shapeObs3.orientation() == CGAL::RIGHT_TURN) ? shapeObs3.reverse_orientation() : (void)true;

	Polygon_2 shapeObs4;
	shapeObs4.push_back(Point_2(685.23778, 103.34458));
	shapeObs4.push_back(Point_2(699.84901, 63.89404));
	shapeObs4.push_back(Point_2(755.37175, 37.5937));
	shapeObs4.push_back(Point_2(800.66658, 79.96649));
	shapeObs4.push_back(Point_2(875.18392, 75.5831));
	shapeObs4.push_back(Point_2(878.10615, 125.26153));
	shapeObs4.push_back(Point_2(784.59421, 93.11668));
	shapeObs4.push_back(Point_2(717.38252, 106.26684));
	(shapeObs4.orientation() == CGAL::RIGHT_TURN) ? shapeObs4.reverse_orientation() : (void)true;

	Polygon_2 shapeObs5;
	shapeObs5.push_back(Point_2(755, 561));
	shapeObs5.push_back(Point_2(650, 447));
	shapeObs5.push_back(Point_2(821, 382));
	shapeObs5.push_back(Point_2(871, 370));
	shapeObs5.push_back(Point_2(887, 401));
	shapeObs5.push_back(Point_2(912, 413));
	shapeObs5.push_back(Point_2(917, 397));
	shapeObs5.push_back(Point_2(973, 445));
	shapeObs5.push_back(Point_2(947, 496));
	shapeObs5.push_back(Point_2(930, 477));
	shapeObs5.push_back(Point_2(951, 430));
	shapeObs5.push_back(Point_2(887, 423));
	shapeObs5.push_back(Point_2(826, 489));
	shapeObs5.push_back(Point_2(947, 565));
	shapeObs5.push_back(Point_2(951, 538));
	shapeObs5.push_back(Point_2(976, 546));
	shapeObs5.push_back(Point_2(956, 600));
	shapeObs5.push_back(Point_2(911, 581));
	shapeObs5.push_back(Point_2(901, 564));
	shapeObs5.push_back(Point_2(797, 503));
	shapeObs5.push_back(Point_2(797, 472));
	shapeObs5.push_back(Point_2(860, 412));
	shapeObs5.push_back(Point_2(837, 399));
	shapeObs5.push_back(Point_2(778, 460));
	shapeObs5.push_back(Point_2(784, 523));
	shapeObs5.push_back(Point_2(874, 593));
	shapeObs5.push_back(Point_2(879, 651));
	shapeObs5.push_back(Point_2(923, 663));
	shapeObs5.push_back(Point_2(941, 624));
	shapeObs5.push_back(Point_2(965, 639));
	shapeObs5.push_back(Point_2(940, 688));
	shapeObs5.push_back(Point_2(841, 667));
	shapeObs5.push_back(Point_2(843, 617));
	(shapeObs5.orientation() == CGAL::RIGHT_TURN) ? shapeObs5.reverse_orientation() : (void)true;

	Polygon_2 shapeObs6;
	shapeObs6.push_back(Point_2(365.25165, 602.23329));
	shapeObs6.push_back(Point_2(451.45795, 624.15024));
	shapeObs6.push_back(Point_2(449.99685, 599.31103));
	(shapeObs6.orientation() == CGAL::RIGHT_TURN) ? shapeObs6.reverse_orientation() : (void)true;

	Polygon_2 shapeObs7;
	shapeObs7.push_back(Point_2(119.78285, 489.08309));
	shapeObs7.push_back(Point_2(55.493399, 464.24385));
	shapeObs7.push_back(Point_2(159.23318, 418.94881));
	shapeObs7.push_back(Point_2(186.99454, 474.47179));
	(shapeObs7.orientation() == CGAL::RIGHT_TURN) ? shapeObs7.reverse_orientation() : (void)true;
	
	Polygon_2 shapeObs8;
	shapeObs8.push_back(Point_2(64.844587, 675.52337));
	shapeObs8.push_back(Point_2(118.90617, 634.61172));
	shapeObs8.push_back(Point_2(134.97853, 666.75658));
	shapeObs8.push_back(Point_2(146.66752, 650.68417));
	shapeObs8.push_back(Point_2(140.82302, 631.68943));
	shapeObs8.push_back(Point_2(152.51202, 630.22828));
	shapeObs8.push_back(Point_2(181.7345, 633.15057));
	shapeObs8.push_back(Point_2(224.10708, 656.52867));
	shapeObs8.push_back(Point_2(215.34033, 693.05692));
	shapeObs8.push_back(Point_2(180.27337, 672.60108));
	shapeObs8.push_back(Point_2(194.88461, 725.2018));
	shapeObs8.push_back(Point_2(240.17945, 663.83432));
	shapeObs8.push_back(Point_2(251.86844, 634.61168));
	shapeObs8.push_back(Point_2(219.72371, 628.76717));
	shapeObs8.push_back(Point_2(196.34574, 621.46153));
	shapeObs8.push_back(Point_2(162.73989, 612.69472));
	shapeObs8.push_back(Point_2(190.50124, 603.92793));
	shapeObs8.push_back(Point_2(203.65135, 614.15588));
	shapeObs8.push_back(Point_2(205.11248, 596.62228));
	shapeObs8.push_back(Point_2(187.57899, 587.85549));
	shapeObs8.push_back(Point_2(243.1017, 580.54983));
	shapeObs8.push_back(Point_2(257.71294, 567.39969));
	shapeObs8.push_back(Point_2(259.17406, 546.94384));
	shapeObs8.push_back(Point_2(262.0963, 527.94913));
	shapeObs8.push_back(Point_2(200.72911, 561.55514));
	shapeObs8.push_back(Point_2(181.7345, 546.94384));
	shapeObs8.push_back(Point_2(155.43427, 527.94913));
	shapeObs8.push_back(Point_2(153.97314, 552.78838));
	shapeObs8.push_back(Point_2(184.65675, 564.47742));
	shapeObs8.push_back(Point_2(175.89001, 580.54983));
	shapeObs8.push_back(Point_2(136.43966, 582.01098));
	shapeObs8.push_back(Point_2(123.28954, 538.17705));
	shapeObs8.push_back(Point_2(85.300309, 535.25479));
	shapeObs8.push_back(Point_2(94.06706, 516.2601));
	shapeObs8.push_back(Point_2(53.155598, 506.0322));
	shapeObs8.push_back(Point_2(76.533576, 564.4774));
	shapeObs8.push_back(Point_2(99.911546, 589.31664));
	shapeObs8.push_back(Point_2(86.761437, 609.77248));
	shapeObs8.push_back(Point_2(67.766834, 624.38377));
	shapeObs8.push_back(Point_2(47.311095, 611.23359));
	shapeObs8.push_back(Point_2(34.160978, 583.47209));
	shapeObs8.push_back(Point_2(23.933121, 602.46679));
	(shapeObs8.orientation() == CGAL::RIGHT_TURN) ? shapeObs8.reverse_orientation() : (void)true;

	Polygon_2 shapeObs9;
	shapeObs9.push_back(Point_2(108, 876));
	shapeObs9.push_back(Point_2(171, 921));
	shapeObs9.push_back(Point_2(44, 950));
	shapeObs9.push_back(Point_2(49, 897));
	shapeObs9.push_back(Point_2(39, 861));
	shapeObs9.push_back(Point_2(70, 832));
	shapeObs9.push_back(Point_2(206, 838));
	shapeObs9.push_back(Point_2(222, 809));
	shapeObs9.push_back(Point_2(221, 778));
	shapeObs9.push_back(Point_2(336, 706));
	shapeObs9.push_back(Point_2(397, 712));
	shapeObs9.push_back(Point_2(396, 735));
	shapeObs9.push_back(Point_2(369, 737));
	shapeObs9.push_back(Point_2(259, 866));
	shapeObs9.push_back(Point_2(184, 875));
	(shapeObs9.orientation() == CGAL::RIGHT_TURN) ? shapeObs9.reverse_orientation() : (void)true;

	Polygon_2 shapeObs10;
	shapeObs10.push_back(Point_2(50 + 447.07458, 645.42408));
	shapeObs10.push_back(Point_2(50 + 505.51954, 725.78628));
	shapeObs10.push_back(Point_2(50 + 528.89753, 684.87462));
	shapeObs10.push_back(Point_2(50 + 664.78207, 651.26861));
	shapeObs10.push_back(Point_2(50 + 631.17619, 614.74032));
	shapeObs10.push_back(Point_2(50 + 542.04765, 608.89582));
	shapeObs10.push_back(Point_2(50 + 487.98607, 649.80743));
	(shapeObs10.orientation() == CGAL::RIGHT_TURN) ? shapeObs10.reverse_orientation() : (void)true;

	Polygon_2 shapeObs11;
	shapeObs11.push_back(Point_2(585.88135, 854.36578));
	shapeObs11.push_back(Point_2(607.7982, 794.45942));
	shapeObs11.push_back(Point_2(748.06612, 721.40288));
	shapeObs11.push_back(Point_2(829.88905, 731.63079));
	(shapeObs11.orientation() == CGAL::RIGHT_TURN) ? shapeObs11.reverse_orientation() : (void)true;

	StaticObstacle* obs1 = new StaticObstacle(vector<Cell*>(), shapeObs1);
	StaticObstacle* obs2 = new StaticObstacle(vector<Cell*>(), shapeObs2);
	StaticObstacle* obs3 = new StaticObstacle(vector<Cell*>(), shapeObs3);
	StaticObstacle* obs4 = new StaticObstacle(vector<Cell*>(), shapeObs4);
	StaticObstacle* obs5 = new StaticObstacle(vector<Cell*>(), shapeObs5);
	StaticObstacle* obs6 = new StaticObstacle(vector<Cell*>(), shapeObs6);
	StaticObstacle* obs7 = new StaticObstacle(vector<Cell*>(), shapeObs7);
	StaticObstacle* obs8 = new StaticObstacle(vector<Cell*>(), shapeObs8);
	StaticObstacle* obs9 = new StaticObstacle(vector<Cell*>(), shapeObs9);
	StaticObstacle* obs10 = new StaticObstacle(vector<Cell*>(), shapeObs10);
	StaticObstacle* obs11 = new StaticObstacle(vector<Cell*>(), shapeObs11);

	obstacles.push_back(obs1);
	obstacles.push_back(obs2);
	obstacles.push_back(obs3);
	obstacles.push_back(obs4);
	obstacles.push_back(obs5);
	obstacles.push_back(obs6);
	obstacles.push_back(obs7);
	obstacles.push_back(obs8);
	obstacles.push_back(obs9);
	obstacles.push_back(obs10);
	obstacles.push_back(obs11);
	return obstacles;
}

vector<StaticObstacle*> createObstacles(double uavCurvature, double scale){
	vector<StaticObstacle*> obstacles;

	Polygon_2 shapeObs1;
	shapeObs1.push_back(Point_2(scale * 350, scale * 990));
	shapeObs1.push_back(Point_2(scale * 450, scale * 950));
	shapeObs1.push_back(Point_2(scale * 480, scale * 840));
	shapeObs1.push_back(Point_2(scale * 300, scale * 805));
	shapeObs1.push_back(Point_2(scale * 220, scale * 580));
	shapeObs1.push_back(Point_2(scale * 120, scale * 580));
	shapeObs1.push_back(Point_2(scale * 80, scale * 640));
	shapeObs1.push_back(Point_2(scale * 120, scale * 810));
	shapeObs1.reverse_orientation();

	StaticObstacle* obs1 = new StaticObstacle(vector<Cell*>(), shapeObs1);

	Polygon_2 shapeObs2;
	shapeObs2.push_back(Point_2(scale * 120, scale * 120));
	shapeObs2.push_back(Point_2(scale * 310, scale * 450));
	shapeObs2.push_back(Point_2(scale * 390, scale * 220));
	shapeObs2.reverse_orientation();

	StaticObstacle* obs2 = new StaticObstacle(vector<Cell*>(), shapeObs2);

	Polygon_2 shapeObs3;
	double x = scale * 450;
	double y = scale * 700;
	
	shapeObs3.push_back(Point_2(x + scale * 30, y));
	shapeObs3.push_back(Point_2(x - scale * 10, y- scale * 50));
	shapeObs3.push_back(Point_2(x- scale * 15, y- scale * 120));

	shapeObs3.push_back(Point_2(x+ scale * 40, y- scale * 200));
	shapeObs3.push_back(Point_2(x+ scale * 200, y- scale * 215));

	shapeObs3.push_back(Point_2(x + scale * 220, y - scale * 120));
	shapeObs3.push_back(Point_2(x + scale * 180, y - scale * 30));
	shapeObs3.push_back(Point_2(x + scale * 160, y + scale * 50));

	shapeObs3.push_back(Point_2(x + scale * 250, y + scale * 10));
	shapeObs3.push_back(Point_2(x + scale * 270, y - scale * 250));
	shapeObs3.push_back(Point_2(x + scale * 240, y - scale * 280));

	shapeObs3.push_back(Point_2(x + scale * 100, y - scale * 245));
	shapeObs3.push_back(Point_2(x, y - scale * 270));


	shapeObs3.push_back(Point_2(x - scale * 80, y - scale * 90));
	shapeObs3.push_back(Point_2(x - scale * 60, y - scale * 30));

	shapeObs3.reverse_orientation();

	StaticObstacle* obs3 = new StaticObstacle(vector<Cell*>(), shapeObs3);

	Polygon_2 shapeObs4;
	x = scale * 550;
	y = scale * 250;

	shapeObs4.push_back(Point_2(x, y));
	shapeObs4.push_back(Point_2(x + scale * 80, y - scale * 40));

	shapeObs4.push_back(Point_2(x + scale * 200, y - scale * 30));
	shapeObs4.push_back(Point_2(x + scale * 250, y + scale * 20));
	shapeObs4.push_back(Point_2(x + scale * 260, y + scale * 110));
	shapeObs4.push_back(Point_2(x + scale * 340, y + scale * 120));

	shapeObs4.push_back(Point_2(x + scale * 300, y - scale * 45));
	shapeObs4.push_back(Point_2(x + scale * 160, y - scale * 90));
	shapeObs4.push_back(Point_2(x, y - scale * 90));
	shapeObs4.push_back(Point_2(x- scale * 60, y - scale * 40));
	shapeObs4.reverse_orientation();

	StaticObstacle* obs4 = new StaticObstacle(vector<Cell*>(), shapeObs4);

	Polygon_2 shapeObs5;
	x = scale * 800;
	y = scale * 700;

	shapeObs5.push_back(Point_2(x, y));
	shapeObs5.push_back(Point_2(x - scale * 100, y + scale * 220));
	shapeObs5.push_back(Point_2(x + scale * 100, y + scale * 230));
	shapeObs5.push_back(Point_2(x + scale * 120, y + scale * 10));

	shapeObs5.reverse_orientation();

	StaticObstacle* obs5 = new StaticObstacle(vector<Cell*>(), shapeObs5);
	
	obstacles.push_back(obs1);
	obstacles.push_back(obs2);
	obstacles.push_back(obs3);
	obstacles.push_back(obs4);
	obstacles.push_back(obs5);

	cout << "\t L \t d " << endl;
	double offset[5] = {0,0,0,0,0};
	vector<StaticObstacle*>::iterator it = obstacles.begin();
	double selectedL;
	int i = 0;
	while(it != obstacles.end()) {
		Polygon_2* shape = (*it)->getShape();
		vector<CPointPH> p;
		int count = 0;
		for (VertexIterator vertex = shape->vertices_begin(); vertex < shape->vertices_end(); vertex++) {
			p.push_back(CPointPH(vertex->x(), vertex->y()));
			count++;
			if (count == 3) {
				double L = G2getMaxL(&p[0], uavCurvature);
				double d = G2getOffSetObstacleDistance(&p[0], L);
				if(offset[i] < d){
					offset[i] = d;
					selectedL = L;
				}
				
				count = 0;
				p.clear();
			}
		}
		cout << "Obs " << i << " : \t" << selectedL << " \t " << offset[i] << endl;
		it++;
		i++;
	}

	obs1->generateOffSet(offset[0] + 2);
	obs2->generateOffSet(offset[1] + 2);
	obs3->generateOffSet(offset[2] + 2);
	obs4->generateOffSet(offset[3] + 2);
	obs5->generateOffSet(offset[4] + 2);

	return obstacles;
}

void startGraphics(int argc, char** argv, RRT* rrt, NavigationEnvironment* navigationSpace, string filePath) {
	Graphics* g = new Graphics();

	if (navigationSpace != nullptr) {
		EnvironmentDisplayer* displayerEnv = new EnvironmentDisplayer{ *navigationSpace };
		g->registerDisplayable(displayerEnv);
	}

	if (rrt != nullptr) {
		RRTDisplayer* displayerRRT = new RRTDisplayer{ rrt };
		g->registerDisplayable(displayerRRT);
	}

	g->init(argc, argv, navigationSpace->getInitialCoordinate()->getNodeWaypoint(), navigationSpace->getFinalCoordinate()->getNodeWaypoint(), filePath);
	g->startGraphics();
}

string getPathToSeedFolder(string experimentName, string algorithmName, string envName, string filePrefix, string fileSufix, string fileExtension) {
	string path, pathBase;
	pathBase = "C:/RRTSimulations/bySeed/";
	if (experimentName.size() != 0) {
		pathBase = pathBase + experimentName;
	}
	path = pathBase + algorithmName + "/" + envName + "/" + filePrefix + "_" + fileSufix + "_" + envName + fileExtension;
	return path;
}

int main(int argc, char** argv) {
	string binaryFilePath = argv[1];
	
	istringstream rrtType(argv[2]);
	//int rrtChoose = *argv[2] - '0';
	int rrtChoose;
	if (!(rrtType >> rrtChoose)) {
		cerr << "Invalid rrt choice number " << argv[1] << '\n';
		return 0;
	}

	string envName = argv[3];	
	istringstream it(argv[4]);
	int iterations;
	if (!(it >> iterations)) {
		cerr << "Invalid iteration number " << argv[1] << '\n';
		return 0;
	}

	istringstream qNewXss(argv[5]), qNewYss(argv[6]), qEndXss(argv[7]), qEndYss(argv[8]);
	double qInitX, qInitY, qEndX, qEndY;
	if (!(qNewXss >> qInitX && qNewYss >> qInitY && qEndXss >> qEndX && qEndYss >> qEndY)) {
		cerr << "Invalid coordination number " << argv[1] << '\n';
		return 0;
	}

	istringstream seed(argv[9]);
	int randomNumberSeed;
	if (!(seed >> randomNumberSeed)) {
		cerr << "Invalid seed number " << argv[1] << '\n';
		return 0;
	}

	/*double maxLatitude = 1000;
	double maxLongitude = 1000;*/	
	//################# Configurações para as comparações com o código do Felipe
	/*double initCoordLat = -22.994583, initCoordLong = -45.99875;
	double endCoordLat = -22.000417, endCoordLong = -45.002917;
	double intervalLatitude = endCoordLat - initCoordLat;
	double intervalLongitude = endCoordLong - initCoordLong;
	Node* qInit = new Node{ initCoordLong + intervalLongitude / 2 , -22.99};
	Node* qGoal = new Node{-45.25, -22.15};*/

	double initCoordLat = 0, initCoordLong = 0;
	double endCoordLat = 1000, endCoordLong = 1000;
	//double endCoordLat = 1, endCoordLong = 1;

	/*double intervalLatitude = endCoordLat - initCoordLat;
	double intervalLongitude = endCoordLong - initCoordLong;*/

	bool visionExperiment = false;
	bool curveExperiment = false;

	if (envName == "natural") {
		visionExperiment = true;
	} else if (envName == "curve") {
		curveExperiment = true;
		endCoordLat = 5000;
		endCoordLong = 5000;
	}

	//Scale based on default [1000 : 1000] navigation interval in both axis
	double scale = (endCoordLat / 1000.0);

	//radiusNeighborhood is the beta of scaling factor fo the ball size given by log n / n.
	//double radiusNeighborhood = scale * 650, distanceToGoal = scale*50;
	double radiusNeighborhood = scale * 650, distanceToGoal = scale*50;

	//double uavCurvature = 0.009;
	double uavCurvature = 0.015;
	//Binary grid read and navigation space configuration
	BinaryGrid binaryGrid{};
	binaryGrid.binaryMatrixRead(binaryFilePath);

	cout << "<<<<<<<< Simulation for planning of the navigation environment: " << envName << ". >>>>>>>>" << endl;
	NavigationEnvironment navigationSpace;

	if (visionExperiment) {
		navigationSpace = NavigationEnvironment{ createObstaclesRRTVisionExperiment(), 
												initCoordLong, initCoordLat, 
												endCoordLong, endCoordLat, 
												randomNumberSeed };
	} else if (curveExperiment) {
		navigationSpace = NavigationEnvironment{createObstacles(uavCurvature, scale), 
												initCoordLong, initCoordLat, 
												endCoordLong, endCoordLat, 
												randomNumberSeed};
	}else{
		navigationSpace = NavigationEnvironment{ binaryGrid, false, 
												initCoordLong, initCoordLat, 
												endCoordLong, endCoordLat, 
												randomNumberSeed };
	}

	//NavigationEnvironment navigationSpace{createObstacles(uavCurvature, scale), initCoordLong, initCoordLat, endCoordLong, endCoordLat, randomNumberSeed};
	//NavigationEnvironment navigationSpace(initCoordLong, initCoordLat, endCoordLong, endCoordLat, randomNumberSeed);

	Node* qInit = new Node{ qInitX, qInitY }; qInit->setStartPosition(true);
	Node* qGoal = new Node{ qEndX, qEndY };

	RRT* rrt = nullptr;
	string fileBySeedName;
	//string experimentName = "grid_compare/";
	string experimentName = "vision_experiments/";

	vector<double> dSukha;
	dSukha.push_back(125);
	dSukha.push_back(90.9091);
	dSukha.push_back(62.5);
	dSukha.push_back(45.4545);
	dSukha.push_back(31.25);

	vector<string> dNames;
	dNames.push_back("64");
	dNames.push_back("128");
	dNames.push_back("256");
	dNames.push_back("512");
	dNames.push_back("1024");

	switch (rrtChoose){		
		case 1: //Case RRT*-Smart
			{
				std::cout << "RRT*-Smart building..." << endl;
				rrt = new RRTStarSmart{ navigationSpace, radiusNeighborhood, iterations };
				((RRTStarSmart*)rrt)->build(qInit, qGoal, distanceToGoal, envName);
				fileBySeedName = "C:/RRTSimulations/bySeed/RRT_Smart/"+ envName +"/rrtstar_smart_bySeed_" + envName + ".txt";

				//string fileImageName = "C:/RRTSimulations/bySeed/RRT_Smart/" + envName + "/rrtstar_smart_bySeed_" + "_print_it_" + envName + ".bmp";
				//startGraphics(argc, argv, rrt, &navigationSpace, fileImageName);

				//Catch data from simulation and input to file
				double planningTime = rrt->getPlanningTime();
				double cost = 0;
				if ((NodeStar*)rrt->getRoute()) {
					cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				}
				int nodes = rrt->getQuantityNodes();
				int collisions = rrt->getCollisionChecker().getCollisionsDetected();
				//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
				saveDataBySeed(fileBySeedName, data, data + 4);
				//rrt->~RRT();
				break;
			}

		case 2:	//Case RRT*-SukharevVertices
			{
				int cellsSukharev = 100;

				cout << "RRT*-SukharevVertices " << cellsSukharev << " cells building..." << endl;
				navigationSpace.buildGrid(cellsSukharev, GridEnum::SUKHAREV_GRID);
				rrt = new RRTStarSukharevVertices{ navigationSpace, radiusNeighborhood, iterations};
				((RRTStarSukharevVertices*)rrt)->buildSukharevVerticesKdTree(qInit, qGoal, distanceToGoal, envName, cellsSukharev);
				fileBySeedName = "C:/RRTSimulations/bySeed/RRT_SV/" + envName + "/rrtstar_sv_kdtree" + to_string(cellsSukharev) + "_cells_bySeed_" + envName + ".txt";

				//Catch data from simulation and input to file
				double planningTime = rrt->getPlanningTime();
				double cost = 0;
				if ((NodeStar*)rrt->getRoute()) {
					cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				}
				int nodes = rrt->getQuantityNodes();
				int collisions = rrt->getCollisionChecker().getCollisionsDetected();
				//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
				saveDataBySeed(fileBySeedName, data, data + 4);

				//do{
				//	//{
				//	//	cout << "RRT*-SukharevVertices " << cellsSukharev << " cells building..." << endl;
				//	//	navigationSpace.buildGrid(cellsSukharev, GridEnum::SUKHAREV_GRID);
				//	//	rrt = new RRTStarSukharevVertices{ navigationSpace, radiusNeighborhood, iterations };
				//	//	((RRTStarSukharevVertices*)rrt)->buildSukharevVertices(qInit, qGoal, distanceToGoal, envName, cellsSukharev);
				//	//	//((RRTStarSukharevVertices*)rrt)->buildSukharevVerticesHough(qInit, qGoal, distanceToGoal, envName, cellsSukharev);
				//	//	//((RRTStarSukharevVertices*)rrt)->buildSukharevVerticesProportion(qInit, qGoal, distanceToGoal, envName, cellsSukharev);
				//	//	fileBySeedName = "C:/RRTSimulations/bySeed/RRT_SV/" + envName + "/rrtstar_sv_" + to_string(cellsSukharev) + "_cells_bySeed_" + envName + ".txt";

				//	//	//Catch data from simulation and input to file
				//	//	double planningTime = rrt->getPlanningTime();
				//	//	double cost = 0;
				//	//	//cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				//	//	int nodes = rrt->getQuantityNodes();
				//	//	int collisions = rrt->getCollisionChecker().getCollisionsDetected();
				//	//	//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				//	//	float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
				//	//	saveDataBySeed(fileBySeedName, data, data + 4);

				//	//	((RRTStarSukharevVertices*)rrt)->~RRTStarSukharevVertices();

				//	//	//Reset qInit and qGoal to delete references to sucessors and antecessors
				//	//	qInit = new Node{ qInitX, qInitY };
				//	//	qGoal = new Node{ qEndX, qEndY };
				//	//	//rrt->~RRT();
				//	//}

				//	//{//RRT-SukharevVerticeTransformed
				//	//	cout << "RRT*-SukharevVerticesTransformed " << cellsSukharev << " cells building..." << endl;
				//	//	navigationSpace.buildGrid(cellsSukharev, GridEnum::SUKHAREV_GRID);
				//	//	rrt = new RRTStarSukharevVertices{ navigationSpace, radiusNeighborhood, iterations };
				//	//	//((RRTStarSukharevVertices*)rrt)->buildSukharevVertices(qInit, qGoal, distanceToGoal, envName, cellsSukharev);
				//	//	((RRTStarSukharevVertices*)rrt)->buildSukharevVerticesHough(qInit, qGoal, distanceToGoal, envName, cellsSukharev);
				//	//	//((RRTStarSukharevVertices*)rrt)->buildSukharevVerticesProportion(qInit, qGoal, distanceToGoal, envName, cellsSukharev);
				//	//	fileBySeedName = "C:/RRTSimulations/bySeed/RRT_SV/" + envName + "/rrtstar_sv_transformed_test_" + to_string(cellsSukharev) + "_cells_bySeed_" + envName + ".txt";

				//	//	//Catch data from simulation and input to file
				//	//	double planningTime = rrt->getPlanningTime();
				//	//	double cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				//	//	int nodes = rrt->getQuantityNodes();
				//	//	int collisions = rrt->getCollisionChecker().getCollisionsDetected();
				//	//	//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				//	//	float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
				//	//	saveDataBySeed(fileBySeedName, data, data + 4);

				//	//	((RRTStarSukharevVertices*)rrt)->~RRTStarSukharevVertices();

				//	//	//Reset qInit and qGoal to delete references to sucessors and antecessors
				//	//	qInit = new Node{ qInitX, qInitY };
				//	//	qGoal = new Node{ qEndX, qEndY };
				//	//	//rrt->~RRT();
				//	//}

				//	//{//RRT-SukharevVerticeProportional
				//	//	cout << "RRT*-SukharevVerticesProportional " << cellsSukharev << " cells building..." << endl;
				//	//	navigationSpace.buildGrid(cellsSukharev, GridEnum::SUKHAREV_GRID);
				//	//	rrt = new RRTStarSukharevVertices{ navigationSpace, radiusNeighborhood, iterations };
				//	//	((RRTStarSukharevVertices*)rrt)->buildSukharevVerticesProportion(qInit, qGoal, distanceToGoal, envName, cellsSukharev);
				//	//	fileBySeedName = "C:/RRTSimulations/bySeed/RRT_SV/" + envName + "/rrtstar_sv_proportion_test" + to_string(cellsSukharev) + "_cells_bySeed_" + envName + ".txt";

				//	//	//Catch data from simulation and input to file
				//	//	double planningTime = rrt->getPlanningTime();
				//	//	double cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				//	//	int nodes = rrt->getQuantityNodes();
				//	//	int collisions = rrt->getCollisionChecker().getCollisionsDetected();
				//	//	//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				//	//	float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
				//	//	saveDataBySeed(fileBySeedName, data, data + 4);

				//	//	((RRTStarSukharevVertices*)rrt)->~RRTStarSukharevVertices();

				//	//	//Reset qInit and qGoal to delete references to sucessors and antecessors
				//	//	qInit = new Node{ qInitX, qInitY };
				//	//	qGoal = new Node{ qEndX, qEndY };
				//	//	//rrt->~RRT();
				//	//}

				//	{//RRT-SukharevVerticeKdTree
				//		std::cout << "RRT*-SukharevVerticesKdTree " << cellsSukharev << " cells building..." << endl;
				//		navigationSpace.buildGrid(cellsSukharev, GridEnum::SUKHAREV_GRID);
				//		rrt = new RRTStarSukharevVertices{ navigationSpace, radiusNeighborhood, iterations };
				//		((RRTStarSukharevVertices*)rrt)->buildSukharevVerticesKdTree(qInit, qGoal, distanceToGoal, envName, cellsSukharev);
				//		fileBySeedName = "C:/RRTSimulations/bySeed/RRT_SV/" + envName + "/rrtstar_sv_kdtree_" + to_string(cellsSukharev) + "_cells_bySeed_" + envName + ".txt";
				//		
				//		string fileImageName = "C:/RRTSimulations/byIteration/RRT_SV/" + envName + "/rrtstar_sv_kdtree_" + "_print_it_" + envName + ".bmp";
				//		//startGraphics(argc, argv, rrt, &navigationSpace, fileImageName);

				//		//Catch data from simulation and input to file
				//		double planningTime = rrt->getPlanningTime();
				//		double cost = 0;
				//		cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				//		int nodes = rrt->getQuantityNodes();
				//		int collisions = rrt->getCollisionChecker().getCollisionsDetected();
				//		//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				//		float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
				//		saveDataBySeed(fileBySeedName, data, data + 4);

				//		((RRTStarSukharevVertices*)rrt)->~RRTStarSukharevVertices();

				//		//Reset qInit and qGoal to delete references to sucessors and antecessors
				//		qInit = new Node{ qInitX, qInitY };
				//		qGoal = new Node{ qEndX, qEndY };
				//		//rrt->~RRT();
				//	}

				//	//cin.get();
				//	if (cellsSukharev == 0) {
				//		cellsSukharev = 75;
				//	}
				//	else {
				//		cellsSukharev = cellsSukharev * 2;
				//	}
				//} while (cellsSukharev <= 100);
				break; 
			}
		
		case 3: //Case RRT*
			{
				for (int k = 3; k < dSukha.size(); k++) {
					//std::cout << "RRT* d" + dNames[k] + " building..." << endl;
					std::cout << "RRT* building..." << endl;
					rrt = new RRTStar{ navigationSpace, radiusNeighborhood, iterations };
					//rrt->setDispersion(dSukha[k]);
					((RRTStar*)rrt)->build(qInit, qGoal, distanceToGoal, envName, "0");
					//fileBySeedName = "C:/RRTSimulations/bySeed/RRT_Star/" + envName + "/rrtstar_bySeed_" + envName + ".txt";
					fileBySeedName = "C:/RRTSimulations/bySeed/" + experimentName + "RRT_Star/" + envName + "/rrtstar_bySeed_" + envName + ".txt";

					//Catch data from simulation and input to file
					double planningTime = rrt->getPlanningTime();
					double cost = 0;
					if ((NodeStar*)rrt->getRoute()) {
						cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
					}
					int nodes = rrt->getQuantityNodes();
					int collisions = rrt->getCollisionChecker().getCollisionsDetected();
					//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
					float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
					saveDataBySeed(fileBySeedName, data, data + 4);

					//Reset qInit and qGoal to delete references to sucessors and antecessors
					qInit = new Node{ qInitX, qInitY }; qInit->setStartPosition(true);
					qGoal = new Node{ qEndX, qEndY };
					rrt->~RRT();
				
					//std::cout << "RRT* d" + dNames[k] + " and seed " + to_string(randomNumberSeed) + " complete!" << endl;
				}

				break;	
			}
		
		case 4: //Case RRT*-SukharevGrid		
			{
				int cellsSukharev = 64, k = 0;
				do{
					std::cout << "RRT*-Sukharev with " << cellsSukharev << " Cells building..." << endl;
					navigationSpace.buildGrid(cellsSukharev, GridEnum::SUKHAREV_GRID);
					rrt = new RRTStarSukharevVertices{ navigationSpace, radiusNeighborhood, iterations };
					rrt->setDispersion(dSukha[k]);
					((RRTStarSukharevVertices*)rrt)->buildSukharev(qInit, qGoal, distanceToGoal, envName, cellsSukharev);
					std::cout << "RRT*-Sukharev with " << cellsSukharev << "Cells constructed" << endl;
				
					fileBySeedName = "C:/RRTSimulations/bySeed/" + experimentName + "RRT_Sukharev/" + envName + "/rrtstar_sukharev_" + to_string(cellsSukharev) + "_cells_bySeed_" + envName + ".txt";
				
					////Catch data from simulation and input to file
					//double planningTime = rrt->getPlanningTime();
					//double cost = 0;
					//if ((NodeStar*)rrt->getRoute()) {
					//	cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
					//}
					//int nodes = rrt->getQuantityNodes();
					//int collisions = rrt->getCollisionChecker().getCollisionsDetected();
					////Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
					//float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
					//saveDataBySeed(fileBySeedName, data, data + 4);

					//Reset qInit and qGoal to delete references to sucessors and antecessors
					qInit = new Node{ qInitX, qInitY }; qInit->setStartPosition(true);
					qGoal = new Node{ qEndX, qEndY };

					cellsSukharev = cellsSukharev * 2;
					rrt->~RRT();
					std::cout << "RRT*-Sukharev cells " + to_string(cellsSukharev) + " and seed " + to_string(randomNumberSeed) + " complete!" << endl;
					k++;
				} while (cellsSukharev <= 1024);
				return 0;
				break; 
			}
		
		case 5: //Case RRT - WorcapExperiment
			{
				//vector<double> dSukha;
				//dSukha.push_back(125);
				//dSukha.push_back(90.9091);
				//dSukha.push_back(62.5);
				//dSukha.push_back(45.4545);
				//dSukha.push_back(31.25);

				//vector<double> dLattice;
				//dLattice.push_back(118.996);
				//dLattice.push_back(67.2057);
				//dLattice.push_back(61.7632);
				//dLattice.push_back(37.1607);
				//dLattice.push_back(18.5804);

				//vector<string> dNames;
				//dNames.push_back("64");
				//dNames.push_back("128");
				//dNames.push_back("256");
				//dNames.push_back("512");
				//dNames.push_back("1024");

				//for (int k = 0; k < dSukha.size(); k ++){
				//	std::cout << "RRT with Sukha " << dNames[k] << " dispersion building..." << endl;
				//	rrt = new RRT{ navigationSpace, iterations };
				//	rrt->setDispersion(dSukha[k]);
				//	rrt->build(qInit, qGoal, distanceToGoal, envName, 1, dNames[k]);
				//	fileBySeedName = "C:/RRTSimulations/bySeed/RRT/" + envName + "/rrt_dsukha_" + dNames[k] + "_cells_bySeed_" + envName + ".txt";

				//	//Catch data from simulation and input to file
				//	double planningTime = rrt->getPlanningTime();
				//	//double cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				//	double cost = 0;
				//	int nodes = rrt->getQuantityNodes();
				//	int collisions = rrt->getCollisionChecker().getCollisionsDetected();
				//	//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				//	float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
				//	saveDataBySeed(fileBySeedName, data, data + 4);
				//	rrt->~RRT();

				//	std::cout << "RRT with Lattice " << dNames[k] << " dispersion building..." << endl;
				//	rrt = new RRT{ navigationSpace, iterations };
				//	rrt->setDispersion(dLattice[k]);
				//	rrt->build(qInit, qGoal, distanceToGoal, envName, 2, dNames[k]);
				//	fileBySeedName = "C:/RRTSimulations/bySeed/RRT/" + envName + "/rrt_dlattice_" + dNames[k] + "_cells_bySeed_" + envName + ".txt";

				//	//Catch data from simulation and input to file
				//	double planningTime2 = rrt->getPlanningTime();
				//	//double cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				//	double cost2 = 0;
				//	int nodes2 = rrt->getQuantityNodes();
				//	int collisions2 = rrt->getCollisionChecker().getCollisionsDetected();
				//	//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				//	float data2[] = { randomNumberSeed, cost2, planningTime2, nodes2, collisions2 };
				//	saveDataBySeed(fileBySeedName, data2, data2 + 4);
				//	rrt->~RRT();
				//}

				std::cout << "RRT building..." << endl;
				rrt = new RRT{ navigationSpace, iterations };
				rrt->setDispersion(55);
				rrt->build(qInit, qGoal, distanceToGoal, envName, 0, "");
				fileBySeedName = "C:/RRTSimulations/bySeed/RRT/" + envName + "/rrt_bySeed_" + envName + ".txt";

				//Catch data from simulation and input to file
				double planningTime = rrt->getPlanningTime();
				//double cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				double cost = 0;
				int nodes = rrt->getQuantityNodes();
				int collisions = rrt->getCollisionChecker().getCollisionsDetected();
				//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
				saveDataBySeed(fileBySeedName, data, data + 4);
				rrt->~RRT();

				break; 
			}

		case 6: 
			{
				NavigationEnvironment emptySpace(0, 0, 5000, 5000, randomNumberSeed);
				rrt = new RRT{ emptySpace, 0 };
				rrt->zigZagTest();
				break;
			}

		case 7: 
			{
				std::cout << "RRT Curve building..." << endl;
				rrt = new RRTCurve{ navigationSpace, iterations };
				//rrt->build(qInit, qGoal, distanceToGoal, envName);

				//fileBySeedName = "C:/RRTSimulations/bySeed/RRT" + envName + "/rrt_bySeed_" + envName + ".txt";
				////Catch data from simulation and input to file
				//double planningTime = rrt->getPlanningTime();
				//double cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				//int nodes = rrt->getQuantityNodes();
				//int collisions = rrt->getCollisionChecker().getCollisionsDetected();
				////Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				//float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
				//saveDataBySeed(fileBySeedName, data, data + 4);
				rrt->~RRT();
				break; 
			}

		case 8: 
			{
				int numberCells = 64;
				do{
					std::cout << "RRTGrid-Lattice with " << numberCells << " Cells building..." << endl;
					navigationSpace.buildGrid(numberCells, GridEnum::LATTICE_GRID);

					fileBySeedName = "C:/RRTSimulations/bySeed/RRT_Grid/Lattice/" + envName + "/rrt_grid_lattice_" + to_string(numberCells) + "_cells_bySeed_" + envName + ".txt";
					rrt = new RRTGrid{ navigationSpace, 0};
					((RRTGrid*)rrt)->build(qInit, qGoal, distanceToGoal, envName, numberCells);
					//Catch data from simulation and input to file
					//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
					float data1[] = { randomNumberSeed, 0, rrt->getPlanningTime(), rrt->getQuantityNodes(),  rrt->getCollisionChecker().getCollisionsDetected()};
					saveDataBySeed(fileBySeedName, data1, data1 + 3);
					//Reset qInit and qGoal to delete references to sucessors and antecessors
					qInit = new Node{ qInitX, qInitY }; qInit->setStartPosition(true);
					qGoal = new Node{ qEndX, qEndY };
					rrt->~RRT();

					//startGraphics(argc, argv, rrt, &navigationSpace);

					cout << "RRTGrid-Sukharev with " << numberCells << " Cells building..." << endl;
					navigationSpace.buildGrid(numberCells, GridEnum::SUKHAREV_GRID);
					fileBySeedName = "C:/RRTSimulations/bySeed/RRT_Grid/Sukharev/" + envName + "/rrt_grid_sukharev_" + to_string(numberCells) + "_cells_bySeed_" + envName + ".txt";
					rrt = new RRTGrid{ navigationSpace, 0 };
					((RRTGrid*)rrt)->build(qInit, qGoal, distanceToGoal, envName, numberCells);
					float data2[] = { randomNumberSeed, 0, rrt->getPlanningTime(), rrt->getQuantityNodes(),  rrt->getCollisionChecker().getCollisionsDetected() };
					saveDataBySeed(fileBySeedName, data2, data2 + 3);
					//Reset qInit and qGoal to delete references to sucessors and antecessors
					qInit = new Node{ qInitX, qInitY };
					qGoal = new Node{ qEndX, qEndY };
					rrt->~RRT();

					//startGraphics(argc, argv, rrt, &navigationSpace);

					numberCells = numberCells * 2;
				} while (numberCells <= 1024);
				//string fileImageName = "C:/RRTSimulations/byIteration/RRT_Grid/Sukharev/" + envName + "/rrtstar_grid_sukharev_" + "_print_it_" + envName + ".bmp";
				//startGraphics(argc, argv, rrt, &navigationSpace, fileImageName);
				return 0;
				break;
			} 
		
		case 9: //Informed-RRT*
			{ 
				std::cout << "Informed RRT* building..." << endl;
				rrt = new InformedRRTStar{ navigationSpace, radiusNeighborhood, iterations, experimentName };
				((InformedRRTStar*)rrt)->build(new NodeStar(qInit), new NodeStar(qGoal), distanceToGoal, envName);

				fileBySeedName = getPathToSeedFolder(experimentName, "Informed_RRT", 
					envName, "informedrrtstar", "bySeed", ".txt");				
				string fileImageName = getPathToSeedFolder(experimentName, "Informed_RRT",
						envName, "informedrrtstar", "finalpath", "");
				rrt->displayNow(fileImageName, iterations);

				//startGraphics(argc, argv, rrt, &navigationSpace, fileImageName);
				//fileBySeedName = "C:/RRTSimulations/bySeed/Informed_RRT/" + envName + "/informedrrtstar_bySeed_" + envName + ".txt";

				//Catch data from simulation and input to file
				double planningTime = rrt->getPlanningTime();
				double cost = 0;
				if ((NodeStar*)rrt->getRoute()) {
					cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				}
				int nodes = rrt->getQuantityNodes();
				int collisions = rrt->getCollisionChecker().getCollisionsDetected();
				//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
				saveDataBySeed(fileBySeedName, data, data + 4);
				rrt->~RRT();

				break; 
			}
		case 11: //Case RRT*-Vertices (No Sukharev)
			{ 	
				for (int k = 3; k < dSukha.size(); k++) {
					std::cout << "RRT*-Vertices d" << dNames[k] << " building..." << endl;
					rrt = new RRTStarSukharevVertices{ navigationSpace, radiusNeighborhood, iterations };
					rrt->setDispersion(dSukha[k]);
					((RRTStarSukharevVertices*)rrt)->buildVerticesKdTree(qInit, qGoal, distanceToGoal, envName, dNames[k]);
					fileBySeedName = "C:/RRTSimulations/bySeed/" + experimentName + "RRT_Vertices/" + envName + "/rrtstar_vertices_bySeed_d" + dNames[k] + "_" + envName + ".txt";

					//string fileImageName = "C:/RRTSimulations/bySeed/RRT_Vision/" + envName + "/rrtstar_vision_" + "_print_it_" + envName + ".bmp";
					//startGraphics(argc, argv, rrt, &navigationSpace, fileImageName);

					//Catch data from simulation and input to file
					double planningTime = rrt->getPlanningTime();
					double cost = 0;
					if ((NodeStar*)rrt->getRoute()) {
						cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
					}
					int nodes = rrt->getQuantityNodes();
					int collisions = rrt->getCollisionChecker().getCollisionsDetected();
					//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
					float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
					saveDataBySeed(fileBySeedName, data, data + 4);

					//Reset qInit and qGoal to delete references to sucessors and antecessors
					qInit = new Node{ qInitX, qInitY }; qInit->setStartPosition(true);
					qGoal = new Node{ qEndX, qEndY };
					((RRTStarSukharevVertices*)rrt)->~RRTStarSukharevVertices();

					//rrt->~RRT();
					std::cout << "RRT*-Vertices d" + dNames[k] + " and seed " + to_string(randomNumberSeed) + " complete!" << endl;
				}
				break; 
			}

		case 12: 
			{
				int cellsSukharev = 64;
				int k = 0;
				do {
					std::cout << "RRT*-SV NoOpt with " << cellsSukharev << " Cells building..." << endl;
					navigationSpace.buildGrid(cellsSukharev, GridEnum::SUKHAREV_GRID);
					rrt = new RRTStarSukharevVertices{ navigationSpace, radiusNeighborhood, iterations };
					rrt->setDispersion(dSukha[k]);
					((RRTStarSukharevVertices*)rrt)->buildSukharevVerticesKdTreeNoOpt(qInit, qGoal, distanceToGoal, envName, cellsSukharev);
					std::cout << "RRT*-SV NoOpt with " << cellsSukharev << " Cells constructed" << endl;

					fileBySeedName = "C:/RRTSimulations/bySeed/" + experimentName + "RRT_SV_NoOpt/" + envName + "/rrtstar_sv_noopt_" + to_string(cellsSukharev) + "_cells_bySeed_" + envName + ".txt";
					//Catch data from simulation and input to file
					double planningTime = rrt->getPlanningTime();
					double cost = 0;
					if ((NodeStar*)rrt->getRoute()) {
						cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
					}
					int nodes = rrt->getQuantityNodes();
					int collisions = rrt->getCollisionChecker().getCollisionsDetected();
					//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
					float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
					saveDataBySeed(fileBySeedName, data, data + 4);

					//Reset qInit and qGoal to delete references to sucessors and antecessors
					qInit = new Node{ qInitX, qInitY }; qInit->setStartPosition(true);
					qGoal = new Node{ qEndX, qEndY };
					cellsSukharev = cellsSukharev * 2;
					rrt->~RRT();
					std::cout << "RRT*-SV NoOpt cells " + to_string(cellsSukharev) + " and seed " + to_string(randomNumberSeed) + " complete!" << endl;
					k++;
				} while (cellsSukharev <= 1024);
				return 0;
				break; 
			}

		case 13: //BIT*
			{
				std::cout << "BIT* building..." << endl;

				rrt = new BITStar{ navigationSpace, radiusNeighborhood, iterations, experimentName};
				NodeStar* qInitStar = new NodeStar(qInit);
				NodeStar* qGoalStar = new NodeStar(qGoal);
				((BITStar*)rrt)->build(qInitStar, qGoalStar, distanceToGoal, envName);

				std::cout << "BIT* constructed" << endl;

				//experimentName = "";
				//fileBySeedName = "C:/RRTSimulations/bySeed/" + experimentName + "BIT_Star/" + envName + "/bitstar_bySeed_" + envName + ".txt";
				fileBySeedName = getPathToSeedFolder(experimentName, "BIT_Star",
					envName, "bitstar", "bySeed", ".txt");
				string fileImageName = getPathToSeedFolder(experimentName, "BIT_Star",
					envName, "bitstar", "finalpath", "");
				rrt->displayNow(fileImageName, iterations);

				//Catch data from simulation and input to file
				double planningTime = rrt->getPlanningTime();
				double cost = 0;
				if ((NodeStar*)rrt->getRoute()) {
					cost = ((NodeStar*)rrt->getRoute()->getGoal())->getCost();
				}
				int nodes = rrt->getQuantityNodes();
				int collisions = rrt->getCollisionChecker().getCollisionsDetected();

				//Save data to current simulation {"seed", "final_cost", "planning_time", "nodes"," collisions"};
				float data[] = { randomNumberSeed, cost, planningTime, nodes, collisions };
				saveDataBySeed(fileBySeedName, data, data + 4);

				rrt->~RRT();
				return 0;
				break; 
			}

		case 14: 
			{
				std::cout << "Visibility Graph building..." << endl;

				VisibilityGraph* graph = new VisibilityGraph{navigationSpace};
				graph->build(qInit, qGoal);

				std::cout << "Visibility Graph constructed" << endl;

				graph->~VisibilityGraph();
				return 0;
				break; 
			}

		default:
			break;
	}

	if (curveExperiment){
		//Optimal Path: 5214.94
		cout.precision(4);
		cout << "Tempo de planejamento: " << fixed << rrt->getPlanningTime() << endl;

		int i = 0;
		cout << "------------Rota não suavizada--------------" << endl;

		Node* cNode = rrt->getRoute()->getGoal();
		Node* iNode = rrt->getRoute()->getInit();
		while (cNode) {
			cout << "Node #" << i << ": x = " << cNode->getX() << "\t y = " << cNode->getY() << endl;
			cNode = cNode->getAntecessor();
			i++;
		}
		cout << "Comprimento da rota não suavizada: " << rrt->getRoute()->getRouteCost() << endl;
		cout << "--------------------------" << endl;
		startGraphics(argc, argv, rrt, &navigationSpace, "C:/RRTSimulations/bySeed/RRT_SV/" + envName + "/curve_nosmoothed_kappa_" + to_string(uavCurvature) +".bmp");

		cout << "------------Rota suavizada com PH curve--------------" << endl;

		Route* smoothedRoute = smoothRoute(rrt->getRoute(), uavCurvature);
		cNode = smoothedRoute->getSmoothGoal();
		iNode = smoothedRoute->getSmoothInit();

		while (cNode) {
			cout << "Node #" << i << ": x = " << cNode->getX() << "\t y = " << cNode->getY() << endl;
			cNode = cNode->getAntecessor();
			i++;
		}
		rrt->setRoute(smoothedRoute);
		cout << "Comprimento da rota suavizada: " << rrt->getRoute()->getSmoothRouteCost() << endl;
		cout << "--------------------------" << endl;

		cout << "------------Curvatura ao longo da rota planejada--------------" << endl;
		rrt->getRoute()->generateCurvatureFile("some_file_name");

		startGraphics(argc, argv, rrt, &navigationSpace, "C:/RRTSimulations/bySeed/RRT_SV/" + envName + "/curve_smoothed_kappa_" + to_string(uavCurvature) + ".bmp");
	}

	//int i = 0;
	//Node* cNode = rrt->getRoute()->getGoal();
	//Node* iNode = rrt->getRoute()->getInit();
	//while (cNode) {
	//	cout << "Node #" << i << ": x = " << cNode->getX() << "\t y = " << cNode->getY() << endl;
	//	cNode = cNode->getAntecessor();
	//	i++;
	//}
	//cout << "--------------------------" << endl;

	//Route* smoothedRoute = smoothRoute(rrt->getRoute(), uavCurvature);

	//cNode = smoothedRoute->getGoal();
	//iNode = smoothedRoute->getInit();

	//while (cNode) {
	//	cout << "Node #" << i << ": x = " << cNode->getX() << "\t y = " << cNode->getY() << endl;
	//	cNode = cNode->getAntecessor();
	//	i++;
	//}
	//rrt->setRoute(smoothedRoute);
	//cout << "Comprimento da rota suavizada: " << rrt->getRoute()->getRouteCost() << endl;
	//rrt->getRoute()->generateCurvatureFile("some_file_name");
	////cout << "Comprimento da rota não suavizada: " << rrt->getRoute()->getRouteCost() << endl; 

	//Graphics* g = new Graphics();
	////Cria o objeto para mostrar o Ambiente de navegação.
	//EnvironmentDisplayer* edDisplayer = new EnvironmentDisplayer{ navigationSpace };
	//RRTDisplayer* rrtDisplayer = new RRTDisplayer{ rrt };

	////A Ordem de registro define quem será mostrado primeiro no layer do OpenGl
	////g->registerDisplayable(edDisplayer);
	//g->registerDisplayable(rrtDisplayer);

	//g->init(argc, argv, navigationSpace.getInitialCoordinate()->getNodeWaypoint(), navigationSpace.getFinalCoordinate()->getNodeWaypoint(), "smooth_simulation.bmp");
	//g->startGraphics();

	cout << "<<<<<<<< RRT Constructed!! >>>>>>>>>" << endl;
	cout << "Exiting program!!" << endl << endl;

	//while (true) {};
}