#ifndef RRT_H
#define RRT_H

#include "BialkowskiCollisionChecker.h"
#include "NavigationEnvironment.h"
#include <vector>
#include "SearchGrid.h"
#include "Graphics.h"
#include "Route.h"

using namespace std;

class RRT{
public:
    RRT(NavigationEnvironment ne, const int n);
    virtual ~RRT();
    virtual void build(Node* qInit, Node *qGoal, double distanceToGenerateRoute, string envName, int selectDispersion, string cells);
    Node* getRoot();
	Node* getGoal();
    Waypoint* getNEInitialCoordinate();
    Waypoint* getNEFinalCoordinate();
	BialkowskiCollisionChecker getCollisionChecker();
	vector<Node*> getStackNodes();
	SearchGrid* getSearchGrid();
	double getPlanningTime();
	int getQuantityNodes();
	NavigationEnvironment* getNavigationEnvironment();
	void displayNow(string pathFile, int iteration);
	void displayNow(string pathFile, double iteration);
	double getDistanceToGoal();
	Route* getRoute();
	void setRoute(Route* route);
	void zigZagTest();
	void setDispersion(double dispersion);
protected:
    void setRoot(Node* node);
    Node* generateRandomNode();
    Node* nearestNode(Node* qrand);
    Node* createNewNode(Node* qNear, Node*qRand);
    NavigationEnvironment navigationSpace;
    void insertNode(Node* qParent, Node* qChild);
	void removeNode(Node* qParent, Node* qChild);
    Node* root;
	Node* goal;
    int qtdNodes;
    vector<Node*> stackNodes;
	SearchGrid* nodesGrid;
	BialkowskiCollisionChecker collisionChecker;
	Node* pathCompleted(Node* qNew, Node* qGoal, double distanceToGenerateRoute);
	int const n; //Number of interaction to execute the simulation.
	double planningTime = 0;
	double distanceToGoal;
	Graphics* g;
	Route* route;
	double deltaQ = 0;
};

#endif /* RRT_H */