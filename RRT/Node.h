#ifndef NODE_H
#define NODE_H

#include <vector>
#include "Waypoint.h"
#include "G2PlanarPH.h"

using namespace std;
        
class Node {
public:
    Node();
    Node(double latitude, double longitude);
	Node(double latitude, double longitude, G2PlanarPH* curve);
    Node(const Node& orig);
    virtual ~Node();
    Waypoint* getNodeWaypoint();
    Node* getAntecessor();
    void setAntecessor(Node* node);
    const vector<Node*> & getSucessors() const;
    void addSucessor(Node* newSucessor);
	bool isSucessorNode(Node * potentialSucessor);
	bool isAntecessor(Node * potentialAntecessor);
	bool isInRoute(Node * potentialRouteMember);
	G2PlanarPH* getCurve();
	short int removeSucessor(Node* sucessorToRemove);
    bool isVisited();
    void visite();
	double getX() const;
	double getY() const;
	inline bool operator==(const Node& nodeToCompare);
	Node* createCopy();
	Node* getReferenceCurveNode();
	void setReferenceCurveNode(Node* referenceNode);
	void incrementTestsValue(int increment);
	int getTestsValue();
	void setStartPosition(bool value);
	bool isStartPosition();
protected:
    Node* antecessor;
    Waypoint* nodeWaypoint;
    vector<Node*> sucessores;
    bool visited = false;
	bool startPosition = false;
	G2PlanarPH* curve = nullptr;
	Node* referenceCurveNode = nullptr;
	int testsValue = 0;
};

#endif /* NODE_H */