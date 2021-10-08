#pragma once

#include "Node.h"

struct NodePointerCompare {
	bool operator() (const Node* lhs, const Node* rhs) const {
		return (lhs->getX() < rhs->getX()) || (lhs->getX() == rhs->getX() && lhs->getY() < rhs->getY());
	}
};