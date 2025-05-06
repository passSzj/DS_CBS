#ifndef DVG_CBS_LOWLEVEL_H
#define DVG_CBS_LOWLEVEL_H

#endif //DVG_CBS_LOWLEVEL_H

#include "Map.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <algorithm>
#include "readMapAndAgent.h"
#include "agent.h"
#include "Constraint.h"
#include<unordered_map>
#pragma once

using namespace std;
class lowLevelCBS{
private:
   int expand=0;

public:
    Map m;
    lowLevelCBS(){}
    std::vector<doubleVertex*> open;
    std::vector<doubleVertex*> closed;

    int getLowExpand();
    int getExpandCount(vector<doubleVertex*> nodes);
    double heuristicCostEstimate(const doubleVertex& a, const doubleVertex& b) const;
    double getEuclidDistance(const Vertex& a, const Vertex& b) const;
    int getNodeWithLeastF(const std::vector<doubleVertex*> &list) const;
    vector<doubleVertex*> ReconstructPathAndJustGetPath(doubleVertex* node);
    void addDvParent(doubleVertex *current,Vertex *v);
    bool hasConflict(Vertex* vertex,int time,const vector<Constraint*> &constraints);
    void openErase(int x,int y);
    bool isAllInClosed(vector<Vertex*> Successors);
    bool Astar(doubleVertex* start, doubleVertex* goal, Path &path,const vector<Constraint*> &constraints,int agentMaxSpeed);


private:
    void initOpenAndClosed(doubleVertex* start);
    void clearSuccessors(vector<Vertex*> &list,vector<doubleVertex*> open);
};