#ifndef DS_CBS_AGENT_H
#define DS_CBS_AGENT_H

#include "CBSDataStructure.h"
#include "Constraint.h"

struct Path;

class agent {
public:
    int index;
    int startX;
    int startY;
    int goalX;
    int goalY;
    Path *path;
    int speed;
    agent(int index,int startX,int startY,int goalX,int goalY,int speed);


};


#endif //DS_CBS_AGENT_H
