#pragma once
#ifndef DS_CBS_CONSTRAINT_H
#define DS_CBS_CONSTRAINT_H
#include "CBSDataStructure.h"
class agent;
struct doubleVertex;

class Constraint {
public:
    Constraint(agent* agent,doubleVertex* doubleVertex,int timeStep){
        this->agent=agent;
        this->doubleVertex=doubleVertex;
        this->timeStep=timeStep;
    }

    agent* agent;
    doubleVertex* doubleVertex;
    int timeStep;
};


#endif //DS_CBS_CONSTRAINT_H
