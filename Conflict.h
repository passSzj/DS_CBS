#ifndef DS_CBS_CONFLICT_H
#define DS_CBS_CONFLICT_H
#include "agent.h"


class Conflict {
public:

    Conflict(agent* agent1,agent*agent2,doubleVertex* V,int Time1, int Time2,int timeStep):V(V),timeStep(timeStep){
        agents[0]=agent1;
        agents[1]=agent2;
        Time[0]=Time1;
        Time[1]=Time2;
    }


    Conflict(agent* agent,doubleVertex* V,int Time1,int timeStep):V(V),timeStep(timeStep){
        agents[0]=agent;
        agents[1]= nullptr;
        Time[0]=Time1;
        Time[1]=-1;
    }

    agent* agents[2];
    doubleVertex* V;
    int timeStep;
    int Time[2];
};




#endif //DS_CBS_CONFLICT_H
