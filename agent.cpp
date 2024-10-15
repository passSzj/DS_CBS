#include "agent.h"
#include "CBSDataStructure.h"

agent::agent(int index,int startX,int startY,int goalX,int goalY,int speed)
:index(index),startX(startX),startY(startY),goalX(goalX),goalY(goalY),speed(speed)
{
    path=new Path(index);
}