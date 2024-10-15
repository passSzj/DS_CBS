#ifndef DS_CBS_READMAP_H
#define DS_CBS_READMAP_H

#endif //DS_CBS_READMAP_H


#include "agent.h"


void initMapFromXML(const char* filename, Map& m);

std::vector<agent*> ReadAgentsFromXML(const char* filename);