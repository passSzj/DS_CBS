#include <iostream>
#include "tinyxml2.h"
#include <unordered_map>
#include <string>
#include "Map.h"
#include "readMapAndAgent.h"
#include "agent.h"
#include <cstdlib>
using namespace tinyxml2;

void initMapFromXML(const char* filename, Map& m) {
    XMLDocument doc;
    if (doc.LoadFile(filename) != XML_SUCCESS) {
        std::cerr << "Error loading XML file: " << filename << std::endl;
        return;
    }

    XMLElement* root = doc.FirstChildElement("map");
    if (!root) {
        std::cerr << "Invalid XML format: Missing root element <map>" << std::endl;
        return;
    }

    std::unordered_map<std::string, doubleVertex*> vertices;

    // Parse vertices
    for (XMLElement* vertexElem = root->FirstChildElement("vertex"); vertexElem; vertexElem = vertexElem->NextSiblingElement("vertex")) {
        const char* id = vertexElem->Attribute("id");
        if (!id) {
            std::cerr << "Invalid XML format: Vertex missing id attribute" << std::endl;
            continue;
        }

        int x = vertexElem->IntAttribute("x");
        int y = vertexElem->IntAttribute("y");

        leftVertex* lv = new leftVertex(x, y);
        rightVertex* rv = new rightVertex(x, y);
        doubleVertex* dv = new doubleVertex(lv, rv);

        vertices[id] = dv;
        m.addVertex(dv);
    }

    // Parse edges
    for (XMLElement* edgeElem = root->FirstChildElement("edge"); edgeElem; edgeElem = edgeElem->NextSiblingElement("edge")) {
        const char* from = edgeElem->Attribute("from");
        const char* to = edgeElem->Attribute("to");

        int length=std::atoi(edgeElem->Attribute("length"));
        int maxSpeed=std::atoi(edgeElem->Attribute("maxSpeed"));

        const char* type = edgeElem->Attribute("type");


        if (!from || !to || !type) {
            std::cerr << "Invalid XML format: Edge missing from, to, or type attribute" << std::endl;
            continue;
        }

        doubleVertex* dvFrom = vertices[from];
        doubleVertex* dvTo = vertices[to];



        if (std::string(type) == "left-right") {
            m.addEdge(&dvFrom->vl, &dvTo->vr,length,maxSpeed);
            m.addEdge(&dvTo->vr, &dvFrom->vl,length,maxSpeed);
        } else if (std::string(type) == "right-left") {
            m.addEdge(&dvFrom->vr, &dvTo->vl,length,maxSpeed);
            m.addEdge(&dvTo->vl, &dvFrom->vr,length,maxSpeed);
        } else {
            std::cerr << "Unknown edge type: " << type << std::endl;
        }
    }
}


std::vector<agent*> ReadAgentsFromXML(const char* filename) {
    std::vector<agent*> agents;
    XMLDocument doc;
    if (doc.LoadFile(filename) != XML_SUCCESS) {
        std::cerr << "Failed to load file: " << filename << std::endl;
        return agents;
    }

    XMLElement* root = doc.RootElement();
    if (root == nullptr) {
        std::cerr << "No root element in XML file." << std::endl;
        return agents;
    }

    int index = 0;

    for (XMLElement* elem = root->FirstChildElement("agent"); elem != nullptr; elem = elem->NextSiblingElement("agent")) {
        int startX, startY, goalX, goalY,speed;

        elem->FirstChildElement("startX")->QueryIntText(&startX);
        elem->FirstChildElement("startY")->QueryIntText(&startY);
        elem->FirstChildElement("goalX")->QueryIntText(&goalX);
        elem->FirstChildElement("goalY")->QueryIntText(&goalY);
        elem->FirstChildElement("speed")->QueryIntText(&speed);

        agents.push_back(new agent(index, startX, startY, goalX, goalY,speed));
        index++;
    }

    return agents;
}