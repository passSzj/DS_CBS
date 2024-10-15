#ifndef DVG_CBS_MAP_H
#define DVG_CBS_MAP_H

#include <vector>
#include <list>
#include <tuple>
#include "CBSDataStructure.h"
#include "iostream"

class Map{
private:
    std::vector<doubleVertex*> vertexs;

public:

    void addVertex(doubleVertex *v){
        vertexs.push_back(v);
    }
    int getVertexNum(){
        return vertexs.size();
    }

    doubleVertex* findVertex(Vertex v) {
        for (auto vertex : vertexs) {
            if (vertex->vl == v||vertex->vr == v) {
                return vertex;
            }
        }
        return nullptr;
    }


    void addEdge(Vertex *v1,Vertex *v2,int length,int maxSpeed) {

        doubleVertex* source = findVertex(*v1);
        doubleVertex* destination = findVertex(*v2);
        if (source && destination) {
            if(v1->leftOrRight) {
                source->vrNeighbors.push_back(&destination->vl);
                source->vrlengthAndMaxSpeed.push_back(std::make_tuple(length, maxSpeed));}
            if(!v1->leftOrRight){
                source->vlNeighbors.push_back(&destination->vr);
                source->vllengthAndMaxSpeed.push_back(std::make_tuple(length, maxSpeed));}
        }
    }




    void printGraph() {
//        for (auto vertex : vertexs) {
//            std::cout <<"当前左节点："<< "("<<vertex->vl.x<< ","<<vertex->vl.y<< ")"<<"  邻接节点：";
//            if(vertex->vlNeighbors.empty())
//                std::cout << "无";
//            for (auto neighbor : vertex->vlNeighbors) {
//                std::cout << "(" << neighbor->x << "," << neighbor->y << ") ";
//            }
//            std::cout << std::endl;
//
//            std::cout <<"当前右节点："<< "("<<vertex->vr.x<< ","<<vertex->vr.y<< ")"<<"  邻接节点：";
//            if(vertex->vrNeighbors.empty())
//                std::cout << "无";
//            for (auto neighbor : vertex->vrNeighbors) {
//                std::cout << "(" << neighbor->x << "," << neighbor->y << ") ";
//            }
//            std::cout << std::endl;
//        }
    }


    std::vector<std::vector<Vertex*>> getSuccessors(doubleVertex* v){
        std::vector<std::vector<Vertex*>> successors;
        for(auto doubleVertex: vertexs){
            if((*v) == (*doubleVertex)){
                successors.push_back(doubleVertex->vlNeighbors);
                successors.push_back(doubleVertex->vrNeighbors);
                return successors;
            }
        }
        return successors;
    }


    std::vector<doubleVertex*> getVertex(){
        return vertexs;
    }


    doubleVertex* getVertex(int x,int y){
        for(auto v:vertexs){
            if(v->vl.x==x&&v->vl.y==y){
                return v;
            }
        }
    }


    void clearMapAstarValues() {
        for (auto vertex: vertexs) {
            vertex->g = 0;
            vertex->h = 0;
            vertex->f = 0;
            vertex->vl.parent = nullptr;
            vertex->vl.depth = 0;
            vertex->visitLeft=false;
            vertex->visitRight=false;
        }
    }

};

#endif //DVG_CBS_MAP_H