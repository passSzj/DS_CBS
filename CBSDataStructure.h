#ifndef DVG_CBS_CBSDATASTRUCTURE_H
#define DVG_CBS_CBSDATASTRUCTURE_H


#include "agent.h"
#include <tuple>
#include <vector>

class Constraint;
struct doubleVertex;

struct Vertex
{
    int depth;
    bool leftOrRight;
    doubleVertex* dvParent;

    Vertex()
    {
        parent = nullptr;
    }

    Vertex(int x, int y,bool leftOrRight) :
        x(x), y(y)
    {
        this->leftOrRight=leftOrRight;
        parent= nullptr;
    }



    Vertex(const Vertex &other)
    {
        x=other.x;
        y=other.y;
    }

    int x;
    int y;


    Vertex* parent;


    inline bool operator ==(const Vertex &v) const
    {
        return v.x==this->x&&v.y==this->y;
    }

};

struct leftVertex : public Vertex{
    leftVertex() : Vertex() { leftOrRight = false; }
    leftVertex(int x, int y) : Vertex(x, y, false) {}
};

struct rightVertex : public Vertex{
    rightVertex() : Vertex() { leftOrRight = true; }
    rightVertex(int x, int y) : Vertex(x, y, true) {}
};


struct doubleVertex{

    doubleVertex(leftVertex *vl, rightVertex *vr): g(0), h(0), f(0), visitLeftSubVertex(false), visitRightSubVertex(false)
        //number(num)
    {
        this->vl=*vl;
        this->vr=*vr;
        parent= nullptr;
        this->vl.dvParent = this;
        this->vr.dvParent = this;
        depth=0;
    }

    doubleVertex(const doubleVertex& other) {
        f=other.f;
        g=other.g;
        h=other.h;
        vr=other.vr;
        vl=other.vl;

        visitRightSubVertex=other.visitRightSubVertex;
        visitLeftSubVertex=other.visitLeftSubVertex;

        parent= nullptr;
        depth=other.depth;
        vrNeighbors=other.vrNeighbors;
        vlNeighbors=other.vlNeighbors;
        vllengthAndMaxSpeed=other.vllengthAndMaxSpeed;
        vrlengthAndMaxSpeed=other.vrlengthAndMaxSpeed;
    }

    doubleVertex(const doubleVertex& other,int newTime) {
        f=other.f;
        g=other.g;
        h=other.h;
        vr=other.vr;
        vl=other.vl;

        visitRightSubVertex=other.visitRightSubVertex;
        visitLeftSubVertex=other.visitLeftSubVertex;

        parent= nullptr;
        depth=newTime;
        vrNeighbors=other.vrNeighbors;
        vlNeighbors=other.vlNeighbors;
        vllengthAndMaxSpeed=other.vllengthAndMaxSpeed;
        vrlengthAndMaxSpeed=other.vrlengthAndMaxSpeed;
    }

    rightVertex vr;
    leftVertex vl;
    doubleVertex* parent;
    int depth;
    double g;
    double h;
    double f;
    bool visitLeftSubVertex;
    bool visitRightSubVertex;
    std::vector<Vertex*> vrNeighbors;
    std::vector<Vertex*> vlNeighbors;
    std::vector<std::tuple<int,int>> vrlengthAndMaxSpeed;
    std::vector<std::tuple<int,int>> vllengthAndMaxSpeed;

    inline bool operator ==( doubleVertex &v)
    {
        return v.vl.x==this->vl.x&&v.vl.y==this->vl.y;
    }

    int getSuccessorsSize(doubleVertex *start,doubleVertex *goal){
        if(this->visitLeftSubVertex){
            return vlNeighbors.size();
        }
        if(this->visitRightSubVertex){
            return vrNeighbors.size();
        }
        if((*this)==(*start)){
            if(goal->vl.x>start->vl.x){
                return vrNeighbors.size();
            }
            else
                return vlNeighbors.size();
        }

    }

    void convertLeftSubvertex(){
        this->visitLeftSubVertex= true;
        this->visitRightSubVertex= false;
    }

    void convertRightSubvertex(){
        this->visitRightSubVertex= true;
        this->visitLeftSubVertex= false;
    }
};

struct Path{
    Path(){}
    Path(int index) : agentIndex(index){}

    int agentIndex;
    std::vector<doubleVertex*> Nodes;
    std::vector<Constraint*> Constraints;
    double getVertexX(int a){
        double X=Nodes[a]->vl.x;
        return X;
    }

    double getVertexY(int a){
        double Y=Nodes[a]->vl.y;
        return Y;
    }

    int get_cost()
    {
        return Nodes.size();
    }

};





#endif //DVG_CBS_CBSDATASTRUCTURE_H