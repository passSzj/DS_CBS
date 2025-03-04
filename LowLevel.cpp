#include "LowLevel.h"

int lowLevelCBS::getLowExpand(){
    return expand;
}

int lowLevelCBS::getExpandCount(vector<doubleVertex*> nodes){
    int size1=nodes.size();
    int size2;
    vector<doubleVertex*> temp;
    for (size_t j = 0; j < nodes.size() - 1; ++j){
        doubleVertex* currentNode = nodes[j];
        doubleVertex* nextNode = nodes[j+1];
        temp.push_back(currentNode);
        for (int t = currentNode->depth + 1; t < nextNode->depth; ++t) {
            doubleVertex* newDoubleVertex = new doubleVertex(*currentNode,t);
            temp.push_back(newDoubleVertex);
        }
    }
    temp.push_back(nodes.back());
    size2=temp.size();

    return (size2-size1);
}

double lowLevelCBS::heuristicCostEstimate(const doubleVertex& a, const doubleVertex& b) const
{
    double ED= getEuclidDistance(a.vl, b.vl);
    return ED == 0 ? 1 : ED;
}

double lowLevelCBS::getEuclidDistance(const Vertex& a, const Vertex& b) const {
    double dx = static_cast<double>(a.x - b.x);
    double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

int lowLevelCBS::getNodeWithLeastF(const std::vector<doubleVertex*> &list) const
{
    //TODO
    double min = INT_MAX;
    int min_index = 0;
    for (int i = 0; i < list.size(); i++)
    {
        if (list[i]->f < min)
        {
            min = list[i]->f;
            min_index = i;
        }
    }

    return min_index;
}

Path lowLevelCBS::ReconstructPath(doubleVertex* node)
{
    std::vector<doubleVertex*> path_reverse;
    while (node != nullptr)
    {
        path_reverse.push_back(node);
        node = node->parent;
    }

    Path new_path;
    for (int i = path_reverse.size() - 1; i >= 0; i--)
    {
        new_path.Nodes.push_back(path_reverse[i]);
    }


    for(auto node:new_path.Nodes){
        std::cout<<node->vl.x<<","<<node->vl.y<<std::endl;
    }

    return new_path;
}

vector<doubleVertex*> lowLevelCBS::ReconstructPathAndJustGetPath(doubleVertex* node)
{
    std::vector<doubleVertex*> path_reverse;
    while (node != nullptr)
    {
        path_reverse.push_back(node);
        node = node->parent;
    }

    std::vector<doubleVertex*> path;
    for (int i = path_reverse.size() - 1; i >= 0; i--)
    {
        path.push_back(path_reverse[i]);
    }
    return path;
}

void lowLevelCBS::addDvParent(doubleVertex *current,Vertex *v){
    doubleVertex* newDoubleVertex = new doubleVertex(*current);
    v->dvParent = newDoubleVertex;
    v->dvParent->parent = current;
}

int lowLevelCBS::findVertexInClosed(int x, int y){
    for(int i=0;i<closed.size();i++){
        if(closed[i]->vl.x==x&&closed[i]->vl.y==y){
            return i;
        }
    }
}

int lowLevelCBS::getSize(doubleVertex *current,doubleVertex *start,doubleVertex *goal){
    if(current->visitLeft){
        return m.getSuccessors(current)[0].size();
    }
    if(current->visitRight){
        return m.getSuccessors(current)[1].size();
    }
    if((*current)==(*start)){
        if(goal->vl.x>start->vl.x){
            return m.getSuccessors(current)[1].size();
        }
        else
            return m.getSuccessors(current)[0].size();
    }

}

bool lowLevelCBS::hasConflict(Vertex* vertex,int time,const vector<Constraint*> &constraints){
    for(int i=0;i<constraints.size();i++){
        if(constraints[i]->timeStep==time && constraints[i]->doubleVertex->vl.x==vertex->x && constraints[i]->doubleVertex->vl.y==vertex->y)
            return true;
    }
    return false;
}

void lowLevelCBS::openErase(int x,int y){
    for(int i=0;i<open.size();i++){
        if(open[i]->vl.x==x&&open[i]->vl.y==y){
            open.erase(open.begin()+i);
        }
    }
}


bool lowLevelCBS::isAllInClosed(vector<Vertex*> Successors){
    int count=0;
    for(int i=0;i<Successors.size();i++){
        if (std::find(closed.begin(), closed.end(), Successors[i]->dvParent) != closed.end())
        {
            count++;
        }
    }
    if(Successors.size()== count)
        return false;
    return true;

}