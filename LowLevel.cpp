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
    if(current->visitLeftSubVertex){
        return m.getSuccessors(current)[0].size();
    }
    if(current->visitRightSubVertex){
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

bool lowLevelCBS::Astar(doubleVertex* start, doubleVertex* goal, Path &path,const vector<Constraint*> &constraints,int agentMaxSpeed){


    m.clearMapAstarValues();
    start->g=0;
    start->vl.depth=0;
    start->f=heuristicCostEstimate(*start,*goal);

    initOpenAndClosed(start);

    int  isStart=0;


    while (!open.empty()&&closed.size() < m.getVertexNum() * 30){
        int index= getNodeWithLeastF(open);
        doubleVertex* current =open[index];


        std::vector<doubleVertex*>::iterator it = (open.begin() + index);
        open.erase(it);
        expand++;
        bool isClosed=false;


        if ((*current) == (*goal))
        {
            path.Nodes = ReconstructPathAndJustGetPath(current);
            expand+=getExpandCount(path.Nodes);
            return true;
        }

        std::vector<std::vector<Vertex*>> successors = m.getSuccessors(current);
        std::vector<Vertex*> leftSuccessors=successors[0];
        std::vector<Vertex*> rightSuccessors=successors[1];




        if((*current)==(*start)&&isStart==0){
            isStart++;
            bool isSelf=false;
            Vertex *l=new Vertex(current->vl);
            Vertex *r=new Vertex(current->vr);
            addDvParent(current,l);
            addDvParent(current,r);
            leftSuccessors.push_back(l);
            rightSuccessors.push_back(r);
            clearSuccessors(leftSuccessors, open);
            clearSuccessors(rightSuccessors,open);


            for(int i=0; i < leftSuccessors.size(); i++){
                int edgeLength;
                int edgeMaxSpeed;

                if(leftSuccessors[i]->x == current->vl.x && leftSuccessors[i]->y == current->vl.y){
                    leftSuccessors[i]->dvParent->g=current->g;
                    leftSuccessors[i]->dvParent->h=current->h;
                    isSelf= true;
                    edgeLength=0;
                    edgeMaxSpeed=0;
                }else{
                    edgeLength=get<0>(current->vllengthAndMaxSpeed[i]);
                    edgeMaxSpeed=get<1>(current->vllengthAndMaxSpeed[i]);
                }

                int speed=edgeMaxSpeed>agentMaxSpeed?speed=agentMaxSpeed:speed=edgeMaxSpeed;

                if (std::find(closed.begin(), closed.end(), leftSuccessors[i]->dvParent) != closed.end())
                {
                    continue;
                }

                int new_cost = current->g + heuristicCostEstimate(*leftSuccessors[i]->dvParent, *current);




                auto it =std::find(open.begin(), open.end(), leftSuccessors[i]->dvParent);

                if(it==open.end())
                {
                    double step;
                    if(speed==0){
                        step=1;
                    }else{
                        step=ceil(double(edgeLength)/double(speed));
                    }

                    if(!hasConflict(leftSuccessors[i], current->depth + step, constraints))
                    {
                        doubleVertex* newDoubleVertex = new doubleVertex(*leftSuccessors[i]->dvParent);
                        leftSuccessors[i]->dvParent = newDoubleVertex;
                        leftSuccessors[i]->dvParent->parent = current;
                        leftSuccessors[i]->dvParent->depth = current->depth + step;
                        leftSuccessors[i]->dvParent->g = new_cost;
                        leftSuccessors[i]->dvParent->f = leftSuccessors[i]->dvParent->f + heuristicCostEstimate(*leftSuccessors[i]->dvParent, *goal);
                        leftSuccessors[i]->dvParent->visitLeftSubVertex=true;
                        open.push_back(leftSuccessors[i]->dvParent);
                    }


                }
                else if (new_cost < leftSuccessors[i]->dvParent->g){
                    if(!hasConflict(leftSuccessors[i], current->depth + 1, constraints)){
                        (*it)->parent=current;
                        (*it)->depth=current->depth+1;
                        (*it)->g=new_cost;
                        (*it)->f=(*it)->g+ heuristicCostEstimate(*(*it),*goal);
                        (*it)->visitLeftSubVertex=true;
                    }
                }
                else{
                    continue;
                }



                if(!isClosed){
                    closed.push_back(current);
                    isClosed= true;
                }


            }





            for(int i=0;i<rightSuccessors.size();i++){

                int edgeLength;
                int edgeMaxSpeed;

                if(rightSuccessors[i]->x==current->vl.x && rightSuccessors[i]->y==current->vl.y){
                    rightSuccessors[i]->dvParent->g=current->g;
                    rightSuccessors[i]->dvParent->h=current->h;
                    edgeLength=0;
                    edgeMaxSpeed=0;
                }else{
                    edgeLength=get<0>(current->vrlengthAndMaxSpeed[i]);
                    edgeMaxSpeed=get<1>(current->vrlengthAndMaxSpeed[i]);
                }

                int speed=edgeMaxSpeed>agentMaxSpeed?speed=agentMaxSpeed:speed=edgeMaxSpeed;



                if (std::find(closed.begin(), closed.end(), rightSuccessors[i]->dvParent) != closed.end())
                {
                    continue;
                }

                int new_cost = current->g + heuristicCostEstimate(*rightSuccessors[i]->dvParent, *current);



                auto it =std::find(open.begin(),open.end(),rightSuccessors[i]->dvParent);

                if(it==open.end())
                {
                    double step;
                    if(speed==0){
                        step=1;
                    }else{
                        step=ceil(double(edgeLength)/double(speed));
                    }

                    if(!hasConflict(rightSuccessors[i],current->depth+step,constraints))
                    {
                        doubleVertex* newDoubleVertex = new doubleVertex(*rightSuccessors[i]->dvParent);
                        rightSuccessors[i]->dvParent = newDoubleVertex;
                        rightSuccessors[i]->dvParent->parent = current;
                        rightSuccessors[i]->dvParent->depth = current->depth + step;
                        rightSuccessors[i]->dvParent->g = new_cost;
                        rightSuccessors[i]->dvParent->f = rightSuccessors[i]->dvParent->f + heuristicCostEstimate(*rightSuccessors[i]->dvParent, *goal);
                        rightSuccessors[i]->dvParent->visitRightSubVertex=true;
                        open.push_back(rightSuccessors[i]->dvParent);
                    }


                }
                else if (new_cost < rightSuccessors[i]->dvParent->g){
                    if(!hasConflict(rightSuccessors[i],current->depth+1,constraints)){
                        (*it)->parent=current;
                        (*it)->depth=current->depth+1;
                        (*it)->g=new_cost;
                        (*it)->f=(*it)->g+ heuristicCostEstimate(*(*it),*goal);
                        (*it)->visitLeftSubVertex=true;
                    }
                }
                else{
                    continue;
                }




                if(!isClosed){
                    closed.push_back(current);
                    isClosed= true;
                }


            }

        }


        if(current->visitLeftSubVertex){

            if(leftSuccessors.size() != 0){
                Vertex *lc=new Vertex(current->vl);
                addDvParent(current,lc);
                leftSuccessors.push_back(lc);
                clearSuccessors(leftSuccessors, open);
            }else{
                if(*(current)==*(start)){
                    continue;
                }
                closed.push_back(current);
                while(current->getSuccessorsSize(start,goal)<=1){
                    if(current->parent!= nullptr){
                        current=current->parent;
                        openErase(current->vl.x,current->vl.y);
                    }

                }
                open.push_back(current);
                continue;
            }



            for(int i=0; i < leftSuccessors.size(); i++){

                int edgeLength;
                int edgeMaxSpeed;


                if(leftSuccessors[i]->x == current->vl.x && leftSuccessors[i]->y == current->vl.y){
                    leftSuccessors[i]->dvParent->g=current->g;
                    leftSuccessors[i]->dvParent->h=current->h;
                    edgeLength=0;
                    edgeMaxSpeed=0;
                }else{
                    edgeLength=get<0>(current->vllengthAndMaxSpeed[i]);
                    edgeMaxSpeed=get<1>(current->vllengthAndMaxSpeed[i]);
                }

                int speed=edgeMaxSpeed>agentMaxSpeed?speed=agentMaxSpeed:speed=edgeMaxSpeed;

                if (std::find(closed.begin(), closed.end(), leftSuccessors[i]->dvParent) != closed.end())
                {
                    continue;
                }

                int new_cost = current->g + heuristicCostEstimate(*leftSuccessors[i]->dvParent, *current);




                auto it =std::find(open.begin(), open.end(), leftSuccessors[i]->dvParent);

                if(it==open.end())
                {
                    double step;
                    if(speed==0){
                        step=1;
                    }else{
                        step=ceil(double(edgeLength)/double(speed));
                    }

                    if(!hasConflict(leftSuccessors[i], current->depth + step, constraints))  // 不存在冲突
                    {
                        doubleVertex* newDoubleVertex = new doubleVertex(*leftSuccessors[i]->dvParent);
                        leftSuccessors[i]->dvParent = newDoubleVertex;
                        leftSuccessors[i]->dvParent->parent = current;
                        leftSuccessors[i]->dvParent->depth = current->depth + step;
                        leftSuccessors[i]->dvParent->g = new_cost;
                        leftSuccessors[i]->dvParent->f = leftSuccessors[i]->dvParent->f + heuristicCostEstimate(*leftSuccessors[i]->dvParent, *goal);
                        leftSuccessors[i]->dvParent->visitLeftSubVertex=true;
                        open.push_back(leftSuccessors[i]->dvParent);
                    }


                }
                else if (new_cost < leftSuccessors[i]->dvParent->g){
                    if(!hasConflict(leftSuccessors[i], current->depth + 1, constraints)){
                        (*it)->parent=current;
                        (*it)->depth=current->depth+1;
                        (*it)->g=new_cost;
                        (*it)->f=(*it)->g+ heuristicCostEstimate(*(*it),*goal);
                        (*it)->visitLeftSubVertex=true;
                    }
                }
                else{
                    continue;
                }




                if(!isClosed){
                    closed.push_back(current);
                    isClosed= true;
                }
            }



        }


        if(current->visitRightSubVertex){
            if(rightSuccessors.size()!=0&& isAllInClosed(rightSuccessors)){
                Vertex *rc=new Vertex(current->vr);
                addDvParent(current,rc);
                rightSuccessors.push_back(rc);
                clearSuccessors(rightSuccessors,open);
            }else{

                if(*(current)==*(start)){
                    continue;
                }
                closed.push_back(current);
                while(current->getSuccessorsSize(start,goal)<=1){
                    if(current->parent!= nullptr){
                        current=current->parent;
                        openErase(current->vl.x,current->vl.y);
                    }
                }
                open.push_back(current);
                continue;
            }



            for(int i=0;i<rightSuccessors.size();i++){

                int edgeLength;
                int edgeMaxSpeed;

                if(rightSuccessors[i]->x==current->vl.x && rightSuccessors[i]->y==current->vl.y){
                    rightSuccessors[i]->dvParent->g=current->g;
                    rightSuccessors[i]->dvParent->h=current->h;
                    edgeLength=0;
                    edgeMaxSpeed=0;
                }else{
                    edgeLength=get<0>(current->vrlengthAndMaxSpeed[i]);
                    edgeMaxSpeed=get<1>(current->vrlengthAndMaxSpeed[i]);
                }

                int speed=edgeMaxSpeed>agentMaxSpeed?speed=agentMaxSpeed:speed=edgeMaxSpeed;


                if (std::find(closed.begin(), closed.end(), rightSuccessors[i]->dvParent) != closed.end())
                {
                    continue;
                }

                int new_cost = current->g + heuristicCostEstimate(*rightSuccessors[i]->dvParent, *current);



                auto it =std::find(open.begin(),open.end(),rightSuccessors[i]->dvParent);

                if(it==open.end())
                {

                    double step;
                    if(speed==0){
                        step=1;
                    }else{
                        step=ceil(double(edgeLength)/double(speed));
                    }

                    if(!hasConflict(rightSuccessors[i],current->depth+step,constraints))
                    {
                        doubleVertex* newDoubleVertex = new doubleVertex(*rightSuccessors[i]->dvParent);
                        rightSuccessors[i]->dvParent = newDoubleVertex;
                        rightSuccessors[i]->dvParent->parent = current;
                        rightSuccessors[i]->dvParent->depth = current->depth + step;
                        rightSuccessors[i]->dvParent->g = new_cost;
                        rightSuccessors[i]->dvParent->f = rightSuccessors[i]->dvParent->f + heuristicCostEstimate(*rightSuccessors[i]->dvParent, *goal);
                        rightSuccessors[i]->dvParent->visitRightSubVertex=true;
                        open.push_back(rightSuccessors[i]->dvParent);
                    }




                }
                else if (new_cost < rightSuccessors[i]->dvParent->g){
                    if(!hasConflict(rightSuccessors[i],current->depth+1,constraints)){
                        (*it)->parent=current;
                        (*it)->depth=current->depth+1;
                        (*it)->g=new_cost;
                        (*it)->f=(*it)->g+ heuristicCostEstimate(*(*it),*goal);
                        (*it)->visitLeftSubVertex=true;
                    }
                }
                else{
                    continue;
                }




                if(!isClosed){
                    closed.push_back(current);
                    isClosed= true;
                }


            }
        }



    }

}


void lowLevelCBS::initOpenAndClosed(doubleVertex* start)
{
    open.clear();
    closed.clear();
    open.push_back(start);
}

void lowLevelCBS::clearSuccessors(vector<Vertex*> &list,vector<doubleVertex*> open){
    for(auto vertex:list){
        if(open.size()==0){
            vertex->dvParent->g=0;
            vertex->dvParent->f=0;
            vertex->dvParent->h=0;
        }
        else{
            for(auto dv:open){
                if(dv!=vertex->dvParent){
                    vertex->dvParent->g=0;
                    vertex->dvParent->f=0;
                    vertex->dvParent->h=0;
                }
            }
        }


    }
}