#include "CBS.h"


bool highLevelCBS::UpdateSolutionByLowLevel(CTNode &node, int agentIndex){
    doubleVertex* startV=m.getVertex(_agents[agentIndex]->startX,_agents[agentIndex]->startY);
    doubleVertex* goalV=m.getVertex(_agents[agentIndex]->goalX,_agents[agentIndex]->goalY);
    if(lowLevelCBS.Astar(startV,goalV,*(_agents[agentIndex]->path),node.getConstraint(),_agents[agentIndex]->speed)){
        _agents[agentIndex]->path->agentIndex=agentIndex;
        node.setSolutionForAgent(*_agents[agentIndex]);
        return true;
    }
    else
        return false;
}


vector <Path*> highLevelCBS::RunCBS(){
    CTNode *root =new CTNode();
    root->setSolution(findPathsForAllAgents(*root));
    root->cost= getSIC(root->getSolution());
    _open.push_back(root);


    while(!_open.empty()){
        CTNode *p;
        if(retrieveAndPopCTNodeWithLowestCost(&p)){
            highExpand++;
            bool valid = ValidPathsInCTNode(*p);

            if(valid){
                p->setExpand(getExpand());
                printSolution(p);
                return p->getSolution();
            }
            int conTime=p->getFirstConflict().timeStep;



            Conflict conflict =p->getFirstConflict();

            bool pathFound;
            if(conflict.agents[1]!= nullptr){



                for(int i=0;i<2;i++){
                    CTNode* newCTNode=new CTNode();

                    pair<pair<int,doubleVertex*>,pair<int,doubleVertex*>> timeAndV=p->checkConflict(&conflict);


                    if(timeAndV.first.first!=conflict.timeStep){
                        if(timeAndV.first.first-1<conflict.timeStep&&timeAndV.second.first<conflict.timeStep){
                            newCTNode->addConstraints(p->getConstraint(),new Constraint(conflict.agents[i],timeAndV.first.second,timeAndV.first.first));
                        }
                        else if(timeAndV.first.first>=timeAndV.second.first){
                            newCTNode->addConstraints(p->getConstraint(),new Constraint(conflict.agents[i],timeAndV.first.second,timeAndV.first.first));

                        }else if (timeAndV.first.first<timeAndV.second.first){
                            newCTNode->addConstraints(p->getConstraint(),new Constraint(conflict.agents[i+1],timeAndV.second.second,timeAndV.second.first));
                        }
                    }else{
                        if(conflict.Time[0]==conflict.Time[1]){
                            if(conflict.V== nullptr){
                                newCTNode->addConstraints(p->getConstraint(),new Constraint(conflict.agents[i],p->getSolution()[i]->Nodes[conflict.Time[i]],conflict.Time[i]));
                            }else{
                                newCTNode->addConstraints(p->getConstraint(),new Constraint(conflict.agents[i],conflict.V,conflict.Time[i]));
                            }

                        }else{
                            if(i==0){
                                if(conflict.Time[0]<conflict.Time[1]){
                                    newCTNode->changeSource();
                                    int Time0=conflict.Time[0];
                                    int Time1=p->getNextTime(conflict.Time[1],p->getSolution()[1]->Nodes);
                                    for(int s=0;s<=Time1-Time0;s++){
                                        if(s==0){
                                            newCTNode->addConstraints(p->getConstraint(),new Constraint(conflict.agents[i],conflict.V,conflict.Time[i]+s));
                                        }else
                                            newCTNode->addConstraints(new Constraint(conflict.agents[i],conflict.V,conflict.Time[i]+s));
                                    }

                                }else{
                                    newCTNode->changeSource();
                                    int Time0=conflict.Time[0];
                                    int Time1=conflict.Time[1];
                                    for(int s=0;s<=Time0-Time1;s++){
                                        if(s==0){
                                            newCTNode->addConstraints(p->getConstraint(),new Constraint(conflict.agents[i],conflict.V,conflict.Time[i]+s));
                                        }else
                                            newCTNode->addConstraints(new Constraint(conflict.agents[i],conflict.V,conflict.Time[i]+s));
                                    }
                                }
                            }else if (i==1){
                                if(conflict.Time[1]<=conflict.Time[0]){
                                    newCTNode->changeSource();
                                    int Time0=conflict.Time[0];
                                    int Time1=conflict.Time[1];
                                    for(int s=0;s<=Time0-Time1;s++){
                                        if(s==0){
                                            newCTNode->addConstraints(p->getConstraint(),new Constraint(conflict.agents[i],conflict.V,conflict.Time[i]+s));
                                        }else
                                            newCTNode->addConstraints(new Constraint(conflict.agents[i],conflict.V,conflict.Time[i]+s));
                                    }

                                }else{
                                    newCTNode->changeSource();
                                    int Time0=conflict.Time[0];
                                    int Time1=conflict.Time[1];
                                    for(int s=0;s<=Time1-Time0;s++){
                                        if(s==0){
                                            newCTNode->addConstraints(p->getConstraint(),new Constraint(conflict.agents[i],conflict.V,conflict.Time[i]+s));
                                        }else
                                            newCTNode->addConstraints(new Constraint(conflict.agents[i],conflict.V,conflict.Time[i]+s));
                                    }
                                }
                            }
                        }

                    }


                    newCTNode->setSolution(p->getSolution());

                    if(timeAndV.first.first==conflict.timeStep){
                        pathFound = UpdateSolutionByLowLevel(*newCTNode,conflict.agents[i]->index);
                    }
                    else if(timeAndV.first.first>=timeAndV.second.first){
                        pathFound = UpdateSolutionByLowLevel(*newCTNode,conflict.agents[i]->index);
                    }
                    else if(timeAndV.first.first<timeAndV.second.first){
                        pathFound = UpdateSolutionByLowLevel(*newCTNode,conflict.agents[i+1]->index);
                    }


                    if(timeAndV.first.first!=conflict.timeStep&&!checkNewSolution(*newCTNode,conflict.agents[i]->index,conflict.agents[i+1]->index)){
                        if(timeAndV.first.first>=timeAndV.second.first){
                            for(int u=0;u<=timeAndV.first.first-timeAndV.second.first;u++){
                                newCTNode->addConstraints(new Constraint(conflict.agents[i+1],timeAndV.second.second,timeAndV.second.first+u));
                            }
                            pathFound = UpdateSolutionByLowLevel(*newCTNode,conflict.agents[i+1]->index);
                            if(pathFound){
                                newCTNode->cost= getSIC(newCTNode->getSolution());
                                if(newCTNode->cost<INT_MAX){
                                    _open.push_back(newCTNode);
                                }
                            }
                            break;
                        }else if(timeAndV.first.first<timeAndV.second.first){
                            for(int u=0;u<timeAndV.second.first-timeAndV.first.first;u++){
                                newCTNode->addConstraints(new Constraint(conflict.agents[i],timeAndV.first.second,timeAndV.first.first+u));
                            }
                            pathFound = UpdateSolutionByLowLevel(*newCTNode,conflict.agents[i]->index);
                            if(pathFound){
                                newCTNode->cost= getSIC(newCTNode->getSolution());

                                if(newCTNode->cost<INT_MAX){
                                    _open.push_back(newCTNode);
                                }
                            }
                            break;
                        }

                    }


                    if(pathFound){
                        newCTNode->cost= getSIC(newCTNode->getSolution());

                        if(newCTNode->cost<INT_MAX){
                            _open.push_back(newCTNode);
                        }
                    }
                }



            }
            else if(conflict.agents[1]== nullptr){



                CTNode* newCTNode=new CTNode();
                newCTNode->addConstraints(p->getConstraint(),new Constraint(conflict.agents[0], conflict.V,conflict.timeStep));
                newCTNode->setSolution(p->getSolution());
                pathFound= UpdateSolutionByLowLevel(*newCTNode,conflict.agents[0]->index);
                if(pathFound){
                    newCTNode->cost= getSIC(newCTNode->getSolution());
                    if(newCTNode->cost<INT_MAX){
                        _open.push_back(newCTNode);
                    }
                }


            }

           delete p;

        }
    }
}






bool highLevelCBS::ValidPathsInCTNode(CTNode&node){
    bool valid_solution=true;
    node.clearConflicts();

    int lastTimeStep=0;
    vector<Path*> solution =node.getSolution();

    if(solution.size()==0){return false;}

    for(int i=0;i<solution.size();i++){
        int currentNodeSize =solution[i]->Nodes.size();
        if(lastTimeStep<currentNodeSize){
            lastTimeStep=currentNodeSize;
        }
    }

    for(int i=0;i<lastTimeStep;i++){

        for(int j=0;j<solution.size();j++){


            if(solution[j]->Nodes.size()==0) continue;

            int a=std::min((int)solution[j]->Nodes.size()-1,i);

            for(int k=0;k<solution.size();k++){
                if(j==k) continue;
                if(solution[k]->Nodes.size()==0) continue;
                int b=std::min((int)solution[k]->Nodes.size()-1,i);


                if(*(solution[j]->Nodes[a]) == *(solution[k]->Nodes[b])){
                    if(solution[k]->Nodes[b]==solution[k]->Nodes[solution[k]->Nodes.size()-1]||solution[j]->Nodes[a]==solution[j]->Nodes[solution[j]->Nodes.size()-1]){  //到达终点认为不产生冲突
                        continue;
                    }

                    int agent1Time=node.getFirstArriveTime(solution[j]->agentIndex,solution[j]->Nodes[a]);
                    int agent2Time=node.getFirstArriveTime(solution[k]->agentIndex,solution[j]->Nodes[a]);

                    node.addConflict(new Conflict(_agents[solution[j]->agentIndex],_agents[solution[k]->agentIndex],solution[j]->Nodes[a],agent1Time,agent2Time,i));
                    valid_solution= false;
                    return valid_solution;
                }


                if(a!=(int)solution[j]->Nodes.size()-1&&b!=(int)solution[k]->Nodes.size()-1){

                    if(doIntersect(solution[j]->Nodes[a],solution[j]->Nodes[a+1],solution[k]->Nodes[b],solution[k]->Nodes[b+1])){



                        int agent1Time=node.getFirstArriveTime(solution[j]->agentIndex,solution[j]->Nodes[a]);
                        int agent2Time=node.getFirstArriveTime(solution[k]->agentIndex,solution[k]->Nodes[b]);

                        node.addConflict(new Conflict(_agents[solution[j]->agentIndex],_agents[solution[k]->agentIndex],
                                                      nullptr,agent1Time,agent2Time,i));
                        valid_solution= false;
                        return valid_solution;
                    }

                }


                if(a!=(int)solution[j]->Nodes.size()-1&&b!=(int)solution[k]->Nodes.size()-1){
                    if(*(solution[j]->Nodes[a+1])==*(solution[k]->Nodes[b])&&*(solution[j]->Nodes[a])==*(solution[k]->Nodes[b+1])){
                        int agent1Time=node.getFirstArriveTime(solution[j]->agentIndex,solution[j]->Nodes[a]);
                        int agent2Time=node.getFirstArriveTime(solution[k]->agentIndex,solution[j]->Nodes[a]);
                        node.addConflict(new Conflict(_agents[solution[j]->agentIndex],_agents[solution[k]->agentIndex],solution[j]->Nodes[a],agent1Time,agent2Time,i));
                        valid_solution= false;
                        return valid_solution;
                    }
                }





            }
        }
    }

    return valid_solution;
}

bool highLevelCBS::checkNewSolution(CTNode& node,int agent0,int agent1){
    bool valid=true;
    int lastTimeStep=0;
    Path *p1 = node.getPathByAgentIndex(agent0);
    Path *p2 = node.getPathByAgentIndex(agent1);
    vector<Path*> solution={p1,p2};
    for(int i=0;i<solution.size();i++){
        int currentNodeSize =solution[i]->Nodes.size();
        if(lastTimeStep<currentNodeSize){
            lastTimeStep=currentNodeSize;
        }
    }


    for(int i=0;i<lastTimeStep;i++){

        for(int j=0;j<solution.size();j++){


            if(solution[j]->Nodes.size()==0) continue;

            int a=std::min((int)solution[j]->Nodes.size()-1,i);

            for(int k=0;k<solution.size();k++){
                if(j==k) continue;
                if(solution[k]->Nodes.size()==0) continue;
                int b=std::min((int)solution[k]->Nodes.size()-1,i);


                if(*(solution[j]->Nodes[a]) == *(solution[k]->Nodes[b])){

                    valid= false;
                    return valid;
                }



                if(a!=(int)solution[j]->Nodes.size()-1&&b!=(int)solution[k]->Nodes.size()-1){
                    if(*(solution[j]->Nodes[a+1])==*(solution[k]->Nodes[b])&&*(solution[j]->Nodes[a])==*(solution[k]->Nodes[b+1])){

                        valid= false;
                        return valid;
                    }
                }



            }
        }
    }
}


double crossProduct(doubleVertex* P1,doubleVertex* P2,doubleVertex* P3,doubleVertex* P4){
    return ((double)P2->vl.x-(double)P1->vl.x)*((double)P4->vl.y-(double)P3->vl.y)-((double)P2->vl.y-(double)P1->vl.y)*((double)P4->vl.x-(double)P3->vl.x);
}

bool highLevelCBS::doIntersect(doubleVertex* A,doubleVertex* B,doubleVertex* C,doubleVertex* D){
    double cross1 = crossProduct(A, B, A, C);
    double cross2 = crossProduct(A, B, A, D);
    double cross3 = crossProduct(C, D, C, A);
    double cross4 = crossProduct(C, D, C, B);

    return (cross1 * cross2 < 0) && (cross3 * cross4 < 0);   //(cross1 * cross2 < 0) 且 (cross3 * cross4 < 0)则相交，存在冲突
}


vector<Path*> highLevelCBS::findPathsForAllAgents(CTNode &node)
{
    vector<Path*> paths;
    //m.printGraph();




    for(int i=0;i<_agents.size();i++){
        doubleVertex* startV=m.getVertex(_agents[i]->startX,_agents[i]->startY);
        doubleVertex* goalV=m.getVertex(_agents[i]->goalX,_agents[i]->goalY);
        lowLevelCBS.Astar(startV,goalV,*_agents[i]->path,node.getConstraint(),_agents[i]->speed);
        paths.push_back(_agents[i]->path);
    }

    return paths;
}

int highLevelCBS::getSIC(const vector < Path*> &solution)
{
    int cost = 0;
    for (int i = 0; i < solution.size(); i++)
    {
        cost += solution[i]->get_cost();
    }

    return cost;
}


void highLevelCBS::printSolution(CTNode* ct){
    int makespan=0;
    cout<<"------------solution------------"<<endl;
    auto it=ct->getSolution();
    for(int ii=0;ii<it.size();ii++){
        cout<<"agent"<<it[ii]->agentIndex<<":"<<" ("<<it[ii]->Nodes[0]->vl.x<<","<<it[ii]->Nodes[0]->vl.y<<") to"<<" ("<<it[ii]->Nodes[it[ii]->Nodes.size()-1]->vl.x<<","<<it[ii]->Nodes[it[ii]->Nodes.size()-1]->vl.y<<") "<<endl;
        if(it[ii]->Nodes.size()==0){
            cout<<"未找到路径"<<endl;
            continue;
        }
        for(int j=0;j<it[ii]->Nodes.size();j++){
            if(it[ii]->Nodes.size()>makespan) makespan=it[ii]->Nodes.size();
            cout<<"timeStep "<<it[ii]->Nodes[j]->depth<<": "<<"("<<it[ii]->Nodes[j]->vl.x<<","<<it[ii]->Nodes[j]->vl.y<<")"<<endl;
        }
        cout<<endl;
    }
    cout<<"------------solution------------"<<endl;
    cout<<"Cost: "<<ct->cost<<endl;
    cout<<"Makespan: "<<makespan<<endl;
    cout<<"Expand: "<<ct->getTotalExpand()<<endl;
}