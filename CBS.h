#ifndef DS_CBS_CBS_H
#define DS_CBS_CBS_H

#include <vector>
#include "LowLevel.h"
#include "Conflict.h"
#include "readMapAndAgent.h"
#include "Map.h"
#include "Constraint.h"
class CTNode{
public:
    int cost;

    const std::vector<Path*>& getSolution(){
        int count[solution.size()];
        for(int i=0;i<solution.size();i++){
            vector<doubleVertex*> temp;
            for (size_t j = 0; j < solution[i]->Nodes.size() - 1; ++j){
                doubleVertex* currentNode = solution[i]->Nodes[j];
                doubleVertex* nextNode = solution[i]->Nodes[j+1];
                temp.push_back(currentNode);

                for (int t = currentNode->depth + 1; t < nextNode->depth; ++t) {
                    doubleVertex* newDoubleVertex = new doubleVertex(*currentNode,t);
                    temp.push_back(newDoubleVertex);
                }
            }
            temp.push_back(solution[i]->Nodes.back());
            solution[i]->Nodes=temp;
        }
        return solution;
    }

    void clearConflicts()
    {
        conflicts.clear();
    }

    void addConflict(Conflict* newConflict)
    {
        conflicts.push_back(newConflict);
    }


    void setSolution(const vector <Path*> &newSolution)
    {
        solution.clear();

        for (int i = 0; i < newSolution.size(); i++)
        {
            Path* new_path = new Path(newSolution[i]->agentIndex);
            new_path->Nodes = std::vector<doubleVertex*>(newSolution[i]->Nodes);

            for (int j = 0; j < newSolution[i]->Constraints.size(); j++)
            {
                new_path->Constraints.push_back(new Constraint(newSolution[i]->Constraints[j]->agent, newSolution[i]->Constraints[j]->doubleVertex, newSolution[i]->Constraints[j]->timeStep));
            }

            solution.push_back(new_path);
        }
    }

    int getFirstArriveTime(int agentIndex,doubleVertex *ConflictVertex){
        int time=-1;
        for(int i=0;i<solution[agentIndex]->Nodes.size();i++){
            if(solution[agentIndex]->Nodes[i]->vl.x==ConflictVertex->vl.x&&solution[agentIndex]->Nodes[i]->vl.y==ConflictVertex->vl.y){
                time=i;
                break;
            }
        }
        if(!time){
            cout<<"路径错误"<<endl;
        } else
            return time;

    }

    const Conflict& getFirstConflict() const
    {
        return *conflicts[0];
    }

    int getConstraintTime(int conflictTime,Path* s1){
        int time=conflictTime;
        int i=conflictTime;
        int i2=conflictTime;
        while(i>=0){
            if(s1->Nodes[i]->getSuccessorsSize(*(s1->Nodes.begin()),*(s1->Nodes.end()-1))<=1){
                i--;
            }
            else
                break;
        }

        if(i<0){
            while(i2<s1->Nodes.size()) {
                if (s1->Nodes[i2]->getSuccessorsSize(*(s1->Nodes.begin()),
                                                     *(s1->Nodes.end()-1)) <= 1){
                    i2++;
                }
                else
                    break;
            }
        }

        if(i>=0){
            time=i;
        }else if(i2!=s1->Nodes.size())
            time=i2;
        return time;
    }

    int getNextTime(int time, vector<doubleVertex*> nodes){
        for(int i=time;i<nodes.size();i++){
            if(nodes[i]->vl.x!=nodes[time]->vl.x){
                return i;
            }
        }
    }

    pair<pair<int,doubleVertex*>,pair<int,doubleVertex*>> checkConflict(Conflict *conflict){
        int j=conflict->agents[0]->index;
        int k=conflict->agents[1]->index;
        int time1=conflict->timeStep;
        int time2=conflict->timeStep;



        if(conflict->V== nullptr){
            return make_pair(make_pair(time1,solution[j]->Nodes[time1]), make_pair(time2,solution[k]->Nodes[time2]));
        }


        if(this->solution[j]->Nodes[conflict->Time[0]-1]->vl.y==this->solution[k]->Nodes[conflict->Time[1]-1]->vl.y&&(this->solution[j]->Nodes[conflict->Time[0]+1]->vl.y==this->solution[k]->Nodes[conflict->Time[1]+1]->vl.y)){
            time1= getConstraintTime(conflict->timeStep,this->solution[j]);
            time2= getConstraintTime(conflict->timeStep,this->solution[k]);


            if(time1<conflict->timeStep&&time2<conflict->timeStep){
                return make_pair(make_pair(time1+1,solution[j]->Nodes[time1+1]), make_pair(time2,solution[k]->Nodes[time2]));
            }


            if(time1>time2){
                if(time1>conflict->timeStep){
                    for(int index=0;index<this->solution[k]->Nodes.size();index++){
                        if(*(this->solution[j]->Nodes[time1])==*(this->solution[k]->Nodes[index])){
                            time2=index;
                            time1= getNextTime(time1,this->solution[j]->Nodes);
                            break;
                        }
                    }
                }
                else
                    time1++;
            }else
            {
                if(time2>conflict->timeStep){
                    time2++;
                    for(int index=0;index<this->solution[j]->Nodes.size();index++){
                        if(*(this->solution[k]->Nodes[time2-1])==*(this->solution[j]->Nodes[index])){
                            time1=index;
                            break;
                        }
                    }
                }
                else
                    time2++;
            }


        }

        return make_pair(make_pair(time1,solution[j]->Nodes[time1]), make_pair(time2,solution[k]->Nodes[time2]));
    }

    void addConstraints(const vector<Constraint*> oldConstraintList,Constraint* newConstraint){
        constraint.clear();
        for(int i=0;i<oldConstraintList.size();i++){
            constraint.push_back(new Constraint(oldConstraintList[i]->agent,oldConstraintList[i]->doubleVertex,oldConstraintList[i]->timeStep));
        }
        constraint.push_back(newConstraint);
    }

    void addConstraints(Constraint* newConstraint){
        constraint.push_back(newConstraint);
    }

    const vector<Constraint*> getConstraint() const {
        return constraint;
    }

    void setSolutionForAgent(agent &agent){
        solution[agent.index]=new Path(agent.index);
        solution[agent.index]->Nodes=std::vector<doubleVertex*>(agent.path->Nodes);
    }

    Path* getPathByAgentIndex(int agentIndex){
        for(auto s:solution){
            if(s->agentIndex==agentIndex){
                return s;
            }
        }
    }

    CTNode(){
        source=-1;
    }

    void changeSource(){
        source=1;
    }

    int getSource(){
        return source;
    }

    void updateExpand(int expand){
        totalExpand+=expand;
    }


    void setExpand(int expand){
       totalExpand=expand;
    }

    int getTotalExpand(){
        return totalExpand;
    }

private:
    std::vector<Path*> solution;
    vector<Constraint*> constraint;
    vector<Conflict*> conflicts;
    int source;
    int totalExpand;
};


class highLevelCBS {

public:
    vector <Path*> RunCBS();
    bool ValidPathsInCTNode(CTNode&node);
    vector <Path*> findPathsForAllAgents(CTNode &node);
    int getSIC(const vector < Path*> &solution);
    bool doIntersect(doubleVertex* A,doubleVertex* B,doubleVertex* C,doubleVertex* D);
    bool UpdateSolutionByLowLevel(CTNode &node, int agentIndex);
    void printSolution(CTNode* ct);
    bool checkNewSolution(CTNode&node,int agent0,int agnet1);
    bool retrieveAndPopCTNodeWithLowestCost(CTNode** node);
    int getExpand();

    void readInput(){
        initMapFromXML("D:\\clone\\DS_CBS\\MapData\\SL_Task3.xml", m);
        lowLevelCBS.m=this->m;
        _agents=ReadAgentsFromXML("D:\\clone\\DS_CBS\\AgentData\\SL_Task3_10Agent.xml");

    }


private:
    vector<agent*> _agents;
    Map m;
    lowLevelCBS lowLevelCBS;
    vector<CTNode*> _open;
    int highExpand=0;
};



#endif //DS_CBS_CBS_H
