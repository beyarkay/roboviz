/*
 *Swarm.h
 *
 */

#ifndef SWARM
#define SWARM

#include "Robot.h"

class Swarm
{
    private:
        std::vector< std::shared_ptr<Robot> > robotVector;
        int swarmSize;

    public:
        Swarm(); //no-argument constructor
        Swarm(int swarmSize); //parameterized constructor
        ~Swarm(); //destructor

        std::shared_ptr<Robot> getRobot(int i) const; //method to return a pointer to the i'th robot in the swarm

        void addRobot(std::shared_ptr<Robot> robot); //method to add a robot to the swarm
};
#endif
