/*
 * Swarm.h
 *
 */

#ifndef ROBOGEN_SWARM_H_
#define ROBOGEN_SWARM_H_

#include "Robot.h"

namespace robogen {
  class Swarm {
    private:
      std::vector<boost::shared_ptr<Robot>> robotVector_;
      int swarmSize_;

    public:
      Swarm(); //no-argument constructor
      Swarm(int swarmSize); //parameterized constructor
      ~Swarm(); //destructor

      boost::shared_ptr<Robot> getRobot(int i) const; //method to return a pointer to the i'th robot in the swarm

      void addRobot(boost::shared_ptr<Robot> robot); //method to add a robot to the swarm

      // Return the size of the swarm
      int getSize();
  };
}
#endif /* ROBOGEN_SWARM_H_ */
