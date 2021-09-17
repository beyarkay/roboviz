/*
 * @file Swarm.h
 */

#ifndef ROBOGEN_SWARM_H_
#define ROBOGEN_SWARM_H_

#include "Robot.h"

namespace robogen {
  /**
   * A Swarm is a collection of robots, usually working together to maximise
   * their collective fitness.
   *
   * A swarm of size 1 should be used in preference to only using the Robot
   * class.
   *
   * A swarm is one or more robots which are usually working cooperatively in the
   * RoboGen task environment to achieve some common goal, such as gathering
   * resources.
   *
   * @see Robot
   */
  class Swarm {
    private:
      std::vector<boost::shared_ptr<Robot>> robotVector_;
      int swarmSize_;

    public:
      /// No-argument constructor
      Swarm();

      /// Destructor.
      ~Swarm();

      /*
       * Return a pointer to the i-th robot in the swarm.
       *
       */
      boost::shared_ptr<Robot> getRobot(int i) const;

      /**
       * Add a robot to the end of the swarm vector.
       */
      void addRobot(boost::shared_ptr<Robot> robot);

      /**
       * Return the size of the swarm
       */
      int getSize();
  };
}
#endif /* ROBOGEN_SWARM_H_ */
