/**
 * @file Swarm.cpp
 */

#include "Swarm.h"

namespace robogen {
  /**
   * Initialises an empty swarm with a size of zero.
   * @see Swarm::addRobot()
   */
  Swarm::Swarm() : swarmSize_(0) {}

  /**
   * Destructor.
   */
  Swarm::~Swarm() {
    robotVector_.clear();
  }

  /**
   * If i is greater than the size of the swarm or less than zero,
   * a `std::out_of_range` exception will be thrown for the underlying
   * `std::vector` containing the robots.
   */
  boost::shared_ptr<Robot> Swarm::getRobot(int i) const {
    return robotVector_.at(i);
  }

  /**
   * The pointer is pushed to the end of the private robot vector and the size
   * of the swarm is incremented. This logs a message to `std::cout`. There is
   * no way to insert a robot at a certain index inside the swarm, as robots in
   * a swarm are considered unordered. Currently there is no way to remove
   * robots from a swarm once they have been added.
   */
  void Swarm::addRobot(boost::shared_ptr<Robot> robot) {
    std::cout << "[D] Adding robot to swarm" << std::endl;
    robotVector_.push_back(robot);
    swarmSize_++;
  }

  /**
   * The size of the swarm is incremented each time a robot is added to the
   * swarm, and there is no way to remove robots from a swarm once added.
   */
  int Swarm::getSize() {
    return swarmSize_;
  }
}
