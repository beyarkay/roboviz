/**
 * Swarm.cpp
 *
 * A swarm is a collection of robots, which
 * usually are working cooperatively in the
 * RoboGen task environment to achieve some
 * common goal, such as gathering resources
 *
 */

// TODO do we need to include something
// to use boost::shared_ptr?
#include "Swarm.h"

namespace robogen {
  Swarm::Swarm() : swarmSize_(0) {}

  Swarm::~Swarm() {
    robotVector_.clear();
  }

  boost::shared_ptr<Robot> Swarm::getRobot(int i) const {
    return robotVector_.at(i);
  }

  void Swarm::addRobot(boost::shared_ptr<Robot> robot) {
    std::cout << "[D] Adding robot to swarm" << std::endl;
    robotVector_.push_back(robot);
    swarmSize_++;
  }

  int Swarm::getSize() {
    return swarmSize_;
  }
}
