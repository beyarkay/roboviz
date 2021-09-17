#ifndef ROBOGEN_SWARM_POSITIONS_CONFIG_H_
#define ROBOGEN_SWARM_POSITIONS_CONFIG_H_

#include <osg/Vec3>
#include <vector>
#include "robogen.pb.h"

namespace robogen {
  /**
   * Contains all the starting positions for the robots in the swarm.
   */
  class SwarmPositionsConfig {
    public:
      /**
       * Initializes an empty starting positions configuration.
       */
      SwarmPositionsConfig() {}

      /**
       * Initialise the starting position configuration with some starting
       * coordinates
       */
      SwarmPositionsConfig(const std::vector<osg::Vec3>& coordinates):
        coordinates_(coordinates) {}

      /**
       * Destructor
       */
      virtual ~SwarmPositionsConfig() {}

      /**
       * Get the starting positions of the swarm as xyz coordinates.
       *
       * @return the coordinates of the starting positions
       */
      const std::vector<osg::Vec3>& getCoordinates() const {
        return coordinates_;
      }

      /**
       * Serialize the SwarmPositionsConfig into a format parseable for
       * ProtoBuf messages
       *
       * @param[out] message        The message to serialise into.
       */
      void serialize(robogenMessage::SimulatorConf &message){
        for (unsigned int i = 0; i < coordinates_.size(); ++i){
          robogenMessage::SwarmPosition *curr = message.add_swarmpositions();
          curr->set_x(coordinates_[i].x());
          curr->set_y(coordinates_[i].y());
          curr->set_z(coordinates_[i].z());
        }
      }

    private:
      /**
       * SwarmPositions coordinates
       */
      std::vector<osg::Vec3> coordinates_;
  };
}
#endif /* ROBOGEN_SWARM_POSITIONS_CONFIG_H_ */
