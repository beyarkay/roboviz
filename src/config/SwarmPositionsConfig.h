#ifndef ROBOGEN_SWARM_POSITIONS_CONFIG_H_
#define ROBOGEN_SWARM_POSITIONS_CONFIG_H_

#include <osg/Vec3>
#include <vector>
#include "robogen.pb.h"

namespace robogen {
  /**
   * SwarmPositions configuration parameters
   */
  class SwarmPositionsConfig {
    public:
      /**
       * Initializes starting positions configuration
       */
      SwarmPositionsConfig() {}

      SwarmPositionsConfig(const std::vector<osg::Vec3>& coordinates):
        coordinates_(coordinates) {}

      /**
       * Destructor
       */
      virtual ~SwarmPositionsConfig() {}

      /**
       * @return the coordinates of the starting positions
       */
      const std::vector<osg::Vec3>& getCoordinates() const {
        return coordinates_;
      }

      /**
       * Serialize the SwarmPositionsConfig into a format parseable for
       * ProtoBuf messages
       *
       * @param message The message to serialise into.
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
