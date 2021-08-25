#ifndef ROBOGEN_SWARM_H_
#define ROBOGEN_SWARM_H_

#include <boost/shared_ptr.hpp>
#include <map>
#include <vector>

#include "Robogen.h"
#include "robogen.pb.h"

extern "C" {
#include "brain/NeuralNetwork.h"
}

namespace robogen {

  /**
   * A ROBOGEN Swarm
   */
  class Swarm {

    public:
      /**
       * Error-less constructor.
       */
      Swarm();

      /**
       * Initializes a Robogen robot from a robogen message
       * @param odeWorld
       * @param odeSpace
       * @param robotSpec
       */
      bool init(dWorldID odeWorld, dSpaceID odeSpace,
          const robogenMessage::Swarm& robotSpec,
          bool printInfo=false, bool printInitErrors=true);

      /**
       * Destructor
       */
      virtual ~Swarm();

      /**
       *  @return  the body parts of which the robot is composed of
       */
      const std::vector<boost::shared_ptr<Model> >& getBodyParts();

      /**
       * @return the neural network that controls the robot
       */
      const boost::shared_ptr<NeuralNetwork>& getBrain() const;

      /**
       * @return the robot ID
       */
      int getId() const;

      /**
       * @return message that generated the robot
       */
      const robogenMessage::Swarm& getMessage();

      const std::vector<boost::shared_ptr<Connection> >& getBodyConnections()
        const;
      void traverseBody(const std::vector<boost::shared_ptr<Model> >,
          const std::vector<boost::shared_ptr<Connection> >);
      int getRoot();

      inline void addJoint(boost::shared_ptr<Joint> joint) {
        joints_.push_back(joint);
      }

      /**
       * Merges bodies connected with fixed joints into complex bodies
       */
      void optimizePhysics();

    private:

      /**
       * ODE physics world
       */
      dWorldID odeWorld_;

      /**
       * ODE collision space
       */
      dSpaceID odeSpace_;

      /**
       * Swarm robots
       */
      std::vector<boost::shared_ptr<Robot> > robots;

      /**
       * Swarm ID
       */
      int id_;


      /**
       * Contains the robot message
       */
      const robogenMessage::Swarm* swarmMessage_;

      bool printInfo_;

      bool printInitErrors_;

  };

}

#endif /* ROBOGEN_SWARM_H_ */
