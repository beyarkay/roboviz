#include <gtest/gtest.h>
#include "Swarm.h"
#include "Robot.h"

namespace robogen {

  /**
   * When a swarm is initialised, then confirm the swarm size is zero
   */
  TEST(SwarmTest, OnInitThenSizeIsZero) {
    boost::shared_ptr<Swarm> swarm(new Swarm);
    EXPECT_EQ(swarm->getSize(), 0);
  }

  /**
   * When a robot is added to a swarm, then confirm the swarm size is
   * incremented
   */
  TEST(SwarmTest, OnAddRobotThenSizeIncrements) {
    boost::shared_ptr<Swarm> swarm(new Swarm);
    for (unsigned int i = 1; i < 100; i++) {
      boost::shared_ptr<Robot> robot(new Robot);
      swarm->addRobot(robot);
      EXPECT_EQ(swarm->getSize(), i);
    }
  }

  /**
   * When a robot is added to the swarm, then confirm getRobot(i) will return
   * that added robot.
   */
  TEST(SwarmTest, OnAddRobotThenReturnsCorrectRobot) {
    boost::shared_ptr<Swarm> swarm(new Swarm);
    for (unsigned int i = 1; i < 100; i++) {
      boost::shared_ptr<Robot> robot(new Robot);
      swarm->addRobot(robot);
      EXPECT_EQ(swarm->getRobot(i-1), robot);
    }
  }

} // namespace robogen
