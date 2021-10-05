/*
 * @(#) Scenario.cpp   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#include <iostream>
#include "config/RobogenConfig.h"
#include "config/TerrainConfig.h"
#include "model/objects/BoxObstacle.h"
#include "scenario/Scenario.h"
#include "scenario/Terrain.h"
#include "Swarm.h"
#include "Robot.h"
#include "Environment.h"

namespace robogen {

  Scenario::Scenario(boost::shared_ptr<RobogenConfig> robogenConfig) :
    robogenConfig_(robogenConfig), startPositionId_(0),
    stopSimulationNow_(false) {

    }

  Scenario::~Scenario() {

  }

  /**
   * Creates a scenario which contains lights, robot(s), obstacles, etc
   *
   * @param odeWorld a dynamics world, that contains all the simulation data
   * @param odeSpace a collision space, used to organize and speed up collision tests
   * @param swarm The swarm to place into the scenario.
   * @return true if the scenario was created successfully, false otherwise
   */
  bool Scenario::init(dWorldID odeWorld, dSpaceID odeSpace, boost::shared_ptr<Swarm> swarm) {

    environment_ = boost::shared_ptr<Environment>(new Environment(
          odeWorld, odeSpace, robogenConfig_));

    if(!environment_->init()) {
      return false;
    }

    stopSimulationNow_ = false;
    swarm_ = swarm;


    // ===========================================================
    // Build the robots in the environment
    //
    // Set their rotation, position, and log some debugging output
    // about their Axis-Aligned-Bounding-Box
    // ===========================================================
    std::vector<double> overlapMaxZ;
    double rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ;
    std::cout << "[D] Creating and adding " << swarm->getSize()
      << " robots to the swarm" << std::endl;
    for (unsigned int i = 0; i < swarm->getSize(); ++i) {
      boost::shared_ptr<Robot> currRobot = swarm->getRobot(i);
      rMinX = 0;
      rMaxX = 0;
      rMinY = 0;
      rMaxY = 0;
      rMinZ = 0;
      rMaxZ = 0;
      currRobot->getAABB(rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ);
      // If the parameter `swarmPositionConfigFile` has been set and
      // every member of the swarm has a location specified, then use
      // those locations.
      if (robogenConfig_->getSwarmPositionsConfig()->getCoordinates().size() > 0) {
        std::cout << "[D] Using different xyz location for each robot"
          << std::endl;
        // Set up the starting orientation of the robot
        osg::Vec3 startingPosition =
          robogenConfig_->getSwarmPositionsConfig()->getCoordinates().at(i);
        // If the user's specified a silly z-value, warn them of the fact
        if (startingPosition.z() < robogenConfig_->getTerrainConfig()->getHeight()) {
          std::cout << "[W] The specified starting z-value '"
            << startingPosition.z()
            << "' is less than the terrain height '"
            << robogenConfig_->getTerrainConfig()->getHeight()
            << "'. This might cause issues" << std::endl;
        }
        // Translate the robot to the correct starting position
        currRobot->translateRobot(startingPosition);
        currRobot->getAABB(rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ);
        overlapMaxZ.push_back(rMinZ);
        std::cout << "[D] Translating robot to: "
          << "x=" << startingPosition.x()
          << ", y= " << startingPosition.y()
          << ", z=" << startingPosition.z() << std::endl;
      } else {
        // The parameter `swarmPositionsConfigFile` hasn't been set, so just
        // use the location specified by the startPositionFile
        std::cout << "[D] Using the same xy location and azimuth for each robot"
          << std::endl;
        // Rotate the robot around the z-axis by startingAzimuth degrees
        float startingAzimuth = robogenConfig_->getStartingPos()->getStartPosition(
            startPositionId_)->getAzimuth();
        osg::Quat robotRotation;
        robotRotation.makeRotate(
            osg::inDegrees(startingAzimuth), osg::Vec3(0,0,1));
        currRobot->rotateRobot(robotRotation);

        // Translate the robot to startingPosition
        osg::Vec2 startingPosition =
          robogenConfig_->getStartingPos()->getStartPosition(
              startPositionId_)->getPosition();
        currRobot->translateRobot(
            osg::Vec3(startingPosition.x(),
              startingPosition.y(),
              robogenConfig_->getTerrainConfig()->getHeight()
              + inMm(2) - rMinZ));
      }

      // Log some debug information
      std::cout
        << "[I] The " << i << "-th robot is enclosed in the "
        << "Axis-Aligned-Bounding-Box(rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ) ("
        << rMinX << ", " << rMaxX << ", " << rMinY << ", " << rMaxY << ", "
        << rMinZ << ", " << rMaxZ << ")" << std::endl;
      std::cout << "[I] Obstacles in this range will not be generated"
        << std::endl;
    }

    // ==========================================================
    // Create and attempt to add the obstacles to the environment
    //
    // If an obstacle collides with a robot, either raise the
    // obstacle above the robot or remove the obstacle
    // completely
    // ==========================================================

    // -----------------------------------------------------
    // Collect all the information from the obstacles config
    // about the various obstacles
    // -----------------------------------------------------
    boost::shared_ptr<ObstaclesConfig> obstaclesConfig =
      robogenConfig_->getObstaclesConfig();

    // Get the coords of the obstacles
    const std::vector<osg::Vec3>& obstaclesCoords =
      obstaclesConfig->getCoordinates();
    // Get the sizes of the obstacles
    const std::vector<osg::Vec3>& obstacleSizes = obstaclesConfig->getSizes();
    // Get the densities of the obstacles
    const std::vector<float>& obstacleDensities = obstaclesConfig->getDensities();
    // Get the rotation Axes of the obstacles
    const std::vector<osg::Vec3>& rotationAxes =
      obstaclesConfig->getRotationAxes();
    // Get the rotation angles of the obstacles
    const std::vector<float>& rotationAngles =
      obstaclesConfig->getRotationAngles();

    obstaclesOrLightsRemoved_ = false;

    for (unsigned int i = 0; i < obstaclesCoords.size(); ++i) {
      boost::shared_ptr<BoxObstacle> obstacle(new BoxObstacle(
            odeWorld, odeSpace,
            obstaclesCoords[i], obstacleSizes[i],
            obstacleDensities[i], rotationAxes[i],
            rotationAngles[i])
          );
      // Get the axis-aligned-bounding-box of the current obstacle
      double oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ;
      obstacle->getAABB(oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ);

      // ----------------------------------------------------------
      // Only add the obstacle if it doesn't collide with the robot
      // ----------------------------------------------------------
      double rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ;
      for (unsigned int i = 0; i < swarm->getSize(); i++) {
        boost::shared_ptr<Robot> currRobot = swarm->getRobot(i);
        double rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ;
        currRobot->getAABB(rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ);
        // Check if the robot's AABB collides with the obstacles AABB in the X
        // axis
        bool collidesX = (oMinX <= rMinX && oMaxX >= rMaxX) ||
          (oMinX >= rMinX && oMinX <= rMaxX) ||
          (oMaxX >= rMinX && oMaxX <= rMaxX);
        // Check if the robot's AABB collides with the obstacles AABB in the Y
        // axis
        bool collidesY = (oMinY <= rMinY && oMaxY >= rMaxY) ||
          (oMinY >= rMinY && oMinY <= rMaxY) ||
          (oMaxY >= rMinY && oMaxY <= rMaxY);
        // Check if the robot's AABB collides with the obstacles AABB in the Z
        // axis
        bool collidesZ = (oMinZ <= rMinZ && oMaxZ >= rMaxZ) ||
          (oMinZ >= rMinZ && oMinZ <= rMaxZ) ||
          (oMaxZ >= rMinZ && oMaxZ <= rMaxZ);

        // Only add obstacles if they don't collide with the robots
        if (!(collidesX && collidesY && collidesZ)) {
          environment_->addObstacle(obstacle);
        } else {
          if (robogenConfig_->getObstacleOverlapPolicy() ==
              RobogenConfig::ELEVATE_ROBOT) {
            // If the obstacle is above the world's height limit then lower it
            // down to be below the height limit. Then add the obstacle to the
            // world
            if (oMaxZ > overlapMaxZ.at(i)) {
              overlapMaxZ.at(i) = oMaxZ;
            }
            environment_->addObstacle(obstacle);
          } else {
            obstacle->remove();
            obstaclesOrLightsRemoved_ = true;
          }
        }
      }
    }

    // ==============================================================
    // Create and attempt to add the light sources to the environment
    //
    // If an obstacle collides with a robot, either raise the
    // obstacle above the robot or remove the obstacle
    // completely
    // ==============================================================

    // ---------------------------------------------------------
    // Collect all the information from the light sources config
    // about the various obstacles
    // ---------------------------------------------------------

    // Load up the config for the light sources
    boost::shared_ptr<LightSourcesConfig> lightSourcesConfig =
      robogenConfig_->getLightSourcesConfig();

    // Instantiate an empty vector of light source objects. We'll only add
    // light sources to the vector once we know they don't collide with the
    // robots
    std::vector<boost::shared_ptr<LightSource>> lightSources;
    // Load up the coords for the light sources
    std::vector<osg::Vec3> lightSourcesCoords =
      lightSourcesConfig->getCoordinates();
    // Load up the intensities for the light sources
    std::vector<float> lightSourcesIntensities =
      lightSourcesConfig->getIntensities();

    for (unsigned int i = 0; i < lightSourcesCoords.size(); ++i) {
      double lMinX = lightSourcesCoords[i].x() - LightSource::RADIUS;
      double lMaxX = lightSourcesCoords[i].x() + LightSource::RADIUS;
      double lMinY = lightSourcesCoords[i].y() - LightSource::RADIUS;
      double lMaxY = lightSourcesCoords[i].y() + LightSource::RADIUS;
      double lMinZ = lightSourcesCoords[i].z() - LightSource::RADIUS;
      double lMaxZ = lightSourcesCoords[i].z() + LightSource::RADIUS;

      // --------------------------------------------------------------
      // Only add the light source if it doesn't collide with the robot
      // --------------------------------------------------------------
      double rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ;
      for (unsigned int i = 0; i < swarm->getSize(); i++) {
        boost::shared_ptr<Robot> currRobot = swarm->getRobot(i);
        double rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ;
        currRobot->getAABB(rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ);

        // Check if the robot's AABB collides with the light's AABB in the X
        // axis
        bool collidesX = (lMinX <= rMinX && lMaxX >= rMaxX) ||
          (lMinX >= rMinX && lMinX <= rMaxX) ||
          (lMaxX >= rMinX && lMaxX <= rMaxX);
        // Check if the robot's AABB collides with the light's AABB in the Y
        // axis
        bool collidesY = (lMinY <= rMinY && lMaxY >= rMaxY) ||
          (lMinY >= rMinY && lMinY <= rMaxY) ||
          (lMaxY >= rMinY && lMaxY <= rMaxY);
        // Check if the robot's AABB collides with the light's AABB in the Z
        // axis
        bool collidesZ = (lMinZ <= rMinZ && lMaxZ >= rMaxZ) ||
          (lMinZ >= rMinZ && lMinZ <= rMaxZ) ||
          (lMaxZ >= rMinZ && lMaxZ <= rMaxZ);

        // Only add lights to the lightSources vector if they don't collide
        // with the robots
        if (!(collidesX && collidesY && collidesZ)) {
          lightSources.push_back(boost::shared_ptr<LightSource>(
                new LightSource(odeSpace, lightSourcesCoords[i],
                  lightSourcesIntensities[i])));
        } else {
          if (robogenConfig_->getObstacleOverlapPolicy() ==
              RobogenConfig::ELEVATE_ROBOT) {

            if (lMaxZ > overlapMaxZ.at(i)) {
              overlapMaxZ.at(i) = lMaxZ;
            }
            lightSources.push_back(boost::shared_ptr<LightSource>(
                  new LightSource(odeSpace, lightSourcesCoords[i],
                    lightSourcesIntensities[i])));
          } else {
            obstaclesOrLightsRemoved_ = true;
          }
        }
      }
    }
    // Now that we know non of the lights in lightSources collide with the
    // robots, add those lightSources to the environment
    environment_->setLightSources(lightSources);

    // If the user asked that the robot be lifted up when there's an overlap,
    // then raise the robot upwards.
    bool shouldElevate = (robogenConfig_->getObstacleOverlapPolicy() ==
      RobogenConfig::ELEVATE_ROBOT);
    for (int i = 0; i < swarm->getSize(); i++) {
      boost::shared_ptr<Robot> currRobot = swarm->getRobot(i);
      if (shouldElevate) {
        // FIXME here we are actually recalculating startingPosition, since
        // we've already calculated it above (around line 100). Find a way to
        // not recalculate the value.
        if (robogenConfig_->getSwarmPositionsConfig()->getCoordinates().size() > 0) {
          // Set up the starting orientation of the robot
          osg::Vec3 startingPosition =
            robogenConfig_->getSwarmPositionsConfig()->getCoordinates().at(i);
          // Translate the robot to the correct starting position
          currRobot->translateRobot(
              osg::Vec3(startingPosition.x(),
                startingPosition.y(),
                overlapMaxZ.at(i) + inMm(2) - rMinZ));
        } else {
          // Translate the robot to startingPosition
          osg::Vec2 startingPosition =
            robogenConfig_->getStartingPos()->getStartPosition(
                startPositionId_)->getPosition();
          currRobot->translateRobot(
              osg::Vec3(startingPosition.x(),
                startingPosition.y(),
                overlapMaxZ.at(i) + inMm(2) - rMinZ));
        }
      }
      // Physics optimisation: replace all fixed joints with composite bodies
      currRobot->optimizePhysics();
    }
    return true;
  }

  boost::shared_ptr<StartPosition> Scenario::getCurrentStartPosition() {
    return robogenConfig_->getStartingPos()->getStartPosition(
        startPositionId_);
  }

  void Scenario::prune(){
    environment_.reset();
    for (int i = 0; i < swarm_->getSize(); i++) {
      boost::shared_ptr<Robot> currRobot = swarm_->getRobot(i);
      currRobot.reset();
    }
  }

  boost::shared_ptr<Swarm> Scenario::getSwarm() {
    return swarm_;
  }

  boost::shared_ptr<RobogenConfig> Scenario::getRobogenConfig() {
    return robogenConfig_;
  }

  void Scenario::setStartingPosition(int id) {
    startPositionId_ = id;
  }

  boost::shared_ptr<Environment> Scenario::getEnvironment() {
    return environment_;
  }
}
