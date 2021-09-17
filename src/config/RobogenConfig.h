/*
 * @(#) RobogenConfig.h   1.0   Mar 12, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_ROBOGEN_CONFIG_H_
#define ROBOGEN_ROBOGEN_CONFIG_H_

#include <boost/shared_ptr.hpp>
#include "config/ObstaclesConfig.h"
#include "config/SwarmPositionsConfig.h"
#include "config/StartPositionConfig.h"
#include "config/TerrainConfig.h"
#include "config/LightSourcesConfig.h"
#include "robogen.pb.h"

namespace robogen {

/**
 * Configuration parameters used to define the experiment which the swarm
 * will go through.
 *
 * This class contains all parameters as specified in the configuration file,
 * and all files referenced by that configuration file. The parameters are used
 * to define the specifics of the scenario, environment, and fitness function
 * which will be used to test the swarm in an experiment.
 *
 */
class RobogenConfig {

public:

    /**
     * Specify what should be done if the positions  of two objects overlap,
     * where those positions have been specified by the user in the
     * configuration file(s).
     */
    enum ObstacleOverlapPolicies {
        REMOVE_OBSTACLES,         /*!< Remove both obstacles if they overlap */
        CONSTRAINT_VIOLATION,     /*!< Raise a constraint violation error if two obstacles overlap */
        ELEVATE_ROBOT             /*!< Raise the robot above the obstacle if the robot overlaps with the obstacle */
    };

    /**
     * Initializes a robogen config object from configuration parameters.
     *
     * @param[in] scenario          The type of scenario which
     *                              dictates the fitness function to be used. One of 'racing' (fitness dictated by
     *                              distance from the starting position), 'chasing' (fitness dictated by distance
     *                              from light sources) or a scenarioFile can be specified containing a custom
     *                              javascript defined scenario and fitness function.
     * @param[in] scenarioFile      The *.js file containing a description of the custom scenario.
     * @param[in] timeSteps         The total number of timesteps to run before ending the scenario.
     * @param[in] timeStepLength    The duration of each timestep. Total duration of the simulation = timeSteps * timeStepLength;
     * @param[in] actuationPeriod   Duration between motor actuations, in number of timesteps.
     * @param[in] terrain           The terrain of the robot. Either defined by a flat plane or by a height image.
     * @param[in] obstacles         The configuration defining all the obstacles in the scenario and related parameters.
     * @param[in] obstacleFile      The file defining each of the obstacles in the scenario.
     * @param[in] startPositions    The starting position for a robot will be chosen from this list of starting position configurations
     * @param[in] startPosFile      The file defining each of the starting positions
     * @param[in] lightSources      The light sources in the scenario, used for the 'chasing' scenario to determine fitness.
     * @param[in] lightSourceFile   The file defining the light sources to be used in the scenario.
     * @param[in] sensorNoiseLevel  The amount of noise to add to the sensors to simulate real-life inaccuracies when recording sensor data.
     * @param[in] motorNoiseLevel   The amount of noise to add to the motors to simulate real-life inaccuracies when sending commands to the motors.
     * @param[in] capAcceleration   If true, limit the maximum allowed linear acceleartion to `maxLinearAcceleration` and the maximum allowed
                                    angular acceleration to `maxAngularAcceleration`. This is used to stop the simulation getting out of control or to
                                    stop the genetic algorithm from exploiting quirks in the physics engine which
                                    might permit unrealistic accelerations.
     * @param[in] maxLinearAcceleration         The maximum allowed linear acceleration, see capAcceleration. Ignored if capAcceleration is false
     * @param[in] maxAngularAcceleration        The maximum allowed angular acceleration, see capAcceleration. Ignored if capAcceleration is false
     * @param[in] maxDirectionShiftsPerSecond   If a motor changes direction more than `maxDirectionShiftsPerSecond`, then the simulator will disable it,
                                                simulating real-life burnout of motors. if the value is -1, no burnout will occur.
     * @param[in] gravity                       The force to apply for gravity, by default -9.81m.s^-2 in the z direction, and zero in x and y.
     * @param[in] disallowObstacleCollisions    If true, then any obstacle collision will raise a constraint violation.
     * @param[in] swarmSize                     The number of robots in the swarm.
     * @param[in] gatheringZoneSize             The size of the gathering zone, as used for custom swarm based scenarios
     * @param[in] gatheringZonePos              The position of the gathering zone, as used for custom swarm based scenarios
     * @param[in] resourcesConfigFile           The configuration file defining the resources that need to be collected by the
                                                swarm, as used for custom swarm based scenarios
     * @param[in] swarmPositions                The locations of the members of the swarm. Takes priority over startPositions. The number of positions
                                                should be equal to swarmSize.
     * @param[in] swarmPositionsFile            The file containing the swarm positions.
     * @param[in] obstacleOverlapPolicy         What to do when obstacles overlap. See the enum ObstacleOverlapPolicies
     */
    RobogenConfig(
        std::string scenario,
        std::string scenarioFile,
        unsigned int timeSteps,
        float timeStepLength,
        int actuationPeriod,
        boost::shared_ptr<TerrainConfig> terrain,
        boost::shared_ptr<ObstaclesConfig> obstacles,
        std::string obstacleFile,
        boost::shared_ptr<StartPositionConfig> startPositions,
        std::string startPosFile,
        boost::shared_ptr<LightSourcesConfig> lightSources,
        std::string lightSourceFile,
        float sensorNoiseLevel,
        float motorNoiseLevel,
        bool capAcceleration,
        float maxLinearAcceleration,
        float maxAngularAcceleration,
        int maxDirectionShiftsPerSecond,
        osg::Vec3 gravity,
        bool disallowObstacleCollisions,
        unsigned int swarmSize,
        osg::Vec3 gatheringZoneSize,
        osg::Vec3 gatheringZonePos,
        std::string resourcesConfigFile,
        boost::shared_ptr<SwarmPositionsConfig> swarmPositions,
        std::string swarmPositionsFile,
        unsigned int obstacleOverlapPolicy) :
      scenario_(scenario),
      scenarioFile_(scenarioFile),
      timeSteps_(timeSteps),
      timeStepLength_(timeStepLength),
      actuationPeriod_(actuationPeriod),
      terrain_(terrain),
      obstacles_(obstacles),
      obstacleFile_(obstacleFile),
      startPositions_(startPositions),
      startPosFile_(startPosFile),
      lightSources_(lightSources),
      lightSourceFile_(lightSourceFile),
      sensorNoiseLevel_(sensorNoiseLevel),
      motorNoiseLevel_(motorNoiseLevel),
      capAcceleration_(capAcceleration),
      maxLinearAcceleration_(maxLinearAcceleration),
      maxAngularAcceleration_(maxAngularAcceleration),
      maxDirectionShiftsPerSecond_(maxDirectionShiftsPerSecond),
      gravity_(gravity),
      disallowObstacleCollisions_(disallowObstacleCollisions),
      swarmSize_(swarmSize),
      gatheringZoneSize_(gatheringZoneSize),
      gatheringZonePos_(gatheringZonePos),
      swarmPositionsFile_(swarmPositionsFile),
      swarmPositions_(swarmPositions),
      //  TODO [resources] include this line to save the resources config into resourcesConfig_(resourcesConfig),
      resourcesConfigFile_(resourcesConfigFile),
      obstacleOverlapPolicy_(obstacleOverlapPolicy) {

        simulationTime_ = timeSteps * timeStepLength;
      }

    /**
     * Initializes a robogen config object from a protobuf robogenMessage::SimulatorConf message
     */
    RobogenConfig(const robogenMessage::SimulatorConf &message);

    /**
     * Destructor, destroy the robogen configuration object.
     */
    virtual ~RobogenConfig() {

    }

    /**
     * Get the type of scenario used.
     *
     * Get the type of scenario which dictates the fitness function to be used. One of
     * 'racing' (fitness dictated by distance from the starting position), 'chasing'
     * (fitness dictated by distance from light sources) or a scenarioFile can be
     * specified containing a custom javascript defined scenario and fitness function.
     *
     * @return The simulation scenario as a string, one of 'racing', 'chasing', or a string
     * giving the path of a custom javascript file.
     */
    std::string getScenario() const {
        return scenario_;
    }

    /**
     * Get the *.js file containing a description of the custom scenario.
     *
     * @return the simulation scenario file (for scripted scenario)
     */
    std::string getScenarioFile() const {
        return scenarioFile_;
    }

    /**
	 * Get the total number of timesteps to run before ending the scenario.
     *
     * @return the number of timesteps
     */
    unsigned int getTimeSteps() const {
        return timeSteps_;
    }

    /**
     * Get the duration of each timestep.
     *
     * Total duration of the simulation = timeSteps * timeStepLength;
     *
     * @return the time step length
     */
    float getTimeStepLength() const {
        return timeStepLength_;
    }

    /**
     * Get the duration between motor actuations, in number of timesteps
     *
     * @return the actuation period
     */
    int getActuationPeriod() const {
        return actuationPeriod_;
    }
    /**
     * Get the terrain configuration of the robot.
     *
     * This is either defined by a flat plane or by a height image.
     *
     * @return the terrain configuration
     */
    boost::shared_ptr<TerrainConfig> getTerrainConfig() {
        return terrain_;
    }

    /**
     * Get the configuration defining all the obstacles in the scenario and related parameters
     *
     * @return the obstacles configuration
     */
    boost::shared_ptr<ObstaclesConfig> getObstaclesConfig() {
        return obstacles_;
    }

    /**
     * Get the file defining each of the obstacles.
     *
     * @return the obstacles configuration file
     */
    std::string getObstacleFile(){
        return obstacleFile_;
    }

    /**
     * Get the list of starting positions for a robot.
     *
     * @return the robot starting positions
     */
    boost::shared_ptr<StartPositionConfig> getStartingPos() {
        return startPositions_;
    }

    /**
     * Get the file defining each of the starting positions.
     *
     * @return the starting position configuration file
     */
    std::string getStartPosFile(){
        return startPosFile_;
    }

    /**
     * Get the config object containing a list of light sources in the
     * scenario.
     *
     * This is used for the 'chasing' scenario to determine fitness.
     *
     * @return the light sources configuration
     */
    boost::shared_ptr<LightSourcesConfig> getLightSourcesConfig() {
        return lightSources_;
    }

    /**
     * Get the file defining each of the light sources
     *
     * @return the light sources configuration file
     */
    std::string getLightSourceFile(){
        return lightSourceFile_;
    }

    /**
     * Get the amount of noise by which the sensor readings are perturbed.
     *
     * This is done in order to mimic real-life inaccuracies in sensor readings due to natural
     * phenomena The value passed to the neural network will be chosen from a
     * standard gaussian distribution with mean of the actual reading and
     * standard deviation of the sensorNoiseLevel
     *
     * @return sensor noise level
     */
    float getSensorNoiseLevel() {
        return sensorNoiseLevel_;
    }

    /**
     * Get the amount of noise by which the motor outputs are perturbed.
     *
     * This is done in order to mimic real-life inaccuracies in motors due to natural
     * phenomena. The value passed from the neural network to the motor will be chosen from a
     * uniform distribution over the range
     * (actualValue - motorNoiseLevel, actualValue + motorNoiseLevel)
     *
     * @return motor noise level
     *
     */
    float getMotorNoiseLevel() {
        return motorNoiseLevel_;
    }

    /**
     * Check if the linear and/or angular acceleration is capped.
     *
     * The spelling typo in `Alleration` is known about, but it's origins aren't.
     *
     * If true, limit the maximum allowed linear acceleartion to `maxLinearAcceleration` and the maximum allowed
     * angular acceleration to `maxAngularAcceleration`. This is used to stop the simulation getting out of control or to
     * stop the genetic algorithm from exploiting quirks in the physics engine which
     * might permit unrealistic accelerations.
     *
     * @return if acceleration is capped
     */
    bool isCapAlleration() {
        return capAcceleration_;
    }

    /**
     * Get the maximum allowed linear acceleration, see capAcceleration.
     *
     * Ignored if capAcceleration is false
     *
     * @return max linear acceleration (if capped)
     */
    float getMaxLinearAcceleration() {
        return maxLinearAcceleration_;
    }

    /**
     * Get the maximum allowed angular acceleration, see capAcceleration.
     *
     * Ignored if capAcceleration is false.
     *
     * @return max angular acceleration (if capped)
     */
    float getMaxAngularAcceleration() {
        return maxAngularAcceleration_;
    }

    /**
     * Get the maximum allowed direction shifts per second.
     *
     * A direction shift is clockwise -> anticlockwise, or vice versa.
     * If a motor changes direction more than `maxDirectionShiftsPerSecond`, then the simulator will disable it,
     * simulating real-life burnout of motors. if the value is -1, no burnout will occur.
     *
     * @return max direction shifts per second for testing motor burnout
     *         if -1, then do not check burnout
     */
    int getMaxDirectionShiftsPerSecond() {
        return maxDirectionShiftsPerSecond_;
    }

    /**
     * Get the force to apply for gravity, by default -9.81m.s^-2 in the z direction, and zero in x and y.
     *
     * @return gravity vector
     */
    osg::Vec3 getGravity() {
        return gravity_;
    }

    /**
     * Check if any obstacle collision will raise a constraint violation.
     *
     * @return if should disallow obstacle collisions
     */
    bool isDisallowObstacleCollisions() {
        return disallowObstacleCollisions_;
    }

    /**
     * Get the size of the swarm.
     *
     * @return the size of the swarm as specified in the configuration file.
     */
    unsigned int getSwarmSize() {
        return swarmSize_;
    }

    /**
     * Get the size of the gathering zone.
     *
     * The gathering zone is used for custom swarm based scenarios.
     *
     * @return the size of the gathering zone
     * @see getGatheringZonePos()
     */
    osg::Vec3 getGatheringZoneSize() {
        return gatheringZoneSize_;
    }

    /**
     * Get the position of the gathering zone.
     *
     * The gathering zone is used for custom swarm based scenarios.
     *
     * @return the position of the gathering zone
     * @see getGatheringZoneSize()
     */
    osg::Vec3 getGatheringZonePos() {
        return gatheringZonePos_;
    }

    /**
     * Get the path of the file containing the resources configurations
     *
     * The resources configuration file contains one resource definition per
     * line as a list of space-separated floating point values. The order of
     * the values on each line of the file is:
     * 1. x-pos
     * 2. y-pos
     * 3. z-pos
     * 4. x-magnitude
     * 5. y-magnitude
     * 6. z-magnitude
     * 7. unknown
     * 8. unknown
     *
     * @return the resources config file
     */
    std::string getResourcesConfigFile(){
        return resourcesConfigFile_;
    }

    /**
     * Get the configuration object describing the swarm positions.
     *
     * Takes priority over startPositions. The number of positions should be
     * equal to swarmSize.
     *
     * @return the swarm positions config
     */
    boost::shared_ptr<SwarmPositionsConfig> getSwarmPositionsConfig() {
        return swarmPositions_;
    }

    /**
     * Get the path of the file defining the swarm positions.
     *
     * Each line of the file should contain space-deliminated float values
     * (like `0 0.2 1.8` or similar) where the values are in the order x-pos y-pox z-pos.
     *
     * @return the swarm positions configuration file
     */
    std::string getSwarmPositionsFile(){
        return swarmPositionsFile_;
    }

    /**
     * Get the policy used when obstacles overlap.
     *
     * @return One of REMOVE_OBSTACLES, CONSTRAINT_VIOLATION, ELEVATE_ROBOT
     * @see ObstacleOverlapPolicies
     */
    unsigned int getObstacleOverlapPolicy() {
        return obstacleOverlapPolicy_;
    }

    /**
     * Get the total running time of the simulation, where simulationTime is = timeSteps * timeStepLength;
     *
     * @return the total simulation time
     */
    float getSimulationTime() const {
        return simulationTime_;
    }

    /**
     * Convert the configuration into a protobuf message.
     *
     * Go through all members in this configuration and serialise them into a
     * format acceptable to Google's protobuf messages. For more complex objects,
     * call their own .serialise methods.
     *
     * @return a simulator configuration protobuf message describing the
     * current robogenConfig object
     */
    robogenMessage::SimulatorConf serialize() const{
        robogenMessage::SimulatorConf ret;

        ret.set_ntimesteps(timeSteps_);
        ret.set_scenario(scenario_);
        ret.set_timestep(timeStepLength_);
        ret.set_actuationperiod(actuationPeriod_);
        ret.set_sensornoiselevel(sensorNoiseLevel_);
        ret.set_motornoiselevel(motorNoiseLevel_);
        ret.set_capacceleration(capAcceleration_);
        ret.set_maxlinearacceleration(maxLinearAcceleration_);
        ret.set_maxangularacceleration(maxAngularAcceleration_);
        ret.set_maxdirectionshiftspersecond(maxDirectionShiftsPerSecond_);
        ret.set_gravityx(gravity_.x());
        ret.set_gravityy(gravity_.y());
        ret.set_gravityz(gravity_.z());
        ret.set_disallowobstaclecollisions(disallowObstacleCollisions_);
        ret.set_swarmsize(swarmSize_);
        ret.set_gatheringzonesizex(gatheringZoneSize_.x());
        ret.set_gatheringzonesizey(gatheringZoneSize_.y());
        ret.set_gatheringzonesizez(gatheringZoneSize_.z());
        ret.set_gatheringzoneposx(gatheringZonePos_.x());
        ret.set_gatheringzoneposy(gatheringZonePos_.y());
        ret.set_gatheringzoneposz(gatheringZonePos_.z());
        ret.set_obstacleoverlappolicy(obstacleOverlapPolicy_);

        terrain_->serialize(ret);
        swarmPositions_->serialize(ret);
        obstacles_->serialize(ret);
        startPositions_->serialize(ret);
        lightSources_->serialize(ret);

        return ret;
    }

private:

    /**
     * The simulation scenario
     */
    std::string scenario_;

    /**
     * The simulation scenario file (if using scripted scenario)
     */
    std::string scenarioFile_;

    /**
     * Total number of simulation timesteps
     */
    unsigned int timeSteps_;

    /**
     * Time step duration
     */
    float timeStepLength_;

    /**
     * Actuation period (in number of time steps)
     */
    int actuationPeriod_;

    /**
     * Terrain configuration
     */
    boost::shared_ptr<TerrainConfig> terrain_;

    /**
     * SwarmPositions configuration
     */
    boost::shared_ptr<SwarmPositionsConfig> swarmPositions_;

    /**
     * SwarmPostition configuration file location
     */
    std::string swarmPositionsFile_;

    /**
     * Obstacles configuration
     */
    boost::shared_ptr<ObstaclesConfig> obstacles_;

    /**
     * Obstacle configuration file location
     */
    std::string obstacleFile_;

    /**
     * List of robot starting positions
     */
    boost::shared_ptr<StartPositionConfig> startPositions_;

    /**
     * Starting positions configuration file location
     */
    std::string startPosFile_;

    /**
     * Light sources configuration
     */
    boost::shared_ptr<LightSourcesConfig> lightSources_;

    /**
     * Light sources configuration file location
     */
    std::string lightSourceFile_;

    /**
     * Simulation time
     */
    float simulationTime_;

    /**
     * Sensor noise level (see getter for details)
     */
    float sensorNoiseLevel_;

    /**
     * Motor noise level (see getter for details)
     */
    float motorNoiseLevel_;


    /**
     *  Flag to enforce acceleration cap.  Useful for preventing unrealistic
     *  behaviors / simulator exploits
     */
    bool capAcceleration_;

    /**
     *  Maximum allowed linear acceleration if acceleration is capped
     */
    float maxLinearAcceleration_;

    /**
     *  Maximum allowed angular acceleration if acceleration is capped
     */
    float maxAngularAcceleration_;

    /**
     *  Maximum allowed direction shifts per second (if specified)
     */
    int maxDirectionShiftsPerSecond_;

    /**
     * Gravity
     */
    osg::Vec3 gravity_;

    /**
     * Size of the swarm of robots to simulate in the world
     */
    unsigned int swarmSize_;

    /**
     * Position of the gathering zone where all robots congretate
     */
    osg::Vec3 gatheringZonePos_;

    /**
     * Size of the gathering zone where all robots congretate
     */
    osg::Vec3 gatheringZoneSize_;

    /**
     * The file in which the definitions of the different resources
     * can be found
     */
    std::string resourcesConfigFile_;

    /**
     * flag to disallow obstacle collisions
     */
    bool disallowObstacleCollisions_;

    /**
     * policy for handling the situation when an obstacle is in the robot's
     * initial AABB. One of REMOVE_OBSTACLES, CONSTRAINT_VIOLATION,
     * ELEVATE_ROBOT
     */
    unsigned int obstacleOverlapPolicy_;
};

}

#endif /* ROBOGEN_ROBOGEN_CONFIG_H_ */
