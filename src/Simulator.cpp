/*
 * @(#) Simulator.cpp   1.0   Nov 27, 2014
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Joshua Auerbach
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

#include "Simulator.h"
#include "utils/RobogenCollision.h"
#include "Models.h"
#include "Robot.h"
#include "Swarm.h"
#include "viewer/WebGLLogger.h"

//#define DEBUG_MASSES

// ODE World
extern dWorldID odeWorld;

// Container for collisions
extern dJointGroupID odeContactGroup;

namespace robogen{

/**
 * By default, set onlyOnce to false and supply a FileViewerLog
 * object, then call the fully parameterised runSimulations method.
 *
 * @see runSimulations()
 */
unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Swarm &swarmMessage,
		IViewer *viewer, boost::random::mt19937 &rng) {
	boost::shared_ptr<FileViewerLog> log;
	return runSimulations(scenario, configuration,
			swarmMessage, viewer, rng, false, log);
}

/**
  * @param[in] scenario      Contains the swarms goal and describes the
  * environment in which it must complete that goal
 * @param[in] configuration
 * @param[in] swarmMessage   A Protobuf Message describing the swarm
 * @param[in] viewer        Where the simulation should (optionally) be rendered to
 * @param[in] rng           Random Noise Generator, used to simulate random
 * fluctuations in the swarm's motors and sensor readings
 * @param[in] onlyOnce      If true, the simulation will end after the first trial.
 * @param[in] log           The object used to keep the logs.
 *
 * Run a certain number of trials in a scenario. A trial is one iteration of
 * the swarm being initialised and then started to complete it's goal. A
 * scenario is the collection of variables that defines the swarms surroundings
 * and environment. The simulation can either be done headless (without being
 * rendered to a screen for a person to view it) or it can be done with a
 * viewer, where a person can view the simulation.
 *
 * Relies on external variables dWorldID odeWorld and dJointGroupID
 * odeContactGroup.
 */
unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Swarm &swarmMessage,
		IViewer *viewer,
		boost::random::mt19937 &rng,
		bool onlyOnce, boost::shared_ptr<FileViewerLog> log) {

	bool constraintViolated = false;

	boost::random::normal_distribution<float> normalDistribution;
	boost::random::uniform_01<float> uniformDistribution;

	while (scenario->remainingTrials() && (!constraintViolated)) {

		// ---------------------------------------
		// Simulator initialization
		// ---------------------------------------

		dInitODE();

		// Create ODE world
		odeWorld = dWorldCreate();

		// Set gravity from config
		osg::Vec3 gravity = configuration->getGravity();
		dWorldSetGravity(odeWorld, gravity.x(), gravity.y(), gravity.z());

        // ERP controls how much error correction is done each time step
		dWorldSetERP(odeWorld, 0.1);
        // Set the global CFM (constraint force mixing) value
		dWorldSetCFM(odeWorld, 10e-6);
        // Set auto disable flag for newly created bodies.
		dWorldSetAutoDisableFlag(odeWorld, 1);

		// Create collision world
		dSpaceID odeSpace = dSimpleSpaceCreate(0);

		// Create contact group with maximum size of zero
		odeContactGroup = dJointGroupCreate(0);

		// wrap all this in block so things get cleaned up before shutting down
		// ode
        // FIXME There are lots of ODE errors when the simulation stops. Maybe it's
        // something to do with this block not enclosing enough of the variables?
		{

		// ---------------------------------------
		// Generate Swarm of Robots
        //
        // Create a swarm object, then create the same robot configuration->getSwarmSize()
        // number of times. After doing a load of checks and setting up the robot,
        // add it to the swarm
		// ---------------------------------------
		boost::shared_ptr<Swarm> swarm(new Swarm);
        for (unsigned int i = 0; i < configuration->getSwarmSize(); i++) {
          // Create a robot. It'll be added to the swarm at the bottom of the for-loop
          boost::shared_ptr<Robot> robot(new Robot);
          if (!robot->init(odeWorld, odeSpace, swarmMessage.robots(i))) {
            std::cout << "[E] Problems decoding the " << i
              << "-th robot.  Quit." << std::endl;
            return SIMULATION_FAILURE;
          }

#ifdef DEBUG_MASSES
          float totalMass = 0;
          for (unsigned int j = 0; j < robot->getBodyParts().size(); ++j) {
            float partMass = 0;
            for (unsigned int k = 0; k < robot->getBodyParts()[j]->getBodies().size(); ++k) {
              dMass mass;
              dBodyGetMass(robot->getBodyParts()[j]->getBodies()[k], &mass);
              partMass += mass.mass;
            }
            std::cout << "[D] " << robot->getBodyParts()[j]->getId() <<  " has mass: "
              << partMass * 1000. << "g" << std::endl;
            totalMass += partMass;
          }
          std::cout << "[D] Total mass is " << totalMass * 1000. << "g" << std::endl;
#endif
          if (log) {
            if (!log->init(robot, configuration)) {
              std::cout << "[E] Problem initializing log!" << std::endl;
              return SIMULATION_FAILURE;
            }
          }

          std::cout << "[I] Evaluating the " << i << "-th individual, with id=" << robot->getId()
            << ", trial: " << scenario->getCurTrial()
            << std::endl;

          // Go through each sensor of the robot. If it's a touch sensor, then add
          // it to the list of touchSensors
          std::vector<boost::shared_ptr<Sensor> > sensors = robot->getSensors();
          std::vector<boost::shared_ptr<TouchSensor> > touchSensors;
          for (unsigned int j = 0; j < sensors.size(); ++j) {
            if (boost::dynamic_pointer_cast<TouchSensor>(sensors[j])) {
              touchSensors.push_back(boost::dynamic_pointer_cast<TouchSensor>(sensors[j]));
            }
          }

          // Initialise the motors of the robot, and add a check to make sure they
          // don't go too fast
          std::vector<boost::shared_ptr<Motor>> motors = robot->getMotors();
          // set cap for checking motor burnout
          for(unsigned int j = 0; j < motors.size(); j++) {
            motors[j]->setMaxDirectionShiftsPerSecond(
                configuration->getMaxDirectionShiftsPerSecond()
            );
          }

          // Register brain and body parts
          boost::shared_ptr<NeuralNetwork> neuralNetwork = robot->getBrain();
          std::vector<boost::shared_ptr<Model>> bodyParts = robot->getBodyParts();

          swarm->addRobot(robot); //add robot to swarm after its been registered
        }
        if (!scenario->init(odeWorld, odeSpace, swarm)) {
          std::cout << "[E] Cannot initialize scenario. Quit." << std::endl;
          return SIMULATION_FAILURE;
        }

        if((configuration->getObstacleOverlapPolicy() == RobogenConfig::CONSTRAINT_VIOLATION)
            && scenario->wereObstaclesRemoved()) {
          std::cout << "[D] Using 'contraintViolation' obstacle overlap policy,"
            << " and ostacles were removed, so will return min fitness." << std::endl;
          constraintViolated = true;
          break;
        }

        // Setup environment and do some error checking
        boost::shared_ptr<Environment> env = scenario->getEnvironment();

        if (!scenario->setupSimulation()) {
          std::cout << "[E] Cannot setup scenario. Quit." << std::endl;
          return SIMULATION_FAILURE;
        }

        // Go through the body parts of every robot, and configure them inside
        // the scenario.
        std::vector<std::vector<boost::shared_ptr<Model>>> swarmBodyParts;
        for (unsigned int i = 0; i < configuration->getSwarmSize(); i++) {
          swarmBodyParts.push_back( swarm->getRobot(i)->getBodyParts() );
        }
        bool visualize = (viewer != NULL);
        if(visualize && !viewer->configureScene(swarmBodyParts, scenario)) {
          std::cout << "[E] Cannot configure scene. Quit." << std::endl;
          return SIMULATION_FAILURE;
        }

        //Init the webGLLogger
        boost::shared_ptr<WebGLLogger> webGLlogger;
        if (log && log->isWriteWebGL()) {
          webGLlogger.reset(new WebGLLogger(log->getWebGLFileName(),
                scenario));
        }

		//setup vectors for keeping velocities
		dReal previousLinVel[3];
		dReal previousAngVel[3];

		// ---------------------------------------
		// Main Loop
        //
        // This is the main simulation loop, in
        // which the forces are calculated,
        // collisions managed, and results
        // displayed to the user
		// ---------------------------------------

        // Number of iterations since start of simulation
		int count = 0;
        // Amount of time elapsed since start of simulation, in seconds
		double t = 0;

        boost::shared_ptr<CollisionData> collisionData(new CollisionData(scenario));

        double step = configuration->getTimeStepLength();
        while ((t < configuration->getSimulationTime()) && (!(visualize && viewer->done()))) {
          if(visualize && (!viewer->frame(t, count))) {
            continue;
          }

          if (scenario->shouldStopSimulationNow()) {
            std::cout << "[E] Scenario has stopped the simulation!" << std::endl;
            break;
          }

          // Send a full stop every 500 iterations
          if ((count++) % 500 == 0) {
            std::cout << "." << std::flush;
          }

          // Collision detection
          dSpaceCollide(odeSpace, collisionData.get(), odeCollisionCallback);

          // Step the world by one timestep
          dWorldStep(odeWorld, step);

          // Empty contact groups used for collisions handling
          dJointGroupEmpty(odeContactGroup);

          if (configuration->isDisallowObstacleCollisions() && collisionData->hasObstacleCollisions()) {
            constraintViolated = true;
            break;
          }

          // If the acceleration is capped,
          if (configuration->isCapAlleration()) {
            // go through every robot and check that no robot is going faster
            // than the acceleration cap
            for (unsigned int i = 0; i < configuration->getSwarmSize(); i++ ) {
              boost::shared_ptr<Robot> robot = swarm->getRobot(i);
              dBodyID robotRootBody = robot->getCoreComponent()->getRoot()->getBody();

              const dReal *angVel, *linVel;
              angVel = dBodyGetAngularVel(robotRootBody);
              linVel = dBodyGetLinearVel(robotRootBody);

              // Don't check for robots going over the speed limit in the zeroth
              // timestep
              if(t > 0) {
                double angAccel = dCalcPointsDistance3(angVel, previousAngVel);
                double linAccel = dCalcPointsDistance3(linVel, previousLinVel);

                if(angAccel > configuration->getMaxAngularAcceleration() ||
                    linAccel > configuration->getMaxLinearAcceleration()) {

                  printf("EVALUATION CANCELED: Maximum Acceleration");
                  printf(" exceeded at time %f.", t);
                  printf(" Angular accel: %f, Linear accel: %f.\n",
                      angAccel, linAccel);
                  printf("Will give minumum fitness (%f).\n", MIN_FITNESS);
                  constraintViolated = true;
                  break;
                }
              }

              // save current velocities as previous
              for(unsigned int j = 0; j < 3; j++) {
                previousAngVel[j] = angVel[j];
                previousLinVel[j] = linVel[j];
              }
            }
          }

          // Elapsed time since last call
          env->setTimeElapsed(step);

          // Go through every component on every robot, and update the sensors
          // if that component contains any sensors
          for (unsigned int i = 0; i < configuration->getSwarmSize(); i++ ) {
            boost::shared_ptr<Robot> robot = swarm->getRobot(i);
            std::vector<boost::shared_ptr<Model>> bodyParts = robot->getBodyParts();

            for (unsigned int j = 0; j < bodyParts.size(); ++j) {
              if (boost::dynamic_pointer_cast<PerceptiveComponent>(bodyParts[j])) {
                boost::dynamic_pointer_cast<PerceptiveComponent>(bodyParts[j])->updateSensors(env);
              }
            }
          }

          bool motorBurntOut = false;
          // =========================================
          // Go through each robot and do two things:
          // 1. Update the Neural Neural networks
          // 2. Check each motor for burnout
          // =========================================
          for (unsigned int i = 0; i < configuration->getSwarmSize(); i++ ) {
            boost::shared_ptr<Robot> robot = swarm->getRobot(i);

            // -----------------------------------------------------------
            // Update the Robot's neural Network
            //
            // The NN is only evaluated on some iterations of the main loop.
            // This is because IRL the microcontroller doesn't run at the same
            // speed as a desktop computer, so will have an upper limit on how
            // frequently it can take in the inputs, feed them through the
            // Neural Network, and actuate the outputs.
            // -----------------------------------------------------------

            boost::shared_ptr<NeuralNetwork> neuralNetwork = robot->getBrain();
            std::vector<boost::shared_ptr<Sensor>> sensors = robot->getSensors();
            std::vector<boost::shared_ptr<Motor>> motors = robot->getMotors();

            float networkInput[MAX_INPUT_NEURONS];
            float networkOutputs[MAX_OUTPUT_NEURONS];

            if(((count - 1) % configuration->getActuationPeriod()) == 0) {
              // Read input from each sensor into networkInput
              for (unsigned int j = 0; j < sensors.size(); ++j) {
                networkInput[j] = sensors[j]->read();

                if (configuration->getSensorNoiseLevel() > 0.0) {
                  networkInput[j] += (normalDistribution(rng) *
                      configuration->getSensorNoiseLevel() * networkInput[j]);
                }
              }
              if (log) {
                log->logSensors(networkInput, sensors.size());
              }

              // Initialise `neuralNetwork` with networkInput[0]
              ::feed(neuralNetwork.get(), &networkInput[0]);

              // Step the neural network
              ::step(neuralNetwork.get(), t);

              // Fetch the neural network ouputs
              ::fetch(neuralNetwork.get(), &networkOutputs[0]);

              // Send controls to the motors
              std::vector<boost::shared_ptr<Motor>> motors = robot->getMotors();
              for (unsigned int j = 0; j < motors.size(); ++j) {
                // Add motor noise:
                // uniform in range +/- motorNoiseLevel * actualValue
                if(configuration->getMotorNoiseLevel() > 0.0) {
                  networkOutputs[j] += (((uniformDistribution(rng) * 2.0 *
                          configuration->getMotorNoiseLevel()) -
                        configuration->getMotorNoiseLevel()) *
                      networkOutputs[j]);
                }

                if (boost::dynamic_pointer_cast<RotationMotor>(motors[j])) {
                  // If the motor is a rotation motor, then set it's velocity
                  boost::dynamic_pointer_cast<RotationMotor>(motors[j])->setDesiredVelocity(
                      networkOutputs[j],
                      step * configuration->getActuationPeriod()
                  );
                } else if (boost::dynamic_pointer_cast< ServoMotor>(motors[j])) {
                  // If the motor is a servo motor, then set it's position
                  boost::dynamic_pointer_cast<ServoMotor>(motors[j])->setDesiredPosition(
                      networkOutputs[j],
                      step * configuration->getActuationPeriod()
                  );
                }
              }

              if(log) {
                log->logMotors(networkOutputs, motors.size());
              }
            }

            // ----------------------------------------------------------------
            // Check all motors for motor burnout
            //
            // Check if any of the motors have been sent commands so frequently
            // that we need to simulate them burning out
            // ----------------------------------------------------------------
            for (unsigned int j = 0; j < motors.size(); ++j) {
              motors[j]->step(step) ; //* configuration->getActuationPeriod() );

              // todo find a cleaner way to do this for now will reuse accel cap infrastructure
              if (motors[j]->isBurntOut()) {
                std::cout << "[E] Motor " << j << " has burnt out, will terminate now." << std::endl;
                motorBurntOut = true;
              }
            }
            if(constraintViolated || motorBurntOut) {
              break;
            }
          }

          if (!scenario->afterSimulationStep()) {
            std::cout << "[E] Cannot execute scenario after simulation step. Quit." << std::endl;
            return SIMULATION_FAILURE;
          }

          for (unsigned int i = 0; i < configuration->getSwarmSize(); i++ ) {
            if(log) {
              log->logPosition(scenario->getSwarm()->getRobot(i)->getCoreComponent()->getRootPosition());
            }
          }

          if(webGLlogger) {
            webGLlogger->log(t);
          }

          t += step;
		}

        if (!scenario->endSimulation()) {
          std::cout << "[E] Cannot complete scenario. Quit." << std::endl;
          return SIMULATION_FAILURE;
        }

		// ---------------------------------------
		// Simulator finalization
		// ---------------------------------------

		// Destroy the WebGlLogger, since contains pointer to scenario
        if(webGLlogger) {
          webGLlogger.reset();
        }
		} // END code block protecting objects for ode code clean up


		// scenario has a shared ptr to the robot, so need to prune it
		scenario->prune();

		// Destroy the joint group
		dJointGroupDestroy(odeContactGroup);

		// Destroy ODE space
		dSpaceDestroy(odeSpace);

		// Destroy ODE world
		dWorldDestroy(odeWorld);

		// Destroy the ODE engine
		dCloseODE();

        if (constraintViolated || onlyOnce) {
          break;
        }
	}
    if (constraintViolated) {
      return CONSTRAINT_VIOLATED;
    }
	return SIMULATION_SUCCESS;
}
}
