/*
 * @(#) Robot.h   1.0   Mar 4, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
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
#ifndef ROBOGEN_ROBOT_H_
#define ROBOGEN_ROBOT_H_

#include <boost/shared_ptr.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/copy.hpp>
#include <map>
#include <vector>

#include "Robogen.h"
#include "robogen.pb.h"
#include "model/Connection.h"

#include "model/CompositeBody.h"

extern "C" {
#include "brain/NeuralNetwork.h"
}

namespace robogen {

class Model;
class Motor;
class Sensor;

struct BodyEdgeDescriptorTag {
	typedef boost::edge_property_tag kind;
};
typedef boost::property<BodyEdgeDescriptorTag, boost::shared_ptr<Connection> >
BodyEdgeProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS,
		boost::no_property, BodyEdgeProperty> BodyGraph;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
		boost::no_property, BodyEdgeProperty> BodyUndirectedGraph;
typedef boost::graph_traits<BodyGraph>::edge_descriptor BodyEdge;

/**
 * A robot is an actor in the robogen world which attempts to maximise some
 * fitness function.
 *
 * The fitness function is defined so that it increases when the Robot does
 * something the research desires (like move towards a light, or walk quickly).
 *
 * Rather use a Swarm of size 1 than the robot class directly.
 *
 * @see Swarm
 */
class Robot {

public:
	/**
	 * Error-less constructor.
	 */
	Robot();

	/**
	 * Initializes a Robogen robot from a robogen message
     * @param[in] odeWorld          The Open Dynamics Engine World which the
     * robot will inhabit.
     * @param[in] odeSpace          The Open Dynamics Engine Space which is
     * used to speed up collisions related to the robot
     * @param[in] robotSpec         The Protobuf message which is a serialised
     * version of all the parameters needed to initialise a robot.
     * @param[in] printInfo         Default is false, If true then print
     * information to std::cout.
     * @param[in] printInitErrors    Default is true, if true then print
     * errors preventing the robot from being initialised to `std::cerr`
	 */
	bool init(dWorldID odeWorld, dSpaceID odeSpace,
			const robogenMessage::Robot& robotSpec,
			bool printInfo=false, bool printInitErrors=true);

	/**
	 * Destructor.
	 */
	virtual ~Robot();

	/**
     *  Get the Model objects which are this robot's body parts
     *
	 *  @return  The body parts of which the robot is composed of.
	 */
	const std::vector<boost::shared_ptr<Model> >& getBodyParts();

	/**
     * Get the NeuralNetwork object which controls the actions of this robot.
     *
	 * @return The neural network that controls the robot.
	 */
	const boost::shared_ptr<NeuralNetwork>& getBrain() const;

	/**
     * Get the vector of Sensor objects which allow the robot to perceive
     * its environment.
     *
	 * @return The sensors of the robot.
	 */
	const std::vector<boost::shared_ptr<Sensor> >& getSensors() const;

	/**
     * Get the vector of Motor objects which allow the robot to move around in
     * it's environment. Motors can either rotate continuously (like wheels) or
     * they can be turned to a certain rotation (like arms).
     *
	 * @return The motors of the robot.
	 */
	const std::vector<boost::shared_ptr<Motor> >& getMotors();

	/**
     * Get a pointer to the robot's core component.
     *
     * If you think of the robot as a directed acyclic graph of
     * different components all connected together, then the core component
     * is the root node of that graph. The position of the core component is
     * often used as the de-facto position of the robot as a whole.
     *
	 * @return The core component of the robot.
	 */
	boost::shared_ptr<Model> getCoreComponent();

	/**
	 * Translate the robot by some change in xyz to a new location.
     *
	 * @param translation Amount of translation.
	 */
	void translateRobot(const osg::Vec3& translation);

	/**
	 * Rotate the robot.
     *
	 * @param rot rotation Quaternion
	 */
	void rotateRobot(const osg::Quat &rot);

	/**
	 * Returns the robot axis-aligned bounding box.
     *
     * This is the smallest box which can contain the robot while still having
     * all six faces aligned to the axes of the coordinate system.
     *
     * @param[out] minX       The minimum X value.
     * @param[out] maxX       The maximum X value.
     * @param[out] minY       The minimum Y value.
     * @param[out] maxY       The maximum Y value.
     * @param[out] minZ       The minimum Z value.
     * @param[out] maxZ       The maximum Z value.
	 */
	void getAABB(double& minX, double& maxX, double& minY, double& maxY,
			double& minZ, double& maxZ);

	/**
     * Return the integer ID of the robot.
     *
	 * @return the robot ID
	 */
	int getId() const;

	/**
     * Get the body part of the robot from a string ID
     *
     * @param[in] id       The id of the body part.
	 * @return             The body part corresponding to the given id
	 */
	boost::shared_ptr<Model> getBodyPart(std::string id);

	/**
     * Get the protobuf serialised message which generated this robot.
     *
	 * @return message that generated the robot
	 */
	const robogenMessage::Robot& getMessage();

    /**
     * Get a vector of Connection objects which define the joints between
     * the various components of the robot.
     *
     * @return the connections between the components of the robot.
     */
	const std::vector<boost::shared_ptr<Connection> >& getBodyConnections()
			const;


    /**
     * Unimplemented.
     *
     * The body of this method is empty and has no usages. It was originally
     * introduced in commit 99297df7 as:
     * ```
     *  commit 99297df7f026c864513b333d131769b5878086f0
     *  Author: Deniz Aydin <deniz.aydin@epfl.ch>
     *  Date:   Wed Nov 6 11:29:19 2013 +0100
     *
     *      WIP Body Compiler for 3D printing and user-readable robot body
     *      representations.
     * ```
     */
	void traverseBody(const std::vector<boost::shared_ptr<Model> >,
			const std::vector<boost::shared_ptr<Connection> >);


    /**
     * Get the index of the core component.
     *
     * The method getCoreComponent should be used in preference to this
     *
     * @see getCoreComponent
     */
	int getRoot();

    /**
     * Push a joint to the end of the private joints_ vector.
     *
     * This is used in the tree_edge method while the search tree of the
     * robot is being created.
     *
     * @param[in] join       The joint to add.
     *
     * @see tree_edge
     *
     */
	inline void addJoint(boost::shared_ptr<Joint> joint) {
		joints_.push_back(joint);
	}

	/**
	 * Merges bodies connected with fixed joints into complex bodies
	 */
	void optimizePhysics();

private:

	/**
	 * Decodes the body of the robot
	 * @param robotBody
	 * @return true if the operation completed successfully
	 */
	bool decodeBody(const robogenMessage::Body& robotBody);

	/**
	 * Decodes the brain of the robot
	 * @param robotBrain
	 * @return true if the operation completed successfully
	 */
	bool decodeBrain(const robogenMessage::Brain& robotBrain);

	/**
	 * Connects all body parts to the root part
	 */
	void reconnect();

	/**
	 * ODE physics world
	 */
	dWorldID odeWorld_;

	/**
	 * ODE collision space
	 */
	dSpaceID odeSpace_;

	/**
	 * Robot body parts
	 */
	std::vector<boost::shared_ptr<Model> > bodyParts_;

	/**
	 * Connections between the body parts.
	 */
	std::vector<boost::shared_ptr<Connection> > bodyConnections_;

	/**
	 * Mapping from body part to associated sensors
	 */
	std::map<unsigned int, std::vector<boost::shared_ptr<Sensor> > > bodyPartsToSensors_;

	/**
	 * Mapping from body part to associated motors
	 */
	std::map<unsigned int, std::vector<boost::shared_ptr<Motor> > > bodyPartsToMotors_;

	/**
	 * Robot sensors
	 */
	std::vector<boost::shared_ptr<Sensor> > sensors_;

	/**
	 * Robot motors
	 */
	std::vector<boost::shared_ptr<Motor> > motors_;

	/**
	 * Neural network
	 */
	boost::shared_ptr<NeuralNetwork> neuralNetwork_;

	/**
	 * Maps the identifier of a body part with the body part in the bodyParts_ vector
	 */
	std::map<std::string, unsigned int> bodyPartsMap_;

	/**
	 * The core component of the robot
	 */
	boost::shared_ptr<Model> coreComponent_;

	/**
	 * Robot ID
	 */
	int id_;

	/**
	 * Root part index of the robot
	 */
	int rootNode_;

	/**
	 * Boost graph of body tree.
	 */
	boost::shared_ptr<BodyGraph> bodyTree_;

	/**
	 * Joint group of connections between parts
	 */
	dJointGroupID connectionJointGroup_;

	/**
	 * Contains the robot message
	 */

	const robogenMessage::Robot* robotMessage_;

	bool printInfo_;

	bool printInitErrors_;

	// store joints created by connecting components
	// not a set because we don't want iteration order to depend on address,
	// but we will use a set to check for uniqueness while iterating
	std::vector<boost::shared_ptr<Joint> > joints_;

	// store the composite bodies formed by replacing fixed joints
	std::vector<boost::shared_ptr<CompositeBody> > composites_;
};

}

#endif /* ROBOGEN_ROBOT_H_ */
