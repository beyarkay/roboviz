/*
 * @(#) RobogenCollision.cpp   1.0   March 21, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
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
#include "utils/RobogenCollision.h"
#include "config/RobogenConfig.h"

#include "Robot.h"
#include "Swarm.h"
#include "model/SimpleBody.h"

#include <algorithm>

// ODE World
extern dWorldID odeWorld;

// Container for collisions
extern dJointGroupID odeContactGroup;

namespace robogen {

const int MAX_CONTACTS = 32; // maximum number of contact points per body


CollisionData::CollisionData(boost::shared_ptr<Scenario> scenario) :
		scenario_(scenario), hasObstacleCollisions_(false) {

	//numCulled = 0;


	boost::shared_ptr<Swarm> swarm_ = scenario->getSwarm();

	for (size_t i = 0; i < swarm_->getSize(); ++i) {
		for (size_t j = 0; j < swarm_->getRobot(i)->getBodyParts().size(); ++j) {
		boost::shared_ptr<Model> model = swarm_->getRobot(i)->getBodyParts()[j];
			for(size_t k=0; k<model->getBodies().size(); ++k) {
				geomModelMap_[model->getBodies()[k]->getGeom()] = model;
			}
		}
	}
}

bool CollisionData::ignoreCollision(dGeomID o1, dGeomID o2) {

	if (geomModelMap_.count(o1) == 0 || geomModelMap_.count(o2) == 0 )
		return false;
	return ( geomModelMap_[o1] == geomModelMap_[o2]);

}

bool CollisionData::isPartOfBody(dGeomID o1) {
	return geomModelMap_.count(o1);
}

void CollisionData::testObstacleCollisons(dGeomID o1, dGeomID o2) {
	if (	(isPartOfBody(o1) && dGeomGetClass(o2) == dBoxClass)
			||
			(isPartOfBody(o2) && dGeomGetClass(o1) == dBoxClass)) {
		//std::cout << "colliding with obstacle!!!" << std::endl;
		hasObstacleCollisions_ = true;
	}
}

/**
 * @param[in] data       Should be a pointer to a RobogenConfig object.
 * @param[in] o1         The first object in the collision between o1 and o2.
 * @param[in] o2         The second object in the collision between o1 and o2.
 *
 * The callback given to ODE to use when there's a collision. From <a href="http://opende.sourceforge.net/docs/group__collide.html#ga9f458413ace07fa9e3e7e52d6652ace0">
 * the ODE docs on dNearCallback</a> :
 *
 * > The callback function can call dCollide on o1 and o2 to generate contact
 * > points between each pair. Then these contact points may be added to the
 * > simulation as contact joints. The user's callback function can of course
 * > chose not to call dCollide for any pair, e.g. if the user decides that
 * > those pairs should not interact.
 *
 * Relies on external variables dWorldID odeWorld and dJointGroupID
 * odeContactGroup.
 */
void odeCollisionCallback(void *data, dGeomID o1, dGeomID o2) {

	CollisionData *collisionData = static_cast<CollisionData*>(data);

	// Since we are now using complex bodies, just because two bodies
	// are connected with a joint does not mean we should ignore their
	// collision.  Instead we need to use the ignoreCollision method define
	// above, which will check if the two geoms are part of the same
	// model, in which case we can ignore.
	// TODO can we make this more efficient?


	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	//if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) {
	if (collisionData->ignoreCollision(o1, o2) ) {
		//collisionData->numCulled++;
		return;
	}


	dContact contact[MAX_CONTACTS];
	for (int i = 0; i < MAX_CONTACTS; i++) {
		contact[i].surface.slip1 = 0.01;
		contact[i].surface.slip2 = 0.01;
		contact[i].surface.mode = dContactSoftERP |
					dContactSoftCFM |
					dContactApprox1 |
					dContactSlip1 | dContactSlip2;
		// TODO use different value for self collisions and/or obstacles?
		contact[i].surface.mu = collisionData->getScenario()->getRobogenConfig(
									)->getTerrainConfig()->getFriction();
		contact[i].surface.soft_erp = 0.96;
		contact[i].surface.soft_cfm = 0.01;



	}

	int collisionCounts = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
		sizeof(dContact));

	if (collisionCounts > 0) {
		collisionData->testObstacleCollisons(o1, o2);
	}


	for (int i = 0; i < collisionCounts; i++) {

		dJointID c = dJointCreateContact(odeWorld, odeContactGroup,
				contact + i);
		dJointAttach(c, b1, b2);


	}
}

}
