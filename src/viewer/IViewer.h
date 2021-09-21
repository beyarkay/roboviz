/*
 * @(#) Viewer.h   1.0   Nov 27, 2014
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * Guillaume Leclerc (guillaume.leclerc@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2014 Joshua Auerbach
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

#ifndef IVIEWER_H_
#define IVIEWER_H_

#include "scenario/Scenario.h"
#include "model/Model.h"
#include "boost/date_time/posix_time/posix_time.hpp"

#define MAX_TIME_BETWEEN_FRAMES 0.05
namespace robogen {
class IViewer {

public:
	virtual ~IViewer() {};

    virtual bool configureScene(
        std::vector<std::vector<boost::shared_ptr<Model>>> swarmBodyParts,
        boost::shared_ptr<Scenario> scenario) = 0;
	virtual bool done() = 0;

	/**
	 * Updates the viewer's frame if it should be updated.
     *
     * Will return false if the simulation is paused or going too fast,
     * indicating the simulator should continue without stepping physics.
     *
     * For example, if only a tiny amount of in-simulator time has passed,
     * ignore the physics and keep showing the same rendering of the simulation
     * until enough time has passed that some meaningful change could have
     * happened in the world.
     *
	 * @param[in] simulatedTime      The amount of time simulated so far (in seconds)
	 * @param[in] numTimeSteps       The number of time steps simulated so far
	 *
     * @return false if paused or simulating too fast for physics to be worth
     * simulating
	 */

	virtual bool frame(double simulatedTime, unsigned int numTimeSteps) = 0;

	virtual bool isPaused() = 0;
};
}
#endif
