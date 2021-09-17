/*
 * @(#) Simulator.h   1.0   Nov 27, 2014
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
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

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <boost/shared_ptr.hpp>
#include <boost/random.hpp>

#include "Robogen.h"
#include "config/RobogenConfig.h"
#include "scenario/Scenario.h"
#include "viewer/FileViewerLog.h"
#include "viewer/IViewer.h"
#include "Swarm.h"
#define MIN_FITNESS (-10000.0) /*!< The fitness level given when a robot exceeds the acceleration limit.*/

/**
 * \file Simulator.h
 *
 * The simulator runs one simulation, which may contain multiple trials.
 *
 * The Simulator wraps the ODE code, and should be used by other classes
 * that wish to run a simulation.
 *
 * Historically there were separate executables for Server, ServerViewer, and
 * FileViewer.  This involved a lot of duplicate code, so this class aims to
 * encapsulate all ODE code in one place.
 */

namespace robogen{

/**
 * Describes how the simulation ended.
 */
enum result{
		SIMULATION_SUCCESS,     /*!< Simulation ended without failure */
		SIMULATION_FAILURE,     /*!< Simulation ended with a failure of some kind */
		CONSTRAINT_VIOLATED     /*!< An ODE physics engine constraint was violated */

	};

/**
 * Wrapper method that calls the full runSimulations, but with a
 * default value for log.
 */
unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Swarm &swarmMessage,
        IViewer *viewer,
		boost::random::mt19937 &rng);

/**
 * Run the simulation in a given scenario for a certain number of trials.
 */

unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Swarm &swarmMessage,
		IViewer *viewer,
		boost::random::mt19937 &rng,
		bool onlyOnce, boost::shared_ptr<FileViewerLog> log);
}

#endif /* SIMULATOR_H_ */
