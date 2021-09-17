/*
 * @(#) ConfigurationReader.h   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
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
#ifndef ROBOGEN_CONFIGURATION_READER_H_
#define ROBOGEN_CONFIGURATION_READER_H_

#include <boost/shared_ptr.hpp>
#include <string>
#include "robogen.pb.h"

namespace robogen {

class ObstaclesConfig;
class SwarmPositionsConfig;
class RobogenConfig;
class StartPositionConfig;
class TerrainConfig;
class LightSourcesConfig;

/**
 * The Configuration Reader takes in various types of config files, and
 * converts them to configuration objects as used by Robogen.
 */
class ConfigurationReader {

public:

	/**
	 * Convert the given configuration file to a RobogenConfig object.
	 */
	static boost::shared_ptr<RobogenConfig> parseConfigurationFile(
			const std::string& fileName);

    /**
     * Convert a protobuf message defining a robogen configuration to
     * a RobogenConfig object.
     */
    static boost::shared_ptr<RobogenConfig> parseRobogenMessage(
            const robogenMessage::SimulatorConf& simulatorConf);

private:

    /**
     * Converts the configuration file specifying swarm positions to a
     * SwarmPositionsConfig object.
     */
    static boost::shared_ptr<SwarmPositionsConfig> parseSwarmPositionsFile(
        const std::string& fileName);

	/**
     * Converts the configuration file for obstacles to a ObstaclesConfig
     * object
	 */
	static boost::shared_ptr<ObstaclesConfig> parseObstaclesFile(
        const std::string& fileName);

	/**
	 * Converts the starting position file to a StartPositionConfig object.
	 */
	static boost::shared_ptr<StartPositionConfig> parseStartPositionFile(
        const std::string& fileName);

	/**
	 * Converts the light sources file to a LightSourcesConfig object.
	 */
	static boost::shared_ptr<LightSourcesConfig> parseLightSourcesFile(
        const std::string& fileName);

};

}

#endif /* ROBOGEN_CONFIGURATION_READER_H_ */
