/*
 * @(#) LightSourcesConfig.h   1.0   Jan 7, 2016
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Joshua Auerbach
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
#ifndef ROBOGEN_LIGHTSOURCESCONFIG_H_
#define ROBOGEN_LIGHTSOURCESCONFIG_H_

#include <osg/Vec3>
#include <vector>
#include "robogen.pb.h"

namespace robogen {

/**
 * A config object to store parallel vectors of light source locations and
 * intensities.
 *
 * Light sources are light emitting objects in the robogen world. They can be
 * added without constraint, but their intended purpose is for use with
 * the 'chasing' scenario (see below). Lights can have a position and intensity.
 *
 * When the chasing scenario is set in the configuration file like
 * ```
 * scenario=chasing
 * ```
 * Then the fitness function rewards robots which are the closest to the
 * nearest light source for as much time as possible.
 *
 * @see ConfigurationReader
 * @see ChasingScenario
 */
class LightSourcesConfig {

public:

	/**
	 * Initialise an empty light sources config.
	 */
	LightSourcesConfig() {}

	/**
     * Initialise a light sources config with a vector of Open Scene Graph Vec3
     * coordinates and a parallel vector of intensities for those light sources.
     *
     * @param[in] coordinates       The xyz coordinates of each light source.
     * @param[in] intensities       The float intensities of each light source.
	 */
	LightSourcesConfig(const std::vector<osg::Vec3>& coordinates,
			const std::vector<float> &intensities) :
			coordinates_(coordinates), intensities_(intensities) {

	}

	/**
	 * Destructor.
	 */
	virtual ~LightSourcesConfig() {

	}

	/**
     * Get the xyz coordinates of the light sources.
     *
	 * @return The coordinates of the light sources
	 */
	const std::vector<osg::Vec3>& getCoordinates() const {
		return coordinates_;
	}

	/**
     * Get the intensities of the light sources.
     *
	 * @return the light source intensities
	 */
	const std::vector<float>& getIntensities() const{
		return intensities_;
	}

	/**
	 * Serialize light sources into a SimulatorConf message.
     *
     * This LightSourcesConfig object will be serialised into a protobuf message
     * which can later be deserialised.
     *
     * @param[out] message       The message into which this object should be
     * serialised.
     *
     * @see robogenMessage::SimulatorConf
     * @see src/robogen.proto
	 */
	void serialize(robogenMessage::SimulatorConf &message){
		for (unsigned int i=0; i<coordinates_.size(); ++i){
			robogenMessage::LightSource *curr = message.add_lightsources();
			curr->set_intensity(intensities_[i]);
			curr->set_x(coordinates_[i].x());
			curr->set_y(coordinates_[i].y());
			curr->set_z(coordinates_[i].z());
		}
	}

private:

	/**
	 * Light sources' coordinates
	 */
	std::vector<osg::Vec3> coordinates_;


	/**
	 * Light sources' intensities
	 */
	std::vector<float> intensities_;
};

}




#endif /* ROBOGEN_LIGHTSOURCESCONFIG_H_ */
