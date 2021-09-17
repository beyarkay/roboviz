/*
 * @(#) ObstaclesConfig.h   1.0   Mar 12, 2013
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
#ifndef ROBOGEN_OBSTACLES_CONFIG_H_
#define ROBOGEN_OBSTACLES_CONFIG_H_

#include <osg/Vec3>
#include <vector>
#include "robogen.pb.h"

namespace robogen {

/**
 * A config object to store various parallel vectors describing the various
 * obstacles in the robogen world.
 *
 *
 * This config objects stores parallel vectors of the:
 * - xyz positions
 * - Sizes
 * - Densities
 * - Rotation Axes
 * - Rotation Angles (in degrees)
 *
 * Of the obstacles which will then be generated into the robogen world.
 *
 * From the sizes and densities, (and the world's gravity), the weight of
 * the obstacles can be calculated and used for interactions with the robots.
 *
 */
class ObstaclesConfig {

public:

	/**
	 * Initializes an empty obstacles configuration.
	 */
	ObstaclesConfig() {}

	/**
	 * Initializes an Obstacles Config object with parallel vectors defining
     * each of the obstacles in the world.
     *
     * @param[in] coordinates       The xyz coordinates of each obstacle.
     * @param[in] sizes             The xyz sizes of each obstacle.
     * @param[in] densities         The float densities of each obstacle.
     * @param[in] rotationAxes      The vectors defining the angle around which
     * the obstacle is rotated
     * @param[in] rotationAngles    The float amount by which each obstacle is
     * rotated around its rotationAxis, in degrees.
	 */
	ObstaclesConfig(const std::vector<osg::Vec3>& coordinates,
			const std::vector<osg::Vec3>& sizes,
			const std::vector<float> &densities,
			const std::vector<osg::Vec3>& rotationAxes,
			const std::vector<float> &rotationAngles) :
			coordinates_(coordinates), sizes_(sizes), densities_(densities),
			rotationAxes_(rotationAxes), rotationAngles_(rotationAngles) {

	}

	/**
	 * Destructor.
	 */
	virtual ~ObstaclesConfig() {

	}

	/**
     * The xyz coordinates of the obstacles.
     *
	 * @return The coordinates of the obstacles
	 */
	const std::vector<osg::Vec3>& getCoordinates() const {
		return coordinates_;
	}

	/**
     * The xyz sizes of each of the obstacles.
     *
	 * @return the size of the obstacles
	 */
	const std::vector<osg::Vec3>& getSizes() const {
		return sizes_;
	}

	/**
     * The float densities of each of the obstacles.
     *
	 * @return the obstacle densities
	 */
	const std::vector<float>& getDensities() const{
		return densities_;
	}

	/**
     * The axes around which the obstacle has been rotated.
     *
	 * @return the obstacle Rotation Axes
	 */
	const std::vector<osg::Vec3>& getRotationAxes() const{
		return rotationAxes_;
	}

	/**
     * Get the angles in degrees by which the obstacles are rotated.
     *
	 * @return the obstacle rotation angles
	 */
	const std::vector<float>& getRotationAngles() const{
		return rotationAngles_;
	}

	/**
	 * Serialize obstacles into a SimulatorConf message.
     *
     * @param[out] message       The message into which this object should be
     * serialised.
	 */
	void serialize(robogenMessage::SimulatorConf &message){
		for (unsigned int i=0; i<coordinates_.size(); ++i){
			robogenMessage::Obstacle *curr = message.add_obstacles();
			curr->set_density(densities_[i]);
			curr->set_x(coordinates_[i].x());
			curr->set_y(coordinates_[i].y());
			curr->set_z(coordinates_[i].z());
			curr->set_xsize(sizes_[i].x());
			curr->set_ysize(sizes_[i].y());
			curr->set_zsize(sizes_[i].z());
			curr->set_xrotation(rotationAxes_[i].x());
			curr->set_yrotation(rotationAxes_[i].y());
			curr->set_zrotation(rotationAxes_[i].z());
			curr->set_rotationangle(rotationAngles_[i]);
		}
	}

private:

	/**
	 * Obstacles coordinates
	 */
	std::vector<osg::Vec3> coordinates_;

	/**
	 * Obstacle sizes
	 */
	std::vector<osg::Vec3> sizes_;

	/**
	 * Obstacle densities. If 0, obstacle is fixed
	 */
	std::vector<float> densities_;

	/**
	 * Obstacle rotationAxes
	 */
	std::vector<osg::Vec3> rotationAxes_;

	/**
	 * Obstacle rotationAngles
	 */
	std::vector<float> rotationAngles_;
};

}

#endif /* ROBOGEN_OBSTACLES_CONFIG_H_ */
