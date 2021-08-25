/*
 * @(#) ConfigurationReader.h   1.0   Mar 13, 2013
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
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/algorithm/string.hpp>

#include "config/ConfigurationReader.h"
#include "config/ObstaclesConfig.h"
#include "config/RobogenConfig.h"
#include "config/StartPosition.h"
#include "config/StartPositionConfig.h"
#include "config/TerrainConfig.h"
#include "config/LightSourcesConfig.h"

#include "utils/RobogenUtils.h"

#define DEFAULT_LIGHT_SOURCE_HEIGHT (0.1)
#define DEFAULT_OBSTACLE_DENSITY (0.)
#define DEFAULT_MAX_LINEAR_ACCELERATION (15.0)
#define DEFAULT_MAX_ANGULAR_ACCELERATION (25.0)

namespace robogen {

void makeAbsolute(std::string &fileName,
		const boost::filesystem::path &filePath) {
	const boost::filesystem::path thisFilePath(fileName);
	if (!thisFilePath.is_absolute()) {
		const boost::filesystem::path absolutePath =
				boost::filesystem::absolute(thisFilePath,
						filePath.parent_path());
		fileName = absolutePath.string();
	}
}


boost::shared_ptr<RobogenConfig> ConfigurationReader::parseConfigurationFile(
    const std::string& fileName) {
  std::cout << "[I] Parsing config file: " << fileName << std::endl;

  // List all the allowed config file options in the format:
  //  (option-name, [option-type,] option-description)
  //  where option-type is not required
  boost::program_options::options_description desc(
      "Allowed options for Simulation Config File");
  desc.add_options()
    ("scenario",
     boost::program_options::value<std::string>(),
     "Experiment scenario: (racing, chasing, "
     "or a provided js file)")
    ("timeStep", boost::program_options::value<float>(),
     "Time step duration (s)")
    ("nTimeSteps", boost::program_options::value<unsigned int>(),
     "Number of timesteps (Either this or simulationTime are required)")
    ("simulationTime", boost::program_options::value<float>(),
     "Length of simulation (s)  (Either this or nTimeSteps "\
     "are required)")
    ("terrainType",
     boost::program_options::value<std::string>(),
     "Terrain type: flat or rugged")
    ("terrainHeightField",
     boost::program_options::value<std::string>(),
     "Height Field for terrain generation")
    ("terrainWidth",
     boost::program_options::value<float>(), "Terrain width")
    ("terrainHeight", boost::program_options::value<float>(),
     "Terrain height")
    ("terrainLength", boost::program_options::value<float>(),
     "Terrain length")
    ("terrainFriction",boost::program_options::value<float>(),
     "Terrain Friction Coefficient")
    ("startPositionConfigFile",
     boost::program_options::value<std::string>(),
     "Start Positions Configuration File")
    ("obstaclesConfigFile", boost::program_options::value<std::string>(),
     "Obstacles configuration file")
    ("lightSourcesConfigFile", boost::program_options::value<std::string>(),
     "Light sources configuration file")
    ("actuationFrequency",boost::program_options::value<int>(),
     "Actuation Frequency (Hz)")
    ("sensorNoiseLevel",boost::program_options::value<float>(),
     "Sensor Noise Level:\n "\
     "Sensor noise is Gaussian with std dev of "\
     "sensorNoiseLevel * actualValue.\n"\
     "i.e. value given to Neural Network is "\
     "N(a, a * s)\n"\
     "where a is actual value and s is sensorNoiseLevel")
    ("motorNoiseLevel",boost::program_options::value<float>(),
     "Motor noise level:\n"\
     "Motor noise is uniform in range +/-"\
     "(motorNoiseLevel * actualValue)")
    ("capAcceleration",boost::program_options::value<bool>(),
     "Flag to enforce acceleration cap."\
     "Useful for preventing unrealistic  behaviors "\
     "/ simulator exploits")
    ("maxLinearAcceleration",boost::program_options::value<float>(),
     "Maximum linear acceleration (if capAcceleration."\
     " is true")
    ("maxAngularAcceleration",boost::program_options::value<float>(),
     "Maximum angular acceleration (if capAcceleration."\
     " is true")
    ("maxDirectionShiftsPerSecond",boost::program_options::value<int>(),
     "Maximum number of direction shifts per second"\
     " for testing motor burnout.  If not set, then there is no"\
     " cap")
    ("gravity",
     boost::program_options::value<std::string>(),
     "Gravity: either a single z-value for g=(0,0,z)"\
     " or x,y,z (comma separated) for full g vector."\
     " Specified in m/(s^2)"\
     " Defaults to (0,0,-9.81)")
    ("disallowObstacleCollisions",
     boost::program_options::value<bool>(),
     "Flag to enforce no obstacle collisions.  If true then "\
     "any obstacle collision will be considered a constraint"\
     " violation. (default false).")
    ("swarmSize", boost::program_options::value<unsigned int>(),
     "The number of duplicate robots to include in the simulation"\
     ". Defaults to 1 if not specified")
    ("gatheringZoneSize", 
     boost::program_options::value<std::string>(),
     "The size as an 'x,y,z' string of the gathering zone, which"\
     " is an area highlighed in a special color, useful for"\
     " certain fitness functions")
    ("gatheringZonePosition", 
     boost::program_options::value<std::string>(),
     "The center as an 'x,y,z' string of the gathering zone,"\
     " which is an area  highlighed in a special color, "\
     " useful for certain fitness functions")
    ("resourcesConfigFile", 
     boost::program_options::value<std::string>(),
     "A config file containing a list of resources, with one resource"
     "per line. Each line must contain a list of space-separated "
     "floating point values defining the resource in the order:"
     "x-pos y-pos z-pos x-magnitude y-magnitude z-magnitude unknown unknown")
    ("obstacleOverlapPolicy",
     boost::program_options::value<std::string>(),
     "Defines the policy for handling obstacles "
     " enclosed in the robot's initial"\
     " axis aligned bounding box (AABB).  Options are\n"\
     "\t'removeObstacles' -- obstacles will be removed,"\
     " and the simulation will proceed (default).\n"\
     "\t'constraintViolation' -- the simulation will be"\
     " terminated with a constrain violation.\n"\
     "\t'elevateRobot' -- the robot will be elevated to be"\
     " above all obstacles before the simulation begins.\n")
    ;

  if (fileName == "help") {
    desc.print(std::cout);
    return boost::shared_ptr<RobogenConfig>();
  }

  // var_map contains all the key=value pairs as specified in the config file
  boost::program_options::variables_map var_map;

  try {
    // The calls to the store, parse_config_file and notify function cause
    // var_map to contain all the options found from the config file
    boost::program_options::store(
        boost::program_options::parse_config_file<char>(fileName.c_str(),
          desc, false), var_map);
    boost::program_options::notify(var_map);
  } catch (std::exception &e) {
    std::cerr << "Error while processing simulator configuration: "
      << e.what() << std::endl;
    return boost::shared_ptr<RobogenConfig>();
  }
  std::cout << "[I] Config file '" << fileName << "' parsed successfully" 
    << std::endl;

  const boost::filesystem::path filePath(fileName);

  // ===========================================
  // Parse all the config file's key=value pairs
  // ===========================================

  // --------------------------
  // Read terrain configuration
  // --------------------------
  float terrainLength;
  float terrainWidth;
  float terrainFriction;
  std::string terrainType;

  if (!var_map.count("terrainType")) {
    std::cerr << "Undefined 'terrainType' parameter in '" << fileName << "'"
      << std::endl;
    return boost::shared_ptr<RobogenConfig>();
  }
  terrainType = var_map["terrainType"].as<std::string>();

  if (!var_map.count("terrainFriction")) {
    std::cerr << "Undefined 'terrainFriction' parameter in '" << fileName
      << "'" << std::endl;
    return boost::shared_ptr<RobogenConfig>();
  }
  terrainFriction = var_map["terrainFriction"].as<float>();

  boost::shared_ptr<TerrainConfig> terrain;
  if (terrainType.compare("empty") == 0) {
    terrain.reset(new TerrainConfig(terrainFriction));
  } else {
    // if have empty terrain don't need these values, and leave terrain NULL
    if (!var_map.count("terrainLength")) {
      std::cerr << "Undefined 'terrainLength' parameter in '" << fileName
        << "'" << std::endl;
      return boost::shared_ptr<RobogenConfig>();
    }
    terrainLength = var_map["terrainLength"].as<float>();

    if (!var_map.count("terrainWidth")) {
      std::cerr << "Undefined 'terrainWidth' parameter in '" << fileName
        << "'" << std::endl;
      return boost::shared_ptr<RobogenConfig>();
    }
    terrainWidth = var_map["terrainWidth"].as<float>();

    if (terrainType.compare("flat") == 0) {

      terrain.reset(new TerrainConfig(terrainLength, terrainWidth,
            terrainFriction));

    } else if (terrainType.compare("rugged") == 0) {

      std::string terrainHeightField;
      float terrainHeight;

      if (!var_map.count("terrainHeightField")) {
        std::cerr << "Undefined 'terrainHeightField' parameter in '"
          << fileName << "'" << std::endl;
        return boost::shared_ptr<RobogenConfig>();
      }

      if (!var_map.count("terrainHeight")) {
        std::cerr << "Undefined 'terrainHeight' parameter in '" << fileName
          << "'" << std::endl;
        return boost::shared_ptr<RobogenConfig>();
      }

      terrainHeightField = var_map["terrainHeightField"].as<std::string>();
      makeAbsolute(terrainHeightField, filePath);



      terrainHeight = var_map["terrainHeight"].as<float>();

      terrain.reset(
          new TerrainConfig(terrainHeightField, terrainLength,
            terrainWidth, terrainHeight, terrainFriction));

    } else {
      std::cerr << "Unknown value of 'terrainType' parameter in '" << fileName
        << "'" << std::endl;
      return boost::shared_ptr<RobogenConfig>();
    }
  }

  // ----------------------------
  // Read obstacles configuration
  // ----------------------------
  boost::shared_ptr<ObstaclesConfig> obstacles;
  std::string obstaclesConfigFile = "";
  if (!var_map.count("obstaclesConfigFile")) {
    obstacles.reset(new ObstaclesConfig());
  } else {
    obstaclesConfigFile = var_map["obstaclesConfigFile"].as<std::string>();

    makeAbsolute(obstaclesConfigFile, filePath);

    obstacles = parseObstaclesFile(
        obstaclesConfigFile);
    if (obstacles == NULL) {
      return boost::shared_ptr<RobogenConfig>();
    }
  }

  // ----------------------------
  // Read start pos configuration
  // ----------------------------
  boost::shared_ptr<StartPositionConfig> startPositions;
  std::string startPositionFile = "";
  if (!var_map.count("startPositionConfigFile")) {
    std::cout << "No startPositionConfigFile provided so will use a single"
      << " evaluation with the robot starting at the origin, and "
      << "having 0 azimuth" << std::endl;
    std::vector<boost::shared_ptr<StartPosition> > startPositionVector;

    boost::shared_ptr<StartPosition> startPosition(new StartPosition());
    if (!startPosition->init(osg::Vec2(0,0), 0)) {
      std::cerr << "Problem initializing start position!" << std::endl;
      return boost::shared_ptr<RobogenConfig>();
    }

    startPositionVector.push_back(startPosition);
    startPositions.reset(new StartPositionConfig(startPositionVector));

  } else {

    startPositionFile = var_map["startPositionConfigFile"].as<std::string>();

    makeAbsolute(startPositionFile, filePath);

    startPositions = parseStartPositionFile(startPositionFile);
    if (startPositions == NULL) {
      return boost::shared_ptr<RobogenConfig>();
    }
  }

  // --------------------------------
  // Read light sources configuration
  // --------------------------------
  boost::shared_ptr<LightSourcesConfig> lightSources;
  std::string lightSourcesFile = "";
  if (!var_map.count("lightSourcesConfigFile")) {
    lightSources.reset(new LightSourcesConfig());
  } else {
    lightSourcesFile =
      var_map["lightSourcesConfigFile"].as<std::string>();

    makeAbsolute(lightSourcesFile, filePath);

    lightSources = parseLightSourcesFile(lightSourcesFile);
    if (lightSources == NULL) {
      return boost::shared_ptr<RobogenConfig>();
    }
  }


  // ---------------------------
  // Read scenario configuration
  // ---------------------------
  if (!var_map.count("scenario")) {
    std::cerr << "Undefined 'scenario' parameter in '" << fileName << "'"
      << std::endl;
    return boost::shared_ptr<RobogenConfig>();
  }
  std::string scenario = var_map["scenario"].as<std::string>();
  std::string scenarioFile = "";
  // If the scenario isn't racing nor chasing, then read in the js definition
  // of what the scenario should be
  if (scenario.compare("racing") != 0 && scenario.compare("chasing") != 0) {
    const boost::filesystem::path scenarioFilePath(scenario);
    if (!scenarioFilePath.is_absolute()) {
      const boost::filesystem::path absolutePath =
        boost::filesystem::absolute(scenarioFilePath,
            filePath.parent_path());
      scenarioFile = absolutePath.string();
    }
    if(boost::filesystem::path(scenarioFile).extension().string().compare(".js")
        == 0) {

      //read entire js file into string buffer

      std::ifstream file(scenarioFile.c_str());
      if (!file.is_open()) {
        std::cout << "Cannot find scenario js: '" << scenarioFile << "'"
          << std::endl;
        return boost::shared_ptr<RobogenConfig>();
      }


      std::stringstream buffer;
      buffer << file.rdbuf();
      scenario = buffer.str();
    } else {
      std::cerr << "Invalid 'scenario' parameter" << std::endl;
      return boost::shared_ptr<RobogenConfig>();
    }
  }

  if (!var_map.count("timeStep")) {
    std::cerr << "Undefined 'timeStep' parameter in '" << fileName << "'"
      << std::endl;
    return boost::shared_ptr<RobogenConfig>();
  }
  float timeStep = var_map["timeStep"].as<float>();
  int timeStepTmp = boost::math::iround (timeStep * 100000);

  if (!var_map.count("nTimeSteps") && !var_map.count("simulationTime")) {
    std::cerr << "Either 'nTimesteps' or 'simulationTime' is required "
      << "in '" << fileName << "'" << std::endl;
    return boost::shared_ptr<RobogenConfig>();
  }
  if (var_map.count("nTimeSteps") && var_map.count("simulationTime")) {
    std::cerr << "Only one of 'nTimesteps' or 'simulationTime' should "
      << "be specified in '" << fileName << "'" << std::endl;
    return boost::shared_ptr<RobogenConfig>();
  }
  unsigned int nTimeSteps;
  if (var_map.count("nTimeSteps")){
    nTimeSteps = var_map["nTimeSteps"].as<unsigned int>();
  } else {
    int simulationTimeTmp = boost::math::iround (
        var_map["simulationTime"].as<float>() * 100000);
    if ((simulationTimeTmp % timeStepTmp) != 0) {
      std::cerr << "'simulationTime' must be a multiple "
        << "of 'timeStep'" << std::endl;
      return boost::shared_ptr<RobogenConfig>();
    }
    nTimeSteps = simulationTimeTmp / timeStepTmp;
  }

  int actuationPeriod;
  if (!var_map.count("actuationFrequency")) {
    actuationPeriod = 1;
    std::cout << "Undefined 'actuationFrequency' parameter in '"
      << fileName << "'" << ", will actuate every timeStep."
      << std::endl;
  } else {
    int actuationFrequencyTmp = boost::math::iround (
        (1.0/((float)var_map["actuationFrequency"].as<int>())) * 100000);
    if ((actuationFrequencyTmp % timeStepTmp) != 0) {
      std::cerr << "Inverse of 'actuationFrequency' must be a multiple "
        << "of 'timeStep'" << std::endl;
      return boost::shared_ptr<RobogenConfig>();
    }
    actuationPeriod = actuationFrequencyTmp / timeStepTmp;
  }

  // -----------------------------------------------
  // Read sensor and motor noise level configuration
  // -----------------------------------------------

  float sensorNoiseLevel = 0;
  float motorNoiseLevel = 0;

  if(var_map.count("sensorNoiseLevel")) {
    sensorNoiseLevel = var_map["sensorNoiseLevel"].as<float>();
  }

  if(var_map.count("motorNoiseLevel")) {
    motorNoiseLevel = var_map["motorNoiseLevel"].as<float>();
  }

  bool capAcceleration = false;
  float maxLinearAcceleration = DEFAULT_MAX_LINEAR_ACCELERATION;
  float maxAngularAcceleration = DEFAULT_MAX_ANGULAR_ACCELERATION;

  if(var_map.count("capAcceleration")) {
    capAcceleration = var_map["capAcceleration"].as<bool>();
  }

  if(var_map.count("maxLinearAcceleration")) {
    maxLinearAcceleration = var_map["maxLinearAcceleration"].as<float>();
  }

  if(var_map.count("maxAngularAcceleration")) {
    maxAngularAcceleration = var_map["maxAngularAcceleration"].as<float>();
  }

  int maxDirectionShiftsPerSecond = -1;
  if(var_map.count("maxDirectionShiftsPerSecond")) {
    maxDirectionShiftsPerSecond = var_map["maxDirectionShiftsPerSecond"
    ].as<int>();
  }

  osg::Vec3 gravity(0,0,-9.81);
  if(var_map.count("gravity")) {
    std::string gravityString = var_map["gravity"].as<std::string>();
    std::vector<std::string> gravityOpts;
    boost::split(gravityOpts, gravityString, boost::is_any_of(","));
    if (gravityOpts.size() == 1) {
      gravity[2] = std::atof(gravityOpts[0].c_str());
    } else if (gravityOpts.size() == 3) {
      for(unsigned int i=0; i<3; ++i) {
        gravity[i] = std::atof(gravityOpts[i].c_str());
      }
    } else {
      std::cerr << "'gravity' must either be a single value for " <<
        "g=(0,0,z) or x,y,z (comma separated) for full g vector" <<
        std::endl;
      return boost::shared_ptr<RobogenConfig>();
    }
  }

  bool disallowObstacleCollisions = false;
  if(var_map.count("disallowObstacleCollisions")) {
    disallowObstacleCollisions = var_map["disallowObstacleCollisions"
    ].as<bool>();
  }

  unsigned int obstacleOverlapPolicy;

  if((!var_map.count("obstacleOverlapPolicy")) ||
      (var_map["obstacleOverlapPolicy"].as<std::string>() ==
       "removeObstacles")) {
    obstacleOverlapPolicy = RobogenConfig::REMOVE_OBSTACLES;
  } else if(var_map["obstacleOverlapPolicy"].as<std::string>() ==
      "constraintViolation") {
    obstacleOverlapPolicy = RobogenConfig::CONSTRAINT_VIOLATION;
  } else if(var_map["obstacleOverlapPolicy"].as<std::string>() ==
      "elevateRobot") {
    obstacleOverlapPolicy = RobogenConfig::ELEVATE_ROBOT;
  } else {
    std::cerr << "Invalid value: '" <<
      var_map["obstacleOverlapPolicy"].as<std::string>() <<
      "' given for 'obstacleOverlapPolicy'" << std::endl;
    return boost::shared_ptr<RobogenConfig>();
  }

  // ------------------------------------
  // Read in the swarmSize configurations
  // ------------------------------------
  unsigned int swarmSize;
  if (var_map.count("swarmSize")){
    std::cout << "[W] parameter swarmSize has been "
      "specified but the code for using it has not been "
      "implemented yet. The value will be ignored." << std::endl;
    swarmSize = var_map["swarmSize"].as<unsigned int>();
  } else {
    swarmSize = 1;
  }
  
  // -----------------------------------------
  // Read in the gathering zone configurations
  // -----------------------------------------
  // Configure the gathering zone position
  osg::Vec3 gatheringZonePos(0,0,0);
  if(var_map.count("gatheringZonePosition")) {
    std::cout << "[W] parameter gatheringZonePos has been "
      "specified but the code for using it has not been "
      "implemented yet. The value will be ignored." << std::endl;
    std::string gatheringZonePosString =
      var_map["gatheringZonePosition"].as<std::string>();
    std::vector<std::string> gatheringZonePosOpts;
    boost::split(gatheringZonePosOpts, gatheringZonePosString,
        boost::is_any_of(","));
    if (gatheringZonePosOpts.size() == 3) {
      for(unsigned int i=0; i<3; ++i) {
        gatheringZonePos[i] = std::atof(gatheringZonePosOpts[i].c_str());
      }
    } else {
      std::cerr << "'gatheringZonePosition' (" << gatheringZonePosString 
        << ") from config file '" << fileName 
        << "' is not a set of 3 floating point values seperated by "
        "commas, for example: 1,2.0,3.4" << std::endl;
      return boost::shared_ptr<RobogenConfig>();
    }
  }

  // Configure the gathering zone size
  osg::Vec3 gatheringZoneSize(0,0,0);
  if(var_map.count("gatheringZoneSize")) {
    std::cout << "[W] parameter gatheringZoneSize has been "
      "specified but the code for using it has not been "
      "implemented yet. The value will be ignored." << std::endl;
    std::string gatheringZoneSizeString =
      var_map["gatheringZoneSize"].as<std::string>();
    std::vector<std::string> gatheringZoneSizeOpts;
    boost::split(gatheringZoneSizeOpts, gatheringZoneSizeString,
        boost::is_any_of(","));
    if (gatheringZoneSizeOpts.size() == 3) {
      for(unsigned int i=0; i<3; ++i) {
        gatheringZoneSize[i] = std::atof(gatheringZoneSizeOpts[i].c_str());
      }
    } else {
      std::cerr << "'gatheringZoneSize' (" << gatheringZoneSizeString 
        << ") from config file '" << fileName 
        << "' is not a set of 3 floating point values seperated by "
        "commas, for example: 1,2.0,3.4" << std::endl;
      return boost::shared_ptr<RobogenConfig>();
    }
  }

  // --------------------------------
  // Read the resources configuration
  // --------------------------------
  std::string resourcesConfigFile = "";
  if (var_map.count("resourcesConfigFile")) {
    std::cout << "[W] parameter resourcesConfigFile has been "
      "specified but the code using it has not been "
      "implemented yet. The value will be ignored." << std::endl;
      resourcesConfigFile = var_map["resourcesConfigFile"].as<std::string>();
  }

  return boost::shared_ptr<RobogenConfig>(
      new RobogenConfig(scenario, scenarioFile, nTimeSteps,
        timeStep, actuationPeriod, terrain,
        obstacles, obstaclesConfigFile, startPositions,
        startPositionFile, lightSources, lightSourcesFile,
        sensorNoiseLevel,
        motorNoiseLevel, capAcceleration, maxLinearAcceleration,
        maxAngularAcceleration, maxDirectionShiftsPerSecond,
        gravity, disallowObstacleCollisions,
        swarmSize, gatheringZoneSize, gatheringZonePos, resourcesConfigFile,
        obstacleOverlapPolicy));
}

const std::string getMatchNFloatPattern(unsigned int n) {
	std::stringstream paternSS;
	paternSS << "^";
	for ( unsigned i=0; i<n; i++) {
		paternSS << "(-?\\d*[\\d\\.]\\d*)";
		if ( i < (n-1) )
			paternSS << "\\s+";
	}
	paternSS << "$";
	return paternSS.str();
}

boost::shared_ptr<ObstaclesConfig> ConfigurationReader::parseObstaclesFile(
		const std::string& fileName) {

	std::ifstream obstaclesFile(fileName.c_str());
	if (!obstaclesFile.is_open()) {
		std::cout << "Cannot find obstacles file: '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<ObstaclesConfig>();
	}

	// old = on ground, no rotation:
	// x y xLength yLength zLength density
	static const boost::regex oldObstacleRegex(getMatchNFloatPattern(6));
	// same as above but can be above ground:
	// x y z xLength yLength zLength density
	static const boost::regex noRotationObstacleRegex(
			getMatchNFloatPattern(7));
	// full (w/ axis+angle rotation)
	// x y z xLength yLength zLength density xRot yRot zRot rotAngle
	static const boost::regex fullObstacleRegex(
				getMatchNFloatPattern(11));

	std::vector<osg::Vec3> coordinates;
	std::vector<osg::Vec3> sizes;
	std::vector<float> densities;
	std::vector<osg::Vec3> rotationAxes;
	std::vector<float> rotationAngles;

	std::string line;
	int lineNum = 0;
	while (!RobogenUtils::safeGetline(obstaclesFile, line).eof()) {
		lineNum++;
		boost::cmatch match;
		float x, y, z, xSize, ySize, zSize, density, xRotation, yRotation,
				zRotation, rotationAngle;
		if(boost::regex_match(line.c_str(), match,
						fullObstacleRegex)){
			x = std::atof(match[1].str().c_str());
			y = std::atof(match[2].str().c_str());
			z = std::atof(match[3].str().c_str());
			xSize = std::atof(match[4].str().c_str());
			ySize = std::atof(match[5].str().c_str());
			zSize = std::atof(match[6].str().c_str());
			density = std::atof(match[7].str().c_str());
			xRotation = std::atof(match[8].str().c_str());
			yRotation = std::atof(match[9].str().c_str());
			zRotation = std::atof(match[10].str().c_str());
			rotationAngle = std::atof(match[11].str().c_str());
		} else {
			xRotation = yRotation = zRotation = rotationAngle = 0.0;
			if (boost::regex_match(line.c_str(), match, oldObstacleRegex)){
				x = std::atof(match[1].str().c_str());
				y = std::atof(match[2].str().c_str());
				xSize = std::atof(match[3].str().c_str());
				ySize = std::atof(match[4].str().c_str());
				zSize = std::atof(match[5].str().c_str());
				density = std::atof(match[6].str().c_str());

				z = zSize/2;
			} else if(boost::regex_match(line.c_str(), match,
					noRotationObstacleRegex)){
				x = std::atof(match[1].str().c_str());
				y = std::atof(match[2].str().c_str());
				z = std::atof(match[3].str().c_str());
				xSize = std::atof(match[4].str().c_str());
				ySize = std::atof(match[5].str().c_str());
				zSize = std::atof(match[6].str().c_str());
				density = std::atof(match[7].str().c_str());
			} else {
				std::cerr << "Error parsing line " << lineNum <<
						" of obstacles file: '" << fileName << "'"
						<< std::endl;
				std::cerr << line.c_str() << std::endl;
				return boost::shared_ptr<ObstaclesConfig>();
			}
		}
		coordinates.push_back(osg::Vec3(x, y, z));
		sizes.push_back(osg::Vec3(xSize, ySize, zSize));
		densities.push_back(density);
		rotationAxes.push_back(osg::Vec3(xRotation, yRotation, zRotation));
		rotationAngles.push_back(rotationAngle);
	}

	return boost::shared_ptr<ObstaclesConfig>(
			new ObstaclesConfig(coordinates, sizes, densities,
					rotationAxes, rotationAngles));
}

boost::shared_ptr<LightSourcesConfig> ConfigurationReader::parseLightSourcesFile(
		const std::string& fileName) {

	std::ifstream lightSourcesFile(fileName.c_str());
	if (!lightSourcesFile.is_open()) {
		std::cout << "Cannot find light sources file: '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<LightSourcesConfig>();
	}

	// either 3 or 4 numbers (x,y,z) or (x,y,z,intensity)
	static const boost::regex noIntensityRegex(getMatchNFloatPattern(3));
	static const boost::regex fullRegex(getMatchNFloatPattern(4));

	std::vector<osg::Vec3> coordinates;
	std::vector<float> intensities;

	std::string line;
	int lineNum = 0;
	while (!RobogenUtils::safeGetline(lightSourcesFile, line).eof()) {
		lineNum++;
		boost::cmatch match;
		float x, y, z, intensity;
		if(boost::regex_match(line.c_str(), match, fullRegex)){
			intensity = std::atof(match[4].str().c_str());
		} else {
			intensity = 1.0;
			if(!boost::regex_match(line.c_str(), match, noIntensityRegex)){

				std::cerr << "Error parsing line " << lineNum <<
						" of light sources file: '" << fileName << "'"
						<< std::endl;
				std::cerr << line.c_str() << std::endl;
				return boost::shared_ptr<LightSourcesConfig>();
			}
		}
		x = std::atof(match[1].str().c_str());
		y = std::atof(match[2].str().c_str());
		z = std::atof(match[3].str().c_str());
		coordinates.push_back(osg::Vec3(x, y, z));
		intensities.push_back(intensity);

	}

	return boost::shared_ptr<LightSourcesConfig>(
			new LightSourcesConfig(coordinates, intensities));
}


boost::shared_ptr<StartPositionConfig> ConfigurationReader::parseStartPositionFile(
		const std::string& fileName) {

	std::ifstream startPosFile(fileName.c_str());
	if (!startPosFile.is_open()) {
		std::cout << "Cannot find start position file: '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<StartPositionConfig>();
	}
	std::vector<boost::shared_ptr<StartPosition> > startPositions;
	float x, y, azimuth;
	while (startPosFile >> x) {
		if (!(startPosFile >> y)) {
			std::cout << "Malformed start position file: '" << fileName << "'"
					<< std::endl;
			return boost::shared_ptr<StartPositionConfig>();
		}
		if (!(startPosFile >> azimuth)) {
			std::cout << "Malformed start position file: '" << fileName << "'"
					<< std::endl;
			return boost::shared_ptr<StartPositionConfig>();
		}
		startPositions.push_back(
				boost::shared_ptr<StartPosition>(new StartPosition()));
		if (!startPositions.back()->init(osg::Vec2(x, y), azimuth)) {
			std::cout << "Problem initializing start position!" << std::endl;
			return boost::shared_ptr<StartPositionConfig>();
		}
	}

	return boost::shared_ptr<StartPositionConfig>(
			new StartPositionConfig(startPositions));

}

boost::shared_ptr<RobogenConfig> ConfigurationReader::parseRobogenMessage(
		const robogenMessage::SimulatorConf& simulatorConf) {

	// Decode obstacles
	std::vector<osg::Vec3> obstaclesCoord;
	std::vector<osg::Vec3> obstaclesSize;
	std::vector<float> obstaclesDensity;
	std::vector<osg::Vec3> obstaclesRotationAxis;
	std::vector<float> obstaclesRotationAngle;
	for (int i = 0; i < simulatorConf.obstacles_size(); ++i) {

		const robogenMessage::Obstacle& o = simulatorConf.obstacles(i);
		obstaclesCoord.push_back(osg::Vec3(o.x(), o.y(), o.z()));
		obstaclesSize.push_back(osg::Vec3(o.xsize(), o.ysize(), o.zsize()));
		obstaclesDensity.push_back(o.density());
		obstaclesRotationAxis.push_back(osg::Vec3(o.xrotation(),
				o.yrotation(), o.zrotation()));
		obstaclesRotationAngle.push_back(o.rotationangle());

	}
	boost::shared_ptr<ObstaclesConfig> obstacles(
			new ObstaclesConfig(obstaclesCoord, obstaclesSize,
					obstaclesDensity, obstaclesRotationAxis,
					obstaclesRotationAngle));

	// Decode light sources
	std::vector<osg::Vec3> lightSourcesCoords;
	std::vector<float> lightSourcesIntensities;
	for (int i = 0; i < simulatorConf.lightsources_size(); ++i) {
		const robogenMessage::LightSource& ls = simulatorConf.lightsources(i);
		lightSourcesCoords.push_back(osg::Vec3(ls.x(), ls.y(), ls.z()));
		lightSourcesIntensities.push_back(ls.intensity());
	}
	boost::shared_ptr<LightSourcesConfig> lightSources(
				new LightSourcesConfig(lightSourcesCoords,
						lightSourcesIntensities));


	// Decode start positions
	std::vector<boost::shared_ptr<StartPosition> > startPositions;
	for (int i = 0; i < simulatorConf.startpositions_size(); ++i) {
		const robogenMessage::StartPosition& s = simulatorConf.startpositions(
				i);
		boost::shared_ptr<StartPosition> newStartPos(new StartPosition());
		newStartPos->init(osg::Vec2(s.x(), s.y()), s.azimuth());
		startPositions.push_back(newStartPos);
	}

	// Decode terrain configuration
	boost::shared_ptr<TerrainConfig> terrain;
	if(simulatorConf.terraintype() == TerrainConfig::EMPTY) {
		terrain.reset(new TerrainConfig(simulatorConf.terrainfriction()));
	} else if(simulatorConf.terraintype() == TerrainConfig::FLAT) {
		terrain.reset(new TerrainConfig(simulatorConf.terrainlength(),
					simulatorConf.terrainwidth(),
					simulatorConf.terrainfriction()));
	} else if(simulatorConf.terraintype() == TerrainConfig::ROUGH) {
		terrain.reset(new TerrainConfig(
							simulatorConf.terrainheightfieldfilename(),
							simulatorConf.terrainlength(),
							simulatorConf.terrainwidth(),
							simulatorConf.terrainheight(),
							simulatorConf.terrainfriction()));
	}

	// Decode simulator configuration
	std::string scenario = simulatorConf.scenario();

	//todo with js check!!

	//if (scenario.compare("racing") != 0 && scenario.compare("chasing") != 0) {
	//	std::cerr << "Undefined 'scenario' parameter" << std::endl;
	//	return boost::shared_ptr<RobogenConfig>();
	//}

	unsigned int timeSteps = simulatorConf.ntimesteps();
	float timeStepLength = simulatorConf.timestep();
	int actuationPeriod = simulatorConf.actuationperiod();

    return boost::shared_ptr<RobogenConfig>(
        new RobogenConfig(
          scenario, "", timeSteps, timeStepLength,
          actuationPeriod, terrain, obstacles, "",
          boost::shared_ptr<StartPositionConfig>(
            new StartPositionConfig(startPositions)), "",
          lightSources, "", simulatorConf.sensornoiselevel(),
          simulatorConf.motornoiselevel(),
          simulatorConf.capacceleration(),
          simulatorConf.maxlinearacceleration(),
          simulatorConf.maxangularacceleration(),
          simulatorConf.maxdirectionshiftspersecond(),
          osg::Vec3(
              simulatorConf.gravityx(),
              simulatorConf.gravityy(),
              simulatorConf.gravityz()),
          simulatorConf.disallowobstaclecollisions(),
          simulatorConf.swarmsize(),
          osg::Vec3(
              simulatorConf.gatheringzonesizex(),
              simulatorConf.gatheringzonesizey(),
              simulatorConf.gatheringzonesizez()),
          osg::Vec3(
              simulatorConf.gatheringzoneposx(),
              simulatorConf.gatheringzoneposy(),
              simulatorConf.gatheringzoneposz()),
          simulatorConf.resourcesconfigfile(),
          simulatorConf.obstacleoverlappolicy()
            ));

}

}
