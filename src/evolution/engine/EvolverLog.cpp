/*
 * @(#) EvolverLog.cpp   1.0   Sep 2, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014 Titus Cieslewski
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
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "evolution/representation/RobotRepresentation.h"
#include "evolution/engine/EvolverLog.h"
#include "evolution/engine/Population.h"
#include "utils/json2pb/json2pb.h"

namespace robogen {

#define LOG_DIRECTORY_PREFIX "results/BrainEvolution_"
#define LOG_DIRECTORY_FACET "%Y%m%d-%H%M%S"
#define BAS_LOG_FILE "BestAvgStd.txt"
#define GENERATION_BEST_PREFIX "GenerationBest-"

EvolverLog::EvolverLog(){
}

bool EvolverLog::init(const std::string& confFile, const std::string& logFolderPostfix) {

	// create log directory with time stamp
	std::stringstream logPathSs;
	logPathSs << LOG_DIRECTORY_PREFIX << logFolderPostfix;

	std::string prefixPath = logPathSs.str();
	std::string tempPath = prefixPath;
	int curIndex = 0;
	while (boost::filesystem::is_directory(tempPath)) {
		std::stringstream newPath;
		newPath << prefixPath << "_" << ++curIndex;
		tempPath = newPath.str();
	}

	logPath_ = tempPath;
	boost::filesystem::path logPath(logPath_);

	try{
		boost::filesystem::create_directories(logPath);
	} catch(const boost::filesystem::filesystem_error &err){
		std::cout << err.what() << std::endl << "Evolver log can't create log"\
				" directory.\n" << std::endl;
		return false;
	}

	// open trajectory log
	std::string basLogPath = logPathSs.str() + "/" + BAS_LOG_FILE;
	bestAvgStd_.open(basLogPath.c_str());
	if (!bestAvgStd_.is_open()){
		std::cout << "Can't open Best/Average/STD log file" << std::endl;
		return false;
	}

	// copy evolution configuration file
	// not copying simulator config file, as hard to also get obstacles and
	// startpos - though probably worth it - so TODO (along with robot)
	boost::filesystem::path confFrom(confFile);
	std::stringstream ss;
	ss << logPath_ << "/" << confFrom.filename().string();
	boost::filesystem::path confTo(ss.str());
	boost::filesystem::copy_file(confFrom, confTo);
	return true;
}

EvolverLog::~EvolverLog() {
}

bool EvolverLog::logGeneration(int step, Population &population) {

	if (!population.areEvaluated()){
		std::cout << "EvolverLog::logGeneration(): Trying to log non-evaluated"\
				" population!" << std::endl;
		return false;
	}

	// log best, avg, stddev
	double best,average,stdev;
	population.getStat(best,average,stdev);
	std::cout << "Best: " << best << " Average: " << average << " STD: " <<
				stdev << std::endl;
	bestAvgStd_ << step << " " << best << " " <<
			average << " "  << stdev << std::endl;

	// save robot file of best robot (don't do with fake robot representation)
	#ifndef FAKEROBOTREPRESENTATION_H

	boost::shared_ptr<RobotRepresentation> bestRobot = population.best();

	std::stringstream ss;
	ss << logPath_ + "/" + GENERATION_BEST_PREFIX << step << ".json";

	// De-comment to save protobuf binary file
	// ss << logPath_ + "/" + GENERATION_BEST_PREFIX << step << ".dat
	// bestRobot->serialize().SerializeToOstream(&curRobotFile);

	std::ofstream curRobotFile(ss.str().c_str(),std::ios::out|std::ios::trunc);
	curRobotFile << pb2json(bestRobot->serialize());
	curRobotFile.close();

	#endif /* FAKEROBOTREPRESENTATION_H */

	return true;
}

} /* namespace robogen */
