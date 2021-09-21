/*
 * @(#) ChasingScenario.cpp   1.0   Mar 20, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani, Joshua Auerbach
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
#include "config/RobogenConfig.h"
#include "config/StartPositionConfig.h"
#include "scenario/ChasingScenario.h"
#include "scenario/Environment.h"
#include "Robot.h"
#include "Swarm.h"
#include "Models.h"

namespace robogen {

ChasingScenario::ChasingScenario(boost::shared_ptr<RobogenConfig> robogenConfig) :
		Scenario(robogenConfig), curTrial_(0) {

}

ChasingScenario::~ChasingScenario() {

}

bool ChasingScenario::setupSimulation() {
//changed this method to initialize the distance for each robot to 0 for the current trial
	this->distances_.push_back(std::vector<double>(0));
	for (int i = 0; i < this->getSwarm()->getSize(); ++i) {
		this->distances_.back().push_back(0);
	}
	return true;
}

bool ChasingScenario::init(dWorldID odeWorld, dSpaceID odeSpace, boost::shared_ptr<Swarm> swarm) {

	Scenario::init(odeWorld, odeSpace, swarm);


	if(this->getEnvironment()->getLightSources().size() == 0) {
		std::cerr << "[E] At least 1 light source is required for the "
				<< "chasing scenario." << std::endl;
		return false;
	}
	if(this->getEnvironment()->getLightSources().size() > 1) {
		std::cerr << "[W] "
          << this->getEnvironment()->getLightSources().size() " lights"
          << " specified, but only the first will be used to evaluate "
          << " swarm fitness for the chasing scenario." << std::endl;
	}


	return true;
}


bool ChasingScenario::afterSimulationStep() {

	// Compute distance from light source
	osg::Vec3 lightSourcePos = this->getEnvironment()->getLightSources()[0]->getPosition();

	for (int i = 0; i < this->getSwarm()->getSize(); ++i) {
		osg::Vec3 curPos = this->getSwarm()->getRobot(i)->getCoreComponent()->getRootPosition();
		osg::Vec3 temp = curPos - lightSourcePos;
		this->distances_.at(curTrial_).at(i) += temp.length(); //set the distance value for the current trial for the i'th robot
	}
	return true;
}

bool ChasingScenario::endSimulation() {

	// Compute robot ending position
	curTrial_++;
	// Set next starting position
	this->setStartingPosition(curTrial_);
	return true;

}

double ChasingScenario::getFitness() {
//changed the way fitness is calculated to take the average of all robots in the swarm
	std::vector<double> fitnesses;
	for (int i = 0; i < this->getSwarm()->getSize(); ++i) {
		fitnesses.push_back(1000);
	}
	for (unsigned int i = 0; i < distances_.size(); ++i) {
		for (unsigned int j = 0; j < this->getSwarm()->getSize(); ++j) {
			double trialFit = -1.0 * distances_.at(i).at(j)/this->getRobogenConfig()->getTimeSteps();
//double trialFit = -1.0 * distances_[i]/this->getRobogenConfig()->getTimeSteps();
			if (trialFit < fitnesses.at(j)) {
				fitnesses.at(j) = trialFit;
			//fitness = trialFit;
			}
		}
	}
	double swarmFitness;
	for (int i = 0; i < fitnesses.size(); ++i) {
		swarmFitness += fitnesses.at(i);
	}
	return swarmFitness/fitnesses.size();
	// We transform everything into a maximization problem
	//return -1*(fitness/distances_.size());
}

bool ChasingScenario::remainingTrials() {
	boost::shared_ptr<StartPositionConfig> startPos = this->getRobogenConfig()->getStartingPos();
	return curTrial_ < startPos->getStartPosition().size();
}

int ChasingScenario::getCurTrial() const {
	return curTrial_;
}

}
