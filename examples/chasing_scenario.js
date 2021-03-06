{
    // here we define variables for record keeping
    trialFits : [],

    // called at the beginning of the simulation
    setupSimulation: function() {
		console.log("checking light sources");
		if(this.getEnvironment().getLightSources().length == 0) {
			console.log( "Error: At least 1 light source is required for the chasing scenario.");
			return false;
		}

		this.trialFits.push(0);
		this.timeSteps = 0;
		return true;
    },

    // called after each step of simulation
    afterSimulationStep: function() {
      // Loop through all robots and get the average distance from the light
      let totalDistance = 0;
      var lightSourcePos =
        this.getEnvironment().getLightSources()[0].getPosition();
      for (let i = 0; i < this.getSwarm().getSize(); i++) {
        let currPos =
          this.getSwarm.getRobot(i).getCoreComponent().getRootPosition();
        totalDistance += this.vectorDistance(currPos, lightSourcePos);
      }
      this.trialFits[this.getCurTrial()] += (totalDistance / this.getSwarm().getSize());
      this.timeSteps++;

      return true;
    },


    // optional function called at the end of the simulation
    endSimulation: function() {
      // We take average distance across trial, and multiply by -1 to make
      // a maximization problem (want to be as close as possible to light
      // source at each time step)
      console.log(this.getCurTrial());
      console.log(this.trialFits[this.getCurTrial()]);
      this.trialFits[this.getCurTrial()] *= -1 / this.timeSteps;
      console.log(this.trialFits[this.getCurTrial()]);

      return true;
    },


    // the one required method... return the fitness!
    // as usual, take minimum across trials
    getFitness: function() {
		var fitness =  Number.MAX_VALUE;
		for (var i = 0; i < this.trialFits.length; i++) {
			if (this.trialFits[i] < fitness)
				fitness = this.trialFits[i];
		}
		return fitness;

    },

}
