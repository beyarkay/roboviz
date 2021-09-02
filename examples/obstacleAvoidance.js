{
    // here we define a variable for record keeping
    velocities : [],
    deltaVelocities : [],
    maxIrVals : [],
    fitnesses : [],

    setupSimulation: function() {
      this.velocities = [];
      this.deltaVelocities = [];
      this.maxIrVals = [];
      for (let i = 0; i < this.getSwarm().getSize(); i++) {
        this.velocities.push([]);
        this.deltaVelocity.push([]);
        this.maxIrVals.push([]);
      }
      return true;
    },

    // optional function called after each step of simulation
    afterSimulationStep: function() {
      // Loop through every robot
      for (let i = 0; i < this.getSwarm().getSize(); i++) {
        // Record whichever sensor had the highest InfraRed distance measurement
        var sensors = this.getSwarm().getRobot(i).getSensors();
        var maxIr = 0
        for (var j = 0; j < sensors.length; j++) {
          if (/^Distance/.test(sensors[j].getLabel())) {
            if (sensors[j].read() > maxIr)
              maxIr = sensors[j].read();
          }
        }
        this.maxIrVals[i].push(maxIr);

        // Now record the mean velocity of the robot's motors
        var motors = this.getSwarm().getRobot(i).getMotors();
        var meanVelocity = (motors[1].getVelocity() - motors[0].getVelocity())
          / (2.0 * 2.0 * Math.PI);
        meanVelocity = (meanVelocity + 1)/2.0;
        this.velocities[i].push(meanVelocity);

        // Also record the delta velocity for each robot
        var deltaVelocity = Math.abs(motors[1].getVelocity() +
          motors[0].getVelocity()) / (2.0 * 2.0 * Math.PI);
        this.deltaVelocities[i].push(deltaVelocity);
      }
      return true;
    },

    // optional function called at the end of the simulation
    endSimulation: function() {
      var sum = 0;
      let count  = 0;
      for (var i = 0; i < this.maxIrVals.length; i++) {
        for (let j = 0; j < this.maxIrVals[j].length; j++) {
          sum += this.velocities[i][j]
                * ( 1 - Math.sqrt(this.deltaVelocities[i][j]) )
                * ( 1 - this.maxIrVals[i][j] );
        }
        count += 1;
      }
      let fitness = sum / count;
      this.fitnesses.push(fitness);

      return true;
    },
    // the one required method... return the fitness!
    getFitness: function() {
      var fitness = this.fitnesses[0];
      for (var i = 1; i < this.fitnesses.length; i++) {
        if (this.fitnesses[i] < fitness)
          fitness = this.fitnesses[i];
      }
      return fitness;
    },

}
