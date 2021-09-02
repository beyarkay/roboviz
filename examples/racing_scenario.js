{
    // here we define a variable for record keeping
    distances : [],
    // optional function called at the beginning of the simulation
    setupSimulation: function() {
      this.startPositions = [];
      for (let i = 0; i < this.getSwarm().getSize(); i++) {
        this.startPositions.push(
          this.getSwarm().getRobot(i).getCoreComponent().getRootPosition()
        );
      }
      return true;
    },
/*
    // optional function called after each step of simulation
    afterSimulationStep: function() {
    return true;
    },
*/
    // optional function called at the end of the simulation
    endSimulation: function() {

      // Calculate the average distance that the swarm has moved away from the
      // origin
      let totalDistance = 0;
      for (let i = 0; i < this.getSwarm().getSize(); i++) {
        var minDistance = Number.MAX_VALUE;
        bodyParts = this.getSwarm().getRobot(i).getBodyParts();
        console.log(bodyParts.length + " body parts");
        for (var j = 0; j < bodyParts.length; j++) {
          var xDiff = (bodyParts[j].getRootPosition().x - this.startPos.x);
          var yDiff = (bodyParts[j].getRootPosition().y - this.startPos.y);
          var dist = Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff,2));

          if (dist < minDistance) {
            minDistance = dist;
          }
        }
        totalDistance += minDistance;
      }

      this.distances.push(totalDistance / this.getSwarm().getSize());
      return true;
    },
    // the one required method... return the fitness!
    // here we return minimum distance travelled across evaluations
    getFitness: function() {
    fitness = this.distances[0];
        for (var i=1; i<this.distances.length; i++) {
        if (this.distances[i] < fitness)
            fitness = this.distances[i];
    }
        return fitness;
    },

}
