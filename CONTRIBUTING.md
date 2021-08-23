# Contributing to RoboGen

This file is intended to help out people who want to start making changes to
the [RoboGen project](https://github.com/lis-epfl/robogen), since the
documentation is pretty terrible and full of adverts.

## Installation
The [online instructions for
installation](http://robogen.org/docs/robogen-with-source/) from source seem to
work okay for Linux-based systems, but don't attempt to install it on MacOS, it
doesn't seem to work at all.

Just make sure the version of ODE you're installing is actually the latest
version when you install it with a command like:

```
wget "https://bitbucket.org/odedevs/ode/downloads/ode-VERSION_NUMBER_HERE.tar.gz"
tar -zxvf ode-VERSION_NUMBER_HERE.tar.gz
```
Using version 0.16.2 seems to work alright for me, like:
```
wget "https://bitbucket.org/odedevs/ode/downloads/ode-0.16.2.tar.gz"
tar -zxvf ode-0.16.2.tar.gz
```

## Testing the installation works
There aren't any instructions on how to do this, which is strange. But to run
an example and make sure the visualiser is up and working.

First compile the code and make sure it gets to 100% without errors:
```
cd build
cmake -DCMAKE_BUILD_TYPE=Release -G"Unix Makefiles" ../src/
make -j3
```
You should see some output at the end of running `make -j3` that looks like:

```
...
[100%] Linking CXX executable robogen-server
[100%] Built target robogen-server
```

## Running the robot file viewer / visualiser
Then, to view a robot in the task environment you'll need to use the
`./robogen-file-viewer` command with two parameters:
    1. The location of the file defining what the robot looks like
    2. The location of the file defining some visualisation parameters and
       configurations for the file viewer

So the full command you should run is:
```
./robogen-file-viewer ../examples/starfish.txt ../examples/conf.txt --debug
```
This command will start up a new window showing the robot (from a side-angle,
not from the top) and will exit after a few seconds.

The file `starfish.txt` looks like this:
```
0 CoreComponent Core 0
	0 ActiveHinge Hip1 1 
		0 FixedBrick UpperLeg1 1
			0 ActiveHinge Knee1 0 
				0 FixedBrick LowerLeg1 0 
	1 ActiveHinge Hip2 3 
		0 FixedBrick UpperLeg2 1
			0 ActiveHinge Knee2 0 
				0 FixedBrick LowerLeg2 0 
	2 ActiveHinge Hip3 0 
		0 FixedBrick UpperLeg3 1
			0 ActiveHinge Knee3 0 
				0 FixedBrick LowerLeg3 0 
	3 ActiveHinge Hip4 0 
		0 FixedBrick UpperLeg4 1
			0 ActiveHinge Knee4 0 
				0 FixedBrick LowerLeg4 0
```

And the `conf.txt` looks like this (without the `#` comments)

```
scenario=racing         # racing = robot goes fast as possible
timeStep=0.005          # simulate the robot moving in jumps of 0.005 seconds
simulationTime=8.0      # Run the simulation for 8 seconds
actuationFrequency=25   # "Actuation frequency of the controller in Hz"
terrainType=flat        # flat = the ground is a flat rectangular plane
terrainLength=2         # The ground has a length of 2 meters
terrainWidth=2          # The ground has a width of 2 meters
terrainFriction=1.0     # How much friction there is between robot and ground
sensorNoiseLevel=0.0    # IRL sensors don't measure perfectly, so add some noise if >0
motorNoiseLevel=0.0     # IRL motors don't move perfectly, so add some noise if >0
capAcceleration=false   # If true, stop unrealistic behaviours or exploitation
 #maxDirectionShiftsPerSecond=16 # uncomment to prevent jittery behavior
```

You can find more information about the following things:
- `./robogen-file-viewer -h` will give you a help page with some descriptions
- [robot text file
  guidelines](https://robogen.org/docs/guidelines-for-writing-a-robot-text-file/) will show you how to write a robot definition text file.
- [configuring the
  simulator](http://robogen.org/docs/evolution-configuration/#Simulator_settings) has a big ol' list of parameters that you can change


## Where to find things in the codebase

### Resources
- [protocol buffers](https://developers.google.com/protocol-buffers) are used
  to allow the Evolution Engine and Simulation Engine to communicate
- [emscripten](https://github.com/kripken/emscripten) is something which
  converts CPP code to WebAssembly/JavaScript so that RoboGen can run in the
  user's browser. If something has the `#ifdef EMSCRIPTEN` preprocessor macro,
  then it's used for the WebAssembly/JavaScript version and not the desktop
  version
- [HyperNEAT](https://github.com/peter-ch/MultiNEAT) is a neural network
  evolution algorithm, and RoboGen provides experimental support for it.

### Evolution Engine Files
- `src/Evolver.cpp` : Does the evolving of robots. Is the main executable for
  `./robogen-evolver`.
- `src/config`: Handles configuration given to RoboGen. Makes heavy use of
  [boost program
  options](http://www.boost.org/doc/libs/1_58_0/doc/html/program_options.html).
- `src/evolution/representation` All the code for describing how robots are
  represented in RoboGen
    - `src/evolution/representation/RobotRepresentation` is the main component
      here, and is made up of a
      `src/evolution/representation/PartRepresentation` and a
      `src/evolution/representation/NeuralNetworkRepresentation`
- `src/evolution/enging/Mutator` describes how various operators act on the
  robot representation
- `src/evolution/engine/Population` is a collection which individual robots are
  organised into. 
    - `src/evolution/engine/IndividualContainer` This is extended by
      `Population`
- `src/evolution/neat` is an experimental port of
  [HyperNEAT](https://github.com/peter-ch/MultiNEAT), a neural network
  evolution program, although I don't think this file is ever used directly.
  Instead it's accessed via `src/evolution/engine/neat/NeatContainer`
- `src/evolution/engine/neat/NeatContainer` Contains logic used to interface
  between RoboGen and HyperNEAT. 

### Simulation Engine Files
The simulation engine is built on top of the [Open Dynamics
Engine](http://www.ode.org/), and it's recommended by the maintainers that you
be well-versed in ODE before attempting to modify the RoboGen Simulator.

The simulation engine is usually invoked in one of two ways:
1. Running `build/robogen-server` to perform fitness evaluations for the evolver
2. Running `build/robogen-file-viewer` to "play back" a robot as specified in a
   configuration file.

Important source files and descriptions
- `src/Simulator.cpp`: The file which actually does the simulating. It relies
  on many pieces of code, but is called via `Simulator::runSimulations` in
  these two programs:
    - `src/RobogenServer.cpp`: Used when the evolver wants to perform a fitness
      evaluation of a certain robot, called via `build/robogen-server`
    - `src/viewer/FileFiewer.cpp`: Used by a person to view a robot and watch
      it move in the task environment. Called via `build/robogen-file-viewer`
- `src/model/` contains physical models of robots parts, obstacles, light
  sources, and any other physical items in the task environment.
    - `src/model/components/` contains descriptions of the various components
      (Hinges, Actuated components, Passive components, Bricks, etc)
    - `src/model/sensors/` Contains descriptions of AMU sensors, light sensors,
      touch sensors, sensor groups, etc
    - `src/model/motors/` Describes general motors and servo motors.
    - `src/model/objects/` Describes light sources and obstacles
- `src/render/components/` Has renderer-friendly versions of every model in
  RoboGen. Note that when the `robogen-file-viewer` is run, you do _not_ see
  the physical model of the robot as the evolver sees it. You see a
  representation of how the robot will look when 3D printed, but the evolution
  algorithm sees a bunch of connected primitive blocks (cubes, joints, planes,
  etc) which have much less detail than the 3D printed parts, although should
  function the same


### Other files
- `src/robogen.proto` Defines what the protocol buffers look like, and is
  automatically converted to CPP code by the
  [protobuf](https://developers.google.com/protocol-buffers/)  compiler.
- `src/utils/network/` Contains most of the code for socket-based communication
  between programs.
- `src/utils/json2pb` Is used for converting protobuf messages to/from json
  (since json is used when we want to save an individual robot)
- `src/arduino/` contains files for generating a `NeuralNetwork.h` file for use
  on Arduino.

## When robogen-file-viewer is run
1. `src/viewer/FileViewer.cpp:465` is where `argv[1]` is passed to be parsed
    1. `src/evolution/representation/RobotRepresentation.cpp:1114` is the method
    that does the parsing of json robot config files
    2. `src/evolution/representation/RobotRepresentation.cpp:1133` is the call of
    the json to protobuffer method to convert the json file to a protobuf
    3. `src/utils/json2pb/json2pb.cpp:288` is a wrapper method
    4. `src/utils/json2pb/json2pb.cpp:258` converts json to Proto Buffer
2. `src/viewer/FileViewer.cpp:147` is the line that calls runSimulation with
   one robot
    1. `src/Simulator.cpp:101` is the line that generates the robot swarm
    2. `src/Simulator.cpp:176` Initialises the simulator with one robot instead
       of a swarm

## Useful information
- Communication between the programs is done via sockets
    - Except for `robogen-server-sio`, which uses socket.io


## TODOs related to swarm things
- `src/Simulator.cpp:100`: // TODO: This needs to be edited to generate a swarm of robots
- `src/Simulator.cpp:110`: // TODO This needs to loop through the body pars of the swarm
- `src/Simulator.cpp:129`: // TODO This This needs to check log->init of the swarm
- `src/Simulator.cpp:137`: // TODO this output message should output the evaluation of the swarm
- `src/Simulator.cpp:142`: // TODO this needs to register sensors of the swarm
- `src/Simulator.cpp:156`: // TODO This should register the motors of the swarm
- `src/Simulator.cpp:167`: // TODO this should register the brain and body of every robot in the swarm
- `src/Simulator.cpp:174`: // TODO Scenario initialisation should take in a swarm of robots, not just one
- `src/scenario/Scenario.h:43`: // TODO class swarm should probably also be in here
- `src/scenario/Scenario.h:74`: // TODO this needs to accept a swarm, not just a robot.
- `src/scenario/Scenario.h:88`: // TODO this needs to return the swarm
- `src/scenario/Scenario.h:174`: // TODO this should define the member swarm variable
- `src/scenario/Scenario.cpp:35`: // TODO Need to #include swarm.h
- `src/scenario/Scenario.cpp:59`: // TODO instead of taking in a robot, this should take in a swarm
- `src/scenario/Scenario.cpp:73`: // TODO this should be a member swarm variable, not a member robot variable
- `src/scenario/Scenario.cpp:76`: // TODO this should be done for each member in the swarm
- `src/scenario/Scenario.cpp:85`: // TODO need a way of specifying the position and orientation of each member in the swarm
- `src/scenario/Scenario.cpp:86`: // TODO The swarm will need something like an array of robogenConfigs
- `src/scenario/Scenario.cpp:96`: // TODO apply rotations, min-max, translations to each robot in the swarm
- `src/scenario/Scenario.cpp:106`: // TODO output debug information about every robot in the swarm
- `src/scenario/Scenario.cpp:140`: // TODO need to check the obstacle position against the position of every robot in the swarm
- `src/scenario/Scenario.cpp:201`: // TODO we need to check for light source collisions with every robot in the swarm
- `src/scenario/Scenario.cpp:245`: // TODO need to perform this check with every robot in the swarm
- `src/scenario/Scenario.cpp:255`: // TODO: optimise the physics of every robot in the swarm
- `src/scenario/Scenario.cpp:269`: // TODO we'll need to reset every robot in the swarm
- `src/scenario/Scenario.cpp:273`: // TODO we'll need a Scenario::getSwarm method to get every robot from the swarm
- `src/viewer/FileViewer.cpp:461`: // TODO this needs to be converted to a swarm Message
- `src/viewer/FileViewer.cpp:465`: // TODO: This needs to be updated to return a swarm of robots
- `src/viewer/FileViewer.cpp:511`: // TODO runSimulations needs to be updated to take in a swarm

