# Zebro-Search-and-Rescue
Code to let a swarm of Zebro robots effectively work together to search an area, locate a target and then form a path from the start to the target using their own bodies.

To run the simulation, you need to have Argos installed: https://www.argos-sim.info
Then, enter the Simulation folder and run the following commands:

mkdir build

cd build

cmake ..

make

cd ..

argos3 -c searchandrescue.argos
