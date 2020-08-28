import os
import sys

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
import simlib




simlib.setUpSimulation("../maps/NormalIntersection/NormalIntersection.sumocfg",3)
step = 0
while step < 5000:
    traci.simulationStep()
    step += 1

traci.close()
