import os
import sys
import logging

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
import simulationmanager
import RandomSimulationManager
import simlib

simlib.setUpSimulation("../maps/NormalIntersection/cross.sumocfg",5)
step = 0
manager = RandomSimulationManager.RandomSimulationManager(30,5,True, False)
maxNumAtTrafficLights = 0
while step < 8000:
    manager.handleSimulationStep()
    traci.simulationStep()
    step += 1

logging.info("Max number of stopped cars: %s", manager.maxStoppedVehicles)
logging.info("Average length of platoon: %s", manager.getAverageLengthOfAllPlatoons())
traci.close()
