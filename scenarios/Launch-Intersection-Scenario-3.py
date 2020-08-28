import os
import sys

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
import logging
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
import simulationmanager
import RandomSimulationManager
import simlib

# if you change scale factor here please also chage to scale factor value down there
simlib.setUpSimulation("../maps/NormalIntersection_no_TLS/cross2.sumocfg",3) 
#simlib.setUpSimulation("../maps/traci_tls_adaptive_Jue_23042020/data/cross.sumocfg",3)
#setUpSimulation("../maps/NormalIntersection/cross.sumocfg",3)
step = 0
#manager = SimulationManager(True, True, False)
#manager = simulationmanager.SimulationManager(True, True, False)
manager = RandomSimulationManager.RandomSimulationManager(0,3,True,True, True,False)
traci.trafficlight.setPhase("intersection", 8)
while step < 5000:
    manager.handleSimulationStep()
    traci.simulationStep()
    
    step += 1

logging.info("Max number of stopped cars: %s", manager.maxStoppedVehicles)
logging.info("Average length of platoon: %s", manager.getAverageLengthOfAllPlatoons())
traci.close()
