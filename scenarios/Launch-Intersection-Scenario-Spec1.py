from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
import logging
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
import simulationmanager
import RandomSimulationManager
import simlib


def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    # demand per second from different directions
    pWE = 1. / 15
    pEW = 1. / 15
    pNS = 1. / 20
    with open("../maps/NormalIntersection_no_TLS/randomSpec1.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="2" length="4.00" minGap="2.50" maxSpeed="14.00" guiShape="passenger" accel="2" decel="4.5" sigma="0.5" />
        <vType id="1" length="3.00" minGap="2.50" maxSpeed="14.00" guiShape="passenger" accel="2" decel="4.5" sigma="0.5" />
        <vType id="0" length="5.00" minGap="2.50" maxSpeed="14.00" guiShape="passenger" accel="2" decel="4.5" sigma="0.5" />
        """, file=routes)
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="right_%i" type="2" depart="%i" > <route edges="west_right east_right"/> </vehicle>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pEW:
                print('    <vehicle id="left_%i" type="1" depart="%i"> <route edges="east_left west_left"/>  </vehicle>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pNS:
                print('    <vehicle id="down_%i" type="0" depart="%i">  <route edges="north_down south_down"/> </vehicle>' % (
                    vehNr, i), file=routes)
                vehNr += 1
        print("</routes>", file=routes)

generate_routefile()
# if you change scale factor here please also chage to scale factor value down there
simlib.setUpSimulation("../maps/NormalIntersection_no_TLS/crossSpec1.sumocfg",3) 
#simlib.setUpSimulation("../maps/traci_tls_adaptive_Jue_23042020/data/cross.sumocfg",3)
#setUpSimulation("../maps/NormalIntersection/cross.sumocfg",3)
step = 0
#manager = SimulationManager(True, True, False)
#manager = simulationmanager.SimulationManager(True, True, False)
manager = RandomSimulationManager.RandomSimulationManager(1,3,600,True,True, True,False)
traci.trafficlight.setPhase("intersection", 8)
while step < 500000:
    manager.handleSimulationStep()
    traci.simulationStep()
    
    step += 1

logging.info("Max number of stopped cars: %s", manager.maxStoppedVehicles)
logging.info("Average length of platoon: %s", manager.getAverageLengthOfAllPlatoons())
traci.close()
