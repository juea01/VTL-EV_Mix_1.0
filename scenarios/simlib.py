import logging
import os
import sys

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from sumolib import checkBinary

def flatten(l):
    # A basic function to flatten a list
    return [item for sublist in l for item in sublist]

def setUpSimulation(configFile, trafficScale = 1):
    # Check SUMO has been set up properly
    sumoBinary = checkBinary("sumo-gui")

    # Set up logger
    logging.basicConfig(format='%(asctime)s %(message)s')
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    # Start Simulation and step through
    traci.start([sumoBinary, "-c", configFile, "--step-length", "0.1", "--collision.action", "true","--collision.mingap-factor", "0",  "--start",
                 "--additional-files", "../output/additional.xml, ../maps/NormalIntersection_no_TLS/cross.det.xml", "--emission-output","../output/emission.xml","--duration-log.statistics", "--scale", str(trafficScale)])