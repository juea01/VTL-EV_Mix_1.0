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

actualGenDistListOfVeh = list()
numberOfTimeSteps = 3600  # number of time steps

def generate_routefile(percentageEmergencyVehicle,numCarsToGenerate,numberOfTimeSteps):
    random.seed(42)  # make tests reproducible
   
    with open("../maps/NormalIntersection_no_TLS/randomSpec1_600_Emergency.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="2" length="4.00" minGap="2.50" maxSpeed="14.00" guiShape="passenger" accel="2" decel="4.5" sigma="0.5" />
        <vType id="1" length="3.00" minGap="2.50" maxSpeed="14.00" guiShape="passenger" accel="2" decel="4.5" sigma="0.5" />
        <vType id="0" length="5.00" minGap="2.50" maxSpeed="14.00" guiShape="passenger" accel="2" decel="4.5" sigma="0.5" />
        <vType id="Emergency" length="7.00" minGap="2.50" maxSpeed="14.00" guiShape="emergency" accel="2" decel="4.5" sigma="0.5"/>
        """, file=routes)
        # here we would like to spread across data so that we would get bell curve distribution
        # we would spread across cars 0 to 3600 by 400 chunks and hence we have 9 chunks in total.
        # we will gradually increase number of cars per chunk till we get the chunk that have medium and in our case 
        # median is 1800 in our case  and then gradually decrease.
        # distributed in following orders:
        # 4% 8%   12%   16%   20%   16%  12%  8%   4%
        # Example would be if we are going to distribute 300 cars per hour (3600 seconds) we would slowly increase till we reach 20% chunk
        # which have 60 cars and slowly decrease distribution amount to each of
        # remaining chunks
        # In addition we would like to spread across cars in each chunk (400 steps/sec in each chunk). For example if we
        # are going to distribute 7 cars to particular chunk there would be car for every 57 seconds
        # Also we are exprementing in three leg intersections (east to west, west to east, and north to south) in 2, 2, 1 ratio
 

        listOfPercentage = [0.04,0.08,0.12,0.16,0.20,0.16,0.12,0.08,0.04]
        listOfDistributedCars = list()
        #average second to generate a car for the chunk (maximum one car per second) 
        averageSecondsEachChunk = list()
        for x in listOfPercentage:
            listOfDistributedCars.append(round(x*numCarsToGenerate))
        #print("List of distributed cars"+str(listOfDistributedCars))

        for x in listOfDistributedCars:
            averageSecondsEachChunk.append(round(400/x))
        #print("Average second"+str(averageSecondsEachChunk))


        #for emergency vehicle
        # emergecny vehicle are distributed mainly in chunk 2,3,4,5 (total 4 chunk) where chunk 4 have highest percentage (20%) or busiest time
        # chunk index start by 0
        totalEmergencyVehicles = percentageEmergencyVehicle * numCarsToGenerate
        print("Total EV to generate"+str(totalEmergencyVehicles))
        numCarToDistributeForEachChunk = round((totalEmergencyVehicles/4))
        print("Num Car To Distribute"+str(numCarToDistributeForEachChunk))

        pWE = 0
        pEW = 0
        pNS = 0
        vehNr = 0
        counter = 0
        subCounter = 1
        countVehInEachChunk = 0
        index = 0

        numEmergencyGenerated = 0
        emergencyEastWest = True
        emergencyNorthSouth = False
        for i in range(numberOfTimeSteps):
            if subCounter == averageSecondsEachChunk[index]:
                #print("index"+str(index)+"numVech"+str(vehNr))
                subCounter = 0
                # 40 % for pWE, 40 % for pEW and 20% for pNS
                if pWE < (round(listOfDistributedCars[index]*0.4)):
                    print('    <vehicle id="right_%i" type="2" depart="%i" > <route edges="west_right east_right"/> </vehicle>' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                    pWE = pWE +1
                    countVehInEachChunk = countVehInEachChunk + 1
                if pEW < (round(listOfDistributedCars[index]*0.4)):
                    print('    <vehicle id="left_%i" type="1" depart="%i"> <route edges="east_left west_left"/>  </vehicle>' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                    pEW = pWE +1
                    countVehInEachChunk = countVehInEachChunk + 1
                if pNS < (round(listOfDistributedCars[index]*0.2)):
                    print('    <vehicle id="down_%i" type="0" depart="%i">  <route edges="north_down south_down"/> </vehicle>' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                    pNS = pWE +1
                    countVehInEachChunk = countVehInEachChunk + 1
                #For emergency vehicle 
                if (index >=2 and index <=5 and numEmergencyGenerated < numCarToDistributeForEachChunk):
                    if emergencyEastWest:
                        print('    <vehicle id="leftEmergency_%i" type="Emergency" depart="%i"> <route edges="east_left west_left"/>  </vehicle>' % (
                            vehNr, i), file=routes)
                        vehNr += 1
                        numEmergencyGenerated += 1
                        #if 1% of EV generate from just one direction and if 2% or more then from two direction
                        if percentageEmergencyVehicle >= 0.02:
                            emergencyEastWest = False
                            emergencyNorthSouth = True
                    elif emergencyNorthSouth:
                        print('    <vehicle id="downEmergency_%i" type="Emergency" depart="%i"> <route edges="north_down south_down"/>  </vehicle>' % (
                            vehNr, i), file=routes)
                        vehNr += 1
                        emergencyNorthSouth = False
                        emergencyEastWest = True
                        numEmergencyGenerated += 1
                    # else:
                    #     print('    <vehicle id="rightEmergency_%i" type="Emergency" depart="%i"> <route edges="west_right east_right"/>  </vehicle>' % (
                    #         vehNr, i), file=routes)
                    #     vehNr += 1
                    #     emergencyEastWest = True
                    #     numEmergencyGenerated += 1


            counter = counter + 1
            subCounter = subCounter + 1

            

            if counter == 400:
                counter = 0
                #print("Number of cars generate for chunk"+ str(index)+"num cars"+ str(countVehInEachChunk))
                actualGenDistListOfVeh.append(countVehInEachChunk)
                countVehInEachChunk = 0
                index = index +1
                pWE = 0
                pEW = 0
                pNS = 0
                subCounter = 1
                numEmergencyGenerated =0
            
        print("</routes>", file=routes)

generate_routefile(0.01,600,numberOfTimeSteps) #include 1 % of EV
#generate_routefile(0.02,600,numberOfTimeSteps) #include 2 % of EV
# if you change scale factor here please also chage to scale factor value down there
simlib.setUpSimulation("../maps/NormalIntersection_no_TLS/crossSpec1_600_Emergency.sumocfg",1) 
step = 0
#manager = RandomSimulationManager.RandomSimulationManager(50,1,actualGenDistListOfVeh,True,True, True,False)  # 50% manual
manager = RandomSimulationManager.RandomSimulationManager(0,1,actualGenDistListOfVeh,True,True, True,False)  # 0% manual
traci.trafficlight.setPhase("intersection", 8)
while step < 500000:    
    manager.handleSimulationStep()
    traci.simulationStep()
    step += 1

logging.info("Max number of stopped cars: %s", manager.maxStoppedVehicles)
logging.info("Average length of platoon: %s", manager.getAverageLengthOfAllPlatoons())
traci.close()



