import intersectionController
import platoon
import vehicle
import VehicleManualAuto
import collisioncheck
import simlib
import os
import sys
from simulationmanager import SimulationManager
import re
import datetime
import random
import time

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci

class RandomSimulationManager(SimulationManager):
    def __init__(self, percentage, scaleFactor, actualGenDistListOfVeh, crossRedLight = True, pCreation=True, iCoordination=True, iZipping=True):
        self.intersections = []
        self.platoons = list()
        self.platoonCreation = pCreation
        self.vehicles = list()

        self.manualVehicles = list()
        self.EmergencyManual = list()
        self.automatedVehicles = list()
        self.inVehicles = list()
        self.outVehicles = list()

        self.maxStoppedVehicles = dict()
        self.percentage = percentage
        self.actualGenDistListOfVeh = actualGenDistListOfVeh
        self.scaleFactor = scaleFactor

        self.manualCounter = 0
        self.automatedCounter = 0
        self.stepCounter = 0
        self.chunkIndex = 0
        self.collisionCheckWest = list()
        self.collisionCheckNorth = list()
        self.collisionCheckNorthEast = list()
        self.collisionCheckEast = list()
        self.collisionCheckNorthEastManual = list()
        self.collisionCheckNorthWestManual = list()

        self.westRightEVIN = 0
        self.westRightEVOut = 0
        self.eastLeftEVIN = 0
        self.eastLeftEVOut = 0
        self.northDownEVIN = 0
        self.northDownEVOut = 0

        self.isGenerateManual = True
        self.crossRedLight = crossRedLight
        self.totalInLans  = 0
        self.totalOutLans = 0
        self.useManualTraffic = False
        self.hasGreenVertical = False
        self.emergencyInVertical = False
        self.emergencyInHorizontal = False
        if iCoordination:
            for intersection in traci.trafficlight.getIDList():
                #print("intersectioID"+intersection)
                controller = intersectionController.IntersectionController(intersection, iZipping)
                self.intersections.append(controller)

    def getNumManualCars(self,percentageOfManualVehicle, totalCarsForEachChunk,scaleFactor):
        return (round(((percentageOfManualVehicle/100.00) * totalCarsForEachChunk))*scaleFactor)
    
    def getReleventPlatoon(self, vehicle):
        # Returns a single platoon that is most relevent to the given
        # vehicle by looking to see if the car in front is part of a platoon
        # It also checks that the platoon is heading in the right direction
        # Platoon size is one and therfore only one Platoon leader itself is in a platoon for his experiment
        leadVeh = vehicle.getLeader()
        if leadVeh and leadVeh[1] < 10:
            possiblePlatoon = self.getPlatoonByVehicle(leadVeh[0])
            if possiblePlatoon:
                if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in possiblePlatoon[0].getAllVehicles() and possiblePlatoon[0].getNumberOfVehicles() < 1 and possiblePlatoon[0].getLanePositionFromFront() > 60:
                    return possiblePlatoon[0]
    
    # calcuate how many cars in each lane by subtracting car that has passed the out induction Loop from in induction loop
    def getNumCars(self,inLoop,outLoop):
         #print("Debug!!!!!!:inside function getNumCars")
         remainingCars = inLoop - outLoop
         return remainingCars 
    
    def set_Duration(self,phaseId,numVehicles):
        #set duration number of vehicle * 5 seconds but no more than 60 seconds
        #print("Debug!!!!!!:inside function set_Duration")
        traci.trafficlight.setPhase("intersection", phaseId)
        if (numVehicles * 5) < 60:
            traci.trafficlight.setPhaseDuration("intersection",30)
            #print("Debug!!!!!!:phase" + str(phaseId) +"duration"+str(traci.trafficlight.getPhaseDuration("intersection")))
        else: 
            traci.trafficlight.setPhaseDuration("intersection",30)
            #print("Debug!!!!!!:phase" + str(phaseId) +"duration"+str(traci.trafficlight.getPhaseDuration("intersection")))
    
    #Based on principal of well known related rate problem and pythagoras theorm
    # we would monitor 4:5 ratio range of west right and longSide down lane
    # if any vehicles from respective lanes in that range within similiar time is detected
    # then we would speed up vehicle from
    # west right lane, which is closer to collision point, a little bit and slow down vehicle from  
    # longSide down lane
    #TODO: make this function reuseable for all checking that satisfy properties for related rate problem and pythagoras theorm
    def checkPossibleCollision(self,platoon,laneIdVertical,laneIdHorizontalWest,laneIdHorizontalEast,verRan1,verRan2,verRanEast1,verRanEast2,horRan1,horRan2,horRanEast1,horRanEast2):
        name = platoon.getLeadVehicle().getName()
        position = platoon.getLeadVehicle().getLanePosition()
        speed = platoon.getLeadVehicle().getSpeed()
        detectedTime = platoon.getLeadVehicle().getLastActionTime()
        laneId = platoon.getLeadVehicle().getLane()
        #print("lane id"+laneId+laneIdVertical+laneIdHorizontalWest)
        #print("veh name"+name)
        #print("position"+str(position))
        #if speed is less 7m/s vehicle might be slowing down for traffic etc...
        if speed > 7 :
            if  laneId ==laneIdVertical and (round(position) in range(verRan1,verRan2)):
                #print("lane Id"+laneId)
                collisionNorth = collisioncheck.CollisionCheck(name, speed,detectedTime)
                self.collisionCheckNorth.append(collisionNorth)
            if  laneId == laneIdHorizontalWest and (round(position) in range(horRan1,horRan2)):
                #print("lane Id"+laneId)
                collisionWest = collisioncheck.CollisionCheck(name, speed,detectedTime)
                self.collisionCheckWest.append(collisionWest)
            if  laneId == laneIdHorizontalEast and (round(position) in range(horRanEast1,horRanEast2)):
                #print("lane Id"+laneId)
                collisionEast = collisioncheck.CollisionCheck(name, speed,detectedTime)
                self.collisionCheckEast.append(collisionEast)
            if  laneId ==laneIdVertical and (round(position) in range(verRanEast1,verRanEast2)):
                #print("lane Id"+laneId)
                collisionNorth = collisioncheck.CollisionCheck(name, speed,detectedTime)
                self.collisionCheckNorthEast.append(collisionNorth)
            
        #Evaluate if there are two cars info
        # if vehicle from north speed is slighter higher than west or same speed then
        # set north speed to -1 and west to +1
        for north in self.collisionCheckNorth:
            northTime = int(north.getDetectedTime())
            if northTime  < (int(traci.simulation.getTime())-2):
                self.collisionCheckNorth.remove(north)
            for west in self.collisionCheckWest:
                westTime = int(west.getDetectedTime())
                northSpeed = int(north.getVehSpeed())
                westSpeed = int(west.getVehSpeed())
                if  northTime in range(westTime-1,westTime+2) and northSpeed in range (westSpeed,westSpeed+2):
                    #inform platoon to maintain given speed till intersection has crossed
                    if  laneId ==laneIdVertical and name == north.getVehName():
                        if (platoon.getCollisionDetected()==False):
                            print("Possible collision detected from platoon"+name)
                            platoon.setAllocatedSpeed(speed-2.5)
                            platoon._updateSpeed(speed-2.5)
                            print("Speed"+str(speed) + "updated speed"+ str(speed-2.5))
                            #print("TIme"+str(traci.simulation.getTime()))
                            platoon.setCollisionDetected(True)
                    elif laneId == laneIdHorizontalWest and name == west.getVehName():  
                        if (platoon.getCollisionDetected()==False):
                            print("Possible collision detected from platoon"+name)
                            platoon.setAllocatedSpeed(speed+2)
                            platoon._updateSpeed(speed+2)
                            print("Speed"+str(speed) + "updated speed"+ str(speed+2))
                            #print("TIme"+str(traci.simulation.getTime()))
                            platoon.setCollisionDetected(True)

        for west in self.collisionCheckWest:
            westTime = int(west.getDetectedTime())
            # For collision detection with manual vehicle we can only set speed to automated vehicle and not to 
            # manual vehicle with driver
            for northManual in self.collisionCheckNorthWestManual:
                northTime = int(northManual.getDetectedTime())
                if  westTime in range(northTime,northTime+2):
                        #inform platoon to maintain given speed till intersection has crossed
                    print("NorthTImeWestTime"+str(northTime)+str(westTime))
                    if laneId == laneIdHorizontalWest and name == west.getVehName():  
                        if (platoon.getCollisionDetected()==False):
                            print("Possible collision detected from platoon"+ name+ "with manual car"+northManual.getVehName())
                            platoon.setAllocatedSpeed(speed+3)
                            platoon._updateSpeed(speed+3)
                            print("Speed"+str(speed) + "updated speed"+ str(speed-2))
                            platoon.setCollisionDetected(True)
                    if northTime  < (int(traci.simulation.getTime())-10):
                        self.collisionCheckNorthWestManual.remove(northManual)
            if westTime  < (int(traci.simulation.getTime())-2):
                self.collisionCheckWest.remove(west)
                
        
        for east in self.collisionCheckEast:
            eastTime = int(east.getDetectedTime())
            #print("East TIme"+ str(eastTime))
            #print("East Id"+east.getVehName())
            #print("East Speed"+str(east.getVehSpeed()))
            if eastTime  < (int(traci.simulation.getTime())-2):
                self.collisionCheckEast.remove(east)
            for north in self.collisionCheckNorthEast:
                #print("East TIme"+ str(eastTime))
                #print("East Id"+east.getVehName())
                #print("East Speed"+str(east.getVehSpeed()))
                northTime = int(north.getDetectedTime())
                eastSpeed = int(east.getVehSpeed())
                northSpeed = int(north.getVehSpeed())
                #print("Nor TIme"+ str(northTime))
                #print("Nor Id"+north.getVehName())
                #print("Nor Speed"+str(northSpeed))
                if  eastTime in range(northTime,northTime+2) and eastSpeed in range (northSpeed,northSpeed+2):
                    #inform platoon to maintain given speed till intersection has crossed
                    if  laneId ==laneIdVertical and name == north.getVehName():
                        if (platoon.getCollisionDetected()==False):
                            print("Possible collision detected from platoon"+name)
                            platoon.setAllocatedSpeed(speed+2)
                            platoon._updateSpeed(speed+2)
                            print("Speed"+str(speed) + "updated speed"+ str(speed+2))
                            #print("TIme"+str(traci.simulation.getTime()))
                            platoon.setCollisionDetected(True)
                    elif laneId == laneIdHorizontalEast and name == east.getVehName():  
                        if (platoon.getCollisionDetected()==False):
                            print("Possible collision detected from platoon"+name)
                            platoon.setAllocatedSpeed(speed-2)
                            platoon._updateSpeed(speed-2)
                            print("Speed"+str(speed) + "updated speed"+ str(speed-2))
                            platoon.setCollisionDetected(True)
                if northTime  < (int(traci.simulation.getTime())-2):
                    self.collisionCheckNorthEast.remove(north)
            # For collision detection with manual vehicle we can only set speed to automated vehicle and not to 
            # manual vehicle with driver
            for northManual in self.collisionCheckNorthEastManual:
                northTime = int(northManual.getDetectedTime())
                eastSpeed = int(east.getVehSpeed())
                northSpeed = int(northManual.getVehSpeed())
                if  eastTime in range(northTime,northTime+2):
                    #inform platoon to maintain given speed till intersection has crossed
                    if laneId == laneIdHorizontalEast and name == east.getVehName():  
                        if (platoon.getCollisionDetected()==False):
                            print("Possible collision detected from platoon"+ name+ "with manual car"+northManual.getVehName())
                            platoon.setAllocatedSpeed(speed-2)
                            platoon._updateSpeed(speed-2)
                            print("Speed"+str(speed) + "updated speed"+ str(speed-2))
                            platoon.setCollisionDetected(True)
                if northTime  < (int(traci.simulation.getTime())-2):
                    self.collisionCheckNorthEastManual.remove(northManual)
                

    """ def northWestCheck(self,collisionCheckLong,collisionCheckShort,platoon):
         #Evaluate if there are two cars info
        # if vehicle from longSide speed is slighter higher than shortSide or same speed then
        # set longSide speed to -1 and west to +1
        for longSide in collisionCheckLong:
            for shortSide in collisionCheckShort:
                longTime = int(longSide.getDetectedTime())
                shortTime = int(shortSide.getDetectedTime())
                longSpeed = int(longSide.getVehSpeed())
                shortSpeed = int(shortSide.getVehSpeed())
                speed = platoon.getLeadVehicle().getSpeed()
                name = platoon.getLeadVehicle().getName()
                if  longTime in range(shortTime,shortTime+1) and longSpeed in range (shortSpeed,shortSpeed+2):
                    print("Possible collision detected from platoon"+name)
                    print("Vehicle from longSide lane"+longSide.getVehName())
                    print("Vehicle from shortSide lane"+shortSide.getVehName())
                    #inform platoon to maintain given speed till intersection has crossed
                    if  laneId ==laneIdVertical and name == longSide.getVehName():
                        if (platoon.getCollisionDetected()==False):
                            platoon.setAllocatedSpeed(speed-1)
                            platoon._updateSpeed(speed-1)
                            print("Speed"+str(speed) + "updated speed"+ str(speed-1))
                            platoon.setCollisionDetected(True)
                    elif laneId == laneIdHorizontalWest and name == shortSide.getVehName():  
                        if (platoon.getCollisionDetected()==False):
                            platoon.setAllocatedSpeed(speed+1)
                            platoon._updateSpeed(speed+1)
                            print("Speed"+str(speed) + "updated speed"+ str(speed+1))
                            platoon.setCollisionDetected(True) """

    def detectManualVehPossibleCollision(self,loopId,phaseId):
        vehicleSpeed = traci.inductionloop.getLastStepMeanSpeed(loopId)
        vehicleId = traci.inductionloop.getLastStepVehicleIDs(loopId)
        detectionTime = traci.simulation.getTime()
        for id in vehicleId:
            if  id in  self.manualVehicles or id in self.EmergencyManual and str(traci.trafficlight.getPhase("intersection")) == phaseId:
                print("Possible Collision Manual"+ str(vehicleSpeed)+str(vehicleId)+str(detectionTime))
                collisionManual = collisioncheck.CollisionCheck(str(vehicleId), vehicleSpeed,detectionTime)
                self.collisionCheckNorthEastManual.append(collisionManual) 
                self.collisionCheckNorthWestManual.append(collisionManual) 
        

    def handleSimulationStep(self):
        #using induction loop detect if there is any manual vehicle and if so switch to normal traffic light 
        #otherwise continue with VTLPlusEv algorithm unless cross red light is true then automated vehicles will ignore 
        #normal traffic rule 
        # TODO: induction loop need to pass vehicle type (not existing one but our own type) whether it is manual or automated but need to add this feature to sumo lib
        # TODO: following code is workaround but still demonstrate the concept of manual vehicles communicating 
        # with loop or RSU as vehicle id are detected from loop
        i = 0
        while i < 8:
             vehicleId = traci.inductionloop.getLastStepVehicleIDs(str(i))
             for id in vehicleId:
                 if  id in  self.manualVehicles and id not in self.inVehicles:
                     self.inVehicles.append(id)
                     self.totalInLans += 1   
                 if "Emergency" in id and i == 0:
                     self.westRightEVIN +=1
                 if "Emergency" in id and i == 3:
                     self.eastLeftEVIN +=1
                 if "Emergency" in id and i == 7:
                     self.northDownEVIN +=1

             i += 1
        
        while i < 16:
             vehicleId = traci.inductionloop.getLastStepVehicleIDs(str(i))
             for id in vehicleId:
                 if  id in  self.manualVehicles and id not in self.outVehicles:
                     self.outVehicles.append(id)
                     self.totalOutLans += 1
                 if "Emergency" in id and i == 8:
                     self.westRightEVOut +=1
                 if "Emergency" in id and i == 11:
                     self.eastLeftEVOut +=1
                 if "Emergency" in id and i == 13:
                     self.northDownEVOut +=1     
             i += 1


        
        # Check collision for manual vehicles
        # For north east TODO: Do for other directions
        self.detectManualVehPossibleCollision("19","0")
        if (self.getNumCars(self.westRightEVIN,self.westRightEVOut) > 0) or (self.getNumCars(self.eastLeftEVIN,self.eastLeftEVOut) > 0):
            #horizontal lanes traffic light need to be green as Evs are there
            #print("West Right"+str(self.westRightEVIN)+""+str(self.westRightEVOut))
            #print("East Left"+str(self.eastLeftEVIN)+""+str(self.eastLeftEVOut))
            self.emergencyInHorizontal = True
            #print("Horizontal emer true")

        if (self.getNumCars(self.northDownEVIN,self.northDownEVOut) > 0):
            #Vertical lanes traffic light need to be green as Evs are there
            #print("north dowin out"+str(self.northDownEVIN)+""+str(self.northDownEVOut))
            self.emergencyInVertical = True
            #print("Vertical emer true")
       


        if(self.getNumCars(self.totalInLans,self.totalOutLans) > 0 or  self.emergencyInVertical or self.emergencyInHorizontal):
            self.useManualTraffic = True
            if self.emergencyInVertical:
                self.set_Duration(0,5)
                self.emergencyInVertical = False
                self.hasGreenVertical = True
                #print("Vertical emer set green")
            elif self.emergencyInHorizontal:
                self.set_Duration(4,5)
                self.emergencyInHorizontal = False
                self.hasGreenVertical = False
                #print("Horizontal emer set green")
            if traci.trafficlight.getPhase("intersection") == 8:
                 if self.hasGreenVertical == False:
                    self.set_Duration(0,5)
                    self.hasGreenVertical = True
                 else:
                    self.set_Duration(4,5)
                    self.hasGreenVertical = False
            else:
                if traci.trafficlight.getPhaseDuration("intersection") == 0:
                    if self.hasGreenVertical == False:
                     self.set_Duration(0,5)
                     self.hasGreenVertical = True
                    else:
                     self.set_Duration(4,5)
                     self.hasGreenVertical = False                                   
        else:
            self.set_Duration(8,5)
            self.useManualTraffic = False


        allVehicles = traci.vehicle.getIDList()
        # Check mark vehicles as in-active if they are outside the map
        stoppedCount = dict()
        for v in self.vehicles:
            if v.getName() not in allVehicles:
                v.setInActive()
            # Get information concerning the number of vehicles queueing on each lane
            if v.isActive() and v.getSpeed() == 0:
                lane = v.getEdge()
                if lane in stoppedCount:
                    stoppedCount[lane] = stoppedCount[lane] + 1
                else:
                    stoppedCount[lane] = 1

        # Gather statistics for amount of vehicles stopped per lane
        for lane in stoppedCount:
            if lane in self.maxStoppedVehicles:
                if stoppedCount[lane] > self.maxStoppedVehicles[lane]:
                    self.maxStoppedVehicles[lane] = stoppedCount[lane]
            else:
                self.maxStoppedVehicles[lane] = stoppedCount[lane]

        # Update all platoons active status
        for p in self.getActivePlatoons():
            p.updateIsActive()

        if self.platoonCreation:
            # See whether there are any vehicles that are not
            # in a platoon that should be in one and also that are not in manual vehicles list
            #set speed mode to 7 so that automated vehicles will cross red lights at intersections
            if (self.crossRedLight):
                for v in allVehicles:
                    if v not in self.manualVehicles:
                        traci.vehicle.setSpeedMode(v,7)
            vehiclesNotInPlatoons = [v for v in allVehicles if v not in self.getAllVehiclesInPlatoons() and v not in self.manualVehicles]
            numberOfManualCarsToGenerate = self.getNumManualCars(self.percentage,self.actualGenDistListOfVeh[self.chunkIndex if self.chunkIndex < 9 else 0],self.scaleFactor)
            #print("num manual vehicle to generate"+str(numberOfManualCarsToGenerate) +"for chunk"+str(self.chunkIndex))
            numberOfAutomatedCarsToGenerate = self.actualGenDistListOfVeh[self.chunkIndex if self.chunkIndex < 9 else 0] - numberOfManualCarsToGenerate
            #print("num automated vehicle to generate"+str(numberOfAutomatedCarsToGenerate) +"for chunk"+str(self.chunkIndex))
            for vehicleID in vehiclesNotInPlatoons:
                vehiclel = VehicleManualAuto.VehicleManualAuto(vehicleID)
                #print("Vehicle length:"+ str(vehiclel.getLength()))
                self.vehicles.append(vehiclel)
                # If we're not in a starting segment (speed starts as 0)
                # here lets put condition for some vehicle to be manual and not to be part of platoon and put color blue
                # set speed mode to 31 so that blue (manual vehicles) will follow traffic rule. (Stop at red light)
                # Vehicles are distributed like this -> 4% 8%   12%   16%   20%   16%  12%  8%   4% from 0 to 3600 in 9 chunks whick have 400 each
                if self.manualCounter < numberOfManualCarsToGenerate and self.isGenerateManual and vehicleID not in self.automatedVehicles and "Emergency" not in vehicleID:
                    #vehicle not joining to platoons, set color to blue 
                    #create properties in vehicle class saying this is manual vehicle and set speed to between 2 and 12 m/s
                    vehiclel.setColor((0, 0, 255))
                    vehiclel.setType("manual")
                    vehiclel.setSpeedMode(31)
                    vehiclel.setSpeed(random.randint(11,14))
                    self.manualVehicles.append(vehiclel.getName())
                    self.manualCounter = self.manualCounter + 1
                    if self.automatedCounter < numberOfAutomatedCarsToGenerate:
                        self.isGenerateManual = False
                elif self.automatedCounter < numberOfAutomatedCarsToGenerate and "Emergency" not in vehicleID:
                    self.automatedVehicles.append(vehiclel.getName())
                    self.automatedCounter = self.automatedCounter +1
                    self.isGenerateManual = True
                    possiblePlatoon = self.getReleventPlatoon(vehiclel)
                    if possiblePlatoon:
                        possiblePlatoon.addVehicle(vehiclel)
                    else:
                        self.createPlatoon([vehiclel, ])
                elif "Emergency" in vehicleID:
                    #print("Emergency Vehicle")
                    self.EmergencyManual.append(vehiclel.getName())
           
        # If we're doing intersection management, update each controller and add any new platoons into their
        # control
        if self.intersections and self.useManualTraffic == False :
            for inControl in self.intersections:
                inControl.removeIrreleventPlatoons()
                inControl.findAndAddReleventPlatoons(self.getActivePlatoons())
                inControl.update()

        if self.platoonCreation:
            # Handles a single step of the simulation
            # Update all active platoons in the scenario
            # Platoon size is one and therfore only one Platoon leader itself is in a platoon for his experiment
            for platoon in self.getActivePlatoons():
                platoon.update()
                # when assigning the ragne please think of 3:4 ratio of pythagoras theorm and speed factor
                # from related rate problem
                # In this scenario 71 mean distance to collision point is 24 m away and 69 means 32 m away.
                self.checkPossibleCollision(platoon,"north_down_0","west_right_0","east_left_0",69,71,71,73,71,73,69,71)
                if platoon.canMerge() and platoon.isActive() and platoon.getNumberOfVehicles() < 1:
                    lead = platoon.getLeadVehicle().getLeader()
                    if lead:
                        leadPlatoon = self.getPlatoonByVehicle(lead[0]) 
                        if leadPlatoon:
                            leadPlatoon[0].mergePlatoon(platoon)
        
        self.stepCounter = self.stepCounter + 1
        if self.stepCounter == 4000:
                self.stepCounter = 0
                self.manualCounter = 0
                self.automatedCounter = 0
                self.chunkIndex = self.chunkIndex + 1


        
