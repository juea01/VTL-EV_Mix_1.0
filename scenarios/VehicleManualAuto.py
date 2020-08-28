import traci
from vehicle import Vehicle

class VehicleManualAuto(Vehicle):
    
    def __init__(self, vehicle):
        Vehicle.__init__(self,vehicle)
        self._type = "automatic"

    def getType(self):
        return self._type

    def setType(self,type):
        self._type = type

    def getCO2Emission(self):
        return traci.vehicle.getCO2Emission(self.getName())

    def getLastActionTime(self):
        return traci.vehicle.getLastActionTime(self.getName())
