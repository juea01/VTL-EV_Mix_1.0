import traci

class CollisionCheck():
    
    def __init__(self,vehName,vehSpeed,detectedTime):
        self._vehName = vehName
        self._vehSpeed = vehSpeed
        self._detectedTime = detectedTime

    def getVehName(self):
        return self._vehName

    def getVehSpeed(self):
        return self._vehSpeed
    
    def getDetectedTime(self):
        return self._detectedTime
