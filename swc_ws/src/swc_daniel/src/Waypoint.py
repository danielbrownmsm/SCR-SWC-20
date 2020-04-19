from geopy import distance

class Waypoint:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude
    
    def getLat(self):
        return self.latitude
    
    def getLon(self):
        return self.longitude

    def getLatLon(self):
        return (self.latitude, self.longitude)
    
    def getDistance(self, waypoint1, waypoint2):
        return distance(waypoint1.getLatLon(), waypoint2.getLatLon())