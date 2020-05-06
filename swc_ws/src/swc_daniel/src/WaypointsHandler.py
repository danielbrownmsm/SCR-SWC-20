from vincenty import vincenty

class WaypointsHandler:
    def __init__(self, waypoint_data):
        self.waypoint_data = waypoint_data
        self.start_point = self.waypoint_data[0]
        self.goal_points = self.waypoint_data[1:]
        self.converted_goal_points = self.convertPoints(self.goal_points)
        self.index = 0
    
    def convertPoints(self, data):
        converted_points = []
        loop_var = 0
        
        for point in data:
            # x1, y1
            #   |\
            #     \
            #   |  \
            #       \
            #   |    \
            #    _ _ _\
            # x1, y2|x2, y2 
            # latitude = y; longitude = x
            # order is (x, y) and (lat, lon) _normally_
            # (x1, y2, x2, y2) (except no because lat, lon)
            # so (y2, x1, y2, x2)
            x_dist = vincenty((self.start_point.latitude, data[loop_var].longitude), (self.start_point.latitude, self.start_point.longitude))

            # (x1, y1, x1, y2) except no so (y1, x1, y2, x2)
            y_dist = vincenty((data[loop_var].latitude, data[loop_var].longitude), (self.start_point.latitude, data[loop_var].longitude))
            
            point = (x_dist * 1000.0, y_dist * 1000.0) # it returns kilometers, and we want meters
            print("Converted: " + str(point[0]) + ", " + str(point[1]))
            converted_points.append(point)

            loop_var += 1
        return converted_points
    
    def getWaypointsXY(self):
        return self.converted_goal_points

    def getStartPoint(self):
        return self.start_point