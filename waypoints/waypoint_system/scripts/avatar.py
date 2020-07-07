#!/usr/bin/env python

import rospy
import json
from json import JSONDecoder
from web_service_jeremy_v1.srv import NamePose
from web_service_jeremy_v1.srv import Strings
from web_service_jeremy_v1.srv import WaypointsList
from web_service_jeremy_v1.srv import Waypoint
from web_service_jeremy_v1.msg import Locations,ID
from std_srvs.srv import SetBool


class Avatar():
    def __init__(self):
        self.databaseDirectory = "/home/lee/catkin_ws/src/waypoints/waypoint_system/database/location.json"
        self.addWaypointService = rospy.Service("/web_service/add_location", NamePose, self.callback_addWaypoint)
        self.deleteWaypointService = rospy.Service("/web_service/delete_location", Strings, self.callback_deleteWaypoint)
        self.deleteAllWaypointsService = rospy.Service("/web_service/delete_all_location", SetBool, self.callback_deleteAllWaypoints)
        self.retrieveWaypointService = rospy.Service("/web_service/retrieve_location", Waypoint, self.callback_retrieveWaypoint)
        self.retrieveAllWaypointsService = rospy.Service("/web_service/retrieve_all_location", WaypointsList, self.callback_retrieveAllWaypoints)


        rospy.spin()

    def callback_addWaypoint(self, req):

        try:
            data = self.loadDatabase()
        except JSONDecoder:
            #add location if empty
            f = open(self.databaseDirectory, "r+")
            json.dump({req.name : {"x" : req.x, "y" : req.y, "z" : req.z, "w" : req.w, }}, f, indent=4, sort_keys=True)
            f.close()
            return True, "Succesfully added " + req.name

        if req.name in data:
            return False, "Duplicate name!!!"
        
        data[req.name] =  {"x" : req.x, "y" : req.y, "z" : req.z, "w" : req.w, }
        self.clearDatabase()
        self.dumpDatabase(data)
        return True,"Succesfully added " + req.name  


    def callback_deleteWaypoint(self, req):
        try:
            data = self.loadDatabase()
        except JSONDecoder:
            return False, "No waypoints stored"

        if not req.name in data:
            return False, "Waypoint not found!!"
        del data[req.name]
        self.clearDatabase()
        self.dumpDatabase(data)
        return True, "Succesfully deleted " + req.name
        

    def callback_deleteAllWaypoints(self, req):
        self.clearDatabase()
        return True, "Succesfully deleted all waywaypoints"

    def callback_retrieveWaypoint(self, req):
        try:
            data = self.loadDatabase()
        except JSONDecoder:
            return False, "No waypoints stored"
        if not req.name in data:
            return False, "Name not found!!!", ID()
        
        waypoint = ID()
        waypoint.name = req.name
        waypoint.pose.x = data[req.name]["x"]
        waypoint.pose.y = data[req.name]["y"]
        waypoint.pose.w = data[req.name]["w"]
        waypoint.pose.z = data[req.name]["z"]
        return True, "Successfully retrieved " + req.name, waypoint
    def callback_retrieveAllWaypoints(self, req):
        try:
            data = self.loadDatabase()
        except JSONDecoder:
            return False, []

        allWaypoints = []

        for i, waypoint in enumerate(data):
            location = ID()
            location.name = waypoint
            location.pose.x = data[waypoint]["x"]
            location.pose.y = data[waypoint]["y"]
            location.pose.z = data[waypoint]["z"]
            location.pose.w = data[waypoint]["w"]
            allWaypoints.append(location)

        return True, allWaypoints

    def clearDatabase(self):
        f = open(self.databaseDirectory, "w+")
        f.write("")
        f.close()

    def loadDatabase(self):
        f = open(self.databaseDirectory, "r+")
        data = json.load(f)
        f.close()
        return data

    def dumpDatabase(self, data):
        f = open(self.databaseDirectory, "r+")
        json.dump(data, f, indent=4, sort_keys=True)
        f.close()


if __name__ == "__main__":
    rospy.init_node("legendOfWebService")
    Avatar()
