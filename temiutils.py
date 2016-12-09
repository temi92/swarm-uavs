from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import numpy as np
from math import radians, cos, sin, asin, sqrt

class QuadController(object):
   def __init__(self,vehicle):
        self.vehicle = vehicle
   @property
   def vehicle(self):
        return self._vehicle
   @vehicle.setter
   def vehicle(self,name):
        if isinstance(name,Vehicle):
           self._vehicle = name
        else:
           raise Exception("invalid object passes to Quadcontroller")

   @property
   def mode(self):
       """
       return current vehicle mode
       """
       return self.vehicle.mode
  
   @mode.setter
   def mode(self,mode):
       """
       set the mode of the vehicle
       """
       self.vehicle.mode = VehicleMode(mode)

   def close(self):
       self.vehicle.close()
   
   @property
   def arm(self):
       """
       return True if vehicle is armed else False
       """
       return self.vehicle.armed
 
   @arm.setter
   def arm(self,value):
       if bool(value) != self.vehicle.armed:
           if value:
               self.vehicle.armed = True
           else:
               self.vehicle.armed = False
   @property 
   def airspeed(self):
       return self.vehicle.airspeed
   @airspeed.setter
   def airspeed(self,speed):
       if speed > 10:
           raise ValueError("cannot set speed of vehicle greater than 10m/s")
       self.vehicle.airspeed = speed


   @property
   def is_armable(self):
       return self.vehicle.is_armable

   def takeoff(self,alt=5):
       if self.vehicle.mode.name == "GUIDED":
           print "vehicle in guided mode"
           self.vehicle.simple_takeoff(alt)
           while True:
               print " Altitude: ", self.current_location.alt
               if self.current_location.alt >= alt*0.95:
                   print "reached target altitude"
                   break   
               time.sleep(1) 
           return True 
           
       return False
   
   def home_position(self):
       #returns latitude and longitude of home positon
       print "getting home position"
       cmds = self.vehicle.commands
       cmds.download()
       cmds.wait_ready()
       home = self.vehicle.home_location
       return (home.lat,home.lon)
       

   def goto(self,(lat,lon), alt=5,groundspeed=None):
       """
       send the uav to specified GPS lat,lon and alt in GUIDED mode
       """
       if self.vehicle.mode == "GUIDED":
           loc = LocationGlobalRelative(lat,lon,alt)
           self.vehicle.simple_goto(loc,groundspeed)
           return True
       return False

   @property
   def current_location(self):
       """
       obtain uav current location with altitude relative to home position
       """
       return self.vehicle.location.global_relative_frame

   def set_yaw(self,heading,relative=False):
       """
       Send MAV_CMD_CONDITION_YAW message to point the vehicle to a specified heading in (degrees).This method returns an absolute heading by default but you can set the relative parameter to 'True' to set the yaw relative to 
       direction of travel 
       """
       if relative:
           is_relative = 1
       else:
           is_relative = 0

       msg = self.vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
       # send command to vehicle
       self.vehicle.send_mavlink(msg)

   @property
   def heading(self):
       return self.vehicle.heading   

   @staticmethod
   def get_distance((x,y),(cx,cy),eps=0.00003):
       """
       returns boolean expression which checks if current lat,lon (x,y) is within eps of target lat,lon (cx,cy)
       """    
       return (x-cx)**2 + (y-cy)**2 <= eps**2

   @staticmethod
   def path((lat0,lon0),(lat1,lon1),k=5):
       """
       returns a path for drone to follow
       """
       lats = np.linspace(lat0,lat1,k)
       lons = np.linspace(lon0,lon1,k)
       p = zip(lats,lons)
       return p 
