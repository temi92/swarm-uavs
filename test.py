import argparse
import time
from dronekit import connect
from math import radians,cos,sin,asin,sqrt,pi
from temiutils import QuadController

def altered_position(original_location,dEast,dNorth=0):
   earth_radius = 6378137.0
   dLat = dNorth/earth_radius
   dLon = dEast/(earth_radius*cos(pi*original_location[0]/180))
   newlat = original_location[0] + (dLat *180/pi)
   newlon = original_location[1] + (dLon *180/pi)
   targetlocation = (newlat,newlon)
   return targetlocation  

def main():
   parser = argparse.ArgumentParser(description='Gps navigation of  a single UAV.')
   parser.add_argument("x",help ="+x -> EAST and -x -> WEST",type=float,metavar="X")
   parser.add_argument("y",help="+y -> NORTH and -y -> SOUTH",type=float,metavar="Y")
   parser.add_argument("d",help="total distance by single uav",type=int,metavar="distance")
   parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
   parser.add_argument('--baudrate',type=int,
                   help="Vehicle baudrate settings specify 57600 when connecting with radio. If not specified, SITL automatically started and used.",default=115200)

   args = parser.parse_args()

   connection_string = args.connect
   sitl = None

   #Start SITL if no connection string specified
   if not connection_string or not args.baudrate:
       from dronekit_sitl import SITL
       sitl = SITL()
       sitl.download('copter', '3.3', verbose=True)
       sitl_args = ['-I0', '--model', 'quad', '--home=53.367063,-3.109593,0,0']
       sitl.launch(sitl_args, await_ready=True, restart=True)
       connection_string = 'tcp:127.0.0.1:5762'

   # Connect to the Vehicle
   print 'Connecting to vehicle on: %s' % connection_string
   vehicle = connect(connection_string, wait_ready=True,baud=args.baudrate)
   quad = QuadController(vehicle)
   home_location = quad.home_position()
   init_pos = altered_position(home_location,args.x,args.y)
   start = init_pos
   end = altered_position(start,0,args.d)

   print "Basic pre-arm checks"
   # Don't try to arm until autopilot is ready
   while not quad.is_armable:
       print " Waiting for vehicle to initialise..."
       time.sleep(1)


   print "Arming motors"
   # Copter should arm in GUIDED mode
   quad.mode = "GUIDED"
   quad.arm = True

    # Confirm vehicle armed before attempting to take off
   while not quad.arm:
       print " Waiting for arming..."
       time.sleep(1)
   print "Vehicl armed status: " + str(quad.arm)

   print "taking off .."
   print quad.mode
   quad.takeoff()
   print "going to first way point..."
   quad.goto(init_pos)
   if not quad.goto(init_pos,groundspeed=1):
       print "some event has gone horribly wrong"
       return
   while not QuadController.get_distance((quad.current_location.lat,quad.current_location.lon),init_pos):
       time.sleep(0.001)#to avoid consuming much CPU
   print "arrived at first way point"
          
   #print "settting yaw"
   #quad.set_yaw(90)
   #time.sleep(5)
   #print quad.heading
 
   print "following path..."
   path = QuadController.path(start,end)
   for p in path[1:]:
       print "going to Lat:%.6f, Lon:%.6f" %(p[0],p[1])
       if not quad.goto((p[0],p[1])):
          print "some event has gone horribly wrong"
          return 
       while not QuadController.get_distance((quad.current_location.lat,quad.current_location.lon),(p[0],p[1])):
           time.sleep(0.01)
   print quad.current_location
   print "vehicle returning to launch..."

   quad.mode = "RTL"

if __name__ == "__main__":
   main()
