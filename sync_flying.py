import argparse
import time
import threading
import Queue
from dronekit import connect,VehicleMode,LocationGlobalRelative
from math import radians,cos,sin,asin,sqrt,pi
from temi_utils import QuadController

parser = argparse.ArgumentParser(description="synchronous movement for 2 uavs (arguments given should be in metres)")
parser.add_argument("master_address",help="path to master_vehicle address")
parser.add_argument("slave_address",help = "path to slave address")
parser.add_argument("x",help ="+x -> EAST and -x -> WEST",type=float,metavar="X")
parser.add_argument("y",help="+y -> NORTH and -y -> SOUTH",type=float,metavar="Y")
parser.add_argument("d",help="total distance travelled for both uavs",type=int,metavar="distance")
parser.add_argument("--baseline",help="distance between the 2 drones",type=int,default=50,metavar="distance")
parser.add_argument("--speed",help="speed at which UAV flies in m/s",type=float,default=1,metavar="speed m/s")
parser.add_argument('--baudrate',type=int,
                   help="Vehicle baudrate settings specify 57600 when connecting with radio. If not specified, SITL automatically started and used.",default=115200)

#consider baudrate settings
args = parser.parse_args()

class Quad(QuadController):
   def __init__(self,vehicle,vehicle_id):
       self.vehicle_types = ["master_vehicle","slave_vehicle"]
       self.vehicle_id = vehicle_id
       super(Quad,self).__init__(vehicle)
   def __str__(self):
       return self.vehicle_types[self.vehicle_id]

def altered_position(original_location,dEast,dNorth=0):
   earth_radius = 6378137.0
   dLat = dNorth/earth_radius
   dLon = dEast/(earth_radius*cos(pi*original_location[0]/180))
   newlat = original_location[0] + (dLat *180/pi)
   newlon = original_location[1] + (dLon *180/pi)
   targetlocation = (newlat,newlon)
   return targetlocation

       
def takeoff_sequence(vehicle):
       global home_location1
       global home_location2
       print vehicle
       #get home location of master_vehicle vehicle
       if str(vehicle) == "master_vehicle":
           home_location1 = vehicle.home_position()
       #get home_location of slave vehicle
       if str(vehicle) == "slave_vehicle":
           homelocation2 = vehicle.home_position()
           time.sleep(0.5) #to ensure master vehicle starts off first
       while not vehicle.is_armable:
           time.sleep(1)
       while vehicle.mode != 'GUIDED':
           print "setting mode to GUIDED"
           vehicle.mode = "GUIDED"
           time.sleep(0.2)
       #confirm vehicle armed before attempting to take-off
       while not vehicle.arm:
           vehicle.arm = True
           time.sleep(0.6)
       print "taking off"
       while not vehicle.takeoff():
           pass
       print "vehicle sucessfully took off "
               
def vehicle_connect(connection_string,idd):
   vehicle = connect(connection_string,wait_ready=True,baud=args.baudrate)
   quad = Quad(vehicle,idd)
   vehicles.append(quad)


vehicles =[]
connect_threads =[]
#vehicle_address = ['127.0.0.1:14550','tcp:172.20.10.7:5762']
vehicle_address = [args.master_address,args.slave_address]
home_location1 = None #home location of master_vehicle
home_location2 = None #home location of slave_vehicle
queue = Queue.Queue()


for i in range(len(vehicle_address)):
   connect_threads.append(threading.Thread(target=vehicle_connect,name ='beginner_thread',args=(vehicle_address[i], i),))
   connect_threads[-1].start()
   

for thread in connect_threads:
   print "waiting for connecting threads"
   thread.join()

def worker((func,arg)):
   queue.put((func,arg))

def takeoff_queue():
   while True:
       func,args = queue.get()
       func(args)
       queue.task_done()

t = threading.Thread(target=takeoff_queue)
t.daemon = True
t.start()
del t

launch_threads = []

for i in range(0,len(vehicles)):
   launch_threads.append(threading.Thread(target=worker,name='sample thread',args=((takeoff_sequence,vehicles[i]),)))
   launch_threads[-1].start()

for thread in launch_threads:
   print "waiting for launch threads"
   thread.join()

queue.join()
print "vehicles taken off"

print "home location of master is at:%.6f, Lon:%.6f" %(home_location1[0],home_location1[1])

#inital gps_position of master
init_pos1 = altered_position(home_location1,args.x,args.y)
print "master vehicle initial  position is  Lat:%.6f, Lon:%.6f" %(init_pos1[0],init_pos1[1])

#initial gps_position of slave  (offset from position of master i.e baseline)
init_pos2 = altered_position(init_pos1,args.baseline,0)
print "slave vehicle initial position is Lat:%.6f, Lon:%.6f" %(init_pos2[0],init_pos2[1])

#gps path for master
start1 = init_pos1
end1 = altered_position(start1,0,args.d)

#gps path for slave
start2 = init_pos2
end2 = altered_position(start2,0,args.d)

event = threading.Event()

def master_path(start,end,vehicle):
   print "master flying"   
   ps = Quad.path(start,end)
   #print ps[0]
   if not vehicle.goto(init_pos1,groundspeed=args.speed):
       return

   while not Quad.get_distance((vehicle.current_location.lat,vehicle.current_location.lon),init_pos1):
       time.sleep(0.001)
   event.set()
   while event.isSet():
       print "waiting for slave vehicle to reach initial waypoint"
       time.sleep(1)
 
   for _,p in enumerate(ps[1:]): 
       print "master vehicle going to Lat:%.6f, Lon:%.6f" %(p[0],p[1])
       vehicle.goto(p)
       while not Quad.get_distance((vehicle.current_location.lat,vehicle.current_location.lon),p):
           time.sleep(0.001) 
       print "master vehicle arrived at GPS position"
       
def slave_path(start,end,vehicle):
   event_wait = event.wait()
   print "slave flying"
   ps = Quad.path(start,end)
   if not vehicle.goto(init_pos2,groundspeed=args.speed):
       return
   while not Quad.get_distance((vehicle.current_location.lat,vehicle.current_location.lon),init_pos2):
       time.sleep(0.001)
   event.clear() #event flag is set to False

   for _,p in enumerate(ps[1:]):
       print "slave vehicle going to Lat:%.6f, Lon:%.6f" %(p[0],p[1])
       vehicle.goto(p)
       while not Quad.get_distance((vehicle.current_location.lat,vehicle.current_location.lon),p):
           time.sleep(0.001)
       print "slave vehicle arrived at GPS position"

t1 = threading.Thread(name='master',target=master_path,args=(start1,end1,vehicles[0]))
t1.start()

t2 = threading.Thread(name='slave',target=slave_path,args=(start2,end2,vehicles[1]))
t2.start()

t1.join()
t2.join()

for i in range(0,len(vehicles)):
    print "setting to RTL on %s" % vehicles[i]
    vehicles[i].mode ="RTL"
    time.sleep(0.1)

print "finished synchronous flight"


