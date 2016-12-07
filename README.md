Synchronous Flight of 2 UAVs

Additional Dependencies
DroneKit2.0 - Provides python bindings for the MAVLink Protocol

This codebase consists of 3 parts 
1) sync-flying.py 
   This allows to 2 uavs to take off to a pre set height and follow pre assigned GPS coordinates at the same speed

   Run python sync-flying.py -h for the various configurations
2) test.py
   Allows 1 single uav to follow preassigned GPS configurations
   Run python test.py -h fo the various configurations
3) temi-utils.py
   Python wrapper class for dronekit that includes added control functionality 
    

