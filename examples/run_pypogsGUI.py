# -*- coding: utf-8 -*-
"""
Run the pypogs GUI
==================

Run this script (i.e. type python run_pypogsGUI.py in a termnial window) to start the pypogs Graphical User Interface.
"""
import sys
sys.path.append('..')

import pypogs

class Site:
  lat  =   34.2  # deg N
  lon  = -118.2  # deg E
  elev =   600   # meters above MSL


sys = pypogs.System()

sys.alignment.set_location_lat_lon(lat=Site.lat, lon=Site.lon, height=Site.elev)
sys.alignment.set_alignment_enu()

#sys.add_mount(model="ASCOM", identity="Simulator")
sys.add_mount(model="ASCOM", identity="DeviceHub")
#sys.mount.axis_directions = (1,-1)
#sys.add_mount(model="iOptron AZMP", identity="COM2")
  
'''
#sys.add_coarse_camera(model="ASCOM", identity="Simulator")
#sys.add_coarse_camera(model="ASCOM", identity="ASICamera2_1")
sys.add_coarse_camera(model="ASCOM", identity="ASICamera2_2")
sys.coarse_camera.exposure_time = 200
try:  sys.coarse_camera.gain = 400
except: pass
sys.coarse_camera.plate_scale = 4.5
'''


tle = sys.target.get_tle_from_sat_id(23712)  # ISS = 25544
sys.target.set_target_from_tle(tle)


#sys.target.get_ephem(obj_id='-48', lat=Site.lat, lon=Site.lon, height=Site.elev)
sys.target.get_ephem(obj_id='7', lat=Site.lat, lon=Site.lon, height=Site.elev)
  
try:
    pypogs.GUI(sys, 500)
except Exception:
    raise
finally:
    sys.deinitialize()

