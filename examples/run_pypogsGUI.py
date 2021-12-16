# -*- coding: utf-8 -*-
"""
Run the pypogs GUI
==================

Run this script (i.e. type python run_pypogsGUI.py in a termnial window) to start the pypogs Graphical User Interface.
"""
import sys
sys.path.append('..')

import pypogs

sys = pypogs.System()

try:
  sys.add_mount(model="ascom", identity="ASCOM.DeviceHub.Telescope")
  sys.mount.axis_directions = (1,-1)
except:  pass


try:
  #sys.add_coarse_camera(model="ascom", identity="ASCOM.Simulator.Camera")
  sys.add_coarse_camera(model="ascom", identity="ASCOM.ASICamera2_2.Camera")
  sys.coarse_camera.exposure_time = 200
  try:  sys.coarse_camera.gain = 400
  except: pass
  sys.coarse_camera.plate_scale = 4.5
except: pass
  
  
sys.alignment.set_location_lat_lon(lat=34.2, lon=-118.2, height=600)
sys.alignment.set_alignment_enu()
try:
  tle = sys.target.get_tle_from_sat_id(39035)  # ISS = 25544
  sys.target.set_target_from_tle(tle)
except:  pass
  
  
try:
    pypogs.GUI(sys, 500)
except Exception:
    raise
finally:
    sys.deinitialize()

