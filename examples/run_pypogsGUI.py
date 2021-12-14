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

sys.add_mount(model="ascom", identity="ASCOM.DeviceHub.Telescope")
sys.add_coarse_camera(model="ascom", identity="ASCOM.ASICamera2_2.Camera")
sys.alignment.set_location_lat_lon(lat=34.2, lon=-118.2, height=600)
sys.alignment.set_alignment_enu()
tle = sys.target.get_tle_from_sat_id(25544)
sys.target.set_target_from_tle(tle)

try:
    pypogs.GUI(sys, 500)
except Exception:
    raise
finally:
    sys.deinitialize()

