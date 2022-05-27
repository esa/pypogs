# -*- coding: utf-8 -*-
"""
Run the pypogs GUI
==================

Run this script (i.e. type python run_pypogsGUI.py in a termnial window) to start the pypogs Graphical User Interface.
"""
import sys
sys.path.append('..')
import pypogs

# ClEAR LOGS:
open('../pypogs/debug/pypogs.txt', 'w').close()
open('../pypogs/debug/gui.txt', 'w').close()

# INITIALIZE PYPOGS SYSTEM:
sys = pypogs.System()

# CONFIGURE GROUND STATION SITE:
class MySite:
  lat  =   34.2  # degrees N
  lon  = -118.2  # degrees E
  elev =   600   # meters above MSL
sys.alignment.set_location_lat_lon(lat=MySite.lat, lon=MySite.lon, height=MySite.elev)
sys.alignment.set_alignment_enu()


# ADD MOUNT:
#sys.add_mount(model="ASCOM", identity="Simulator")
#sys.add_mount(model="ASCOM", identity="DeviceHub", axis_directions=(1, -1))  # ascom inverts alt axis?
sys.add_mount(model="iOptron AZMP", identity="COM2", max_rate=(16, 16))
#sys.add_mount(model="Celestron", identity="COM5")

# APPLICATION LINKS
# Use address 127.0.0.1 if the external application runs on this computer.
# Use address 0.0.0.0 if the external application runs on another computer on your local network.
sys.stellarium_telescope_server.start(address='127.0.0.1', port=10001, poll_period=1)   # Stellarium connection
sys.target_server.start(address='127.0.0.1', port=12345, poll_period=1)  # SkyTrack connection


# START GUI:
try:
    pypogs.GUI(sys, 500)
    #sys.do_auto_star_alignment(max_trials=2, rate_control=True, pos_list=[(40, -135), (60, -135)])

except Exception:
    raise
finally:
    sys.deinitialize()