# -*- coding: utf-8 -*-
"""
Run the pypogs GUI
==================

Run this script (i.e. type python run_pypogsGUI.py in a termnial window) to start the pypogs Graphical User Interface.
"""
import sys
sys.path.append('..')
import pypogs

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
#sys.add_mount(model="ASCOM", identity="DeviceHub", axis_directions=(1, -1))
sys.add_mount(model="iOptron AZMP", identity="COM2")
#sys.add_mount(model="Celestron", identity="COM2")


# ADD COARSE CAMERA:
coarsePlateScale = 206 * 5.86 / (400*0.65) # arcsec/pixel,  206 * pixel_pitch_um / focal_length_mm
#sys.add_coarse_camera(model="ASCOM", identity="Simulator")
#sys.add_coarse_camera(model="ASCOM", identity="ASICamera2_1")

sys.add_coarse_camera(
  model="ASCOM", 
  #identity="ASICamera2",
  identity="ASICamera2_2",
  #identity="Simulator",
  exposure_time = 150,
  gain = 400,
  plate_scale = round(coarsePlateScale, 3),  
  binning = 2
)


# ADD STAR CAMERA:
sys.add_star_camera_from_coarse()


# ADD FINE CAMERA:
finePlateScale = 206 * 5.86 / 2350 # arcsec/pixel,  206 * pixel_pitch_um / focal_length_mm
#sys.add_fine_camera(model="ASCOM", identity="ASICamera2", exposure_time=500, gain=260, plate_scale=finePlateScale)



# SET TARGET:
#sys.target.get_and_set_tle_from_sate_id(23712)  # ISS = 25544

#sys.target.get_ephem(obj_id='-48', lat=MySite.lat, lon=MySite.lon, height=MySite.elev)
#sys.target.get_ephem(obj_id='7', lat=MySite.lat, lon=MySite.lon, height=MySite.elev)
#sys.target.get_ephem(obj_id='-170', lat=MySite.lat, lon=MySite.lon, height=MySite.elev)


# START GUI:
try:
    pypogs.GUI(sys, 500)
    sys.do_auto_star_alignment(max_trials=2, rate_control=True, pos_list=[(40, -135), (60, -135)])

except Exception:
    raise
finally:
    sys.deinitialize()