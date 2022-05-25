# -*- coding: utf-8 -*-
"""
Run the pypogs GUI
==================

Run this script (i.e. type python run_pypogsGUI.py in a termnial window) to start the pypogs Graphical User Interface.
"""
import sys
from pathlib import Path
sys.path.append('..')
import pypogs

# ClEAR LOGS:
open('../pypogs/debug/pypogs.txt', 'w').close()
open('../pypogs/debug/gui.txt', 'w').close()

# INITIALIZE PYPOGS SYSTEM:
sys = pypogs.System()

# ADD TARGETS TO SAVED TARGETS LIST:
#sys.saved_targets['YAOGAN 1'] = 29092

# CONFIGURE GROUND STATION SITE:
# class MySite:
#  lat  =  0  # degrees N
#  lon  =  0  # degrees E
#  elev =  500 # meters above MSL
#sys.alignment.set_location_lat_lon(lat=MySite.lat, lon=MySite.lon, height=MySite.elev)
#sys.alignment.set_alignment_enu()   # uses mount-internal alignment model with no corrections
#sys.alignment.get_alignment_data_form_file('../pypogs/data/2022-03-09T050113_Alignment_from_obs.csv')  # load a previous alignment solution


# ADD MOUNT:
'''
sys.add_mount(
  model="ASCOM",        identity="Simulator"
# model="ASCOM",        identity="DeviceHub"
# model="iOptron AZMP", identity="COM2"
# model="Celestron",    identity="COM3"
)
if sys.mount is not None:
  sys.mount.max_rate = (6, 6)
  sys.mount.alt_limit = (-8, 80)  
  sys.stellarium_telescope_server.start(address='0.0.0.0', port=10001, poll_period=1)
'''



# ADD COARSE CAMERA:
'''
coarsePlateScale = 206 * 5.86 / (400*0.65) # arcsec/pixel,  206 * pixel_pitch_um / focal_length_mm
sys.add_coarse_camera(
  model="ASCOM", 
  #identity="ASICamera2",
  #identity="ASICamera2_2",
  identity="Simulator",
  #identity="QHYCCD_GUIDER",
  exposure_time = 100,
  #gain = 400,
  plate_scale = round(coarsePlateScale, 3),  
  binning = 2
)
'''

# ADD STAR CAMERA:
#if sys.coarse_camera is not None:
#  sys.add_star_camera_from_coarse()


# ADD FINE CAMERA:
#finePlateScale = 206 * 5.86 / 2350 # arcsec/pixel,  206 * pixel_pitch_um / focal_length_mm
#sys.add_fine_camera(model="ASCOM", identity="ASICamera2", exposure_time=500, gain=260, plate_scale=finePlateScale)


# CONFIGURE TRACKING SETTINGS (FEEDBACK PROPERTIES):
#sys.control_loop_thread.OL_speed_limit    = 7200
#sys.control_loop_thread.CCL_speed_limit   = 360
#sys.control_loop_thread.CCL_transition_th = 180
#sys.control_loop_thread.FCL_transition_th = 100

# CHANGE COARSE/FINE TRACKING SETTINGS:
# (MOUNT AND RESPECTIVE COARSE/FINE CAMERA MUST BE DEFINED PREVIOUSLY)
'''
if sys.mount is not None and sys.coarse_camera is not None:
  sys.coarse_track_thread.spot_tracker.smoothing_parameter = 4
  sys.coarse_track_thread.spot_tracker.sigma_mode = 'global_root_square'
  sys.coarse_track_thread.spot_tracker.bg_subtract_mode = 'local_mean'
  sys.coarse_track_thread.spot_tracker.filtsize = 25
'''

# ENABLE SAVING IMAGES DURING TRACKING:
# (MOUNT AND RESPECTIVE COARSE/FINE CAMERA MUST BE DEFINED PREVIOUSLY)
#sys.coarse_track_thread.img_save_frequency = 1
#sys.coarse_track_thread.image_folder = Path('D:\pypogs')
#sys.fine_track_thread.img_save_frequency = 1
#sys.fine_track_thread.image_folder = Path('D:\pypogs')

# SET TARGET:
#sys.target.get_and_set_tle_from_sat_id(23712)  # ISS = 25544
sys.target.get_and_set_tle_from_sat_id(25544)  # ISS = 25544
#sys.target.get_ephem(obj_id='-48', lat=MySite.lat, lon=MySite.lon, height=MySite.elev)
#sys.target.get_ephem(obj_id='7', lat=MySite.lat, lon=MySite.lon, height=MySite.elev)
#sys.target.get_ephem(obj_id='-170', lat=MySite.lat, lon=MySite.lon, height=MySite.elev)

# APPLICATION LINKS
# Use address 127.0.0.1 if the external application runs on this computer.
# Use address 0.0.0.0 if the external application runs on another computer on your local network.
sys.stellarium_telescope_server.start(address='127.0.0.1', port=10001, poll_period=1)   # Stellarium connection
sys.target_server.start(address='127.0.0.1', port=12345, poll_period=1)  # SkyTrack connection



# START GUI:
try:
    pypogs.GUI(sys, 50)
except Exception:
    raise
finally:
    sys.deinitialize()