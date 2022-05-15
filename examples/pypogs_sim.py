# -*- coding: utf-8 -*-
"""
Run the pypogs GUI
==================

Run this script (i.e. type python run_pypogsGUI.py in a termnial window) to start the pypogs Graphical User Interface.
"""
import sys, pathlib
sys.path.append('..')
import pypogs

# ClEAR LOGS:
open('../pypogs/debug/pypogs.txt', 'w').close()
open('../pypogs/debug/gui.txt', 'w').close()

# INITIALIZE PYPOGS SYSTEM:
sys = pypogs.System()


# Set custom tetra3 database:
#sys.tetra3.load_database('my_custom_tetra3_database')  # If my_custom_tetra3_database.npz is in your tetra3 installation directory
#sys.tetra3.load_database(pathlib.Path('my_custom_tetra3_database.npz'))  # If my_custom_tetra3_database.npz is in current working directory


# CONFIGURE GROUND STATION SITE:
class MySite:
  lat  =   34.24  # degrees N
  lon  = -118.24  # degrees E
  elev =   600   # meters above MSL
sys.alignment.set_location_lat_lon(lat=MySite.lat, lon=MySite.lon, height=MySite.elev)
sys.alignment.set_alignment_enu()
#sys.alignment.get_alignment_data_form_file('../pypogs/data/2022-03-09T050113_Alignment_from_obs.csv')


'''
auto_align_azi = (-30, 60, 150, -120)
auto_align_alt = (50, 60)
auto_align_vectors = []
for azi in auto_align_azi:
  for alt in auto_align_alt:
    auto_align_vectors.append((alt, azi))
sys.auto_align_vectors = auto_align_vectors
'''

# ADD MOUNT:
sys.add_mount(model="ASCOM", identity="Simulator")
#sys.add_mount(model="ASCOM", identity="DeviceHub", axis_directions=(1, -1))
#sys.add_mount(model="iOptron AZMP", identity="COM2")
#sys.add_mount(model="Celestron", identity="COM2")

#sys.stellarium_telescope_server.start(address='127.0.0.1', port=10001, poll_period=1)  # use address 127.0.0.1 for access on this computer only
sys.stellarium_telescope_server.start(address='0.0.0.0', port=10001, poll_period=1)  # use address 0.0.0.0 to make accessible across network

sys.target_server.start(address='127.0.0.1', port=12345, poll_period=1)  # Target server to receive target data from SkyTrack


# ADD COARSE CAMERA:
sys.add_coarse_camera(
  model="ASCOM", 
  #identity="ASICamera2",
  #identity="ASICamera2_2",
  identity="Simulator",
  exposure_time = 150,
  gain = 400,
  plate_scale = 5,  
  binning = 1
)


# ADD STAR CAMERA:
#sys.add_star_camera_from_coarse()
sys.add_star_camera(
  model="ASCOM", 
  #identity="ASICamera2",
  #identity="ASICamera2_2",
  identity="Simulator",
  exposure_time = 150,
  gain = 400,
  plate_scale = 20,  
  binning = 1
)


# ADD FINE CAMERA:
#finePlateScale = 206 * 5.86 / 2350 # arcsec/pixel,  206 * pixel_pitch_um / focal_length_mm
#sys.add_fine_camera(model="ASCOM", identity="ASICamera2", exposure_time=500, gain=260, plate_scale=finePlateScale)



# SET TARGET:
#sys.target.get_and_set_tle_from_sat_id(23712)  # ISS = 25544
sys.target.get_and_set_tle_from_sat_id(25544)  # ISS = 25544
#sys.target.get_ephem(obj_id='-48', lat=MySite.lat, lon=MySite.lon, height=MySite.elev)
#sys.target.get_ephem(obj_id='7', lat=MySite.lat, lon=MySite.lon, height=MySite.elev)
#sys.target.get_ephem(obj_id='-170', lat=MySite.lat, lon=MySite.lon, height=MySite.elev)


# START GUI:
try:
    pypogs.GUI(sys, 500)
    #sys.do_auto_star_alignment(max_trials=2, rate_control=True, pos_list=[(40, -135), (60, -135)])

except Exception:
    raise
finally:
    sys.deinitialize()