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

# APPLICATION LINKS
# Use address 127.0.0.1 if the external application runs on this computer.
# Use address 0.0.0.0 if the external application runs on another computer on your local network.
#sys.stellarium_telescope_server.start(address='127.0.0.1', port=10001, poll_period=1)   # Stellarium connection
sys.stellarium_telescope_server.start(address='0.0.0.0', port=10001, poll_period=1)   # Stellarium connection
sys.target_server.start(address='127.0.0.1', port=12345, poll_period=1)  # SkyTrack connection


# Set custom tetra3 database:
#sys.tetra3.load_database('my_custom_tetra3_database')  # If my_custom_tetra3_database.npz is in your tetra3 installation directory
#sys.tetra3.load_database(pathlib.Path('my_custom_tetra3_database.npz'))  # If my_custom_tetra3_database.npz is in current working directory


# CONFIGURE GROUND STATION SITE:
sys.alignment.set_location_lat_lon(
  lat = 34,      # degrees N
  lon = -118,    # degrees E
  height = 500    # meters MSL
)
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
# Note:  it appears ASCOM inverts the alt axis, hence -1 axis direction.
sys.add_mount(model="ASCOM", identity="Simulator", max_rate=(10, 10), alt_limit=(-8, 95), axis_directions=(1, -1))
#sys.add_mount(model="ASCOM", identity="DeviceHub", axis_directions=(1, -1))
#sys.add_mount(model="iOptron AZMP", identity="COM2")
#sys.add_mount(model="Celestron", identity="COM2")


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

if sys.mount is not None:

    # GENERAL FEEDBACK SETTINGS:
    sys.control_loop_thread.integral_max_add = 36 #30
    sys.control_loop_thread.integral_max_subtract = 360 #30
    sys.control_loop_thread.integral_min_rate = 0 #5

    # OPEN LOOP TRACKING SETTINGS:
    sys.control_loop_thread.OL_P = 0.5 #1
    sys.control_loop_thread.OL_I = 10
    sys.control_loop_thread.OL_speed_limit = 10*3600  # increased from 7200 arcsec/sec to improve zenith recovery    

    # COARSE TRACKING SETTINGS:
    if sys.coarse_camera is not None:
        sys.coarse_track_thread.spot_tracker.smoothing_parameter = 4
        sys.coarse_track_thread.spot_tracker.sigma_mode = 'global_root_square'
        sys.coarse_track_thread.spot_tracker.bg_subtract_mode = 'local_mean'
        sys.coarse_track_thread.spot_tracker.filtsize = 25

        sys.coarse_track_thread.spot_tracker.max_search_radius = 1000 #500
        sys.coarse_track_thread.spot_tracker.min_search_radius = 200
        sys.coarse_track_thread.spot_tracker.spot_min_sum = 50 #500
        sys.coarse_track_thread.spot_tracker.spot_min_area = 6 #3
        sys.coarse_track_thread.spot_tracker.fails_to_drop = 10
        sys.coarse_track_thread.spot_tracker.smoothing_parameter = 4 #8
        sys.coarse_track_thread.spot_tracker.rmse_smoothing_parameter = 8
        sys.coarse_track_thread.feedforward_threshold = 10

        sys.control_loop_thread.CCL_P = 0.5 #1
        sys.control_loop_thread.CCL_I = 8 #5
        sys.control_loop_thread.CCL_speed_limit = 3600
        sys.control_loop_thread.CCL_transition_th = 200 # increased from 100 to tolerate more drift

    # FINE TRACKING SETTINGS:
    if sys.fine_camera is not None:
        sys.fine_track_thread.spot_tracker.smoothing_parameter = 4
        sys.fine_track_thread.spot_tracker.sigma_mode = 'global_root_square'
        sys.fine_track_thread.spot_tracker.bg_subtract_mode = 'local_mean'
        sys.fine_track_thread.spot_tracker.filtsize = 25

        sys.control_loop_thread.FCL_P = 1 #2
        sys.control_loop_thread.FCL_I = 5 #5
        sys.control_loop_thread.FCL_speed_limit = 60  # keep this small to reduce smear in primary imaging camera
        sys.control_loop_thread.FCL_transition_th = 200 # increased from 100 to tolerate more drift



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