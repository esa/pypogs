"""Feedback and tracking algorithms
===================================

- :class:`pypogs.ControlLoopThread` is the main control loop for pypyogs. It manages the transition
   logic and feedback loop.

- :class:`pypogs.TrackingThread` is the target tracking loop. One is created for each camera to
  read arriving images. It provides the current error etc. to the pypogs.ControlLoopThread.

- :class:`pypogs.SpotTracker` is attached to each pypogs.TrackingThread and implements the spot
  detection and tracking.

This is Free and Open-Source Software originally written by Gustav Pettersson at ESA.

License:
    Copyright 2019 the European Space Agency

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
"""

# Standard imports:
from pathlib import Path
import logging
import sys
from threading import Thread, Event
from time import sleep, perf_counter as precision_timestamp
from datetime import datetime
from csv import writer as csv_write

# External imports:
from astropy.time import Time as apy_time
from astropy import units as apy_unit
import numpy as np
from tifffile import imwrite as tiff_write

# Internal imports:
sys.path.append('..')  # Add one directory up to path
from tetra3 import get_centroids_from_image
from .hardware_cameras import Camera

EPS = 10**-6  # Epsilon for use in non-zero check
DEG = chr(176)  # Degree (unicode) character


class ControlLoopThread:
    """Run a control loop for satellite tracking.

    This thread holds a reference to the *parent* pypogs.System instance and takes control of all
    devices. Set up this thread with your desired tracking parameters (such as gains, frequency,
    feedback maximum rates, etc.) and then call ControlLoopThread.start(). The thread runs with a
    maximum frequency defined by max_frequency, it defaults to 5 Hz. The target (and start and end
    times if desired) must be defined in the *parent*.target object.

    Upon reaching a transition criteria, the tracking mode will be changed (e.g. open-loop to
    closed-loop) automatically either to a "better" or "worse" mode. The available modes and their
    parameters are defined below:

        - OL: Open-loop control. The mount angles are read out and the control signal is based on
          the difference between the mount angles and the target angles. Gains OL_P (proportional,
          default 1.0), OL_I (integral, default 10 seconds), and rate limit OL_speed_limit
          (default 3600 arcsec/s) apply.

        - CCL: Coarse closed-loop control. The measured position from the coarse camera, returned
          by *parent*.coarse_track_thread.track_x_y, is used (scaled to degrees) for the control
          signal. Gains CCL_P (proportional, default .5), CCL_I (integral, default 10 s), and rate
          limit CCL_speed_limit (default 180 arcsec/s) apply. The transition OL > CCL requires the
          following:

              - *parent*.coarse_track_thread exists and has a track
              - aforementioned track standard deviation is below CCL_transition_th (default 100
                arcsec)

        - FCL: Fine closed-loop control. The measured position from the fine camera, returned by
          *parent*.fine_track_thread.track_x_y, is used (scaled to degrees) for the control signal.
          Gains FCL_P (proportional, default 1.0), FCL_I (integral, default 10 s), and rate limit
          FCL_speed_limit (default 180 arcsec/s) apply. The transition CCL > FCL requires the
          following:

              - *parent*.fine_track_thread exists and has a track
              - aforementioned track standard deviation is below FCL_transition_th (default 30
                arcsec)

        - CTFSP: Coarse-to-fine spiral acquisition. Only available if CTFSP_enable is set to True.
          (defaults to False). It is recommended to use this only to automatically find the
          intercamera alignment, and then disable again for fastest possible acquisition. Enabling
          CTFSP has several effects:

              - When in CCL, the transition CCL > FCL will not occur as described above. Instead,
                when in CCL and the *parent*.coarse_track_thread.rms_error falls below
                CTFSP_transition_th (default 20 arcsec), the transition CCL > CTFSP  occurs.
              - When in CTFSP, CCL tracking parameters are used. If CTFSP_zero_integral = True is
                set (defaults to False) the integral term is set to zero.
              - When in CTFSP, the *parent*.coarse_track_thread.spot_tracker goal is supplemented
                by the goal offset with a spiral which has arms spaced by CTFSP_spacing (default
                100 arcsec), at a rate of CTFSP_speed (default 50 arcsec/s), until a radius
                CTFSP_max_radius (default 500 arcsec) is reached, upon which the mode is changed
                back to CCL and the spiral offset is deleted.
              - When in CTFSP, the transition CTFSP > FCL happens *immediately* when
                *parent*.fine_track_thread exists and has a track.
              - When in FCL, if CTFSP_auto_update_CCL_goal = True, the
                *parent*.coarse_track_thread.spot_tracker goal is updated with the current track
                position, effectively calibrating the intercamera alignment. This only happens when
                *parent*.fine_track_thread.rms_error is below CTFSP_auto_update_CCL_goal_th
                (default 10 arcsec). If CTFSP_disable_after_goal_update is True (the default),
                CTFSP mode will be disabled automatically after successful alignment.

    The availability of each mode can be controlled by setting CCL_enable, FCL_enable, CTFSP_enable
    to True or False, OL mode may not be disabled.

    Note:
        Integral windup protection is achieved by two means:

        1. The integral term is reset if the control signal desired is greater than the speed limit
           (disable by reset_integral_if_saturated=False).
        2. The integral term may grow (in magnitude) by a maximum value of integral_max_add
           (default 36 arcsec) per loop. (It may shrink by a maximum value of
           integral_max_subtract, default 360 arcsec).

    Tracking data is saved to a .csv file in the data_folder. The filenames are auto-generated with
    the start time (as an ISO timestamp) and _ControlLoopThread.csv.

    Args:
        parent (pypogs.System): The System to control.
        data_folder (pathlib.Path, optional): The folder for data saving. If None (the default) the
            folder *pypogs*/data will be used/created.
        debug_folder (pathlib.Path, optional): The folder for debug logging. If None (the default)
            the folder *pypogs*/debug will be used/created.
    """
    def __init__(self, parent, data_folder=None, debug_folder=None):
        """Create ControlLoopThread instance. See class documentation."""
        from . import system
        assert isinstance(parent, system.System), 'Parent must be pypogs.System instance.'
        # Logger setup
        self._debug_folder = None
        if debug_folder is None:
            self.debug_folder = Path(__file__).parent / 'debug'
        else:
            self.debug_folder = debug_folder
        self._logger = logging.getLogger('pypogs.tracking.ControlLoopThread')
        if not self._logger.hasHandlers():
            # Add new handlers to the logger if there are none
            self._logger.setLevel(logging.DEBUG)
            # Console handler at INFO level
            ch = logging.StreamHandler()
            ch.setLevel(logging.INFO)
            # File handler at DEBUG level
            fh = logging.FileHandler(self.debug_folder / 'pypogs.txt')
            fh.setLevel(logging.DEBUG)
            # Format and add
            formatter = logging.Formatter('%(asctime)s:%(name)s-%(levelname)s: %(message)s')
            fh.setFormatter(formatter)
            ch.setFormatter(formatter)
            self._logger.addHandler(fh)
            self._logger.addHandler(ch)

        # Start of constructor
        self._logger.debug('ControlLoopThread called with: parent=' + str(parent) + ' data_folder'
                           + str(data_folder))
        # Data folder setup
        self._data_folder = None
        if data_folder is None:
            self.data_folder = Path(__file__).parent / 'data'
        else:
            self.data_folder = data_folder
        self._thread = None
        self._name = 'ControlLoopThread'
        self._parent = parent
        self._min_loop_time = .2  # 5 Hz
        # Loop management
        self._stop_loop = True
        # Cache of the state of the telescope
        self._empty_state_cache = {'mode': None, 'freq': None, 'fb_alt': None, 'fb_azi': None,
                                   'err_alt': None, 'err_azi': None, 'int_alt': None,
                                   'int_azi': None, 'ct_has_track': None, 'ct_track_alt': None,
                                   'ct_mean_alt': None, 'ct_track_azi': None, 'ct_mean_azi': None,
                                   'ct_track_sd': None, 'ct_rmse': None, 'ct_search_x': None,
                                   'ct_search_y': None, 'ct_search_rad': None,
                                   'ft_has_track': None, 'ft_track_alt': None, 'ft_mean_alt': None,
                                   'ft_track_azi': None, 'ft_mean_azi': None, 'ft_track_sd': None,
                                   'ft_rmse': None, 'ft_search_x': None, 'ft_search_y': None,
                                   'ft_search_rad': None}
        self._state_cache = self._empty_state_cache
        # Tracking feedback settings
        self._reset_integral_if_saturated = True
        self._int_max_add = .01  # Maximum error (deg) to add to the integral term per loop.
        self._int_max_sub = .1
        self._int_min_rate = 0.0  # Minimum rate (deg/s) of the mount to activate the integral term
        # Open loop
        self._OL_Kp = 1.0  # Feedback gain (deg/s per degree error)
        self._OL_Ki = 1/10  # Integral gain (1/integral time [s])
        self._OL_speed_limit = 1.0  # deg/sec internally
        self._OL_goal_x_y = np.array([0, 0], dtype='float64')
        self._OL_goal_offset_x_y = np.array([0, 0], dtype='float64')
        # Coarse closed loop
        self._CCL_enable = True
        self._CCL_Kp = .5  # Feedback gain (deg/s per degree error)
        self._CCL_Ki = 1/10  # Integral gain (1/integral time [s])
        self._CCL_speed_limit = .05  # deg/sec internally
        self._CCL_transition_th = 100.0  # Coarse sd required to move to coarse tracking (arcsec)
        # Fine closed loop
        self._FCL_enable = True
        self._FCL_Kp = 1.0  # Feedback gain (deg/s per degree error)
        self._FCL_Ki = 1/10  # Integral gain (1/integral time [s])
        self._FCL_speed_limit = .05  # deg/sec
        self._FCL_transition_th = 30.0  # Fine sd required to move to fine tracking (arcsec)
        # Spiralling acquisition settings
        # Coarse to Fine spiral
        self._CTFSP_enable = False
        self._CTFSP_spacing = 100.0  # Spacing between arms (arcsec)
        self._CTFSP_speed = 50.0  # Spiral speed (arcsec/sec)
        self._CTFSP_max_radius = 500.0  # Radius where to give up and reset (arcsec)
        # Coarse RMS error needed to transition to spiral (arcsec)
        self._CTFSP_transition_th = 20.0
        # If True the CCL goal (intercam alignment) will be automatically
        # updated when fine tracking is achieved
        self._CTFSP_auto_update_CCL_goal = True
        # Fine RMS error needed to update the coarse goal (intercam alignment)
        self._CTFSP_auto_update_CCL_goal_th = 10.0
        self._CTFSP_zero_integral = False  # Disable coarse integral term for coarse spiralling
        self._CTFSP_disable_after_goal_update = True
        self._CTFSP_delay = 3.0  # Seconds delay before spiralling starts
        self._CTFSP_ramp = 12.0  # Time constant (seconds) for smooth spiral start
        self._logger.info('ControlLoopThread instance created.')

    def _log_debug(self, msg, **kwargs):
        self._logger.debug(msg, **kwargs)

    def _log_info(self, msg, **kwargs):
        self._logger.info(msg, **kwargs)

    def _log_warning(self, msg, **kwargs):
        self._logger.warning(msg, **kwargs)

    def _log_exception(self, msg, **kwargs):
        self._logger.exception(msg, **kwargs)

    @property
    def is_running(self):
        """bool: Returns True if control thread is running."""
        return self._thread is not None and self._thread.is_alive()

    @property
    def state_cache(self):
        """dict: Get dictionary with last cached state."""
        if self.is_running:
            return self._state_cache
        else:
            return self._empty_state_cache

    @property
    def data_folder(self):
        """pathlib.Path: Get or set the path for data saving. Will create folder if not existing.
        """
        return self._data_folder

    @data_folder.setter
    def data_folder(self, path):
        assert isinstance(path, Path), 'Must be pathlib.Path object'
        self._logger.debug('Got set data folder with: '+str(path))
        if path.is_file():
            path = path.parent
        if not path.is_dir():
            path.mkdir(parents=True)
        self._data_folder = path
        self._logger.debug('Set data folder to: '+str(self._data_folder))

    @property
    def debug_folder(self):
        """pathlib.Path: Get or set the path for debug logging. Will create folder if not existing.
        """
        return self._debug_folder

    @debug_folder.setter
    def debug_folder(self, path):
        # Do not do logging in here! This will be called before the logger is set up
        assert isinstance(path, Path), 'Must be pathlib.Path object'
        if path.is_file():
            path = path.parent
        if not path.is_dir():
            path.mkdir(parents=True)
        self._debug_folder = path

    @property
    def name(self):
        """str: Get or set a name for the thread. Default 'ControlLoopThread'."""
        return self._name

    @name.setter
    def name(self, name):
        self._log_debug('Set name called with: '+str(name))
        assert isinstance(name, str), 'Name must be a string'
        self._name = name
        self._log_debug('Name set to: ' + str(self._name))

    @property
    def available_properties(self):
        """tuple of str: Get the available tracking parameters (e.g. gains)."""
        return ('max_frequency', 'reset_integral_if_saturated', 'integral_max_add',
                'integral_max_subtract', 'integral_min_rate', 'OL_P', 'OL_I', 'OL_speed_limit',
                'CCL_enable', 'CCL_P', 'CCL_I', 'CCL_speed_limit', 'CCL_transition_th',
                'FCL_enable', 'FCL_P', 'FCL_I', 'FCL_speed_limit', 'FCL_transition_th',
                'CTFSP_enable', 'CTFSP_spacing', 'CTFSP_speed', 'CTFSP_max_radius',
                'CTFSP_transition_th', 'CTFSP_auto_update_CCL_goal',
                'CTFSP_auto_update_CCL_goal_th', 'CTFSP_disable_integral',
                'CTFSP_disable_after_goal_update', 'CTFSP_delay', 'CTFSP_ramp')

    @property
    def max_frequency(self):
        """float or None: Get or set the maximum frequency in hertz."""
        return 1/self._min_loop_time if self._min_loop_time > 0 else None

    @max_frequency.setter
    def max_frequency(self, hz):
        self._log_debug('Got set max freq with: ' + str(hz))
        if hz is None:
            self._min_loop_time = None
        else:
            self._min_loop_time = 1 / float(hz)
        self._log_debug('Set min loop time to: ' + str(self._min_loop_time))

    @property
    def reset_integral_if_saturated(self):
        """bool: Get or set if the integral term should be reset (zeroed) if control output is at
        the speed limit.
        """
        return self._reset_integral_if_saturated

    @reset_integral_if_saturated.setter
    def reset_integral_if_saturated(self, reset):
        self._log_debug('Reset int if saturating set got: ' + str(reset))
        self._reset_integral_if_saturated = bool(reset)
        self._log_debug('Reset int if saturating set to: ' + str(self.reset_integral_if_saturated))

    @property
    def integral_max_add(self):
        """float: Get or set the maximum error (arcseconds) allowed to be added (in magnitude) to
        the integral.
        """
        return self._int_max_add * 3600

    @integral_max_add.setter
    def integral_max_add(self, max_add):
        self._log_debug('Int max add set got: ' + str(max_add))
        self._int_max_add = float(max_add) / 3600  # deg/s internally
        self._log_debug('Int max add set to: ' + str(self.integral_max_add))

    @property
    def integral_max_subtract(self):
        """float: Get or set the maximum error (arcseconds) allowed to be subtracted (in magnitude)
        to the integral.
        """
        return self._int_max_sub * 3600

    @integral_max_subtract.setter
    def integral_max_subtract(self, max_sub):
        self._log_debug('Int max subtract set got: ' + str(max_sub))
        self._int_max_sub = float(max_sub) / 3600  # deg/s internally
        self._log_debug('Int max subtract set to: ' + str(self.integral_max_subtract))

    @property
    def integral_min_rate(self):
        """float: Get or set the minimum target rate (arcsec per second) required to enable the
        integral term.
        """
        return self._int_min_rate * 3600  # Deg/s internally

    @integral_min_rate.setter
    def integral_min_rate(self, min_rate):
        self._log_debug('Int min rate set got: ' + str(min_rate))
        self._int_min_rate = float(min_rate) / 3600
        self._log_debug('Int max add set to: ' + str(self.integral_min_rate))

    @property
    def OL_P(self):
        """float: Get or set Open-Loop proportional gain."""
        return self._OL_Kp

    @OL_P.setter
    def OL_P(self, gain):
        self._log_debug('OL P Gain set got: ' + str(gain))
        self._OL_Kp = float(gain)
        self._log_debug('OL P Gain set to: ' + str(self.OL_P))

    @property
    def OL_I(self):
        """float: Get or set Open-Loop integral time (1/gain) in seconds."""
        return 1 / self._OL_Ki  # Gain internally

    @OL_I.setter
    def OL_I(self, time):
        self._log_debug('OL I time set got: ' + str(time))
        self._OL_Ki = 1 / float(time)  # Gain internally
        self._log_debug('OL I time set to: ' + str(self.OL_I))

    @property
    def OL_speed_limit(self):
        """float: Get or set Open-Loop speed limit (arcsec per second)."""
        return self._OL_speed_limit * 3600  # deg/sec internally

    @OL_speed_limit.setter
    def OL_speed_limit(self, gain):
        self._log_debug('OL speed set got: ' + str(gain))
        self._OL_speed_limit = float(gain) / 3600  # deg/sec internally
        self._log_debug('OL speed set to: ' + str(self.OL_speed_limit))

    @property
    def OL_goal_x_y(self):
        """tuple of float: Get or set the open-loop goal (arcseconds)."""
        return tuple(self._OL_goal_x_y * 3600)

    @OL_goal_x_y.setter
    def OL_goal_x_y(self, goal):
        self._log_debug('OL goal set got: ' + str(goal))
        self._OL_goal_x_y = np.array(goal) / 3600  # deg internally
        self._log_debug('OL goal set to: ' + str(self.OL_goal_x_y))

    @property
    def OL_goal_offset_x_y(self):
        """tuple of float: Get or set the open-loop goal offset (arcseconds)."""
        return tuple(self._OL_goal_offset_x_y * 3600)

    @OL_goal_offset_x_y.setter
    def OL_goal_offset_x_y(self, offset):
        self._log_debug('OL goal offset set got: ' + str(offset))
        self._OL_goal_offset_x_y = np.array(offset) / 3600  # deg internally
        self._log_debug('OL goal offset set to: ' + str(self.OL_goal_offset_x_y))

    @property
    def CCL_enable(self):
        """bool: Get or set if Coarse Closed-Loop is allowed."""
        return self._CCL_enable

    @CCL_enable.setter
    def CCL_enable(self, enable):
        self._log_debug('CCL enable set got: ' + str(enable))
        self._CCL_enable = bool(enable)
        self._log_debug('CCL enable set to: ' + str(self.CCL_enable))

    @property
    def CCL_P(self):
        """float: Get or set Coarse Closed-Loop proportional gain."""
        return self._CCL_Kp

    @CCL_P.setter
    def CCL_P(self, gain):
        self._log_debug('CCL P Gain set got: ' + str(gain))
        self._CCL_Kp = float(gain)
        self._log_debug('CCL P Gain set to: ' + str(self.CCL_P))

    @property
    def CCL_I(self):
        """float: Get or set Coarse Closed-Loop integral time (1/gain) in seconds."""
        return 1/self._CCL_Ki  # Gain internally

    @CCL_I.setter
    def CCL_I(self, time):
        self._log_debug('CCL P time set got: ' + str(time))
        self._CCL_Ki = 1 / float(time)  # Gain internally
        self._log_debug('CCL P time set to: ' + str(self.CCL_I))

    @property
    def CCL_speed_limit(self):
        """float: Get or set Coarse Closed-Loop speed limit (arcsec per second)."""
        return self._CCL_speed_limit * 3600  # deg/sec internally

    @CCL_speed_limit.setter
    def CCL_speed_limit(self, gain):
        self._log_debug('CCL speed set got: ' + str(gain))
        self._CCL_speed_limit = float(gain) / 3600  # deg/sec internally
        self._log_debug('CCL speed set to: ' + str(self.CCL_speed_limit))

    @property
    def CCL_transition_th(self):
        """float: Get or set the coarse track SD required to transition OL to CCL (arcsec)."""
        return self._CCL_transition_th

    @CCL_transition_th.setter
    def CCL_transition_th(self, th):
        self._log_debug('CCL threshold set got: ' + str(th))
        self._CCL_transition_th = float(th)
        self._log_debug('CCL threshold set to: ' + str(self.CCL_transition_th))

    @property
    def FCL_enable(self):
        """bool: Get or set if Fine Closed-Loop is allowed."""
        return self._FCL_enable

    @FCL_enable.setter
    def FCL_enable(self, enable):
        self._log_debug('FCL enable set got: ' + str(enable))
        self._FCL_enable = bool(enable)
        self._log_debug('FCL enable set to: ' + str(self.FCL_enable))

    @property
    def FCL_P(self):
        """float: Get or set Fine Closed-Loop proportional gain."""
        return self._FCL_Kp

    @FCL_P.setter
    def FCL_P(self, gain):
        self._log_debug('FCL P Gain set got: ' + str(gain))
        self._FCL_Kp = float(gain)
        self._log_debug('FCL P Gain set to: ' + str(self.FCL_P))

    @property
    def FCL_I(self):
        """float: Get or set Fine Closed-Loop integral time (1/gain) in seconds."""
        return 1 / self._FCL_Ki  # Gain internally

    @FCL_I.setter
    def FCL_I(self, time):
        self._log_debug('FCL P time set got: ' + str(time))
        self._FCL_Ki = 1 / float(time)  # Gain internally
        self._log_debug('FCL P time set to: ' + str(self.FCL_I))

    @property
    def FCL_speed_limit(self):
        """float: Get or set Fine Closed-Loop speed limit (arcsec per second)."""
        return self._FCL_speed_limit * 3600  # deg/sec internally

    @FCL_speed_limit.setter
    def FCL_speed_limit(self, gain):
        self._log_debug('FCL speed set got: ' + str(gain))
        self._FCL_speed_limit = float(gain) / 3600  # deg/sec internally
        self._log_debug('FCL speed set to: ' + str(self.FCL_speed_limit))

    @property
    def FCL_transition_th(self):
        """float: Get or set the fine track SD required to transition OL to CCL (arcsec)."""
        return self._FCL_transition_th

    @FCL_transition_th.setter
    def FCL_transition_th(self, th):
        self._log_debug('FCL threshold set got: ' + str(th))
        self._FCL_transition_th = float(th)
        self._log_debug('FCL threshold set to: ' + str(self.FCL_transition_th))

    @property
    def CTFSP_enable(self):
        """bool: Get or set if Coarse-to-Fine Spiral auto-alignment is allowed. Disabled by
        default.

        It is recommended to only enable this when necessary for alignment, and then disable for
        faster acquisitions.
        """
        return self._CTFSP_enable

    @CTFSP_enable.setter
    def CTFSP_enable(self, enable):
        self._log_debug('CTFSP enable set got: ' + str(enable))
        self._CTFSP_enable = bool(enable)
        self._log_debug('CTFSP enable set to: ' + str(self.CTFSP_enable))

    @property
    def CTFSP_spacing(self):
        """float: Get or set the spacing between arms in Coarse-to-Fine Spiral auto-alignment in
        arcseconds."""
        return self._CTFSP_spacing

    @CTFSP_spacing.setter
    def CTFSP_spacing(self, spacing):
        self._log_debug('CTFSP spacing set got: ' + str(spacing))
        self._CTFSP_spacing = float(spacing)
        self._log_debug('CTFSP spacing set to: ' + str(self.CTFSP_spacing))

    @property
    def CTFSP_speed(self):
        """float: Get or set the speed in Coarse-to-Fine Spiral auto-alignment in arcsec per
        second.
        """
        return self._CTFSP_speed

    @CTFSP_speed.setter
    def CTFSP_speed(self, speed):
        self._log_debug('CTFSP speed set got: ' + str(speed))
        self._CTFSP_speed = float(speed)
        self._log_debug('CTFSP speed set to: ' + str(self.CTFSP_speed))

    @property
    def CTFSP_max_radius(self):
        """float: Get or set the maximum (reset) radius for Coarse-to-Fine Spiral auto-alignment in
        arcseconds.
        """
        return self._CTFSP_max_radius

    @CTFSP_max_radius.setter
    def CTFSP_max_radius(self, radius):
        self._log_debug('CTFSP max radius set got: ' + str(radius))
        self._CTFSP_max_radius = float(radius)
        self._log_debug('CTFSP max radius set to: ' + str(self.CTFSP_max_radius))

    @property
    def CTFSP_transition_th(self):
        """float: Get or set the CCL RMSE required to start Coarse-to-Fine Spiral auto-alignment in
        arcseconds.
        """
        return self._CTFSP_transition_th

    @CTFSP_transition_th.setter
    def CTFSP_transition_th(self, threshold):
        self._log_debug('CTFSP start threshold set got: ' + str(threshold))
        self._CTFSP_transition_th = float(threshold)
        self._log_debug('CTFSP start threshold set to: ' + str(self.CTFSP_transition_th))

    @property
    def CTFSP_auto_update_CCL_goal(self):
        """bool: Get or set if CTFSP should update the CCL goal (i.e. save the alignment)."""
        return self._CTFSP_auto_update_CCL_goal

    @CTFSP_auto_update_CCL_goal.setter
    def CTFSP_auto_update_CCL_goal(self, save):
        self._log_debug('CTFSP auto update set got: ' + str(save))
        self._CTFSP_auto_update_CCL_goal = bool(save)
        self._log_debug('CTFSP auto update set to: ' + str(self.CTFSP_auto_update_CCL_goal))

    @property
    def CTFSP_auto_update_CCL_goal_th(self):
        """float: Get or set the required FCL RMSE to auto update the CCL goal in arcseconds."""
        return self._CTFSP_auto_update_CCL_goal_th

    @CTFSP_auto_update_CCL_goal_th.setter
    def CTFSP_auto_update_CCL_goal_th(self, threshold):
        self._log_debug('CTFSP auto update threshold set got: ' + str(threshold))
        self._CTFSP_auto_update_CCL_goal_th = float(threshold)
        self._log_debug('CTFSP auto update threshold set to: '
                        + str(self.CTFSP_auto_update_CCL_goal_th))

    @property
    def CTFSP_disable_integral(self):
        """bool: Get or set if CTFSP should disable (zero) the integral term."""
        return self._CTFSP_zero_integral

    @CTFSP_disable_integral.setter
    def CTFSP_disable_integral(self, disable):
        self._log_debug('CTFSP disable int set got: ' + str(disable))
        self._CTFSP_zero_integral = bool(disable)
        self._log_debug('CTFSP disable int set to: ' + str(self.CTFSP_disable_integral))

    @property
    def CTFSP_disable_after_goal_update(self):
        """bool: Get or set if CTFSP should be automatically disabled when the alignment is
        finished.
        """
        return self._CTFSP_disable_after_goal_update

    @CTFSP_disable_after_goal_update.setter
    def CTFSP_disable_after_goal_update(self, disable):
        self._log_debug('CTFSP disable after update set got: ' + str(disable))
        self._CTFSP_disable_after_goal_update = bool(disable)
        self._log_debug('CTFSP disable after update set to: '
                        + str(self.CTFSP_disable_after_goal_update))

    @property
    def CTFSP_delay(self):
        """float: Get or set the delay from starting CTFSP mode until spiraling begins in seconds.
        """
        return self._CTFSP_delay

    @CTFSP_delay.setter
    def CTFSP_delay(self, delay):
        self._log_debug('CTFSP delay set got: ' + str(delay))
        self._CTFSP_delay = float(delay)
        self._log_debug('CTFSP delay set to: ' + str(self.CTFSP_delay))

    @property
    def CTFSP_ramp(self):
        """float: Get or set the CTFSP speed ramp up in seconds."""
        return self._CTFSP_ramp

    @CTFSP_ramp.setter
    def CTFSP_ramp(self, ramp):
        self._log_debug('CTFSP ramp set got: ' + str(ramp))
        self._CTFSP_ramp = float(ramp)
        self._log_debug('CTFSP ramp set to: ' + str(self.CTFSP_ramp))

    def stop(self):
        """Stop the thread."""
        self._log_debug('Got stop command')
        self._stop_loop = True
        if self._thread is not None and self._thread.is_alive():
            self._log_debug('Waiting for worker thread to exit')
            self._thread.join()
        self._state_cache = self._empty_state_cache
        self._log_info('Control thread stopped.')

    def start(self):
        """Starts the control loop in a background thread. Must have target and be aligned,
        located, and initialised.
        """
        self._log_debug('Got start command')
        assert self._parent.target.has_target, 'No target set'
        assert self._parent.alignment.is_aligned, 'Not aligned'
        assert self._parent.alignment.is_located, 'Not located'
        assert self._parent.is_init, 'System not initialized'
        self._stop_loop = False
        self._thread = Thread(target=self._run, name=self.name+'Worker')
        self._thread.start()
        self._log_info('Started tracking thread with name: '+self._thread.name)

    def _run(self):
        """PRIVATE: Worker method, run by calling ControlLoopThread.start()"""
        # Check if we're ready:
        if self._parent.target.start_time is None \
                or apy_time.now() > self._parent.target.start_time:
            start_time = apy_time.now()
        else:
            start_time = self._parent.target.start_time
        self._log_info('Track start time (UTC): ' + str(start_time))
        end_time = self._parent.target.end_time
        if end_time is None:
            self._log_info('No end time (tracking indefinitely)')
        else:
            self._log_info('Track end time (UTC): ' + str(end_time))
        # Create logfile
        data_filename = Path(start_time.strftime('%Y-%m-%dT%H%M%S') + '_ControlLoopThread.csv')
        data_file = self.data_folder / data_filename
        if data_file.exists():
            self._log_debug('File name clash. Iterating...')
            append = 1
            while data_file.exists():
                data_file = self.data_folder / (data_filename.stem + str(append)
                                                + data_filename.suffix)
                append += 1
            self._log_debug('Found allowable file: '+str(data_file))
        with open(data_file, 'w') as file:
            writer = csv_write(file)

            writer.writerow(['T_UTC_ISO', 'T_ELAPSED', 'INDEX', 'MODE', 'TARG_MNT_ALT',
                             'TARG_MNT_AZ', 'TARG_MNT_RATE_ALT', 'TARG_MNT_RATE_AZ',
                             'TARG_ENU_ALT', 'TARG_ENU_AZ', 'MOUNT_MNT_ALT', 'MOUNT_MNT_AZ',
                             'COARSE_EXIST', 'COARSE_TRACK', 'COARSE_ALT_TRACK', 'COARSE_ALT_MEAN',
                             'COARSE_AZ_TRACK', 'COARSE_AZ_MEAN', 'COARSE_SD', 'COARSE_RMSE',
                             'FINE_EXIST', 'FINE_TRACK', 'FINE_ALT_TRACK', 'FINE_ALT_MEAN',
                             'FINE_AZ_TRACK', 'FINE_AZ_MEAN', 'FINE_SD', 'FINE_RMSE', 'FB_ERR_ALT',
                             'FB_ERR_AZ', 'FB_INT_ALT', 'FB_INT_AZ', 'FB_ANGVEL_ALT',
                             'FB_ANGVEL_AZ', 'FB_SATURATED', 'FB_COMMAND_ALT', 'FB_COMMAND_AZ',
                             'FB_KP', 'FB_KI', 'FF_ALT', 'FF_AZ', 'REC_EXIST', 'REC_POWER',
                             'REC_SMOOTH'])
        # Check if we need to slew
        target_alt_az = self._parent.get_alt_az_of_target(start_time)[0]
        mount_alt_az = np.array(self._parent.mount.get_alt_az())
        difference = np.sqrt(np.sum(((target_alt_az - mount_alt_az + 180) % 360-180) ** 2))
        if difference > 1:  # If more than 1 degree off
            self._log_info('Slewing to target start position.')
            self._parent.slew_to_target(start_time)
        while start_time > apy_time.now():  # Wait to start
            self._log_info('Waiting for target to rise.')
            sleep(min(10, (start_time - apy_time.now()).sec))
        self._log_info('Starting closed loop tracking.')
        # Start the trackers
        if self._parent.coarse_track_thread is not None:
            self._parent.coarse_track_thread.start()
        if self._parent.fine_track_thread is not None:
            self._parent.fine_track_thread.start()
        if self._parent.receiver is not None:
            self._parent.receiver.start()
        mode = 'OL'
        err_alt_az = np.array([0, 0], dtype='float64')
        err_integral = np.array([0, 0], dtype='float64')
        start_timestamp = precision_timestamp()
        def seconds_since_start(): return precision_timestamp() - start_timestamp
        last_timestamp = None
        loop_utctime = start_time
        loop_index = 0
        # Only used if we do fine acquisition by spiralling
        CTFSP_t = 0.0
        CTFSP_offset = np.array([0, 0], dtype='float64')
        try:
            while not self._stop_loop and (loop_utctime < end_time
                                           if end_time is not None else True):
                # CONTROL LOOP #
                # Time info:
                loop_timestamp = seconds_since_start()
                loop_utctime = apy_time.now()  # Astropy timestamp in UTC
                dt = loop_timestamp - last_timestamp if last_timestamp is not None else 0.0
                last_timestamp = loop_timestamp
                self._log_debug('Control loop timestamp: '+str(loop_timestamp))
                self._log_debug('Actual loop dt = '+str(dt))
                # TARGET position and rate (by discrete differentiation) in the MNT frame
                step = .2
                target_itrf_xyz = self._parent. \
                    get_itrf_direction_of_target(loop_utctime + [0, step]*apy_unit.s)
                # Convert all necessary coordinate frames
                target_mnt_altaz = self._parent.alignment. \
                    get_mnt_altaz_from_itrf_xyz(target_itrf_xyz)
                target_mnt_rate = (((target_mnt_altaz[:, 1]
                                     - target_mnt_altaz[:, 0] + 180) % 360) - 180) / step
                target_mnt_altaz = target_mnt_altaz[:, 0]
                # This one we keep just for fun ;)
                target_enu_altaz = self._parent.alignment. \
                    get_enu_altaz_from_itrf_xyz(target_itrf_xyz[:, 0])
                # MOUNT position in the MNT frame
                mount_com_altaz = np.array(self._parent.mount.get_alt_az())
                mount_mnt_altaz = self._parent.alignment. \
                    get_mnt_altaz_from_com_altaz(mount_com_altaz)

                ct_exists = self._parent.coarse_track_thread is not None
                if ct_exists:
                    ct_has_track = self._parent.coarse_track_thread.has_track
                    ct_track_alt_az = self._parent.coarse_track_thread.track_alt_az
                    ct_mean_alt_az = self._parent.coarse_track_thread.mean_alt_az
                    ct_mean_abs_x_y = self._parent.coarse_track_thread.mean_x_y_absolute
                    ct_track_sd = self._parent.coarse_track_thread.track_sd
                    ct_rmse = self._parent.coarse_track_thread.rms_error
                    ct_search_rad = self._parent.coarse_track_thread.pos_search_rad
                    ct_search_pos = self._parent.coarse_track_thread.pos_search_x_y
                else:
                    ct_has_track = None
                    ct_track_alt_az = (None, None)
                    ct_mean_alt_az = (None, None)
                    ct_track_sd = None
                    ct_rmse = None
                    ct_search_rad = None
                    ct_search_pos = (None, None)
                ft_exists = self._parent.fine_track_thread is not None
                if ft_exists:
                    ft_has_track = self._parent.fine_track_thread.has_track
                    ft_track_alt_az = self._parent.fine_track_thread.track_alt_az
                    ft_mean_alt_az = self._parent.fine_track_thread.mean_alt_az
                    ft_track_sd = self._parent.fine_track_thread.track_sd
                    ft_rmse = self._parent.fine_track_thread.rms_error
                    ft_search_rad = self._parent.fine_track_thread.pos_search_rad
                    ft_search_pos = self._parent.fine_track_thread.pos_search_x_y
                else:
                    ft_has_track = None
                    ft_track_alt_az = (None, None)
                    ft_mean_alt_az = (None, None)
                    ft_track_sd = None
                    ft_rmse = None
                    ft_search_rad = None
                    ft_search_pos = (None, None)
                rec_exists = self._parent.receiver is not None
                if rec_exists:
                    rec_power_smooth = self._parent.receiver.smooth_power
                    rec_power = self._parent.receiver.instant_power
                else:
                    rec_power_smooth = None
                    rec_power = None

                # Mode switching logic
                self._log_debug('Doing mode logic. Currently in mode: ' + str(mode))
                if mode == 'OL':
                    self._log_debug('Mode switching started, currently in OL')
                    if self._CCL_enable and ct_exists:
                        self._log_debug('Evaluating CCL requrements')
                        if ct_has_track and ct_track_sd is not None:
                            self._log_debug('CCL track with SD: ' + str(ct_track_sd))
                            if ct_track_sd < self._CCL_transition_th:
                                self._log_info('Switching OL > CCL.')
                                mode = 'CCL'
                                # Rescale integral term!
                                if self._OL_Ki > 0 and self._CCL_Ki > 0:
                                    err_integral *= (self._OL_Kp * self._OL_Ki
                                                     / self._CCL_Kp / self._CCL_Ki)
                elif mode == 'CCL':
                    self._log_debug('Mode switching started, currently in CCL')
                    if not self._CCL_enable or not ct_exists or not ct_has_track:
                        self._log_debug('CCL not available')
                        self._log_info('Switching CCL > OL.')
                        mode = 'OL'
                        # Rescale integral term
                        if self._CCL_Ki > 0 and self._OL_Ki > 0:
                            err_integral *= self._CCL_Kp * self._CCL_Ki / self._OL_Kp / self._OL_Ki
                    elif self._CTFSP_enable:
                        self._log_debug('Evaluating CTFSP requirements')
                        # Make sure spiral is cleared (this should do nothing)
                        CTFSP_t = 0.0
                        CTFSP_offset = np.array([0, 0], dtype='float64')
                        self._parent.coarse_track_thread.goal_offset_x_y = [0, 0]
                        if ct_rmse is not None:
                            self._log_debug('Current CCL RMSE: ' + str(ct_rmse))
                            if ct_rmse < self._CTFSP_transition_th:
                                self._log_info('Switching CCL > CTFSP.')
                                mode = 'CTFSP'
                                # Do not need to rescale integral, will continue with CCL gains.
                    elif self._FCL_enable and ft_exists:
                        self._log_debug('Evaluating FCL requirements')
                        if ft_has_track and ft_track_sd is not None:
                            self._log_debug('FCL track with SD: ' + str(ft_track_sd))
                            if ft_track_sd < self._FCL_transition_th:
                                self._log_info('Switching CCL > FCL.')
                                mode = 'FCL'
                                # Rescale integral term
                                if self._CCL_Ki > 0 and self._FCL_Ki > 0:
                                    err_integral *= (self._CCL_Kp * self._CCL_Ki
                                                     / self._FCL_Kp / self._FCL_Ki)
                elif mode == 'CTFSP':
                    self._log_debug('Mode switching started, currently in CTFSP')
                    switched = False
                    if not ct_exists or not ct_has_track:
                        self._log_debug('CT not available')
                        self._log_info('Switching CTFSP > OL.')
                        mode = 'OL'
                        switched = True
                        # Rescale integral term
                        if self._CCL_Ki > 0 and self._OL_Ki > 0:
                            err_integral *= (self._CCL_Kp * self._CCL_Ki
                                             / self._OL_Kp / self._OL_Ki)
                    elif not self._CTFSP_enable:
                        self._log_debug('CTFSP disabled')
                        self._log_info('Switching CTFSP > CCL.')
                        mode = 'CCL'
                        switched = True
                        # Do not need to rescale integral, will continue with CCL gains.
                    elif self._FCL_enable and ft_exists:
                        self._log_debug('Evaluating FCL requirements')
                        if ft_has_track:
                            self._log_info('Switching CTFSP > FCL.')
                            mode = 'FCL'
                            switched = True
                            # Rescale integral term
                            if self._CCL_Ki > 0 and self._FCL_Ki > 0:
                                err_integral *= (self._CCL_Kp * self._CCL_Ki
                                                 / self._FCL_Kp / self._FCL_Ki)
                    elif self._CTFSP_max_radius is not None:
                        self._log_debug('Evaluating CTFSP reset condition')
                        if None not in CTFSP_offset:
                            sp_radius = np.sqrt(np.sum(CTFSP_offset**2))
                            self._log_debug('CTFSP radius: ' + str(sp_radius))
                            if sp_radius > self._CTFSP_max_radius:
                                self._log_info('Switching CTFSP > CCL.')
                                mode = 'CCL'
                                switched = True
                                # Do not need to rescale integral, will continue with CCL gains.
                    if switched:
                        # Clear spiral variables
                        CTFSP_t = 0.0
                        CTFSP_offset = np.array([0, 0], dtype='float64')
                        self._parent.coarse_track_thread.goal_offset_x_y = [0, 0]
                elif mode == 'FCL':
                    self._log_debug('Mode switching started, currently in FCL')
                    if not self._FCL_enable or not ft_exists or not ft_has_track:
                        self._log_debug('FCL not available')
                        if self._CCL_enable and ct_exists and ct_has_track:
                            self._log_info('Switching FCL > CCL.')
                            mode = 'CCL'
                            if self._FCL_Ki > 0 and self._CCL_Ki > 0:
                                err_integral *= (self._FCL_Kp * self._FCL_Ki
                                                 / self._CCL_Kp / self._CCL_Ki)
                        else:
                            self._log_info('Switching FCL > OL.')
                            mode = 'OL'
                            if self._FCL_Ki > 0 and self._OL_Ki > 0:
                                err_integral *= (self._FCL_Kp * self._FCL_Ki
                                                 / self._OL_Kp / self._OL_Ki)
                    elif self._CTFSP_enable and self._CTFSP_auto_update_CCL_goal:
                        # We are fine tracking and have spiral auto-align enabled. Check if we
                        # should update coarse tracker goal with the current (absolute) position.
                        self._log_debug('Evaluating CTFSP alignment update condition')
                        self._log_debug('Current FCL RMSE: ' + str(ft_rmse))
                        if ft_rmse is not None and ft_rmse < self._CTFSP_auto_update_CCL_goal_th:
                            self._log_debug('Should update goal!')
                            self._log_debug('CCL absolute mean: ' + str(ct_mean_abs_x_y)
                                            + ', old goal: '
                                            + str(self._parent.coarse_track_thread.goal_x_y))
                            if None in ct_mean_abs_x_y:
                                self._log_warning(
                                        'Want to update coarse goal, but have no coarse mean.')
                            else:
                                try:
                                    # Set new goal to current mean
                                    self._parent.coarse_track_thread.goal_x_y = ct_mean_abs_x_y
                                    # Remove offset
                                    self._parent.coarse_track_thread.goal_offset_x_y = [0, 0]
                                    # Zero mean estimator
                                    self._parent.coarse_track_thread.spot_tracker \
                                        .change_mean_relative(-ct_mean_abs_x_y[0],
                                                              -ct_mean_abs_x_y[1])
                                except BaseException:
                                    self._log_warning('Failed to update goal', exc_info=True)
                                self._log_info('Auto updated coarse goal to: ' + str(np.round(
                                        self._parent.coarse_track_thread.goal_x_y, 3))
                                               + '.')
                                if self._CTFSP_disable_after_goal_update:
                                    self._log_info('Disabling CTFSP.')
                                    self._CTFSP_enable = False
                else:
                    self._log_warning('Unknown tracking mode, going to open loop')
                    mode = 'OL'

                # Set gains and calculate error based on mode
                self._log_debug('Calculating error terms, in mode: '+str(mode))
                saturated = False
                if mode == 'OL':
                    fb_kp = self._OL_Kp
                    fb_ki = self._OL_Ki
                    speed_limit = self._OL_speed_limit
                    # XY to altaz flip order
                    offset = self._OL_goal_x_y[::-1] + self._OL_goal_offset_x_y[::-1]
                    offset[1] /= np.cos(np.deg2rad(np.clip(mount_mnt_altaz[0], 0, 85)))
                    err_alt_az = (((target_mnt_altaz - mount_mnt_altaz
                                    - offset + 180) % 360) - 180)
                    self._log_debug('OL error set: ' + str(err_alt_az))
                elif mode == 'CCL' or mode == 'CTFSP':
                    fb_kp = self._CCL_Kp
                    fb_ki = self._CCL_Ki
                    speed_limit = self._CCL_speed_limit
                    if None in ct_track_alt_az:
                        self._log_debug('No update from coarse tracker')
                        err_alt_az = np.array([0, 0], dtype='float64')
                    else:
                        err_alt_az = np.array(ct_track_alt_az) / 3600
                        err_alt_az[1] /= np.cos(np.deg2rad(np.clip(mount_mnt_altaz[0], -85, 85)))
                    self._log_debug('CCL/CTFSP error set: ' + str(err_alt_az))
                elif mode == 'FCL':
                    fb_kp = self._FCL_Kp
                    fb_ki = self._FCL_Ki
                    speed_limit = self._FCL_speed_limit
                    if None in ft_track_alt_az:
                        self._log_debug('No update from fine tracker')
                        err_alt_az = np.array([0, 0], dtype='float64')
                    else:
                        err_alt_az = np.array(ft_track_alt_az) / 3600
                        err_alt_az[1] /= np.cos(np.deg2rad(np.clip(mount_mnt_altaz[0], -85, 85)))
                    self._log_debug('FCL error set: ' + str(err_alt_az))
                else:
                    # We're in an undefined tracking mode. Bad times...
                    self._log_warning('In an undefined tracking mode, going to open-loop')
                    mode = 'OL'
                    err_alt_az = np.array([0, 0], dtype='float64')
                    fb_kp = 0.0
                    fb_ki = 0.0

                # Update integral term, both check if it should be cleared and limit the max add
                if (not fb_ki) or (mode == 'CTFSP' and self._CTFSP_zero_integral) \
                        or (self._int_min_rate ** 2 > np.sum(target_mnt_rate ** 2)):
                    err_integral = np.array([0, 0], dtype='float64')
                    self._log_debug('Cleared integral')
                else:
                    # Limit how much (away from zero) can be added
                    add_integral = np.where(err_alt_az * err_integral > 0,  # Same sign
                                            err_alt_az.clip(-self._int_max_add,
                                                            self._int_max_add) * dt,
                                            err_alt_az.clip(-self._int_max_sub,
                                                            self._int_max_sub) * dt)
                    err_integral += add_integral
                    self._log_debug('Integral calculated to: ' + str(err_integral))
                # Calculate correction term
                angvel_correction = fb_kp * (err_alt_az + fb_ki * err_integral)
                (angvel_correction, saturated) = self._clip_feedback_rates(angvel_correction,
                                                                           speed_limit)
                self._log_debug('FB calculated: ' + str(angvel_correction) + ', saturated: '
                                + str(saturated))
                # Check if we are in spiral mode, then we should feed-forward the spiral rates
                if mode == 'CTFSP':
                    CTFSP_t += dt
                    # Spiral pos is in azi alt
                    new_spiral_pos = np.array(self._get_spiral_goal(
                            CTFSP_t, spacing=self._CTFSP_spacing, speed=self._CTFSP_speed,
                            ramp=self._CTFSP_ramp, delay=self._CTFSP_delay))
                    spiral_angvel = (new_spiral_pos - CTFSP_offset) / dt / 3600
                    CTFSP_offset = new_spiral_pos
                    self._log_debug('CTFSP t: '+str(CTFSP_t))
                    self._log_debug('CTFSP offs: '+str(CTFSP_offset))
                    self._log_debug('STFSP angvel: '+str(spiral_angvel))
                    angvel_correction[0] -= spiral_angvel[1]
                    angvel_correction[1] -= (spiral_angvel[0] / np.cos(
                            np.deg2rad(np.clip(mount_mnt_altaz[0], 0, 85))))
                    rotation = self._parent.coarse_camera.rotation
                    rotmx = np.array([[np.cos(np.deg2rad(rotation)),
                                       - np.sin(np.deg2rad(rotation))],
                                      [np.sin(np.deg2rad(rotation)),
                                       np.cos(np.deg2rad(rotation))]])
                    rot_offset = rotmx @ CTFSP_offset
                    self._parent.coarse_track_thread.goal_offset_x_y = list(rot_offset)
                    self._log_debug('FB updated: ' + str(angvel_correction))
                    self._log_debug('Offset set: ' + str(rot_offset))
                # Calculate total rates
                angvel_total = self._get_safe_rates(angvel_correction + target_mnt_rate,
                                                    mount_mnt_altaz)
                self._log_debug('Sending rates: ' + str(angvel_total))
                # Check post-calc clearing conditions
                if saturated and self._reset_integral_if_saturated:
                    err_integral = np.array([0, 0], dtype='float64')
                    self._log_debug('Cleared integral')

                # Command the mount
                self._parent.mount.set_rate_alt_az(*angvel_total)
                # Update feed-forward terms
                ff_alt, ff_azi = (angvel_total - target_mnt_rate) * 3600
                ff_azi *= np.cos(np.deg2rad(mount_mnt_altaz[0]))
                if ct_exists:
                    ff_x_y = np.array([-ff_azi, -ff_alt])
                    rotation = self._parent.coarse_camera.rotation
                    rotmx = np.array([[np.cos(np.deg2rad(rotation)),
                                       - np.sin(np.deg2rad(rotation))],
                                      [np.sin(np.deg2rad(rotation)),
                                       np.cos(np.deg2rad(rotation))]])
                    ff_x_y = rotmx @ ff_x_y
                    self._parent.coarse_track_thread.feedforward_rate = list(ff_x_y)
                if ft_exists:
                    ff_x_y = np.array([-ff_azi, -ff_alt])
                    rotation = self._parent.fine_camera.rotation
                    rotmx = np.array([[np.cos(np.deg2rad(rotation)),
                                       - np.sin(np.deg2rad(rotation))],
                                      [np.sin(np.deg2rad(rotation)),
                                       np.cos(np.deg2rad(rotation))]])
                    ff_x_y = rotmx @ ff_x_y
                    self._parent.fine_track_thread.feedforward_rate = list(ff_x_y)

                # Write state to cache
                self._state_cache = {'mode': mode,
                                     'freq': 1 / dt if dt > 0 else None,
                                     'fb_alt': angvel_correction[0] * 3600,
                                     'fb_azi': angvel_correction[1] * 3600,
                                     'err_azi': err_alt_az[1] * 3600,
                                     'err_alt': err_alt_az[0] * 3600,
                                     'int_azi': err_integral[1] * 3600,
                                     'int_alt': err_integral[0] * 3600,
                                     'ct_has_track': ct_has_track,
                                     'ct_track_alt': ct_track_alt_az[0],
                                     'ct_mean_alt': ct_mean_alt_az[0],
                                     'ct_track_azi': ct_track_alt_az[1],
                                     'ct_mean_azi': ct_mean_alt_az[1],
                                     'ct_track_sd': ct_track_sd,
                                     'ct_rmse': ct_rmse,
                                     'ct_search_rad': ct_search_rad,
                                     'ct_search_x': ct_search_pos[0],
                                     'ct_search_y': ct_search_pos[1],
                                     'ft_has_track': ft_has_track,
                                     'ft_track_alt': ft_track_alt_az[0],
                                     'ft_mean_alt': ft_mean_alt_az[0],
                                     'ft_track_azi': ft_track_alt_az[1],
                                     'ft_mean_azi': ft_mean_alt_az[1],
                                     'ft_track_sd': ft_track_sd,
                                     'ft_rmse': ft_rmse,
                                     'ft_search_rad': ft_search_rad,
                                     'ft_search_x': ft_search_pos[0],
                                     'ft_search_y': ft_search_pos[1]}
                # Save to file
                with open(data_file, 'a') as file:
                    writer = csv_write(file)

                    writer.writerow([loop_utctime.isot, loop_timestamp, loop_index, mode,
                                     target_mnt_altaz[0], target_mnt_altaz[1], target_mnt_rate[0],
                                     target_mnt_rate[1], target_enu_altaz[0], target_enu_altaz[1],
                                     mount_mnt_altaz[0], mount_mnt_altaz[1], ct_exists,
                                     ct_has_track, ct_track_alt_az[0], ct_mean_alt_az[0],
                                     ct_track_alt_az[1], ct_mean_alt_az[1], ct_track_sd, ct_rmse,
                                     ft_exists, ft_has_track, ft_track_alt_az[0],
                                     ft_mean_alt_az[0], ft_track_alt_az[1], ft_mean_alt_az[1],
                                     ft_track_sd, ft_rmse, err_alt_az[0], err_alt_az[1],
                                     err_integral[0], err_integral[1], angvel_correction[0],
                                     angvel_correction[1], saturated, angvel_total[0],
                                     angvel_total[1], fb_kp, fb_ki, ff_alt, ff_azi, rec_exists,
                                     rec_power, rec_power_smooth])
                # Update loop
                loop_index += 1
                if self._min_loop_time is not None:
                    time_to_next = (loop_timestamp + self._min_loop_time) - seconds_since_start()
                else:
                    time_to_next = 0
                if time_to_next > EPS:
                    self._log_debug('Sleeping for (ms): ' + str(np.round(time_to_next*1000)))
                    sleep(time_to_next)
        except BaseException:
            self._log_debug('Was interrupted, stopping!', exc_info=True)
            if self._parent.coarse_track_thread is not None:
                self._parent.coarse_track_thread.stop()
            if self._parent.fine_track_thread is not None:
                self._parent.fine_track_thread.stop()
            if self._parent.receiver is not None and self._parent.receiver.is_running:
                self._parent.receiver.stop()
            self._parent.mount.stop()
            self.stop()
            self._parent.mount.stop()
            raise

        self._parent.mount.stop()
        if self._parent.coarse_track_thread is not None:
            self._parent.coarse_track_thread.stop()
        if self._parent.fine_track_thread is not None:
            self._parent.fine_track_thread.stop()
        if self._parent.receiver is not None and self._parent.receiver.is_running:
            self._parent.receiver.stop()
        self._log_info('Tracking ended')

    def _get_safe_rates(self, desired_rates, curr_alt_az=None):
        """PRIVATE: Limit the desired rates to safe values such that the maximum speed and position
        of the Mount is not violated.

        Args:
            desired_rates (numpy.ndarray, tuple, list): length 2 array with desired altitude and
                azimuth rates in degrees per second.
            curr_alt_az (numpy.ndarray, tuple, list, optional): length 2 array with current
                telescope altitude and azimuth angles in degrees. If None (the default) *no
                position checking will be done*.

        Returns:
            numpy.ndarray: length 2 array with desired rates clipped to respect maximum rate and
                position.
        """
        rate_limit = self._parent.mount.max_rate
        rlim_alt = [-rate_limit[0], rate_limit[0]]
        rlim_azi = [-rate_limit[1], rate_limit[1]]
        if curr_alt_az is not None:  # Check alt-azi limit cases
            alt_lim = self._parent.mount.alt_limit
            azi_lim = self._parent.mount.azi_limit
            if alt_lim[0] is not None and curr_alt_az[0] <= alt_lim[0]:
                rlim_alt[0] = 0
            if alt_lim[1] is not None and curr_alt_az[0] >= alt_lim[1]:
                rlim_alt[1] = 0
            if azi_lim[0] is not None and curr_alt_az[1] <= azi_lim[0]:
                rlim_azi[0] = 0
            if azi_lim[1] is not None and curr_alt_az[1] >= azi_lim[1]:
                rlim_azi[1] = 0
        return np.array([np.clip(desired_rates[0], *rlim_alt),
                         np.clip(desired_rates[1], *rlim_azi)])

    @staticmethod
    def _get_spiral_goal(t, spacing=100, speed=100, ramp=5, delay=0):
        """Get spiral goal for times t.

        Args:
            t (numpy.ndarray): array of time points (seconds) to calculate spiral position for.
            spacing (float, optional): spacing between arms in arcseconds. Default 100.
            speed (float, optional): speed in arcseconds per second to traverse spiral. Default
                100.
            ramp (float, optional): Time constant in seconds for smooth start
                (by (1 - e**-t/ramp)).
        """
        f = lambda t: (1 - np.exp(- t / ramp)) * t
        t = np.where(t < delay, 0, t - delay)
        x = (np.sqrt(spacing / np.pi * speed * f(t))
             * np.cos(2 * np.sqrt(np.pi / spacing * speed * f(t))))
        y = (np.sqrt(spacing / np.pi * speed * f(t))
             * np.sin(2 * np.sqrt(np.pi / spacing * speed * f(t))))
        return np.vstack((x, y)).squeeze()

    @staticmethod
    def _clip_feedback_rates(desired_rate, speed_limit):
        """Limit the desired feedback rate to the saturation limit. Returns limited rates and
        saturation flag.

        Args:
            desired_rate (numpy.ndarray, tuple, list): Desired rate.
            speed_limit (float): Rate limit in same units as desired_rate. If None no speed limit
                is applied.

        Returns:
            numpy.ndarray: Rate limited to the speed_limit.
            bool: True if the rate was changed, False otherwise.
        """
        if speed_limit is None:
            return (desired_rate, False)
        else:
            clipped_corr = desired_rate
            did_clip = False
            if np.abs(clipped_corr[0]) > speed_limit:
                clipped_corr[0] = np.clip(clipped_corr[0], -speed_limit, speed_limit)
                did_clip = True
            if np.abs(clipped_corr[1]) > speed_limit:
                clipped_corr[1] = np.clip(clipped_corr[1], -speed_limit, speed_limit)
                did_clip = True
            return (clipped_corr, did_clip)


class TrackingThread:
    """Run a thread to track a point on a camera.

    This thread should be given a pypogs.Camera object and a pypogs.SpotTracker object. An empty
    SpotTracker will be created and attached if not given as input. The camera acquisition
    properties are set directly in the Camera and the spot detection parameters are set directly in
    the SpotTracker. They may be accessed (and set) by TrackingThread.camera and
    TrackingThread.spot_tracker respectively. If an image_folder is given to the TrackingThread the
    images read are saved (as .tiff) to the path, the maximum saving frequency can be limited by
    setting img_save_frequency (default 1 Hz) to avoid saving massive amounts of data.

    The TrackingThread will update on every image event received from the Camera, so the frequency
    is controlled by the Camera.frame_rate. If the thread is already busy, the image will be
    ignored and a frame drop warning logged.

    After setting up and starting the thread, read TrackingThread.track_alt_az to get the last
    measured position (error from the goal) of the satellite being tracked. See note below for
    coordinate definition. The tracker keeps an estimate of the mean and standard deviation of
    position, signal sum, and signal area which may be used to keep the track. Please read the
    SpotTracker (in this module) documentation carefully to set up the spot detection and tracking
    parameters.

    The TrackingThread can also pass feed-forward information to the SpotTracker to compensate for
    the movement of the telescope on the position of the target. The SpotTracker mean position will
    be moved at a speed of TrackingThread.feedforward_rate (tuple of floats; arcsec per second) but
    only if the step change is greater than TrackingThread.feedforward_threshold (default 10
    arcsec). It is also possible to move the SpotTracker goal offset by a speed defined by
    TrackingThread.goal_offset_rate (arcsec per second) to follow a moving goal offset (i.e. during
    spiralling).

    Note:
        The TrackingThread has two distinct coordinate systems. The alt_az system measures
        positions relative to the defined goal and is (1) derotated with the Camera.rotation
        parameter and (2) switched (i.e. x and y becomes az and alt respectively) in order to
        coincide with the control loop's alt_az system compared to the SpotTracker x and y
        positions. However, many parameters (e.g. the goal and anything appended by _absolute) are
        in the same x_y coordinates of the SpotTracker *without any derotation*. All distance
        measurements are scaled by the Camera.plate_scale such that the outputs are directly in
        arcseconds.

    Args:
        camera (pypogs.Camera, optional): The Camera object to read from.
        spot_tracker (pypogs.SpotTracker, optional): The SpotTracker used to detect and track from
            the images. If this is not supplied an empty SpotTracker will be created.
        name (str, optional): A name for the TrackingThread.
        image_folder (pathlib.Path, optional): The folder for data saving. If None (the default)
            images will not be saved.
        img_save_frequency (float, optional): Maximum frequency for saving images. If None every
            image will be saved (this may use a lot of disk space and computer resources).
        data_folder (pathlib.Path, optional): The folder for data saving. If None (the default) the
            folder *pypogs*/data will be used/created.
        debug_folder (pathlib.Path, optional): The folder for debug logging. If None (the default)
            the folder *pypogs*/debug will be used/created.

    Example:
        ::

            # Create a camera instance (see pypogs.camera.Camera)
            cam = pypogs.Camera(model='ptgrey', identity='18285284', name='CoarseCam')
            # Create a TrackingThread instance
            tt = pypogs.TrackingThread(camera=cam, name='CoarseTrackThread')
            # Set up tracking parameters (see SpotTracker in this module for details)
            tt.spot_tracker.max_search_radius = 500
            tt.spot_tracker.min_search_radius = 100
            tt.spot_tracker.position_sigma = 5
            # (Optional) set up a directory for image saving at .5 Hz
            tt.image_folder = Path('./tracking_images')
            tt.img_save_frequency = .5
            # Start the tracker
            tt.start()
            # Wait for a while
            time.sleep(2)
            # Read the position
            print(tt.track_alt_az)
            # Stop the tracker
            tt.stop()
            # Deinitialise the camera
            cam.deinitialize()
    """
    def __init__(self, camera=None, spot_tracker=None, name=None, image_folder=None,
                 img_save_frequency=1.0, data_folder=None, debug_folder=None):
        """Create TrackingThread instance. See class documentation."""
        # Logger setup
        self._debug_folder = None
        if debug_folder is None:
            self.debug_folder = Path(__file__).parent / 'debug'
        else:
            self.debug_folder = debug_folder
        self._logger = logging.getLogger('pypogs.tracking.TrackingThread')
        if not self._logger.hasHandlers():
            # Add new handlers to the logger if there are none
            self._logger.setLevel(logging.DEBUG)
            # Console handler at INFO level
            ch = logging.StreamHandler()
            ch.setLevel(logging.INFO)
            # File handler at DEBUG level
            fh = logging.FileHandler(self.debug_folder / 'pypogs.txt')
            fh.setLevel(logging.DEBUG)
            # Format and add
            formatter = logging.Formatter('%(asctime)s:%(name)s-%(levelname)s: %(message)s')
            fh.setFormatter(formatter)
            ch.setFormatter(formatter)
            self._logger.addHandler(fh)
            self._logger.addHandler(ch)

        # Start of constructor
        self._logger.debug('TrackingThread called with camera=' + str(camera) + ' spot_tracker='
                           + str(spot_tracker) + ' name=' + str(name) + ' image_folder='
                           + str(image_folder) + ' img_save_frequency=' + str(img_save_frequency)
                           + ' data_folder=' + str(data_folder))
        super().__init__()
        # Data folder setup
        self._data_folder = None
        if data_folder is None:
            self.data_folder = Path(__file__).parent / 'data'
        else:
            self.data_folder = data_folder
        self._thread = None
        self._camera = None
        self._spot_tracker = None
        self._name = 'UnnamedTrackingThread'
        self._freq_target = None
        self._freq_actual = None
        self._image_folder = None
        self._img_save_frequency = None
        self._stop_running = True
        self._last_processed_img = None
        # Callbacks on image event
        self._process_image = Event()
        self._image_data = None
        self._image_timestamp = None
        # Feedforward terms
        self._feedforward_threshold = 10  # arcsec/sec minimum rate to do feed-forward
        self._feedforward_rate = (0, 0)  # How much the telescope is correcting
        self._goal_offset_rate = (0, 0)  # How much the goal should move

        if camera is not None:
            self.camera = camera
        if name is not None:
            self.name = name
        if spot_tracker is not None:
            self.spot_tracker = spot_tracker
        else:
            if name is not None:
                self.spot_tracker = SpotTracker(name=name)
            else:
                self.spot_tracker = SpotTracker()
        if image_folder is not None:
            self.image_folder = image_folder
        if img_save_frequency is not None:
            self.img_save_frequency = img_save_frequency
        self._logger.info('TrackingThread instance created with name: ' + self.name + '.')

    def _log_debug(self, msg, **kwargs):
        self._logger.debug(self.name + ': ' + msg, **kwargs)

    def _log_info(self, msg, **kwargs):
        self._logger.info(self.name + ': ' + msg, **kwargs)

    def _log_warning(self, msg, **kwargs):
        self._logger.warning(self.name + ': ' + msg, **kwargs)

    def _log_exception(self, msg, **kwargs):
        self._logger.exception(self.name + ': ' + msg, **kwargs)

    @property
    def data_folder(self):
        """pathlib.Path: Get or set the path for data saving. Will create folder if not existing.
        """
        return self._data_folder

    @data_folder.setter
    def data_folder(self, path):
        assert isinstance(path, Path), 'Must be pathlib.Path object'
        self._logger.debug('Got set data folder with: '+str(path))
        if path.is_file():
            path = path.parent
        if not path.is_dir():
            path.mkdir(parents=True)
        self._data_folder = path
        self._logger.debug('Set data folder to: '+str(self._data_folder))

    @property
    def debug_folder(self):
        """pathlib.Path: Get or set the path for debug logging. Will create folder if not existing.
        """
        return self._debug_folder

    @debug_folder.setter
    def debug_folder(self, path):
        # Do not do logging in here! This will be called before the logger is set up
        assert isinstance(path, Path), 'Must be pathlib.Path object'
        if path.is_file():
            path = path.parent
        if not path.is_dir():
            path.mkdir(parents=True)
        self._debug_folder = path

    @property
    def image_folder(self):
        """pathlib.Path: Get or set the path for saving images. Will create folder if not existing.

        The saving frequency can be limited by setting TrackingThread.img_save_frequency
        """
        return self._image_folder

    @image_folder.setter
    def image_folder(self, path):
        self._log_debug('Set img path called with: '+str(path))
        assert not self.is_running, 'Can not change path while running'
        if path is None:
            self._image_folder = None
        else:
            assert isinstance(path, Path), 'Path must be pathlib.Path or None'
            if path.is_file():
                path = path.parent
            if not path.is_dir():
                path.mkdir(parents=True)
            self._image_folder = path
        self._log_debug('Img save path set to: '+str(self._image_folder))

    def _run(self):
        """PRIVATE: Worker function for the thread to run."""
        self._logger.debug('Worker thread started: ' + self._thread.name)
        if not self._camera.is_running:
            self._camera.start()
        self.spot_tracker.clear_tracker()
        start_datetime = datetime.utcnow()
        # Create logfile
        data_filename = Path(start_datetime.strftime('%Y-%m-%dT%H%M%S') + '_' + self.name
                             + '_TrackingThread.csv')
        data_file = self.data_folder / data_filename
        if data_file.exists():
            self._log_debug('File name clash. Iterating...')
            append = 1
            while data_file.exists():
                data_file = self.data_folder / (data_filename.stem + str(append)
                                                + data_filename.suffix)
                append += 1
            self._log_debug('Found allowable file: '+str(data_file))
        with open(data_file, 'w') as file:
            writer = csv_write(file)
            writer.writerow(['T_UTC_ISO', 'T_ELAPSED', 'INDEX', 'IMG_SAVED', 'IMG_USED',
                             'HAS_TRACK', 'RMSE', 'X_TRACK', 'X_MEAN', 'Y_TRACK', 'Y_MEAN',
                             'TRACK_SD', 'SUM_TRACK', 'SUM_MEAN', 'SUM_SD', 'AREA_TRACK',
                             'AREA_MEAN', 'AREA_SD', 'FF_RATE_X', 'FF_RATE_Y', 'FF_STEP_X',
                             'FF_STEP_Y', 'OFFS_RATE_X', 'OFFS_RATE_Y', 'OFFS_STEP_X',
                             'OFFS_STEP_Y'])
        start_timestamp = precision_timestamp()
        old_timestamp = start_timestamp - 1
        last_img_timestamp = None
        loop_index = 0
        try:
            while not self._stop_running:
                # Synchronisation and time management
                if not self._process_image.wait(timeout=3):
                    self._log_warning('Timeout waiting for image in loop')
                image = self._image_data.copy()
                loop_timestamp = precision_timestamp() - start_timestamp
                loop_datetime_utc = self._image_timestamp
                self._log_debug('New loop. Timestamp: '+str(loop_timestamp))
                dt = loop_timestamp - old_timestamp
                old_timestamp = loop_timestamp
                self._actual_freq = 1/dt
                # Feedforward
                ff_step_abs = np.sqrt(np.sum(np.array(self._feedforward_rate)**2))
                self._log_debug('Feedforward step magnitude: ' + str(ff_step_abs))
                if ff_step_abs > self._feedforward_threshold:
                    ff_step = np.array(self._feedforward_rate) * dt
                    self._log_debug('Adding step to mean: (x,y): ' + str(ff_step))
                    self.spot_tracker.change_mean_relative(*ff_step)
                else:
                    ff_step = np.array([0, 0])
                # Update the goal offset
                offs_step = np.array(self._goal_offset_rate) * dt
                self._log_debug('Adding step to goal offset: (x,y): ' + str(offs_step))
                self.spot_tracker.goal_offset_x_y += offs_step
                # Update spottracker from image
                try:
                    img_used = self.spot_tracker.update_from_image(image, self._camera.plate_scale)
                except BaseException:
                    self._log_exception('Failed to process image')
                    raise

                # Saving images
                img_saved = False
                if self._image_folder is not None:
                    # Do we have frequency limit?
                    self._log_debug('Have image save path, checking rate limit, last timestamp:'
                                    + str(last_img_timestamp))
                    if (self._img_save_frequency is None or last_img_timestamp is None
                            or (loop_timestamp-last_img_timestamp) > 1 / self._img_save_frequency):
                        last_img_timestamp = loop_timestamp
                        imgname = Path(start_datetime.strftime('%Y-%m-%dT%H%M%S') + '_'
                                       + self.name + '_img_' + str(loop_index) + '.tiff')
                        self._log_debug('Saving image to disk as name: ' + str(imgname))
                        tiff_write(self._image_folder / imgname, image)
                        img_saved = True
                # Get state
                has_track = self.spot_tracker.has_track
                rmse = self.spot_tracker.rms_error
                (x, y) = self.spot_tracker.track_x_y
                (mx, my) = self.spot_tracker.mean_x_y
                sd = self.spot_tracker.track_sd
                s = self.spot_tracker.track_sum
                ms = self.spot_tracker.mean_sum
                ss = self.spot_tracker.sd_sum
                a = self.spot_tracker.track_area
                ma = self.spot_tracker.mean_area
                sa = self.spot_tracker.sd_area
                ff_rate = self._feedforward_rate
                offs_rate = self._goal_offset_rate
                # Saving log
                with open(data_file, 'a') as file:
                    writer = csv_write(file)
                    writer.writerow([loop_datetime_utc.isoformat(), loop_timestamp, loop_index,
                                     int(img_saved), int(img_used), int(has_track), rmse, x, mx,
                                     y, my, sd, s, ms, ss, a, ma, sa, ff_rate[0], ff_rate[1],
                                     ff_step[0], ff_step[1], offs_rate[0], offs_rate[1],
                                     offs_step[0], offs_step[1]])
                loop_index += 1
                self._process_image.clear()  # Clear processing flag, used to sync
        except BaseException:
            self._log_debug('Worker loop threw exception', exc_info=True)
            try:
                self.stop()
            except BaseException:
                self._log_debug('Failed to stop when worker thread threw exception', exc_info=True)
            raise

    def stop(self):
        """Stop the thread."""
        self._log_debug('Got stop command')
        self._stop_running = True
        if self._thread is not None and self._thread.is_alive():
            self._log_debug('Waiting for worker thread to exit')
            self._thread.join()
        try:
            self._log_debug('Stopping camera')
            self._camera.stop()
        except BaseException:
            self._log_debug('Failed to stop camera, continuing...', exc_info=True)
        self.spot_tracker.clear_tracker()
        self._log_info('Tracking thread stopped with name: '+str(self.name))

    def start(self):
        """Starts the tracking in a background thread. Must have Camera and SpotTracker."""
        self._log_debug('Got start command')
        assert None not in (self.camera, self.spot_tracker), \
            'Must define a Camera and SpotTracker before starting'
        assert self.camera.is_init, 'Camera must be initialised before starting'
        self._stop_running = False
        self._process_image.clear()
        self._thread = Thread(target=self._run, name=self.name+'Worker')
        self._thread.start()
        self._log_info('Started worker thread.')

    @property
    def is_running(self):
        """bool: Returns True if the thread is running."""
        alive = self._thread is not None and self._thread.is_alive()
        return alive

    @property
    def frequency(self):
        """float: Get or set the target loop frequency in Hz. Must be greater than zero."""
        raise DeprecationWarning('Frequncy now controlled by the camera')

    @frequency.setter
    def frequency(self, freq_hz):
        raise DeprecationWarning('Frequncy now controlled by the camera')

    @property
    def frequency_actual(self):
        """float: Get the actual loop frequency (for the last update) in Hz."""
        assert self.is_running, 'Thread is not running'
        return self._actual_freq

    @property
    def name(self):
        """str: Get or set a name for the thread"""
        return self._name

    @name.setter
    def name(self, name):
        self._log_debug('Set name called with: '+str(name))
        assert isinstance(name, str), 'Name must be a string'
        self._name = name

    @property
    def img_save_frequency(self):
        """float: Get or set the maximum frequency to save images. If None (the default) all images
        will be saved.
        """
        return self._img_save_frequency

    @img_save_frequency.setter
    def img_save_frequency(self, freq_hz):
        self._log_debug('Set save freq called with: '+str(freq_hz))
        if freq_hz is None:
            self._img_save_frequency = None
        else:
            assert isinstance(freq_hz, (int, float)), \
                'Save frequency must be None or a number (int or float)'
            self._img_save_frequency = float(freq_hz)
        self._log_debug('Save freq set to: '+str(self.img_save_frequency))

    @property
    def camera(self):
        """pypogs.Camera: Get or set the Camera object for the tracking thread."""
        return self._camera

    @camera.setter
    def camera(self, camera):
        self._log_debug('Set camera called with: '+str(camera))
        assert not self.is_running, 'Can not set while running'
        if self.camera is not None:
            self.camera.remove_event_callback(self._on_image_event)
        if camera is not None:
            assert isinstance(camera, Camera), 'Camera must be pypogs.Camera object'
            self._camera = camera
            self._camera.add_event_callback(self._on_image_event)
        else:
            self._camera = None
        self._log_debug('Set camera to: ' + str(self.camera))

    @property
    def spot_tracker(self):
        """SpotTracker: Get or set the SpotTracker object."""
        return self._spot_tracker

    @spot_tracker.setter
    def spot_tracker(self, spot_tracker):
        self._log_debug('Set spot_tracker called with: '+str(spot_tracker))
        assert not self.is_running, 'Can not set while running'
        assert isinstance(spot_tracker, SpotTracker), 'Must be SpotTracker object'
        self._spot_tracker = spot_tracker

    @property
    def feedforward_rate(self):
        """"tuple of float: Get or set the speed (arcsec/sec) with which the SpotTracker mean
        position should be moved.
        """
        return self._feedforward_rate

    @feedforward_rate.setter
    def feedforward_rate(self, ff_rate):
        self._log_debug('Set feedforward rate got with: ' + str(ff_rate))
        if ff_rate is None:
            self._feedforward_rate = (0, 0)
            self._log_debug('FF rate set to: ' + str(self.feedforward_rate))
        else:
            try:
                ff_rate = tuple(float(x) for x in ff_rate)
                assert len(ff_rate) == 2
                self._feedforward_rate = ff_rate
                self._log_debug('FF rate set to: ' + str(self.feedforward_rate))
            except BaseException:
                self._log_warning('Failed to set FF rate from: ' + str(ff_rate), exc_info=True)

    @property
    def feedforward_threshold(self):
        """float: Get or set the minimum feedforward step (arcsec)."""
        return self._feedforward_threshold

    @feedforward_threshold.setter
    def feedforward_threshold(self, th):
        self._log_debug('Set feedforward threshold got with: ' + str(th))
        self._feedforward_threshold = float(th)
        self._log_debug('FF threshold set to: ' + str(self.feedforward_threshold))

    @property
    def goal_offset_rate(self):
        """"tuple of float: Get or set the speed (arcsec/sec) with which the SpotTracker goal
        offset position should be moved.
        """
        return self._goal_offset_rate

    @goal_offset_rate.setter
    def goal_offset_rate(self, offset_rate):
        self._log_debug('Set goal offset rate got with: ' + str(offset_rate))
        if offset_rate is None:
            self._goal_offset_rate = (0, 0)
            self._log_debug('Offset rate set to: ' + str(self.goal_offset_rate))
        else:
            try:
                offset_rate = tuple([float(x) for x in offset_rate])
                assert len(offset_rate) == 2
                self._goal_offset_rate = offset_rate
                self._log_debug('Offset rate set to: ' + str(self.goal_offset_rate))
            except BaseException:
                self._log_warning('Failed to set goal offset rate from: ' + str(offset_rate),
                                  exc_info=True)

    @property
    def has_track(self):
        """bool: Returns true if there is a track. Returns None if not running."""
        if not self.is_running:
            return None
        return self._spot_tracker.has_track

    @property
    def auto_acquire_track(self):
        """bool: Get or set if tracks should be automatically acquired."""
        return self._spot_tracker.auto_acquire_track

    @auto_acquire_track.setter
    def auto_acquire_track(self, auto):
        self._spot_tracker.auto_acquire_track = auto

    @property
    def track_alt_az(self):
        """tuple of float: Get the latest alt and az position relative to the goal (derotated).
        None if no track or latest update failed.
        """
        if not self.is_running:
            return (None, None)
        st_track = np.array(self._spot_tracker.track_x_y)
        if None in st_track:
            return (None, None)
        else:
            rotation = self._camera.rotation
            rotmx = np.array([[np.cos(np.deg2rad(rotation)), np.sin(np.deg2rad(rotation))],
                              [-np.sin(np.deg2rad(rotation)), np.cos(np.deg2rad(rotation))]])
            return tuple(rotmx @ st_track)[::-1]

    @property
    def track_x_y_absolute(self):
        """tuple of float: Get the latest absolute x and y position (relative to image centre).
        None if no track or latest update failed.
        """
        if not self.is_running:
            return (None, None)
        return self._spot_tracker.track_x_y_absolute

    @property
    def mean_alt_az(self):
        """tuple of float: Get the mean (smoothed) alt and az position relative to the goal
        (derotated). None if no track.

        Smoothing determined by SpotTracker.smoothing_parameter
        """
        if not self.is_running:
            return (None, None)
        st_mean = np.array(self._spot_tracker.mean_x_y)
        if None in st_mean:
            return (None, None)
        else:
            rotation = self._camera.rotation
            rotmx = np.array([[np.cos(np.deg2rad(rotation)), np.sin(np.deg2rad(rotation))],
                              [-np.sin(np.deg2rad(rotation)), np.cos(np.deg2rad(rotation))]])
            return tuple(rotmx @ st_mean)[::-1]

    @property
    def mean_x_y_absolute(self):
        """tuple of float: Get the absolute mean x and y position (relative to image centre).
        None if no track.

        Smoothing determined by SpotTracker.smoothing_parameter
        """
        if not self.is_running:
            return (None, None)
        return self._spot_tracker.mean_x_y_absolute

    @property
    def track_sd(self):
        """float: Get the total standard deviation of the track. None if no track."""
        if not self.is_running:
            return None
        return self._spot_tracker.track_sd

    @property
    def pos_search_rad(self):
        """float: Get or set the current search radius."""
        if not self.is_running:
            return None
        return self._spot_tracker.pos_search_rad

    @pos_search_rad.setter
    def pos_search_rad(self, rad):
        self._spot_tracker.pos_search_rad = float(rad)

    @property
    def pos_search_x_y(self):
        """tuple of float: Get or set the current search position. Will also set search radius to
        max_search_radius.
        """
        if not self.is_running:
            return (None, None)
        return self._spot_tracker.pos_search_x_y

    @pos_search_x_y.setter
    def pos_search_x_y(self, pos):
        self._spot_tracker.pos_search_x_y = pos
        self._spot_tracker.pos_search_rad = self._spot_tracker.max_search_radius

    @property
    def rms_error(self):
        """float: Get the total smoothed RMS error relative to the goal. None if no track.

        Smoothing determined by SpotTracker.rmse_smoothing_parameter
        """
        if not self.is_running:
            return None
        return self._spot_tracker.rms_error

    @property
    def goal_x_y(self):
        """tuple of float: Get or set the goal (in absolute x and y relative to image centre) the
        track is measured against.

        Defaults to (0,0)
        """
        return self.spot_tracker.goal_x_y

    @goal_x_y.setter
    def goal_x_y(self, goal):
        self.spot_tracker.goal_x_y = goal

    @property
    def goal_offset_x_y(self):
        """tuple of float: Set temporary (cleared on lost track) offset from the defined goal."""
        return self.spot_tracker.goal_offset_x_y

    @goal_offset_x_y.setter
    def goal_offset_x_y(self, goal):
        self.spot_tracker.goal_offset_x_y = goal

    def _on_image_event(self, image, timestamp, *args, **kwargs):
        """PRIVATE: Method to attach as camera callback."""
        self._log_debug('Got image event, saving')
        self._image_data = image
        self._image_timestamp = timestamp
        if not self.is_running:
            self._log_debug('Not running')
        else:
            if self._process_image.is_set():
                self._log_warning('Already processing, dropping frame.')
            else:
                self._process_image.set()
                self._log_debug('Set processing flag')


class SpotTracker:
    """Class for detecting and tracking spots (i.e. satellites) from a stream or series of images.

    Typically this class is not used directly, but attached to a TrackingThread (in this module).

    The tracker works by keeping an estimate of the mean and standard deviation (SD) of position,
    signal sum, and signal area and searching for spots which fall within *sigma* SDs away from the
    mean. By default the position and signal sum are used with 5-sigma limits to keep the track. To
    change the limits see e.g SpotTracker.sigma_position; setting any of these to None will disable
    using that parameter to discern tracks. If more than one spot falls within the track limits the
    spot closest in position to the mean position will be used. *It is mandatory to set a
    reasonable maximum search radius (default 1000 arcsec).* This will also be used to initialise
    the position SD estimate. *It is recommended to set a reasonable minimum search radius
    (default None)* to at least one pixel's width. It is also possible to set maximum and minimum
    limits on the signal sum and area SD estimates, otherwise it will be limited to between zero
    and the mean value of the respective signal.

    You may define a goal position against which the output should be measured. By default this is
    (0,0) which is the centre of the image. The units for all positions will be in the units you
    provide via plate_scale (default 1 arcsec/pixel) to SpotTracker.update_from_image(). The goal
    and properties ending in _absolute are always measured relative to image centre.

    The estimators use exponentially weighted moving mean and variance.
    SpotTracker.smoothing_parameter (default 10) defines how samples are weighted and roughly
    corresponds to the number of samples to average.

    If SpotTracker.success_to_start_track (default 3) spots are found in a row a new track will be
    established. If SpotTracker.fails_to_drop_track (default 10) failures occur in a row the track
    is dropped. If an update fails during tracking the tracking parameters will be penalised by
    SpotTracker.failure_sd_penalty (default 25%) to account for drift in the mean.

    A performance metric, the root-mean-squared error (RMSE), is provided which measures the
    position error relative to the goal (instead of the mean, as the SD does). By default this uses
    SpotTracker.smoothing_parameter with exponential averaging, but
    SpotTracker.rmse_smoothing_parameter may be defined to control this individually.

    You may elect to use your own spot detection system and directly update the tracker with
    position with SpotTracker.update_from_observation(). If this is done, the tracker will accept
    the updated position without checking if it falls within the defined tracking range.

    Note:
        The SpotTracker uses position information in (x,y) coordinates where (0,0) is the centre of
        the image, x increases to the right and y increases upwards. When images are used to update
        the tracker (via SpotTracker.update_from_image()) a plate scale should be supplied such
        that the output is in useful units, i.e. arcseconds.

    Args:
        name (str, optional): A name for the SpotTracker.
        data_folder (pathlib.Path, optional): The folder for data saving. If None (the default) the
            folder *pypogs*/data will be used/created. *TODO! Currently does nothing*
        debug_folder (pathlib.Path, optional): The folder for debug logging. If None (the default)
            the folder *pypogs*/debug will be used/created.

    """
    def __init__(self, name=None, data_folder=None, debug_folder=None):
        """Create SpotTracker instance. See class documentation."""
        # Logger setup
        self._debug_folder = None
        if debug_folder is None:
            self.debug_folder = Path(__file__).parent / 'debug'
        else:
            self.debug_folder = debug_folder
        self._logger = logging.getLogger('pypogs.tracking.SpotTracker')
        if not self._logger.hasHandlers():
            # Add new handlers to the logger if there are none
            self._logger.setLevel(logging.DEBUG)
            # Console handler at INFO level
            ch = logging.StreamHandler()
            ch.setLevel(logging.INFO)
            # File handler at DEBUG level
            fh = logging.FileHandler(self.debug_folder / 'pypogs.txt')
            fh.setLevel(logging.DEBUG)
            # Format and add
            formatter = logging.Formatter('%(asctime)s:%(name)s-%(levelname)s: %(message)s')
            fh.setFormatter(formatter)
            ch.setFormatter(formatter)
            self._logger.addHandler(fh)
            self._logger.addHandler(ch)

        # Start of constructor
        self._logger.debug('SpotTracker called with: name=' + str(name) + ' data_folder='
                           + str(data_folder))
        # TODO: Actually do data saving from spot trackers!
        self._data_folder = None
        if data_folder is None:
            self.data_folder = Path(__file__).parent / 'data'
        else:
            self.data_folder = data_folder
        self._name = 'UnnamedSpotTracker'
        if name is not None:
            self.name = name

        # Tracking goals
        self._goal_x_y = (0., 0.)
        self._goal_offs_x_y = (0., 0.)
        # Tracking parameters
        self._has_track = False
        self._success_count = 0
        self._succ_to_start = 3
        self._fail_count = 0
        self._fail_to_drop = 10
        self._fail_sd_penalty = 25.
        self._pos_sig = 5.  # Standard deviations range for tracking in position
        self._min_search_rad = None
        self._max_search_rad = 1000.
        self._sum_sig = 5.  # Standard deviations range for tracking in sum
        self._sum_max_sd = None
        self._sum_min_sd = None
        self._area_sig = None  # Standard deviations range for tracking in area
        self._area_max_sd = None
        self._area_min_sd = None
        # Active cropping
        self.active_crop_enable = True
        self.active_crop_padding = 16
        # Image extraction parameters
        self._crop = None
        self._downsample = None
        self._filtsize = 7
        self._sigma_mode = 'global_median_abs'
        self._bg_subtract_mode = 'global_median'
        self._image_sigma_th = 3.  # Standard deviations above noise to threshold at
        self._image_th = None
        self._binary_open = True
        self._centroid_window = None
        self._spot_min_area = 3  # Default min area
        self._spot_max_area = None
        self._spot_min_sum = 100.  # Default min sum
        self._spot_max_sum = None
        self._spot_max_axis_ratio = 1.5  # Default maximum major/minor axis ratio
        # Latest position
        self._x = None
        self._y = None
        self._sum = None
        self._area = None
        # Mean and sd estimator
        self._alpha = .1
        self._alpha_rmse = None
        self._x_mean = None
        self._y_mean = None
        self._pos_variance = None
        self._pos_search_rad = None
        self._pos_search_x = None
        self._pos_search_y = None
        self._sum_mean = None
        self._sum_variance = None
        self._area_mean = None
        self._area_variance = None
        self._rmse2 = None
        self._auto_aquire = True
        self._logger.info('SpotTracker instance created with name: ' + self.name + '.')

    def _log_debug(self, msg, **kwargs):
        self._logger.debug(self.name + ': ' + msg, **kwargs)

    def _log_info(self, msg, **kwargs):
        self._logger.info(self.name + ': ' + msg, **kwargs)

    def _log_warning(self, msg, **kwargs):
        self._logger.warning(self.name + ': ' + msg, **kwargs)

    def _log_exception(self, msg, **kwargs):
        self._logger.exception(self.name + ': ' + msg, **kwargs)

    @property
    def available_properties(self):
        """tuple of str: Get the available tracking parameters (e.g. gains)."""
        return ('successes_to_track', 'fails_to_drop', 'failure_sd_penalty', 'max_search_radius',
                'min_search_radius', 'position_sigma', 'sum_sigma', 'sum_max_sd', 'sum_min_sd',
                'area_sigma', 'area_max_sd', 'area_min_sd', 'crop', 'downsample', 'filtsize',
                'sigma_mode', 'bg_subtract_mode', 'image_sigma_th', 'image_th', 'binary_open',
                'centroid_window', 'spot_min_sum', 'spot_max_sum', 'spot_min_area',
                'spot_max_area', 'spot_max_axis_ratio', 'active_crop_enable',
                'active_crop_padding')

    @property
    def data_folder(self):
        """pathlib.Path: Get or set the path for data saving. Will create folder if not existing.
        """
        return self._data_folder

    @data_folder.setter
    def data_folder(self, path):
        self._logger.debug('Got set data folder with: '+str(path))
        path = Path(path)
        if path.is_file():
            path = path.parent
        if not path.is_dir():
            path.mkdir(parents=True)
        self._data_folder = path
        self._logger.debug('Set data folder to: '+str(self.data_folder))

    @property
    def debug_folder(self):
        """pathlib.Path: Get or set the path for debug logging. Will create folder if not existing.
        """
        return self._debug_folder

    @debug_folder.setter
    def debug_folder(self, path):
        # Do not do logging in here! This will be called before the logger is set up
        path = Path(path)
        if path.is_file():
            path = path.parent
        if not path.is_dir():
            path.mkdir(parents=True)
        self._debug_folder = path

    @property
    def name(self):
        """str: Get or set a name for the SpotTracker"""
        return self._name

    @name.setter
    def name(self, name):
        self._log_debug('Set name called with: '+str(name))
        self._name = str(name)
        self._log_debug('Name set to: ' + str(self.name))

    @property
    def goal_x_y(self):
        """tuple of float: Get or set the tracking goal in absolute x and y."""
        return self._goal_x_y

    @goal_x_y.setter
    def goal_x_y(self, goal):
        self._log_debug('Set goal called with: ' + str(goal))
        goal = tuple([float(x) for x in goal])
        assert len(goal) == 2
        self._goal_x_y = goal
        self._log_debug('Goal set to: ' + str(self.goal_x_y))

    @property
    def goal_offset_x_y(self):
        """tuple of float: Get or set the tracking offset from the goal."""
        return self._goal_offs_x_y

    @goal_offset_x_y.setter
    def goal_offset_x_y(self, offset):
        self._log_debug('Set goal offset called with: ' + str(offset))
        offset = tuple([float(x) for x in offset])
        assert len(offset) == 2
        self._goal_offs_x_y = offset
        self._log_debug('Offset set to: ' + str(self.goal_offset_x_y))

    @property
    def crop(self):
        """tuple of int or or None: Get or set the cropping of incoming images. If None no cropping
        is applied.

        Can be set in two ways:
            - 2-tuple: The image is cropped to this (width, height) number of pixels in the centre.
            - 4-tuple: The image is cropped to this (width, height, offset_right, offset_down)
        """
        return self._crop

    @crop.setter
    def crop(self, cr):
        self._log_debug('Got set crop with: ' + str(cr))
        if cr is None:
            self._crop = None
        else:
            cr = tuple(int(x) for x in cr)
            assert len(cr) in (2, 4), 'Input must be 2-tuple or 4-tuple convertible to int.'
            self._crop = cr
        self._log_debug('Set crop to: ' + str(self.crop))

    @property
    def active_crop_enable(self):
        """bool: Get or set if active cropping should be used.

        If enabled and the SpotTracker has a track, the incoming image will be automatically
        cropped to only contain the search region (to speed up computation). This may have a slight
        effect on background subtraction and SD estimators.
        """
        return self._active_crop_enable

    @active_crop_enable.setter
    def active_crop_enable(self, en):
        self._log_debug('Got set active crop enable with: ' + str(en))
        self._active_crop_enable = bool(en)
        self._log_debug('Set active crop enable to: ' + str(self.active_crop_enable))

    @property
    def active_crop_padding(self):
        """int: Get or set the number of pixels to pad on each side of the search region for active
        cropping.
        """
        return self._active_crop_padding

    @active_crop_padding.setter
    def active_crop_padding(self, pad):
        self._log_debug('Got set active crop padding with: ' + str(pad))
        pad = int(pad)
        assert not pad < 0, 'Padding can not be negative'
        self._active_crop_padding = pad
        self._log_debug('Set active crop padding to: ' + str(self.active_crop_padding))

    @property
    def downsample(self):
        """int or None: Get or set downsampling (binning) of incoming images.

        Note:
            If a received image's shape can not be divided by this factor an error will be thrown!
        """
        return self._downsample

    @downsample.setter
    def downsample(self, ds):
        self._log_debug('Got set downsample with: ' + str(ds))
        if ds is None:
            self._downsample = None
        else:
            self._downsample = int(ds)
        self._log_debug('Set downsample to: ' + str(self.downsample))

    @property
    def bg_subtract_mode(self):
        """str: Get or set the background subtraction mode.

        Allowed values:
            - global_median: Subtract the global median value.
            - local_median: Subtract the median in a SpotTracker.filtsize wide window around each
              pixel.
            - global_mean: Subtract the global mean value.
            - local_mean: Subtract the median in a SpotTracker.filtsize wide window around each
              pixel.
        """
        return self._bg_subtract_mode

    @bg_subtract_mode.setter
    def bg_subtract_mode(self, mode):
        self._log_debug('Got set bg subtract mode with: ' + str(mode))
        mode = str(mode)
        allowed = ('global_median', 'local_median', 'global_mean', 'local_mean')
        assert mode in allowed, 'Forbidden bg subtract string'
        self._bg_subtract_mode = mode
        self._log_debug('Set bg subtract mode to: ' + str(self.bg_subtract_mode))

    @property
    def sigma_mode(self):
        """str: Get or set the mode used to calculate the image sigma (standard deviation) after
        background subtraction.

        This value multiplied with SpotTracker.image_sigma_th will be used to threshold the image.
        If SpotTracker.image_th is not None (default is None), that will be used as the threshold
        instead, and sigma_mode and image_sigma_th have no effect.

        Allowed values:
            - global_median_abs: Use the median of the absolute value of the image and multiply by
              1.48.
            - local_median_abs: As above, but in a SpotTracker.filtsize wide window around each
              pixel.
            - global_root_square: Use the root mean square of the image.
            - local_root_square: As above, but in a SpotTracker.filtsize wide window around each
              pixel.
        """
        return self._sigma_mode

    @sigma_mode.setter
    def sigma_mode(self, mode):
        self._log_debug('Got set sigma mode with: ' + str(mode))
        mode = str(mode)
        allowed = ('global_median_abs', 'local_median_abs', 'global_root_square',
                   'local_root_square')
        assert mode in allowed, 'Forbidden sigma string'
        self._sigma_mode = mode
        self._log_debug('Set sigma mode to: ' + str(self.sigma_mode))

    @property
    def filtsize(self):
        """int: Get or set the filter size (width and height) used for local background subtraction
        and SD estimate.
        """
        return self._filtsize

    @filtsize.setter
    def filtsize(self, size):
        self._log_debug('Got set filtsize with: ' + str(size))
        size = int(size)
        assert size > 0 and size % 2 == 1, 'Size must be odd and positive'
        self._filtsize = size
        self._log_debug('Set filtsize to: ' + str(self.filtsize))

    @property
    def image_sigma_th(self):
        """float: Get or set the number of standard deviations of image noise used for image
        thresholding.

        If SpotTracker.image_th is set (not None), this value will be ignored.
        """
        return self._image_sigma_th

    @image_sigma_th.setter
    def image_sigma_th(self, sig):
        self._log_debug('Got set image sigma th with: ' + str(sig))
        sig = float(sig)
        assert sig > 0, 'Threshold must be greater than zero'
        self._image_sigma_th = sig
        self._log_debug('Sigma th set to: ' + str(self.image_sigma_th))

    @property
    def image_th(self):
        """int or None: Get or set the image thresholding value.

        If this value is set (not None), the image standard deviation estimator and image_sigma_th
        will be ignored and the provided value used directly.
        """
        return self._image_th

    @image_th.setter
    def image_th(self, th):
        self._log_debug('Got set image th with: ' + str(th))
        if th is None:
            self._image_th = None
        else:
            th = int(th)
            self._image_th = th
        self._log_debug('Image th set to: ' + str(self.image_th))

    @property
    def binary_open(self):
        """bool: Get or set if binary opening (with 1-connected structure element) should be used
        to clean the thresholded image.
        """
        return self._binary_open

    @binary_open.setter
    def binary_open(self, do):
        self._log_debug('Got set binary open with: ' + str(do))
        self._binary_open = bool(do)
        self._log_debug('Binary open set to: ' + str(self.binary_open))

    @property
    def centroid_window(self):
        """int or None: Get or set the second round centroiding window size.

        If None, the centroid will be calculated from the spot region in the thresholded image. If
        set, a square window of the given size will be used to recalculate the centroid."""
        return self._centroid_window

    @centroid_window.setter
    def centroid_window(self, window):
        self._log_debug('Got set centroid window with: ' + str(window))
        if window is None:
            self._centroid_window = None
        else:
            window = int(window)
            assert window > 0 and window % 2 == 1, 'Window must be odd and positive'
            self._centroid_window = window
        self._log_debug('Set centroid window to: ' + str(self.centroid_window))

    @property
    def spot_min_area(self):
        """int or None: Get or set the minimum number of pixels in the thresholded image which
        constitute a star.
        """
        return self._spot_min_area

    @spot_min_area.setter
    def spot_min_area(self, min_area):
        self._log_debug('Got set min spot area with: ' + str(min_area))
        if min_area is None:
            self._spot_min_area = None
        else:
            self._spot_min_area = int(min_area)
        self._log_debug('Min spot area set to: ' + str(self.spot_min_area))

    @property
    def spot_max_area(self):
        """int or None: Get or set the maximum number of pixels in the thresholded image which
        constitute a star.
        """
        return self._spot_max_area

    @spot_max_area.setter
    def spot_max_area(self, max_area):
        self._log_debug('Got set max spot area with: ' + str(max_area))
        if max_area is None:
            self._spot_max_area = None
        else:
            self._spot_max_area = int(max_area)
        self._log_debug('Max spot area set to: ' + str(self.spot_max_area))

    @property
    def spot_min_sum(self):
        """float or None: Get or set the minimum sum of values in a region to constitute a star."""
        return self._spot_min_sum

    @spot_min_sum.setter
    def spot_min_sum(self, min_sum):
        self._log_debug('Got set min spot sum with: ' + str(min_sum))
        if min_sum is None:
            self._spot_min_sum = None
        else:
            self._spot_min_sum = float(min_sum)
        self._log_debug('Min spot sum set to: ' + str(self.spot_min_sum))

    @property
    def spot_max_sum(self):
        """float or None: Get or set the maximum sum of values in a region to constitute a star."""
        return self._spot_max_sum

    @spot_max_sum.setter
    def spot_max_sum(self, max_sum):
        self._log_debug('Got set max spot sum with: ' + str(max_sum))
        if max_sum is None:
            self._spot_max_sum = None
        else:
            self._spot_max_sum = float(max_sum)
        self._log_debug('Max spot sum set to: ' + str(self.spot_max_sum))

    @property
    def spot_max_axis_ratio(self):
        """float or None: Get or set the maximum ratio of major to minor axes for a region to
        constitute a star.

        The major and minor axes are calculated from second moments.
        """
        return self._spot_max_axis_ratio

    @spot_max_axis_ratio.setter
    def spot_max_axis_ratio(self, ratio):
        self._log_debug('Got set max spot axis ratio with: ' + str(ratio))
        if ratio is None:
            self._spot_max_axis_ratio = None
        else:
            ratio = float(ratio)
            assert ratio > 1, 'Axis ratio must be greater than one'
            self._spot_max_axis_ratio = ratio
        self._log_debug('Set max axis ratio to: ' + str(self.spot_max_axis_ratio))

    @property
    def successes_to_track(self):
        """int: Get or set the number of successive successful extractions to start a track."""
        return self._succ_to_start

    @successes_to_track.setter
    def successes_to_track(self, num):
        self._log_debug('Got set success to start with: '+str(num))
        num = int(num)
        assert num > 0, 'Number must be greater than zero'
        self._succ_to_start = num
        self._log_debug('Set success to start to: '+str(self.successes_to_track))

    @property
    def fails_to_drop(self):
        """int: Get or set the number of successive failed extractions after which a track is
        dropped.
        """
        return self._fail_to_drop

    @fails_to_drop.setter
    def fails_to_drop(self, num):
        self._log_debug('Got set fails to drop with: '+str(num))
        num = int(num)
        assert num > 0, 'Number must be greater than zero'
        self._fail_to_drop = num
        self._log_debug('Set fails to drop to: '+str(self.fails_to_drop))

    @property
    def failure_sd_penalty(self):
        """float or None: Get or set the penalty in percent to give to the standard deviation on a
        failed extraction.
        """
        return self._fail_sd_penalty

    @failure_sd_penalty.setter
    def failure_sd_penalty(self, percent):
        self._log_debug('Got set failure penalty with: '+str(percent))
        if percent is None:
            self._fail_sd_penalty = None
        else:
            self._fail_sd_penalty = float(percent)
        self._log_debug('Set failure penalty to: '+str(self._fail_sd_penalty))

    @property
    def max_search_radius(self):
        """float: Get or set the maximum search radius. Also sets initial standard deviation. Must
        be set; default 1000.
        """
        return self._max_search_rad

    @max_search_radius.setter
    def max_search_radius(self, max_rad):
        self._log_debug('Got set max search rad to: ' + str(max_rad))
        assert isinstance(max_rad, (int, float)), 'Input must be a scalar (int or float)'
        self._max_search_rad = float(max_rad)
        self._log_debug('Set max search rad to: '+str(self.max_search_radius))

    @property
    def min_search_radius(self):
        """float or None: Get or set the minimum search radius. Also sets initial standard
        deviation.
        """
        return self._min_search_rad

    @min_search_radius.setter
    def min_search_radius(self, min_rad):
        self._log_debug('Got set min search rad to: ' + str(min_rad))
        if min_rad is None:
            self._min_search_rad = None
        else:
            self._min_search_rad = float(min_rad)
        self._log_debug('Set min search rad to: '+str(self.min_search_radius))

    @property
    def position_sigma(self):
        """float or None: Get or set the number of standard deviations the search radius should be.
        Set None to disable tracking in position.
        """
        return self._pos_sig

    @position_sigma.setter
    def position_sigma(self, sig_pos):
        self._log_debug('Got set sigma position with: '+str(sig_pos))
        if sig_pos is None:
            self._pos_sig = None
        else:
            self._pos_sig = float(sig_pos)
        self._log_debug('Set sigma position to: '+str(self.position_sigma))

    @property
    def sum_sigma(self):
        """float or None: Get or set the sigmal sum standard deviation threshold. Set None to
        disable tracking in sum.
        """
        return self._sum_sig

    @sum_sigma.setter
    def sum_sigma(self, sig_sum):
        self._log_debug('Got set sigma sum with: '+str(sig_sum))
        if sig_sum is None:
            self._sum_sig = None
        else:
            self._sum_sig = float(sig_sum)
        self._log_debug('Set sigma sum to: '+str(self.sum_sigma))

    @property
    def sum_max_sd(self):
        """float or None: Get or set the maximum (and initialisation) for signal sum standard
        deviation estimator.
        """
        return self._sum_max_sd

    @sum_max_sd.setter
    def sum_max_sd(self, sum_max_sd):
        self._log_debug('Got set sum max SD with: '+str(sum_max_sd))
        if sum_max_sd is None:
            self._sum_max_sd = None
        else:
            assert isinstance(sum_max_sd, (int, float)), \
                'Input must be None or a scalar (int or float)'
            self._sum_max_sd = float(sum_max_sd)
        self._log_debug('Set sum max SD to: '+str(self.sum_max_sd))

    @property
    def sum_min_sd(self):
        """float or None: Get or set the minimum for signal sum standard deviation estimator."""
        return self._sum_min_sd

    @sum_min_sd.setter
    def sum_min_sd(self, sum_min_sd):
        self._log_debug('Got set sum min SD with: '+str(sum_min_sd))
        if sum_min_sd is None:
            self._sum_min_sd = None
        else:
            assert isinstance(sum_min_sd, (int, float)), \
                'Input must be None or a scalar (int or float)'
            self._sum_min_sd = float(sum_min_sd)
        self._log_debug('Set sum min SD to: '+str(self.sum_min_sd))

    @property
    def area_sigma(self):
        """float or None: Get or set the signal area standard deviation threshold.

        Set None to disable tracking in spot area.
        """
        return self._area_sig

    @area_sigma.setter
    def area_sigma(self, sig_area):
        self._log_debug('Got set sigma area with: ' + str(sig_area))
        if sig_area is None:
            self._area_sig = None
        else:
            assert isinstance(sig_area, (int, float)), \
                'Input must be None or a scalar (int or float)'
            self._area_sig = float(sig_area)
        self._log_debug('Set sigma area to: ' + str(self._area_sig))

    @property
    def area_max_sd(self):
        """float: Get or set the maximum (and initialisation) for signal area standard deviation
        estimator.
        """
        return self._area_max_sd

    @area_max_sd.setter
    def area_max_sd(self, area_max_sd):
        self._log_debug('Got set area max SD with: ' + str(area_max_sd))
        if area_max_sd is None:
            self._area_max_sd = None
        else:
            assert isinstance(area_max_sd, (int, float)), \
                'Input must be None or a scalar (int or float)'
            self._area_max_sd = float(area_max_sd)
        self._log_debug('Set area max SD to: ' + str(self._area_max_sd))

    @property
    def area_min_sd(self):
        """float: Get or set the minimum for signal area standard deviation estimator."""
        return self._area_min_sd

    @area_min_sd.setter
    def area_min_sd(self, area_min_sd):
        self._log_debug('Got set area min SD with: ' + str(area_min_sd))
        if area_min_sd is None:
            self._area_min_sd = None
        else:
            assert isinstance(area_min_sd, (int, float)), \
                'Input must be None or a scalar (int or float)'
            self._area_min_sd = float(area_min_sd)
        self._log_debug('Set area min SD to: ' + str(self._area_min_sd))

    @property
    def has_track(self):
        """bool: Returns True if there currently is a track."""
        return self._has_track

    @property
    def auto_acquire_track(self):
        """bool: Get or set if tracks should be automatically acquired."""
        return self._auto_aquire

    @auto_acquire_track.setter
    def auto_acquire_track(self, auto):
        self._auto_aquire = bool(auto)

    @property
    def track_x_y(self):
        """tuple of float: Get the latest x and y position relative to the goal. None if no track
        or latest update failed.
        """
        if not self._has_track or None in (self._x, self._y):
            return (None, None)
        else:
            x_rel = self._x - self.goal_x_y[0] - self.goal_offset_x_y[0]
            y_rel = self._y - self.goal_x_y[1] - self.goal_offset_x_y[1]
            return (x_rel, y_rel)

    @property
    def track_x_y_absolute(self):
        """tuple of float: Get the latest absolute x and y position (relative to image centre).
        None if no track or latest update failed.
        """
        if not self._has_track:
            return (None, None)
        else:
            return (self._x, self._y)

    @property
    def mean_x_y(self):
        """tuple of float: Get the mean (smoothed) x and y position relative to the goal. None if
        no track.

        Smoothing determined by SpotTracker.smoothing_parameter
        """
        if not self._has_track:
            return (None, None)
        else:
            x_rel = self._x_mean - self.goal_x_y[0] - self.goal_offset_x_y[0]
            y_rel = self._y_mean - self.goal_x_y[1] - self.goal_offset_x_y[1]
            return (x_rel, y_rel)

    @property
    def mean_x_y_absolute(self):
        """tuple of float: Get the absolute mean x and y position (relative to image centre). None
        if no track.

        Smoothing determined by SpotTracker.smoothing_parameter
        """
        if not self._has_track:
            return (None, None)
        else:
            return (self._x_mean, self._y_mean)

    @property
    def track_sd(self):
        """float: Get the total standard deviation of the track. None if no track."""
        if not self._has_track:
            return None
        else:
            return np.sqrt(self._pos_variance)

    @property
    def pos_search_rad(self):
        """float: Get or set the current search radius."""
        return self._pos_search_rad

    @pos_search_rad.setter
    def pos_search_rad(self, rad):
        if rad is None:
            self._pos_search_rad = None
        else:
            self._pos_search_rad = float(rad)

    @property
    def pos_search_x_y(self):
        """tuple of float or None: Get or set the x y position (absolute) where we search for
        track.

        Will clear tracker if set.
        """
        return (self._pos_search_x, self._pos_search_y)

    @pos_search_x_y.setter
    def pos_search_x_y(self, pos):
        self.clear_tracker()
        if pos is None or None in pos:
            self._pos_search_x = None
            self._pos_search_y = None
        else:
            px = float(pos[0])
            py = float(pos[1])
            self._pos_search_x = px
            self._pos_search_y = py

    @property
    def rms_error(self):
        """float: Get the total smoothed RMS error *relative to the goal*. None if no track.

        Smoothing determined by SpotTracker.rmse_smoothing_parameter
        """
        if not self._has_track:
            return None
        else:
            return np.sqrt(self._rmse2)

    @property
    def track_sum(self):
        """float: Get the latest signal sum. None if no track or latest update failed."""
        if not self._has_track:
            return None
        else:
            return self._sum

    @property
    def mean_sum(self):
        """float: Get the mean (smoothed) signal sum. None if no track.

        Smoothing determined by SpotTracker.smoothing_parameter
        """
        if not self._has_track:
            return None
        else:
            return self._sum_mean

    @property
    def sd_sum(self):
        """float: Get the signal sum standard deviation. None if no track.

        Smoothing determined by SpotTracker.smoothing_parameter
        """
        if not self._has_track:
            return None
        else:
            return np.sqrt(self._sum_variance)

    @property
    def track_area(self):
        """float: Get the latest signal area. None if no track or latest update failed."""
        if not self._has_track:
            return None
        else:
            return self._area

    @property
    def mean_area(self):
        """float: Get the mean (smoothed) signal area. None if no track.

        Smoothing determined by SpotTracker.smoothing_parameter
        """
        if not self._has_track:
            return None
        else:
            return self._area_mean

    @property
    def sd_area(self):
        """float: Get the signal area standard deviation. None if no track.

        Smoothing determined by SpotTracker.smoothing_parameter
        """
        if not self._has_track:
            return None
        else:
            return np.sqrt(self._area_variance)

    @property
    def smoothing_parameter(self):
        """int or float: Get or set the smoothing parameter. It roughly corresponds to the number
        of samples to average.

        - This parameter is used for estimating the mean and standard deviation of the position,
          sum, and area.
        - The total RMS Error smoothing can be set independently by
          SpotTracker.rmse_smoothing_parameter. If not, this value is used there as well.
        - Exponential smoothing is used. smoothing_parameter is the *inverse* of 'alpha'. Each
          smoothed value s is defined from the measurements x by:

          ``mean[n] = alpha*x[n] + (1-alpha)*mean[n-1]``
          ``mean[0] = x[0]``

          ``sd[n]**2 = (1-alpha)*(sd[n-1]**2 + alpha*(x[n]-mean[n-1])**2)``

        The SD estimator is initialised to given maximum values max_search_radius, sum_max_sd,
        area_max_sd if available, otherwise it is set to the magnitude of the first given value.
        """
        return 1 / self._alpha

    @smoothing_parameter.setter
    def smoothing_parameter(self, param):
        assert isinstance(param, (int, float)), 'Parameter must be scalar (float or int)'
        assert param > 0, 'Parameter must be >0'
        self._alpha = 1/float(param)
        self._log_debug('Smoothing xy alpha set to '+str(param))

    @property
    def rmse_smoothing_parameter(self):
        """float: Get or set the smoothing parameter. It roughly corresponds to the number of
        samples to average.

        - This parameter is used for estimating the RMS Error relative to the goal position.
        - If not set, SpotTracker.smoothing_parameter is used
        - Exponential smoothing is used. rmse_smoothing_parameter is the *inverse* of 'alpha'. Each
          smoothed value s is defined from the measurements x by:

          ``rmse[n]**2 = (1-alpha) * (rmse[n-1]**2 + alpha*((x[n]-goal_x)**2 + (y[n]-goal_y)**2))``

          ``rmse[0]**2 = (x[0]-goal_x)**2 + (y[0]-goal_y)**2``
        """
        if self._alpha_rmse is None:
            return self.smoothing_parameter
        else:
            return 1 / self._alpha_rmse

    @rmse_smoothing_parameter.setter
    def rmse_smoothing_parameter(self, param):
        if param is None:
            self._alpha_rmse = None
        else:
            assert param > 0, 'Parameter must be >0'
            self._alpha_rmse = 1/float(param)
        self._log_debug('Smoothing rmse alpha set to '+str(param))

    def update_from_image(self, img, plate_scale=1):
        """Update the SpotTracker from a new image.

        Will use tetra3.extract_star_positions() to find spots in the image with the extraction
        parameters which have been set in the SpotTracker. A spot falling within the current
        tracking range will be searched for and, if found, the tracker will be updated. If not
        found, the estimator SD will be penalised by SpotTracker.failure_penalty (default 10%).

        Args:
            img (numpy.ndarray): The image to update with.
            plate_scale (float, optional): Image plate scale (typically arcseconds per pixel).
            Defaults to 1 (i.e. will output directly in pixels).

        Returns:
            bool: True if has_track *and* the provided image was successfully used to update the
                tracker.
        """
        imshape = img.shape
        return_value = False
        ds = 1 if self.downsample is None else self.downsample
        if plate_scale is None:
            plate_scale = 1
        assert isinstance(plate_scale, (int, float)), \
            'Plate scale must be a number (int or float) or None'

        if self.crop is not None:
            assert len(self.crop) in (2, 4), 'Crop must be length 2 or 4 if tuple/list'
            if len(self.crop) == 2:
                cr = list(self.crop[::-1]) + [- (self.goal_x_y[1] + self.goal_offset_x_y[1])
                                              / plate_scale,
                                              (self.goal_x_y[0] + self.goal_offset_x_y[0])
                                              / plate_scale]
            else:
                cr = list(self.crop[1::-1]) + list(self.crop[3:1:-1])
        else:
            cr = None
        # If we have a track we only need to look in the allowed region anyway (if active crop)
        search_rad = self.pos_search_rad
        (x_search, y_search) = self.pos_search_x_y
        if self._has_track and self.active_crop_enable and search_rad is not None:
            limit = cr[0:2] if cr is not None else None
            x_range = 2 * search_rad / plate_scale
            y_range = 2 * search_rad / plate_scale
            cr = [np.ceil(y_range), np.ceil(x_range), -y_search/plate_scale, x_search/plate_scale]
            if self.active_crop_padding is not None:
                cr[0] += 2*self.active_crop_padding
                cr[1] += 2*self.active_crop_padding
            if limit is not None and cr[0] > limit[0]:
                cr[0] = limit[0]
            if limit is not None and cr[1] > limit[1]:
                cr[1] = limit[1]

        # Process image
        if self._has_track:
            if self._sum_sig is not None:
                sum_max = self.mean_sum + self._sum_sig*self.sd_sum
                sum_min = self.mean_sum - self._sum_sig*self.sd_sum
                if self.spot_min_sum is not None:
                    sum_min = max(self.spot_min_sum, sum_min)
                if self.spot_max_sum is not None:
                    sum_max = min(self.spot_max_sum, sum_max)
            else:
                sum_min = self.spot_min_sum
                sum_max = self.spot_max_sum
            if self._area_sig is not None:
                area_max = self.mean_area + self._area_sig*self.sd_area
                area_min = self.mean_area - self._area_sig*self.sd_area
                if self.spot_min_area is not None:
                    area_min = max(self.spot_min_area, area_min)
                if self.spot_max_area is not None:
                    area_max = min(self.spot_max_area, area_max)
            else:
                area_min = self.spot_min_area
                area_max = self.spot_max_area

            ret = get_centroids_from_image(img, image_th=self.image_th,
                                           binary_open=self.binary_open, filtsize=self.filtsize,
                                           crop=cr, downsample=ds, min_area=area_min,
                                           max_area=area_max, min_sum=sum_min, max_sum=sum_max,
                                           max_axis_ratio=self.spot_max_axis_ratio,
                                           sigma_mode=self.sigma_mode,
                                           bg_sub_mode=self.bg_subtract_mode,
                                           return_moments=True, sigma=self.image_sigma_th,
                                           centroid_window=self.centroid_window)
            success = False
            if ret[0][:, 0].size > 0:
                x = (ret[0][:, 1] - imshape[1] / 2) * plate_scale
                # Image Y coordinates increase down!
                y = - (ret[0][:, 0] - imshape[0] / 2) * plate_scale
                if None not in (x_search, y_search):
                    dx = np.abs(x - x_search)
                    dy = np.abs(y - y_search)
                else:
                    (x_mean, y_mean) = self.mean_x_y_absolute
                    dx = np.abs(x - x_mean)
                    dy = np.abs(y - y_mean)
                index = (dx**2+dy**2).argmin()
                x = float(x[index])
                y = float(y[index])
                dx = dx[index]
                dy = dy[index]
                if search_rad is None or (abs(dx) < search_rad and abs(dy) < search_rad):
                    self._success_count += 1
                    self._fail_count = 0
                    summ = float(ret[1][index])
                    area = float(ret[2][index])
                    self.update_from_observation(x, y, summ, area)
                    return_value = True
                    success = True
            if not success:
                self._success_count = 0
                self._fail_count += 1
                self.update_from_observation(None, None, None, None)
                self.penalize_track(percentage=self.failure_sd_penalty)
                if self._fail_count >= self._fail_to_drop:
                    self._log_info('Lost track.')
                    self._has_track = False
                    self.clear_tracker()

        else:  # Does not have track
            # Process image
            ret = get_centroids_from_image(img, image_th=self.image_th,
                                           binary_open=self.binary_open, filtsize=self.filtsize,
                                           crop=cr, downsample=ds, min_area=self.spot_min_area,
                                           max_area=self.spot_max_area, min_sum=self.spot_min_sum,
                                           max_sum=self.spot_max_sum,
                                           max_axis_ratio=self.spot_max_axis_ratio,
                                           sigma_mode=self.sigma_mode,
                                           bg_sub_mode=self.bg_subtract_mode,
                                           return_moments=True, sigma=self.image_sigma_th,
                                           centroid_window=self.centroid_window)
            if ret[0][:, 0].size > 0:
                self._success_count += 1
                self._fail_count = 0
                x = (ret[0][:, 1] - imshape[1] / 2) * plate_scale
                # Image Y coordinates increase down!
                y = - (ret[0][:, 0] - imshape[0] / 2) * plate_scale
                (x_search, y_search) = self.pos_search_x_y
                if self._auto_aquire:
                    summ = float(ret[1][0])
                    area = float(ret[2][0])
                    x = float(x[0])
                    y = float(y[0])
                    self.update_from_observation(x, y, summ, area)
                    if self._success_count >= self._succ_to_start:
                        self._log_info('Found track')
                        self._has_track = True
                elif None not in (x_search, y_search):
                    dx = np.abs(x - x_search)
                    dy = np.abs(y - y_search)
                    index = (dx**2 + dy**2).argmin()
                    x = float(x[index])
                    y = float(y[index])
                    dx = dx[index]
                    dy = dy[index]
                    if search_rad is None or (abs(dx) < search_rad and abs(dy) < search_rad):
                        summ = float(ret[1][0])
                        area = float(ret[2][0])
                        self.update_from_observation(x, y, summ, area)
                        if self._success_count >= self._succ_to_start:
                            self._log_info('Found track')
                            self._has_track = True
            else:
                self._success_count = 0
                self._fail_count += 1
                self.update_from_observation(None, None, None, None)
                self.clear_tracker()
        return return_value

    def update_from_observation(self, x, y, summ, area):
        """Update with new observed data for the track. Will not check any tracking validity (see
        update_from_image()).

        If any of x, y, summ, area are None it will be interpreted as a failed track cycle and
        nothing updated.
        """
        if None in (x, y, summ, area):
            self._x = None
            self._y = None
            self._sum = None
            self._area = None
        else:
            x = float(x)
            y = float(y)
            summ = float(summ)
            area = float(area)
            self._x = x
            self._y = y
            self._sum = summ
            self._area = area
            if None in (self._x_mean, self._y_mean, self._pos_variance, self._sum_mean,
                        self._sum_variance, self._area_mean, self._area_variance):
                # Initialise means to first received value
                self._x_mean = x
                self._y_mean = y
                self._sum_mean = summ
                self._area_mean = area
                # Initialise variances to max values or heuristic
                if self._max_search_rad is not None:
                    self._pos_variance = self._max_search_rad**2
                else:
                    self._pos_variance = x**2 + y**2
                if self._sum_max_sd is not None:
                    self._sum_variance = self._sum_max_sd**2
                else:
                    self._sum_variance = abs(summ)**2
                if self._area_max_sd is not None:
                    self._area_variance = self._area_max_sd**2
                else:
                    self._area_variance = abs(area)**2
                # Initialise MSE to first error
                dx = x - self.goal_x_y[0] - self.goal_offset_x_y[0]
                dy = y - self.goal_x_y[1] - self.goal_offset_x_y[1]
                self._rmse2 = dx**2 + dy**2
            else:
                # Update position mean and variance
                dx = x - self._x_mean
                dy = y - self._y_mean
                self._x_mean += self._alpha * dx
                self._y_mean += self._alpha * dy
                self._pos_variance = (1 - self._alpha) * (self._pos_variance
                                                          + self._alpha * (dx**2 + dy**2))
                # Iff we have a track, set the search to the current mean
                if self._has_track:
                    self._pos_search_x = self._x_mean
                    self._pos_search_y = self._y_mean
                    # Update search radius to sigma SD, clip with min and max search radius
                    if self._pos_sig is not None:
                        self._pos_search_rad = np.sqrt(self._pos_variance) * self._pos_sig
                    if (self._max_search_rad is not None
                            and self._pos_search_rad > self._max_search_rad):
                        self._pos_search_rad = self._max_search_rad
                    if (self._min_search_rad is not None
                            and self._pos_search_rad < self._min_search_rad):
                        self._pos_search_rad = self._min_search_rad
                # Update sum mean and variance
                d = summ - self._sum_mean
                self._sum_mean += self._alpha * d
                self._sum_variance = (1 - self._alpha) * (self._sum_variance + self._alpha * d**2)
                # Clip sum variance to either set max value or to the mean value squared
                if self._sum_max_sd is not None:
                    if self._sum_variance > self._sum_max_sd**2:
                        self._sum_variance = self._sum_max_sd**2
                elif self._sum_variance > self._sum_mean**2:
                    self._sum_variance = self._sum_mean**2
                # Clip sum variance to the minimum value
                if self._sum_min_sd is not None and self._sum_variance < self._sum_min_sd**2:
                    self._sum_variance = self._sum_min_sd**2
                # Update area mean and variance
                d = area - self._area_mean
                self._area_mean += self._alpha*d
                self._area_variance = (1-self._alpha)*(self._area_variance + self._alpha*d**2)
                # Clip area variance to either set max value or to the mean value squared
                if self._area_max_sd is not None:
                    if self._area_variance > self._area_max_sd**2:
                        self._area_variance = self._area_max_sd**2
                elif self._area_variance > self._area_mean**2:
                    self._area_variance = self._area_mean**2
                # Clip area variance to the minimum value
                if self._area_min_sd is not None and self._area_variance < self._area_min_sd**2:
                    self._area_variance = self._area_min_sd**2
                # Update MSE estimator
                a_rmse = 1/self.rmse_smoothing_parameter
                dx = x - self.goal_x_y[0] - self.goal_offset_x_y[0]
                dy = y - self.goal_x_y[1] - self.goal_offset_x_y[1]
                self._rmse2 = (1 - a_rmse) * (self._rmse2 + a_rmse * (dx**2 + dy**2))

    def penalize_track(self, percentage=25):
        """Scale the search radius, sum SD, and areaSD used for tracking by the given percentage.
        """
        if percentage is not None:
            f = (1 + percentage / 100)
            if self._pos_search_rad is not None:
                self._pos_search_rad *= f**2
                if (self._max_search_rad is not None
                        and self._pos_search_rad > self._max_search_rad):
                    self._pos_search_rad = self._max_search_rad
            if None not in (self._sum_mean, self._sum_variance, self._area_mean,
                            self._area_variance):
                if self.sum_sigma is not None:
                    self._sum_variance *= f**2
                    if self._sum_max_sd is not None:
                        if self._sum_variance > self._sum_max_sd**2:
                            self._sum_variance = self._sum_max_sd**2
                    elif self._sum_variance > self._sum_mean**2:
                        self._sum_variance = self._sum_mean**2
                if self.area_sigma is not None:
                    self._area_variance *= f**2
                    if self._area_max_sd is not None:
                        if self._area_variance > self._area_max_sd**2:
                            self._area_variance = self._area_max_sd**2
                    elif self._area_variance > self._area_mean**2:
                        self._area_variance = self._area_mean**2

    def change_mean_relative(self, dx, dy):
        """Add an offset to SpotTracker.mean_x_y positions. Useful for feed-forward of control
        signals.

        Args:
            dx (float): Amount to add in x.
            dy (float): Amount to add in y.
        """
        if None not in (self._x_mean, self._y_mean):
            self._x_mean += dx
            self._y_mean += dy
        if None not in (self._pos_search_x, self._pos_search_y):
            self._pos_search_x += dx
            self._pos_search_y += dy

    def clear_tracker(self):
        self._x_mean = None
        self._y_mean = None
        self._pos_variance = None
        self._pos_search_rad = None
        self._pos_search_x = None
        self._pos_search_y = None
        self._rmse2 = None
        self._sum_mean = None
        self._sum_variance = None
        self._area_mean = None
        self._area_variance = None
        self._has_track = False
        self.goal_offset_x_y = np.array([0, 0])
