"""High-level control of pypogs core
====================================

- :class:`pypogs.System` is the main instance for interfacing with and controlling the pypogs core.
  It allows the user to create and manage the hardware, tracking threads, targets etc.

- :class:`pypogs.Alignment` manages the alignment and location of the system, including coordinate
  transformations.

- :class:`pypogs.Target` manages the target and start/end times.

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
from threading import Thread
from csv import writer as csv_write

# External imports:
import numpy as np
from astropy.time import Time as apy_time
from astropy import units as apy_unit, coordinates as apy_coord, utils as apy_util
from skyfield import sgp4lib as sgp4
from skyfield import api as sf_api
from tifffile import imwrite as tiff_write

# Internal imports:
from tetra3 import Tetra3
from .hardware_cameras import Camera
from .hardware_mounts import Mount
from .hardware_receivers import Receiver
from .tracking import TrackingThread, ControlLoopThread

# Useful definitions:
EPS = 10**-6  # Epsilon for use in non-zero check
DEG = chr(176)  # Degree (unicode) character
_system_data_dir = Path(__file__).parent / '_system_data'
if not _system_data_dir.exists():
    _system_data_dir.mkdir()


class System:
    """Instantiate and control pypogs functionality and hardware.

    The first step is always to create a System instance. The hardware and all other classes
    are accessible as properties of the System (e.g. :attr:`coarse_camera`,
    :attr:`control_loop_thread`) and should be created by asking the System instance to do
    so.

    Note:
        Tables of Earth's rotation must be downloaded to do coordinate transforms. Calling
        :meth:`update_databases()` will attempt to download these from the internet (if expired).
        To facilitate offline use of pypogs, these will otherwise not be automatically downloaded
        unless strictly neccessary. If you use pypogs offline do this update every few months to
        keep the coordinate transforms accurate.

    Example:
        ::

            import pypogs
            sys = pypogs.System()

    Add and control hardware:
        There are five possible hardware instances:

        - :attr:`mount` (:class:`pypogs.Mount`)

        - :attr:`star_camera` (:class:`pypogs.Camera`)

        - :attr:`coarse_camera` (:class:`pypogs.Camera`)

        - :attr:`fine_camera` (:class:`pypogs.Camera`)

        - :attr:`receiver` (:class:`pypogs.Receiver`)

        They should be added to the System by using an add method, e.g. :meth:`add_mount` for
        :attr:`mount`, and may be removed by the respective :meth:`clear_mount`. One special case
        is if the star and coarse cameras are the same physical camera, then use
        :meth:`add_star_camera_from_coarse` to copy the :attr:`coarse_camera` object to
        :attr:`star_camera`. After adding, see the respective class documentation for controlling
        the settings.

        Example:
            ::

                sys.add_coarse_camera(model='ptgrey', identity='18285284')
                sys.add_star_camera_from_coarse()
                sys.add_fine_camera(model='ptgrey', identity='18285254')
                sys.coarse_camera.exposure_time = 100  # milliseconds
                sys.coarse_camera.frame_rate = 2  # hertz
                sys.coarse_camera is sys.star_camera  # returns True

    Settings for closed loop tracking:
        When a coarse or fine Camera object is added, a corresponding tracking thread (e.g.
        :attr:`coarse_track_thread` for :attr:`coarse_camera`) is also created. This is a
        :class:`pypogs.TrackingThread` object which also holds a :class:`pypogs.SpotTracker` object
        (accessible via :attr:`coarse_track_thread.spot_tracker`). The parameters for these (see
        respective documentation) are used to set up the detection for tracking. For best
        performance it is critical to set up these well.

        Further, a :class:`pypogs.ControlLoopThread` object is available as
        :attr:`control_loop_thread` and defines the feedback parameters used for closed loop
        tracking (e.g. gains, modes, switching logic).

        Example:
            ::

                sys.coarse_track_thread.feedforward_threshold = 10
                sys.coarse_track_thread.spot_tracker.bg_subtract_mode = 'global_median'
                sys.control_loop_thread.CCL_P = 1

    Set location and alignment:
        :attr:`alignment` references the :class:`pypogs.Alignment` instance (auto created) used to
        determine and calibrate the location, alignment, and mount corrections. See the class
        documentation for how to set location and alignments. To do auto-alignment, use the
        :meth:`do_auto_star_alignment` method (requires a star camera).

        If your mount has built in alignment (and/or is physically aligned to the earth) you may
        call :meth:`alignment.set_alignment_enu` to set the telescope alignment to East, North, Up
        (ENU) coordinates, which will also disable the corrections done in pypogs. ENU is the
        traditional astronomical coordinate system for altitide (elevation) and azimuth telescopes,
        measured as degrees above the horizon and degrees away from north (towards east)
        respectively.

        Example:
            ::

                sys.alignment.set_location_lat_lon(lat=52.2155, lon=4.4194, height=45)
                sys.do_auto_star_alignment()

    Set target:
        :attr:`target` references the :class:`pypogs.Target` instance (auto created) which holds
        the target and (optional) tracking start and end times. The target may be set directly to
        an *astropy SkyCoord* or a *skyfield EarthSatellite* by :meth:`target.set_target` or these
        can be created by pypogs by e.g. calling :meth:`target.set_target_from_tle` with a
        Two Line Element (TLE) for a satellite or :meth:`target.set_target_from_ra_dec`
        with right ascension and declination (in decimal degrees) for a star.

        Example satellite:
            ::

                tle = ['1 28647U 05016B   19180.65078896  .00000014  00000-0  19384-4 0  9991',\\
                       '2 28647  56.9987 238.9694 0122260 223.0550 136.0876 15.05723818  6663']
                sys.target.set_target_from_tle(tle)

        Example star:
            ::

                sys.target.set_target_from_ra_dec(37.95456067, 89.26410897)  # Polaris

    Control tracking:
        You are now ready! Just call :meth:`start_tracking` and then :meth:`stop_tracking` when you
        are finished.

    Release hardware:
        Before exiting, it's strongly recommended to call :meth:`deinitialize` to release the
        hardware resources.

    Args:
        data_folder (pathlib.Path, optional): The folder for data saving. If None (the default) the
            folder *pypogs*/data will be used/created.
        debug_folder (pathlib.Path, optional): The folder for debug logging. If None (the default)
            the folder *pypogs*/debug will be used/created.
    """

    @staticmethod
    def update_databases():
        """Download and update Skyfield and Astropy databases (of earth rotation)."""
        sf_api.Loader(_system_data_dir).timescale()
        apy_util.iers.IERS_Auto.open()

    def __init__(self, data_folder=None, debug_folder=None):
        """Create System instance. See class documentation."""
        # Logger setup
        self._debug_folder = None
        if debug_folder is None:
            self.debug_folder = Path(__file__).parent / 'debug'
        else:
            self.debug_folder = debug_folder
        self._logger = logging.getLogger('pypogs.system.System')
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
        self._logger.debug('System constructor called')
        # Data folder setup
        self._data_folder = None
        if data_folder is None:
            self.data_folder = Path(__file__).parent / 'data'
        else:
            self.data_folder = data_folder
        # Threads
        self._coarse_track_thread = None
        self._fine_track_thread = None
        self._control_loop_thread = ControlLoopThread(self)  # Create the control loop thread
        # Hardware not managed by threads
        self._star_cam = None
        self._receiver = None
        self._mount = None
        self._alignment = Alignment()
        self._target = Target()
        # Variable to stop system thread
        self._stop_loop = True
        self._thread = None
        import atexit
        import weakref
        atexit.register(weakref.ref(self.__del__))
        self._logger.info('System instance created.')

    def __del__(self):
        """Destructor. Calls deinitialize()."""
        try:
            self._logger.debug('System destructor called')
        except BaseException:
            pass
        try:
            self.deinitialize()
            self._logger.debug('Deinitialised')
        except BaseException:
            pass

    def initialize(self):
        """Initialise cameras, mount, and receiver if they are not already."""
        self._logger.debug('Initialise called')
        if self.star_camera is not None:
            self._logger.debug('Has star cam')
            if not self.star_camera.is_init:
                try:
                    self.star_camera.initialize()
                    self._logger.debug('Initialised')
                except BaseException:
                    self._logger.warning('Failed to init', exc_info=True)
            else:
                self._logger.debug('Already initialised')
        if self.coarse_camera is not None:
            self._logger.debug('Has coarse cam')
            if not self.coarse_camera.is_init:
                try:
                    self.coarse_camera.initialize()
                    self._logger.debug('Initialised')
                except BaseException:
                    self._logger.warning('Failed to init', exc_info=True)
            else:
                self._logger.debug('Already initialised')
        if self.fine_camera is not None:
            self._logger.debug('Has coarse cam')
            if not self.fine_camera.is_init:
                try:
                    self.fine_camera.initialize()
                    self._logger.debug('Initialised')
                except BaseException:
                    self._logger.warning('Failed to init', exc_info=True)
            else:
                self._logger.debug('Already initialised')
        if self.mount is not None:
            self._logger.debug('Has mount')
            if not self.mount.is_init:
                try:
                    self.mount.initialize()
                    self._logger.debug('Initialised')
                except BaseException:
                    self._logger.warning('Failed to init', exc_info=True)
            else:
                self._logger.debug('Already initialised')
        if self.receiver is not None:
            self._logger.debug('Has receiver')
            if not self.receiver.is_init:
                try:
                    self.receiver.initialize()
                    self._logger.debug('Initialised')
                except BaseException:
                    self._logger.warning('Failed to init', exc_info=True)
            else:
                self._logger.debug('Already initialised')
        self._logger.info('System initialised')

    def deinitialize(self):
        """Deinitialise camera, mount, and receiver if they are initialised."""
        self._logger.debug('Deinitialise called')
        if self.star_camera is not None:
            self._logger.debug('Has star cam')
            if self.star_camera.is_init:
                try:
                    self.star_camera.deinitialize()
                    self._logger.debug('Deinitialised')
                except BaseException:
                    self._logger.warning('Failed to deinit', exc_info=True)
            else:
                self._logger.debug('Not initialised')
        if self.coarse_camera is not None:
            self._logger.debug('Has coarse cam')
            if self.coarse_camera.is_init:
                try:
                    self.coarse_camera.deinitialize()
                    self._logger.debug('Deinitialised')
                except BaseException:
                    self._logger.warning('Failed to deinit', exc_info=True)
            else:
                self._logger.debug('Not initialised')
        if self.fine_camera is not None:
            self._logger.debug('Has coarse cam')
            if self.fine_camera.is_init:
                try:
                    self.fine_camera.deinitialize()
                    self._logger.debug('Deinitialised')
                except BaseException:
                    self._logger.warning('Failed to deinit', exc_info=True)
            else:
                self._logger.debug('Not initialised')
        if self.mount is not None:
            self._logger.debug('Has mount')
            if self.mount.is_init:
                try:
                    self.mount.deinitialize()
                    self._logger.debug('Deinitialised')
                except BaseException:
                    self._logger.warning('Failed to deinit', exc_info=True)
            else:
                self._logger.debug('Not initialised')
        if self.receiver is not None:
            self._logger.debug('Has receiver')
            if self.receiver.is_init:
                try:
                    self.receiver.deinitialize()
                    self._logger.debug('Deinitialised')
                except BaseException:
                    self._logger.warning('Failed to deinit', exc_info=True)
            else:
                self._logger.debug('Not initialised')
        self._logger.info('System deinitialised')

    @property
    def is_init(self):
        """bool: Returns True if all attached hardware (cameras, mount, and receiver) are
        initialised."""
        init = True
        if self.star_camera is not None and not self.star_camera.is_init:
            init = False
        if self.coarse_camera is not None and not self.coarse_camera.is_init:
            init = False
        if self.fine_camera is not None and not self.fine_camera.is_init:
            init = False
        if self.mount is not None and not self.mount.is_init:
            init = False
        if self.receiver is not None and not self.receiver.is_init:
            init = False
        return init

    @property
    def is_busy(self):
        """bool: Returns True if system is doing some control task."""
        if self._thread is not None and self._thread.is_alive():
            return True
        if self._control_loop_thread is not None and self._control_loop_thread.is_running:
            return True
        if self.mount is not None and self.mount.is_moving:
            return True
        return False

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
    def control_loop_thread(self):
        """System.ControlLoopThread: Get the system control loop thread."""
        return self._control_loop_thread

    @property
    def alignment(self):
        """pypogs.Alignment: Get the system alignment object."""
        return self._alignment

    @property
    def target(self):
        """pypogs.Alignment: Get the system target object."""
        return self._target

    @property
    def star_camera(self):
        """pypogs.Camera: Get or set the star camera."""
        return self._star_cam

    @star_camera.setter
    def star_camera(self, cam):
        self._logger.debug('Got set star camera with: '+str(cam))
        if self.star_camera is not None:
            self._logger.debug('Already have a star camera, try to clear it')
            try:
                self.star_camera.deinitialize()
                self._logger.debug('Deinit')
            except BaseException:
                self._logger.debug('Failed to deinit', exc_info=True)
            self._star_cam = None
            self._logger.debug('Cleared')
        if cam is not None:
            assert isinstance(cam, Camera), 'Must be pypogs.Camera'
            self._star_cam = cam
        self._logger.debug('Star camera set to: ' + str(self.star_camera))

    def add_star_camera(self, model=None, identity=None, name='StarCamera', auto_init=True):
        """Create and set the star camera. Calls pypogs.Camera constructor with
        name='StarCamera' and the given arguments.

        Args:
            model (str, optional): The model used to determine the correct hardware API. Supported:
                'ptgrey' for PointGrey/FLIR Machine Vision cameras (using Spinnaker/PySpin APIs).
            identity (str, optional): String identifying the device. For *ptgrey* this is
                'serial number' *as a string*.
            name (str, optional): Name for the device, defaults to 'StarCamera'.
            auto_init (bool, optional): If both model and identity are given when creating the
                Camera and auto_init is True (the default), Camera.initialize() will be called
                after creation.
        """
        self._logger.debug('Got add star camera with model=' + str(model)
                           + ' identity='+str(identity) + ' auto_init=' + str(auto_init))
        if self.star_camera is not None:
            self._logger.debug('Already have a star camera with model='
                               + str(self.star_camera.model)
                               + ' identity=' + str(self.star_camera.identity))
            if self.star_camera.model == model and self.star_camera.identity == identity:
                self._logger.debug('Already attached, just set the name')
                self.star_camera.name = name
            else:
                self._logger.debug('Got a different camera, clear the old one')
                self.star_camera = None
                self._logger.debug('Create new camera')
                self.star_camera = Camera(model=model, identity=identity, name=name,
                                          auto_init=auto_init)
        else:
            self._logger.debug('Dont have anything old to clean up, create new camera')
            self.star_camera = Camera(model=model, identity=identity, name=name,
                                      auto_init=auto_init)
        return self.star_camera

    def add_star_camera_from_coarse(self):
        """Set the star camera to be the current coarse camera object."""
        try:
            self.star_camera = self.coarse_camera
            return self.star_camera
        except BaseException:
            self._logger.debug('Could not link star from coarse: ', exc_info=True)
            return None

    def clear_star_camera(self):
        """Set the star camera to None."""
        self.star_camera = None

    @property
    def coarse_camera(self):
        """pypogs.Camera: Get or set the coarse camera. Will create System.coarse_track_thread if
        not existing.
        """
        if self._coarse_track_thread is None:
            return None
        else:
            return self._coarse_track_thread.camera

    @coarse_camera.setter
    def coarse_camera(self, cam):
        self._logger.debug('Got set coarse camera with: ' + str(cam))
        if self.coarse_camera is not None:
            self._logger.debug('Already have a coarse camera, try to clear it')
            try:
                self.coarse_camera.deinitialize()
                self._logger.debug('Deinit')
            except BaseException:
                self._logger.debug('Failed to deinit', exc_info=True)
            self._coarse_track_thread.camera = None
            self._logger.debug('Cleared')
        if cam is not None:
            assert isinstance(cam, Camera), 'Must be pypogs.Camera object'
            if self._coarse_track_thread is None:
                self._logger.info('Creating coarse tracker thread')
                self._coarse_track_thread = TrackingThread(name='CoarseTracker')
            self._coarse_track_thread.camera = cam
        self._logger.debug('Set coarse camera to: ' + str(self.coarse_camera))

    def add_coarse_camera(self, model=None, identity=None, name='CoarseCamera', auto_init=True):
        """Create and set the coarse camera. Calls pypogs.Camera constructor with
        name='CoarseCamera' and the given arguments.

        Args:
            model (str, optional): The model used to determine the correct hardware API. Supported:
                'ptgrey' for PointGrey/FLIR Machine Vision cameras (using Spinnaker/PySpin APIs).
            identity (str, optional): String identifying the device. For *ptgrey* this is
                'serial number' *as a string*.
            name (str, optional): Name for the device, defaults to 'CoarseCamera'.
            auto_init (bool, optional): If both model and identity are given when creating the
                Camera and auto_init is True (the default), Camera.initialize() will be called
                after creation.
        """
        self._logger.debug('Got add coarse camera with model=' + str(model)
                           + ' identity=' + str(identity) + ' auto_init=' + str(auto_init))

        if self.coarse_camera is not None:
            self._logger.debug('Already have a coarse camera with model='
                               + str(self.coarse_camera.model) + ' identity='
                               + str(self.coarse_camera.identity))
            if self.coarse_camera.model == model and self.coarse_camera.identity == identity:
                self._logger.debug('Already attached, just set the name')
                self.coarse_camera.name = name
            else:
                self._logger.debug('Got a different camera, clear the old one')
                self.coarse_camera = None
                self._logger.debug('Create new camera')
                self.coarse_camera = Camera(model=model, identity=identity, name=name,
                                            auto_init=auto_init)
                return self.coarse_camera
        else:
            self._logger.debug('Dont have anything old to clean up, create new camera')
            self.coarse_camera = Camera(model=model, identity=identity, name=name,
                                        auto_init=auto_init)
        return self.coarse_camera

    def add_coarse_camera_from_star(self):
        """Set the coarse camera to be the current star camera object."""
        try:
            self.coarse_camera = self.star_camera
            return self.coarse_camera
        except BaseException:
            self._logger.debug('Could not link coarse from star: ', exc_info=True)
            return None

    def clear_coarse_camera(self):
        """Set the coarse camera to None."""
        self.coarse_camera = None

    @property
    def fine_camera(self):
        """pypogs.Camera: Get or set the fine camera. Will create System.fine_track_thread if not
        existing.
        """
        if self._fine_track_thread is None:
            return None
        else:
            return self._fine_track_thread.camera

    @fine_camera.setter
    def fine_camera(self, cam):
        self._logger.debug('Got set fine camera with: ' + str(cam))
        if self.fine_camera is not None:
            self._logger.debug('Already have a fine camera, try to clear it')
            try:
                self.fine_camera.deinitialize()
                self._logger.debug('Deinit')
            except BaseException:
                self._logger.debug('Failed to deinit', exc_info=True)
            self._fine_track_thread.camera = None
            self._logger.debug('Cleared')
        if cam is not None:
            assert isinstance(cam, Camera), 'Must be pypogs.Camera object'
            if self._fine_track_thread is None:
                self._logger.info('Creating fine tracker thread')
                self._fine_track_thread = TrackingThread(name='FineTracker')
            self._fine_track_thread.camera = cam
        self._logger.debug('Set fine camera to: ' + str(self.fine_camera))

    def add_fine_camera(self, model=None, identity=None, name='FineCamera', auto_init=True):
        """Create and set the fine camera. Calls pypogs.Camera constructor with
        name='FineCamera' and the given arguments.

        Args:
            model (str, optional): The model used to determine the correct hardware API. Supported:
                'ptgrey' for PointGrey/FLIR Machine Vision cameras (using Spinnaker/PySpin APIs).
            identity (str, optional): String identifying the device. For *ptgrey* this is
                'serial number' *as a string*. name (str, optional): Name for the device, defaults
                to 'FineCamera'.
            auto_init (bool, optional): If both model and identity are given when creating the
                Camera and auto_init is True (the default), Camera.initialize() will be called
                after creation.
        """
        self._logger.debug('Got add fine camera with model=' + str(model) + ' identity='
                           + str(identity) + ' auto_init=' + str(auto_init))
        if self.fine_camera is not None:
            self._logger.debug('Already have a fine camera with model='
                               + str(self.fine_camera.model)
                               + ' identity=' + str(self.fine_camera.identity))
            if self.fine_camera.model == model and self.fine_camera.identity == identity:
                self._logger.debug('Already attached, just set the name')
                self.fine_camera.name = name
            else:
                self._logger.debug('Got a different camera, clear the old one')
                self.fine_camera = None
                self._logger.debug('Create new camera')
                self.fine_camera = Camera(model=model, identity=identity, name=name,
                                          auto_init=auto_init)
                return self.fine_camera
        else:
            self._logger.debug('Dont have anything old to clean up, create new camera')
            self.fine_camera = Camera(model=model, identity=identity, name=name,
                                      auto_init=auto_init)
        return self.fine_camera

    def clear_fine_camera(self):
        """Set the fine camera to None."""
        self.fine_camera = None

    @property
    def coarse_track_thread(self):
        """pypogs.TrackingThread: Get the coarse tracking thread."""
        return self._coarse_track_thread

    @property
    def fine_track_thread(self):
        """pypogs.TrackingThread: Get the fine tracking thread."""
        return self._fine_track_thread

    @property
    def receiver(self):
        """Get or set a pypogs.Receiver for the telescope."""
        return self._receiver

    @receiver.setter
    def receiver(self, rec):
        self._logger.debug('Got set receiver with: '+str(rec))
        if self.receiver is not None:
            self._logger.debug('Already have a receiver, try to clear it')
            try:
                self.receiver.deinitialize()
                self._logger.debug('Deinit')
            except BaseException:
                self._logger.debug('Failed to deinit', exc_info=True)
            self._receiver = None
            self._logger.debug('Cleared')
        if rec is not None:
            assert isinstance(rec, Receiver), 'Must be pypogs.Receiver'
            self._receiver = rec
        self._logger.debug('Receiver set to: ' + str(self._receiver))

    def add_receiver(self, *args, **kwargs):
        """Create and set a pypogs.Receiver for the system. Arguments passed to contructor.

        Args:
            model (str, optional): The model used to determine the correct hardware API. Supported:
                'ni_daq' for National Instruments DAQ cards (tested on USB-6211).
            identity (str, optional): String identifying the device and input. For *ni_daq* this is
                'device/input' eg. 'Dev1/ai1' for device 'Dev1' and analog input 1; only
                differential input is supported for *ni_daq*.
            name (str, optional): Name for the device.
            save_path (pathlib.Path, optional): Save path to set. See Receiver.save_path for
                details.
            auto_init (bool, optional): If both model and identity are given when creating the
                Receiver and auto_init is True (the default), Receiver.initialize() will be called
                after creation.
        """
        self._logger.debug('Got add receiver with args: '+str(args)+' kwargs'+str(kwargs))
        if self.receiver is not None:
            self._logger.debug('Already have a receiver, clear first')
            self.receiver = None
        self.receiver = Receiver(*args, **kwargs)
        return self.receiver

    def clear_receiver(self):
        """Set the receiver to None."""
        self.receiver = None

    @property
    def mount(self):
        """pypogs.Mount: Get or set the mount."""
        return self._mount

    @mount.setter
    def mount(self, mount):
        self._logger.debug('Got set mount with: ' + str(mount))
        if self.mount is not None:
            self._logger.debug('Already have a mount, try to clear it')
            try:
                self.mount.deinitialize()
                self._logger.debug('Deinit')
            except BaseException:
                self._logger.debug('Failed to deinit', exc_info=True)
            self._mount = None
            self._logger.debug('Cleared')
        if mount is not None:
            assert isinstance(mount, Mount), 'Must be pypogs.Mount'
            self._mount = mount
        self._logger.debug('Mount set to: ' + str(self._mount))

    def add_mount(self, *args, **kwargs):
        """Create and set a pypogs.Mount for System.mount. Arguments passed to constructor.

        Args:
            model (str, optional): The model used to determine the the hardware control interface.
                Supported: 'celestron' for Celestron NexStar and Orion/SkyWatcher SynScan (all the
                same) hand controller communication over serial.
            identity (str or int, optional): String or int identifying the device. For model
                *celestron* this can either be a string with the serial port (e.g. 'COM3' on
                Windows or '/dev/ttyUSB0' on Linux) or an int with the index in the list of
                available ports to use (e.g. identity=0 i if only one serial device is connected.)
            name (str, optional): Name for the device.
            auto_init (bool, optional): If both model and identity are given when creating the
                Mount and auto_init is True (the default), Mount.initialize() will be called after
                creation.
        """
        self._logger.debug('Got add mount with args: '+str(args)+' kwargs'+str(kwargs))
        if self.mount is not None:
            self._logger.debug('Already have a mount, clear first')
            self.mount = None
        self.mount = Mount(*args, **kwargs)
        return self.mount

    def clear_mount(self):
        """Set the mount to None."""
        self.mount = None

    def do_auto_star_alignment(self, max_trials=1, rate_control=True):
        """Do the auto star alginment procedure by taking eight star images across the sky.

        Will call System.Alignment.set_alignment_from_observations() with the captured images.

        Args:
            max_trials (int, optional): Maximum attempts to take each image and solve the position.
                Default 1.
            rate_control (bool, optional): If True (the default) rate control
                (see pypogs.Mount) is used.
        """
        assert self.mount is not None, 'No mount'
        assert self.star_camera is not None, 'No star camera'
        assert self.is_init, 'System not initialized'
        assert not self.is_busy, 'System is busy'

        def run():
            self._logger.info('Starting auto-alignment.')
            try:
                # TODO: tetra3 should be loaded and configurable from System.
                t3 = Tetra3('default_database')
                pos_list = [(40, -135), (60, -135), (60, -45), (40, -45), (40, 45), (60, 45),
                            (60, 135), (40, 135)]
                alignment_list = []
                start_time = apy_time.now()
                # Create logfile
                data_filename = Path(start_time.strftime('%Y-%m-%dT%H%M%S')
                                     + '_System_star_align.csv')
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
                    writer.writerow(['RA', 'DEC', 'ROLL', 'FOV', 'PROB', 'TIME', 'ALT', 'AZI',
                                     'TRIAL'])

                for idx, (alt, azi) in enumerate(pos_list):
                    self._logger.info('Getting measurement at Alt: ' + str(alt)
                                      + ' Az: ' + str(azi) + '.')
                    self.mount.move_to_alt_az(alt, azi, rate_control=rate_control, block=True)
                    for trial in range(max_trials):
                        assert not self._stop_loop, 'Thread stop flag is set'
                        img = self.star_camera.get_next_image()
                        timestamp = apy_time.now()
                        # TODO: Test
                        fov_estimate = self.star_camera.plate_scale * img.shape[1] / 3600
                        solve = t3.solve_from_image(img, fov_estimate=fov_estimate,
                                                    fov_max_error=.1)
                        self._logger.debug('TIME:  ' + timestamp.iso)
                        # Save image
                        tiff_write(self.data_folder / (start_time.strftime('%Y-%m-%dT%H%M%S')
                                                       + '_Alt' + str(alt) + '_Azi' + str(azi)
                                                       + '_Try' + str(trial+1) + '.tiff'), img)
                        # Save result to logfile
                        with open(data_file, 'a') as file:
                            writer = csv_write(file)
                            data = np.hstack((solve['RA'], solve['Dec'], solve['Roll'],
                                              solve['FOV'], solve['Prob'], timestamp.iso, alt, azi,
                                              trial + 1))
                            writer.writerow(data)
                        if solve['RA'] is not None:
                            break
                        elif trial + 1 < max_trials:
                            self._logger.debug('Failed attempt '+str(trial+1))
                        else:
                            self._logger.debug('Failed attempt '+str(trial+1)+', skipping...')
                    alignment_list.append((solve['RA'], solve['Dec'], timestamp, alt, azi))

                self.mount.move_home(block=False)
                # Set the alignment!
                assert len(alignment_list) > 0, 'Did not identify any star patterns'
                self.alignment.set_alignment_from_observations(alignment_list)
            except AssertionError:
                self._logger.warning('Auto-align failed.', exc_info=True)
                self._stop_loop = True

            self._stop_loop = True
            self.mount.wait_for_move_to()
        self._thread = Thread(target=run)
        self._stop_loop = False
        self._thread.start()

    def get_alt_az_of_target(self, times=None, time_step=.1):
        """Get the corrected altitude and azimuth angles and rates of the target from the current
        alignment.

        Args:
            times (astropy.time.Time, optional): The time(s) to calculate for. If None (the
                default) the current time is used. If time array the calculation is done for each
                time in the array.
            time_step (float, optional): The time step in seconds used to calculate the angular
                rates (default .1).

        Returns:
            numpy.ndarray: Nx2 array with altitude and azimuth angles in degrees.
            numpy.ndarray: Nx2 array with altitude and azimuth rates in degrees per second.
        """
        if times is None:
            times = apy_time.now()
        assert isinstance(times, apy_time), 'Times must be astropy time'
        # Extend the time vector by 0.1 second if only one time
        single_time = (times.size == 1)
        if single_time:
            times += [0, time_step] * apy_unit.second
        dt = (times[1:] - times[:-1]).sec
        assert np.all(dt > EPS), 'Times must be different!'

        if isinstance(self.target.target_object, sgp4.EarthSatellite):
            pos = self.target.get_target_itrf_xyz(times)
            alt_az = self.alignment.get_com_altaz_from_itrf_xyz(pos, position=True)
        elif isinstance(self.target.target_object, apy_coord.SkyCoord):
            vec = self.target.get_target_itrf_xyz(times)
            alt_az = self.alignment.get_com_altaz_from_itrf_xyz(vec)
        else:
            raise RuntimeError('The target is of unknown type!')
        angvel_alt_az = (((alt_az[:, 1:] - alt_az[:, :-1] + 180) % 360) - 180) / dt

        if single_time:
            return alt_az[:, 0], angvel_alt_az[:, 0]
        else:
            return alt_az, np.hstack((angvel_alt_az, angvel_alt_az[:, -1]))

    def get_itrf_direction_of_target(self, times=None):
        """Get direction (unit vector) in ITRF from the telescope position to the target.

        Args:
            times (astropy.time.Time, optional): The time(s) to calculate for. If None (the
                default) the current time is used. If time array the calculation is done for each
                time in the array.

        Returns:
            numpy.ndarray: Shape (3,) if single time, shape (3,N) if array time.
        """
        assert self.target.has_target, 'No target set'
        if times is None:
            times = apy_time.now()
        assert isinstance(times, apy_time), 'Times must be astropy time'
        if isinstance(self.target.target_object, sgp4.EarthSatellite):
            pos = self.target.get_target_itrf_xyz(times)
            itrf_xyz = self.alignment.get_itrf_relative_from_position(pos)
        elif isinstance(self.target.target_object, apy_coord.SkyCoord):
            itrf_xyz = self.target.get_target_itrf_xyz(times)
        else:
            raise RuntimeError('The target is of unknown type!')
        itrf_xyz /= np.linalg.norm(itrf_xyz, axis=0, keepdims=True)
        return itrf_xyz

    def slew_to_target(self, time=None, block=True, rate_control=True):
        """Slew the mount to the defined target.

        Args:
            time (astropy.time.Time, optional): The time to calculate target position. If None (the
                 default) the current time is used.
            block (bool, optional): If True (the default), excecution is blocked until move
                finishes.
            rate_control (bool, optional): If True (the default) rate control
                (see pypogs.Mount) is used.
        """
        assert self.mount is not None, 'No Mount'
        assert self.is_init, 'System not initialized'
        if time is None:
            time = apy_time.now()
        assert isinstance(time, apy_time), 'Times must be astropy time'
        if time.size == 1:
            alt_azi = self.get_alt_az_of_target(time)[0]
        else:
            alt_azi = self.get_alt_az_of_target(time[0])[0]

        self.mount.move_to_alt_az(alt_azi[0], alt_azi[1], block=block)

    def start_tracking(self):
        """Track the target, using closed loop feedback if defined.

        The target will be tracked between the start and end times defined in the target. To stop
        manually call System.stop_tracking().
        """
        assert self.mount is not None, 'No mount'
        assert self.target.has_target, 'No target set'
        assert self.alignment.is_aligned, 'Telescope not aligned'
        assert self.alignment.is_located, 'No telescope location'
        assert self.is_init, 'System not initialized'
        assert not self.is_busy, 'System is busy'
        self._logger.info('Starting closed loop tracking')
        self.control_loop_thread.start()

    def stop(self):
        """Stop all tasks."""
        self._logger.info('Stop command received.')
        try:
            self.control_loop_thread.stop()
        except BaseException:
            self._logger.warning('Failed to stop control loop thread.', exc_info=True)
        if self._thread is not None and self._thread.is_alive:
            self._stop_loop = True
            try:
                self._thread.join()
            except BaseException:
                self._logger.warning('Failed to join system worker thread.', exc_info=True)
        if self.mount is not None and self.mount.is_init:
            try:
                self.mount.stop()
            except BaseException:
                self._logger.warning('Failed to stop mount.', exc_info=True)

    def do_alignment_test(self, max_trials=2, rate_control=True):
        """Move to 40 positions spread across the sky and measure the alignment errors.

        Data is saved to CSV file named with the current time and '_System_align_test_hemisp.csv'.

        Args:
            max_trials (int, optional): Maximum attempts to take each image and solve the position.
                Default 2.
            rate_control (bool, optional): If True (the default) rate control
                (see pypogs.Mount) is used.
        """
        # Took 12 min without backlash_comp (no images)
        # Took 21 min with backlash_comp (no images)
        assert self.alignment.is_aligned, 'Not aligned'
        assert self.mount is not None, 'No mount'
        assert self.star_camera is not None, 'No star camera'
        assert self.is_init, 'System not initialized'
        assert not self.is_busy, 'System is busy'

        # TODO: Thread and test this one
        # TODO: t3 loded and set up from System
        t3 = Tetra3('default_database')
        self._logger.info('Starting alignment test, 2x20 positions.')
        pos_LH = [(53, -16), (71, -23), (80, -9), (44, -114), (56, -135), (50, -100), (65, -65),
                  (26, -72), (23, -30), (59, -37), (35, -177), (47, -142), (20, -86), (38, -79),
                  (41, -51), (77, -2), (74, -170), (29, -44), (62, -156), (68, -163)]
        pos_RH = [(80, 166), (53, 138), (56, 54), (68, 180), (35, 26), (23, 33), (44, 75),
                  (38, 152), (65, 19), (50, 159), (32, 82), (26, 96), (41, 110), (29, 89),
                  (20, 103), (77, 5), (74, 47), (59, 117), (47, 173), (71, 124)]

        test_time = apy_time.now()
        # Create datafile
        data_filename = Path(test_time.strftime('%Y-%m-%dT%H%M%S')+'_System_align_test_hemisp.csv')
        data_file = self.data_folder / data_filename
        if data_file.exists():
            self._log_debug('File name clash. Iterating...')
            append = 1
            while data_file.exists():
                data_file = self.data_folder / (data_filename.stem + str(append)
                                                + data_filename.suffix)
                append += 1
            self._log_debug('Found allowable file: '+str(data_file))
        with open(data_file, 'w+') as file:
            writer = csv_write(file)
            writer.writerow(['RA', 'DEC', 'ROLL', 'FOV', 'PROB', 'TIME', 'ALT', 'AZI', 'ALT_OBS',
                             'AZI_OBS', 'TRIAL'])

        for k in range(2):
            if k == 0:
                self._logger.info('Left half')
                positions = pos_LH
                (alt, azi) = (0, -90)
                altaz = self.alignment.get_com_altaz_from_enu_altaz((alt, azi))
                self.mount.move_to_alt_az(*altaz, block=True)
            else:
                self._logger.info('Right half')
                positions = pos_RH
                (alt, azi) = (0, 90)
                altaz = self.alignment.get_com_altaz_from_enu_altaz((alt, azi))
                self.mount.move_to_alt_az(*altaz, block=True)
            for i in range(len(positions)):
                (alt, azi) = positions[i]
                self._logger.info('Getting measurement at Alt: ' + str(alt) + ' Az: ' + str(azi)
                                  + ' ENU.')
                altaz = self.alignment.get_com_altaz_from_enu_altaz((alt, azi))
                self.mount.move_to_alt_az(*altaz, rate_control=rate_control, block=True)
                for trial in range(max_trials):
                    img = self.star_camera.get_next_image()
                    timestamp = apy_time.now()
                    # TODO: Test
                    fov_estimate = self.star_camera.plate_scale * img.shape[1] / 3600
                    solve = t3.solve_from_image(img, fov_estimate=fov_estimate, fov_max_error=.1)
                    self._logger.debug('TIME:  ' + timestamp.iso)
                    # Save image
                    tiff_write(self.data_folder / (test_time.strftime('%Y-%m-%dT%H%M%S') + '_Alt'
                                                   + str(alt) + '_Azi' + str(azi) + '_Try'
                                                   + str(trial + 1) + '.tiff'), img)
                    if solve['RA'] is not None:
                        # ra,dec,time to ITRF
                        c = apy_coord.SkyCoord(solve['RA'], solve['Dec'], obstime=timestamp,
                                               unit='deg')
                        c = c.transform_to(apy_coord.ITRS)
                        xyz_observed = [c.x.value, c.y.value, c.z.value]
                        (alt_obs, azi_obs) = self.alignment. \
                            get_enu_altaz_from_itrf_xyz(xyz_observed)
                        err_alt = (alt_obs - alt) * 3600
                        err_azi = (azi_obs - azi) * 3600
                        self._logger.info('Observed position degrees Alt: '
                                          + str(round(alt_obs, 3))
                                          + ' Azi:' + str(round(azi_obs, 3)))
                        self._logger.info('Difference measured arcsec Alt: '
                                          + str(round(err_alt, 1))
                                          + ' Azi:' + str(round(err_azi, 1)))
                    else:
                        (alt_obs, azi_obs) = [None]*2
                    # Save result to logfile
                    with open(data_file, 'a') as file:
                        writer = csv_write(file)
                        data = np.hstack((solve['RA'], solve['Dec'], solve['Roll'], solve['FOV'],
                                          solve['Prob'], timestamp.iso, alt, azi, alt_obs, azi_obs,
                                          trial+1))
                        print(data)
                        writer.writerow(data)
                    if solve['RA'] is not None:
                        break
                    elif trial + 1 < max_trials:
                        print('Failed attempt '+str(trial+1))
                    else:
                        print('Failed attempt '+str(trial+1)+', skipping...')

        print('Done! Moving home')
        self.mount.move_to_alt_az(0, 0, block=True)


class Alignment:
    """Alignment and location of a telescope and coordinate transforms.

    Location is the position of the telescope on Earth. It is provided as latitude, longitude,
    height relative to a reference ellipsoid. WGS84 (the GPS ellipsoid) is used in pypogs (and
    almost everywhere else).

    Alignment refers to the different coordinate frames in use to get the telescope to point in the
    correct direction. A direction in some coordinate frame can either be represented as a
    cartesian unit vector or as two angles: altitude and azimuth. In the former case they are
    appended by _xyz and in the latter by _altaz. There are four coodinate frames to consider,
    ITRF, ENU, MNT, and COM, see below for descriptions.

    The fundamental coordinates used are ITRF_xyz unit vectors. They give direction in an earth
    fixed cartesian frame. This can often be called the ECEF (Earth-Centered Earth-Fixed) or ECR
    (Earth-Centered Rotating) frame. pypogs uses external packages (Astropy and Skyfield) to get
    directions in ITRF_xyz, but the other three are managed here.

    If using auto-align, all you really need to know is the ITRF or ENU direction you want
    the telescope to point towards, use this class to get COM_altaz, and send that to the mount.

    Note:
        If you have a ITRF position vector (e.g. from centre of earth to a satellite) use
        Alignment.get_itrf_relative_from_position() to get the unit vector direction from the
        telescope location or pass the optional argument *position=True* when converting from
        ITRF_xyz.

    If you have a telescope with internal alignment and corrective terms such that the communicated
    values with the mount are in traditional ENU_altaz coordinates, set the location and then call
    Alignment.set_alignment_enu(). This will make MNT coincide with the present ENU frame and
    disable corrections.

    To understand this class deeply, one needs to understand the different coordinate systems that
    are used. They are:

        1. ITRF: *International Terrestrial Reference Frame* is the common ECEF (Earth-Centered
           Earth-Fixed) cartesian coordinate frame. This gives a position in (x, y, z) where the
           coordinates rotate along with Earth, so a point on Earth always has the same
           coordinates. (0, 0, 0) is the centre of Earth. This is the "base" coordinate frame
           in this class which everything else is referenced to.
        2. ENU: *East North Up* is the local tangent plane coordinate frame and depends on the
           location of the telescope. The coordinates may be described in their cartesian
           components (e, n, u) but are more commonly referenced by the two angles ENU altitude
           (degrees above the horizon) and ENU azimuth (degrees rotation east from north).
           ENU_altaz is the traditional astronomical coordinate frame seen e.g. in star charts. To
           transform into this frame the location must be known.
        3. MNT: *Mount* is the *local* coordinate system of the telescope mount. This is another
           cartesian coordinate where the coordinate axes coincide with the telescope mount axes.
           In particular, MNT c is the azimuth rotation axis of the gimbal, MNT a is the altitude
           rotation axis of the gimbal (when the gibals are at the zero position), and MNT b
           completes the right hand system (pointing along the telescope boresight). The
           coordinates are commonly expressed as MNT altitude (degrees above the plane normal to
           the azimuth rotation axis) and MNT azimuth (degrees rotation around the azimuth axis).
           Traditionally, telescopes are physically aligned such that MNT and ENU coincide, but we
           relax this harsh constraint via software!
        4. COM: *Commanded* is the angles which must be send to the mount to actually end up at the
           desired MNT_altaz. The nonperpendicularity (Cnp), vertical deflection (Cvd), and
           altitude zero offset (Alt0) of the physical mount must be corrected for to get better
           than about 1000 arcseconds pointing. In an ideal mount COM and MNT coincide.

    Args:
        data_folder (pathlib.Path, optional): The folder for data saving. If None (the default) the
            folder *pypogs*/data will be used/created.
        debug_folder (pathlib.Path, optional): The folder for debug logging. If None (the default)
            the folder *pypogs*/debug will be used/created.
    """
    def __init__(self, data_folder=None, debug_folder=None):
        """Create Alignment instance. See class documentation."""
        # Logger setup
        self._debug_folder = None
        if debug_folder is None:
            self.debug_folder = Path(__file__).parent / 'debug'
        else:
            self.debug_folder = debug_folder
        self._logger = logging.getLogger('pypogs.system.Alignment')
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

        self._logger.debug('Alignment constructor called')
        # Data folder setup
        self._data_folder = None
        if data_folder is None:
            self.data_folder = Path(__file__).parent / 'data'
        else:
            self.data_folder = data_folder
        # Telescope location
        self._telescope_ITRF = None  # Tel. location ITRF (xyz) in metres
        self._location = None  # Telescope location astropy EarthLocation
        # Transformation matrices
        self._MX_itrf2enu = None  # Matrix tranforming vectors in ITRF-xyz to ENU-xyz
        self._MX_enu2itrf = None  # Inverse of above
        self._MX_itrf2mnt = None  # Matrix transforming vectors in ITRF-xyz to MNT-xyz
        self._MX_mnt2itrf = None  # Inverse of above
        # Correction factors
        self._Alt0 = 0.0  # Altitude zero offset
        self._Cvd = 0.0  # Vertical deflection coefficient
        self._Cnp = 0.0  # Nonperpendicularity

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
    def is_aligned(self):
        """bool: Returns True if telescope has location."""
        return self._MX_itrf2mnt is not None

    @property
    def is_located(self):
        """bool: Returns True if telescope has location."""
        return self._location is not None

    def get_location_lat_lon_height(self):
        """tuple of float: Get the location in latitude (deg), longitude (deg), height (m) above
        ellipsoid.
        """
        assert self.is_located, 'Not located'
        gd = self._location.geodetic
        return (gd.lat.to_value(apy_unit.deg),
                gd.lon.to_value(apy_unit.deg),
                gd.height.to_value(apy_unit.m))

    def get_location_itrf(self):
        """numpy.ndarray: Get the location in ITRF x (m), y (m), z (m) as shape (3,) array."""
        assert self.is_located, 'Not located'
        gc = self._location.geocentric
        return np.array((gc[0].to_value(apy_unit.m),
                         gc[1].to_value(apy_unit.m),
                         gc[2].to_value(apy_unit.m)))

    def set_location_lat_lon(self, lat, lon, height=0):
        """Set the location via latitude (deg), longitude (deg), height (m, default 0) above
        ellipsoid.
        """
        self._logger.debug('Got set location with lat=' + str(lat) + ' lon=' + str(lon)
                           + ' height=' + str(height))
        self._location = apy_coord.EarthLocation.from_geodetic(lat=lat, lon=lon, height=height)
        self._logger.debug('Location set to: '+str(self._location))
        # Set ITRF-ENU transformations for new location
        self._set_mx_enu_itrf()

    def set_location_itrf(self, x, y, z):
        """Set the location via ITRF position x (m), y (m), z (m)."""
        self._logger.debug('Got set location with x='+str(x)+' y='+str(y)+' z='+str(z))
        self._location = apy_coord.EarthLocation.from_geocentric(x, y, z, unit=apy_unit.m)
        self._logger.debug('Location set to: '+str(self._location))
        # Set ITRF-ENU transformations for new location
        self._set_mx_enu_itrf()

    def _set_mx_enu_itrf(self):
        """PRIVATE: Calculate transformation matrices between ENU and ITRF based on our location.
        """
        self._logger.debug('Got set MX ENU-ITRF')
        # Get lat lon in radians
        (lat, lon) = np.deg2rad(self.get_location_lat_lon_height()[:2])
        self._logger.debug('At lat='+str(lat)+' lon='+str(lon)+' (rad)')
        # Find basis vectors
        up = np.asarray([np.cos(lat)*np.cos(lon), np.cos(lat)*np.sin(lon), np.sin(lat)])
        north = np.asarray([-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)])
        east = np.asarray([-np.sin(lon), np.cos(lon), 0])
        # Calculate matrices
        self._MX_itrf2enu = np.vstack((east, north, up))
        self._MX_enu2itrf = self._MX_itrf2enu.transpose()
        self._logger.debug('Set MX_enu2itrf to: '+str(self._MX_enu2itrf))
        self._logger.debug('Set MX_itrf2enu to: '+str(self._MX_itrf2enu))

    def get_itrf_relative_from_position(self, itrf_pos):
        """Get the vector in ITRF pointing from the telescope to the given position. Not
        normalised.
        """
        assert self.is_located, 'No location'
        itrf_pos = np.asarray(itrf_pos)
        if itrf_pos.size == 3:  # Makes size 3 vectors into (3,1) vectors
            itrf_pos = itrf_pos.reshape((3, 1))
        assert itrf_pos.shape[0] == 3, 'Input must be size 3 or shape (3,N)'
        # Calculate relative vector
        relative = itrf_pos - self.get_location_itrf().reshape((3, 1))
        return relative.squeeze()

    def get_itrf_xyz_from_enu_altaz(self, enu_altaz):
        """Transform the given ENU AltAz coordinate to ITRF xyz. May be numpy array-like.

        Args:
            enu_altaz (numpy array-like): Size 2 or shape (2,N).

        Returns:
            numpy.ndarray: Shape (3,) if single input, shape (3,N) if array input.
        """
        assert self.is_located, 'Not located'
        enu_altaz = np.asarray(enu_altaz)
        if enu_altaz.size == 2:
            enu_altaz = enu_altaz.reshape((2, 1))
        # Degrees to radians
        enu_alt = np.deg2rad(enu_altaz[0, :])
        enu_azi = np.deg2rad(enu_altaz[1, :])
        # AltAz to xyz
        enu_xyz = np.vstack((np.cos(enu_alt) * np.sin(enu_azi),
                             np.cos(enu_alt) * np.cos(enu_azi),
                             np.sin(enu_alt)))
        # ENU to ITRF
        itrf_xyz = self._MX_enu2itrf @ enu_xyz
        return itrf_xyz.squeeze()

    def get_enu_altaz_from_itrf_xyz(self, itrf_xyz, position=False):
        """Transform the given ITRF xyz coordinates to ENU AltAz.

        Args:
            itrf_xyz (numpy array-like): Size 3 or shape (3,N).
            position(bool, optional): If True, itrf_xyz is treated as the position in ITRF we want
                to observe, and the ENU AltAz required to see it from our location is returned. If
                False (the default), itrf_xyz is treated as a vector with the desired direction to
                point in ITRF.

        Returns:
            numpy.ndarray: Shape (2,) if single input, shape (2,N) if array input.
        """
        assert self.is_located, 'Not located'
        # Make sure we have (3,N) vectors
        itrf_xyz = np.asarray(itrf_xyz)
        if position:  # Get vector pointing in desired direction
            itrf_xyz = self.get_itrf_relative_from_position(itrf_xyz)
        if itrf_xyz.size == 3:  # Makes size 3 vectors into (3,1) vectors
            itrf_xyz = itrf_xyz.reshape((3, 1))
        assert itrf_xyz.shape[0] == 3, 'Input must be size 3 or shape (3,N)'
        # Normalise
        itrf_xyz /= np.linalg.norm(itrf_xyz, axis=0, keepdims=True)
        # ITRF to ENU
        enu_xyz = self._MX_itrf2enu @ itrf_xyz
        # xyz to AltAz
        alt = np.arcsin(enu_xyz[2, :])
        azi = np.arctan2(enu_xyz[0, :], enu_xyz[1, :])
        # radians to degrees
        return np.rad2deg(np.vstack((alt, azi))).squeeze()

    def get_itrf_xyz_from_mnt_altaz(self, mnt_altaz):
        """Transform the given MNT AltAz coordinate to ITRF xyz. May be numpy array-like.

        Args:
            mnt_altaz (numpy array-like): Size 2 or shape (2,N).

        Returns:
            numpy.ndarray: Shape (3,) if single input, shape (3,N) if array input.
        """
        assert self.is_aligned, 'Not aligned'
        mnt_altaz = np.asarray(mnt_altaz)
        if mnt_altaz.size == 2:
            mnt_altaz = mnt_altaz.reshape((2, 1))
        # Degrees to radians
        mnt_alt = np.deg2rad(mnt_altaz[0, :])
        mnt_azi = np.deg2rad(mnt_altaz[1, :])
        # AltAz to xyz
        mnt_xyz = np.asarray([np.cos(mnt_alt) * np.sin(mnt_azi),
                              np.cos(mnt_alt) * np.cos(mnt_azi),
                              np.sin(mnt_alt)])
        # ENU to ITRF
        itrf_xyz = self._MX_mnt2itrf @ mnt_xyz
        return itrf_xyz.squeeze()

    def get_mnt_altaz_from_itrf_xyz(self, itrf_xyz, position=False):
        """Transform the given ITRF xyz coordinates to MNT AltAz.

        Args:
            itrf_xyz (numpy array-like): Size 3 or shape (3,N).
            position(bool, optional): If True, itrf_xyz is treated as the position in ITRF we want
                to observe, and the ENU AltAz required to see it from our location is returned. If
                False (the default), itrf_xyz is treated as a vector with the desired direction to
                point in ITRF.

        Returns:
            numpy.ndarray: Shape (2,) if single input, shape (2,N) if array input.
        """
        assert self.is_aligned, 'Not aligned'
        # Make sure we have (3,N) vectors
        itrf_xyz = np.asarray(itrf_xyz)
        if position:  # Get vector pointing in desired direction
            itrf_xyz = self.get_itrf_relative_from_position(itrf_xyz)
        if itrf_xyz.size == 3:  # Makes size 3 vectors into (3,1) vectors
            itrf_xyz = itrf_xyz.reshape((3, 1))
        assert itrf_xyz.shape[0] == 3, 'Input must be size 3 or shape (3,N)'
        # Normalise
        itrf_xyz /= np.linalg.norm(itrf_xyz, axis=0, keepdims=True)
        # ITRF to MNT
        mnt_xyz = self._MX_itrf2mnt @ itrf_xyz
        # xyz to AltAz
        alt = np.arcsin(mnt_xyz[2, :])
        azi = np.arctan2(mnt_xyz[0, :], mnt_xyz[1, :])
        # radians to degrees
        return np.rad2deg(np.vstack((alt, azi))).squeeze()

    def get_mnt_altaz_from_com_altaz(self, com_altaz):
        """Transform the given COM AltAz coordinate to MNT AltAz.

        Args:
            mnt_altaz (numpy array-like): Size 2 or shape (2,N).

        Returns:
            numpy.ndarray: Shape (2,) if single input, shape (2,N) if array input.
        """
        com_altaz = np.asarray(com_altaz)
        if com_altaz.size == 2:
            com_altaz = com_altaz.reshape((2, 1))
        # Undo corrections
        mnt_alt = (com_altaz[0, :] - self._Alt0) / (1 + self._Cvd)
        mnt_azi = com_altaz[1, :] + self._Cnp * np.tan(np.deg2rad(mnt_alt))
        return np.vstack((mnt_alt, mnt_azi)).squeeze()

    def get_com_altaz_from_mnt_altaz(self, mnt_altaz):
        """Transform the given MNT AltAz coordinate to COM AltAz.

        Args:
            mnt_altaz (numpy array-like): Size 2 or shape (2,N).

        Returns:
            numpy.ndarray: Shape (2,) if single input, shape (2,N) if array input.
        """
        mnt_altaz = np.asarray(mnt_altaz)
        if mnt_altaz.size == 2:
            mnt_altaz = mnt_altaz.reshape((2, 1))
        # Add corrections
        com_azi = mnt_altaz[1, :] - self._Cnp * np.tan(np.deg2rad(np.clip(mnt_altaz[0, :],
                                                                          -85, 85)))
        com_alt = (1 + self._Cvd) * mnt_altaz[0, :] + self._Alt0
        return np.vstack((com_alt, com_azi)).squeeze()

    # Below here are just longer chainings of transformations
    def get_mnt_altaz_from_enu_altaz(self, enu_altaz):
        """Transform the given ENU AltAz coordinate to MNT AltAz.

        Args:
            enu_altaz (numpy array-like): Size 2 or shape (2,N).

        Returns:
            numpy.ndarray: Shape (2,) if single input, shape (2,N) if array input.
        """
        # ENU>ITRF
        itrf_xyz = self.get_itrf_xyz_from_enu_altaz(enu_altaz)
        # ITRF>MNT
        return self.get_mnt_altaz_from_itrf_xyz(itrf_xyz)

    def get_enu_altaz_from_mnt_altaz(self, mnt_altaz):
        """Transform the given MNT AltAz coordinate to ENU AltAz.

        Args:
            mnt_altaz (numpy array-like): Size 2 or shape (2,N).

        Returns:
            numpy.ndarray: Shape (2,) if single input, shape (2,N) if array input.
        """
        # MNT>ITRF
        itrf_xyz = self.get_itrf_xyz_from_mnt_altaz(mnt_altaz)
        # ITRF>ENU
        return self.get_enu_altaz_from_itrf_xyz(itrf_xyz)

    def get_com_altaz_from_enu_altaz(self, enu_altaz):
        """Transform the given ENU AltAz coordinate to COM AltAz.

        Args:
            enu_altaz (numpy array-like): Size 2 or shape (2,N).

        Returns:
            numpy.ndarray: Shape (2,) if single input, shape (2,N) if array input.
        """
        # ENU>MNT
        mnt_altaz = self.get_mnt_altaz_from_enu_altaz(enu_altaz)
        # MNT>COM
        return self.get_com_altaz_from_mnt_altaz(mnt_altaz)

    def get_enu_altaz_from_com_altaz(self, com_altaz):
        """Transform the given COM AltAz coordinate to ENU AltAz.

        Args:
            mnt_altaz (numpy array-like): Size 2 or shape (2,N).

        Returns:
            numpy.ndarray: Shape (2,) if single input, shape (2,N) if array input.
        """
        # COM>MNT
        mnt_altaz = self.get_mnt_altaz_from_com_altaz(com_altaz)
        # COM>ENU
        return self.get_enu_altaz_from_com_altaz(mnt_altaz)

    def get_com_altaz_from_itrf_xyz(self, itrf_xyz, position=False):
        """Transform the given ITRF xyz coordinates to COM AltAz.

        Args:
            itrf_xyz (numpy array-like): Size 3 or shape (3,N).
            position(bool, optional): If True, itrf_xyz is treated as the position in ITRF we want
                to observe, and the ENU AltAz required to see it from our location is returned. If
                False (the default), itrf_xyz is treated as a vector with the desired direction to
                point in ITRF.

        Returns:
            numpy.ndarray: Shape (2,) if single input, shape (2,N) if array input.
        """
        mnt_altaz = self.get_mnt_altaz_from_itrf_xyz(itrf_xyz, position=position)
        return self.get_com_altaz_from_mnt_altaz(mnt_altaz)

    def set_alignment_from_observations(self, obs_data, alt0=None, Cvd=None, Cnp=None):
        """Use star camera/plate solving observations to set the alignment and mount correction
        terms.

        The observation data must be a list containing eight tuples, each from observing these
        specific COM AltAz coordinates in order: (40,-135), (60,-135), (60,-45), (40,-45), (40,45),
        (60,45), (60,135), (40,135). Each tuple in the list must be a 5-tuple with: (Right
        Ascension (deg), Declination (deg), timestamp (astropy Time), COM Alt (deg), COM Az (deg))
        for the measurement. Set Right Ascension and Declination to None if the measurement failed.

        This method also solves for the mount correction terms alt0, Cvd, Cnp. If some of these are
        well known pass the argument to use that value instead of solving for it. You may also pass
        zeros to disable the corrections.

        Args:
            obs_data (list of tuple): See specifics above. The correct sequence is generated from
                System.do_auto_star_alignment().
            alt0 (float, optional): Altitude offset (deg). If None (default) it will be solved for.
            Cvd (float, optional): Vertical deflecion coefficient. If None (default) it will be
                solved for.
            Cnp (float, optional): Axes nonperpendicularity (deg). If None (default) it will be
                solved for.
        """
        self._logger.debug('Got set alignment from obs with: obs_data=' + str(obs_data) + ' alt0='
                           + str(alt0) + ' Cvd=' + str(Cvd) + ' Cnp=' + str(Cvd))
        assert len(obs_data) == 8, 'Must have 8 observations'
        assert all(isinstance(obs, tuple) and len(obs) == 5 for obs in obs_data), \
            'Observation must be tuple with [RA,DEC,TIME,ALT,AZ]'
        assert all(isinstance(obs[2], apy_time) for obs in obs_data), \
            'Timestamp must be astropy object'
        self._logger.info('Calculating alignment and corrections from observations.')
        # Check which have data
        valid = np.fromiter((k[0] is not None for k in obs_data), bool)
        self._logger.debug('Valid: ' + str(valid))
        # Convert measurements to ITRF:
        obs_in_itrf = np.zeros((3, 8))
        for obs in range(len(valid)):
            if valid[obs]:
                # Convert Ra,Dec,Time to ECEF:
                c = apy_coord.SkyCoord(obs_data[obs][0], obs_data[obs][1],
                                       obstime=obs_data[obs][2], unit='deg', frame='icrs')
                c = c.transform_to(apy_coord.ITRS)
                obs_in_itrf[:, obs] = [c.x.value, c.y.value, c.z.value]
            else:
                self._logger.debug('Skipping observation ' + str(obs))
        self._logger.debug('In ITRF: ' + str(obs_in_itrf))
        # Pairs with same alt and opposite azi
        opposing_pairs = ([0, 4], [1, 5], [3, 7], [2, 6])
        # Pairs with same azi different alt
        azi_pairs = ([0, 1], [2, 3], [4, 5], [6, 7])

        # 1) Estimate azimuth axis
        Mz_obs = np.zeros((3, len(opposing_pairs)))
        n = 0
        for pos in opposing_pairs:
            if np.all(valid[pos]):
                Mz_obs[:, n] = np.sum(obs_in_itrf[:, pos], axis=1)
                Mz_obs[:, n] = Mz_obs[:, n] / np.linalg.norm(Mz_obs[:, n])
                n += 1
        assert n > 1, 'Less than two valid opposing pairs'
        Mz_obs = Mz_obs[:, :n]  # Trim the end!
        self._logger.debug('c-axis estimates: ' + str(Mz_obs))
        # Average
        Mz = np.sum(Mz_obs, axis=1)
        Mz = Mz / np.linalg.norm(Mz)
        # Find residuals
        Mz_sstd = np.sqrt(np.sum(np.rad2deg(np.arccos(np.dot(Mz, Mz_obs)))**2) / (n - 1))
        self._logger.info('Solved MNT c axis: ' + str(np.round(Mz, 3).squeeze()) + ' | RMS: '
                          + str(round(Mz_sstd * 3600, 1)) + ' asec (n=' + str(n) + ').')
        # 2) Calculate true alt and vector projected into plane perp to c for observations
        alt_meas = np.zeros((len(valid),))
        V_perp = np.zeros((3, len(valid)))
        for obs in range(len(valid)):
            if valid[obs]:
                alt_meas[obs] = 90 - np.rad2deg(np.arccos(np.dot(Mz, obs_in_itrf[:, obs])))
                V_perp[:, obs] = obs_in_itrf[:, obs] - np.dot(obs_in_itrf[:, obs], Mz) * Mz
                V_perp[:, obs] = V_perp[:, obs] / np.linalg.norm(V_perp[:, obs])
        self._logger.debug('Measured MNT Alt: ' + str(alt_meas))
        self._logger.debug('Perpendicular vector: ' + str(V_perp))
        # 3) Calculate vertical deflection
        if Cvd is None:
            Cvd_obs = np.zeros((len(azi_pairs),))
            n = 0
            for pos in azi_pairs:
                if np.all(valid[pos]):
                    Cvd_obs[n] = (obs_data[pos[0]][3] - obs_data[pos[1]][3]) \
                                / (alt_meas[pos[0]] - alt_meas[pos[1]]) - 1
                    n += 1
            assert n > 1, 'Less than two valid stacked pairs'
            Cvd_obs = Cvd_obs[:n]  # Trim the end!
            self._logger.debug('Cvd estimates: ' + str(Cvd_obs))
            # Average and find residuals
            Cvd = np.mean(Cvd_obs)
            Cvd_sstd = np.sqrt(np.sum((Cvd_obs - Cvd)**2) / (n - 1))
            self._logger.info('Solved Vert. Defl.: ' + str(round(Cvd * 100, 3)) + '% | RMS: '
                              + str(round(Cvd_sstd * 100, 3)) + '% (n=' + str(n) + ').')
        else:
            self._logger.info('Given Vert. Defl.: ' + str(np.round(Cvd * 100, 3)) + '%.')
            Cvd_sstd = -1
        # 4) Find alt offset
        if alt0 is None:
            alt0_obs = np.zeros((len(valid),))
            n = 0
            for obs in range(len(valid)):
                if valid[obs]:
                    alt0_obs[n] = obs_data[obs][3] - (1 + Cvd)*alt_meas[obs]
                    n += 1
            alt0_obs = alt0_obs[:n]  # Trim the end!
            self._logger.debug('Alt0 estimates: ' + str(alt0_obs))
            # Average and find residuals
            alt0 = np.mean(alt0_obs)
            alt0_sstd = np.sqrt(np.sum((alt0_obs - alt0)**2) / (n - 1))
            self._logger.info('Solved Alt. Zero: ' + str(round(alt0, 4)) + DEG + ' | RMS: '
                              + str(round(alt0_sstd, 4)) + DEG + ' (n=' + str(n) + ').')
        else:
            self._logger.info('Given Alt. Zero: ' + str(round(alt0, 4)) + DEG + '.')
            alt0_sstd = -1
        # 5) Find nonperpendicularity
        if Cnp is None:
            Cnp_obs = np.zeros((len(azi_pairs),))
            n = 0
            for pos in azi_pairs:
                if np.all(valid[pos]):
                    d_Azi = np.rad2deg(np.arcsin(np.dot(Mz, np.cross(V_perp[:, pos[0]],
                                                                     V_perp[:, pos[1]]))))
                    Cnp_obs[n] = d_Azi / (np.tan(np.deg2rad(alt_meas[pos[0]]))
                                          - np.tan(np.deg2rad(alt_meas[pos[1]])))
                    n += 1
            Cnp_obs = Cnp_obs[:n]  # Trim the end!
            self._logger.debug('Cnp estimates: ' + str(Cnp_obs))
            # Average and find residuals
            Cnp = np.mean(Cnp_obs)
            Cnp_sstd = np.sqrt(np.sum((Cnp_obs - Cnp)**2) / (n - 1))
            self._logger.info('Solved Nonperp.: ' + str(round(Cnp, 4)) + DEG + ' | RMS: '
                              + str(round(Cnp_sstd, 4)) + DEG + ' (n=' + str(n) + ').')
        else:
            self._logger.info('Given Nonperp.: ' + str(round(Cnp, 4)) + DEG + '.')
            Cnp_sstd = -1
        # 6) Backcalculate azi correction
        azi_backcalc = np.zeros((len(valid),))
        for obs in range(len(valid)):
            azi_backcalc[obs] = obs_data[obs][4] + Cnp * np.tan(np.deg2rad(alt_meas[obs]))
        self._logger.debug('Backcalculated MNT Azi: ' + str(azi_backcalc))
        # 7) Rotate around azi axis to find y
        My_obs = np.zeros((3, len(valid)))
        n = 0
        for obs in range(len(valid)):
            if valid[obs]:
                rad = np.deg2rad(azi_backcalc[obs])
                My_obs[:, n] = (V_perp[:, obs] * np.cos(rad)
                                + (np.cross(Mz, V_perp[:, obs])) * np.sin(rad))
                n += 1
        My_obs = My_obs[:, :n]  # Trim the end!
        self._logger.debug('b-axis estimates: ' + str(My_obs))
        # Average
        My = np.sum(My_obs, axis=1)
        My = My / np.linalg.norm(My)
        # Find residuals
        My_sstd = np.sqrt(np.sum(np.rad2deg(np.arccos(np.dot(My, My_obs)))**2) / (n - 1))
        self._logger.info('Solved MNT b axis: ' + str(np.round(My, 3).squeeze()) + ' | RMS: '
                          + str(round(My_sstd * 3600, 1)) + ' asec (n=' + str(n) + ').')
        # 8) Cross product for last axis
        Mx = np.cross(My, Mz)
        self._logger.info('MNT b cross c gives a: ' + str(np.round(Mx, 3).squeeze()) + '.')
        # Set the values
        self._MX_itrf2mnt = np.vstack((Mx, My, Mz))
        self._MX_mnt2itrf = self._MX_itrf2mnt.transpose()
        self._Alt0 = alt0
        self._Cvd = Cvd
        self._Cnp = Cnp

        # Create logfile
        log_path = self.data_folder / (apy_time.now().strftime('%Y-%m-%dT%H%M%S')
                                       + '_Alignment_from_obs.csv')
        with open(log_path, 'w') as logfile:
            logwriter = csv_write(logfile)
            logwriter.writerow(['Ma', 'Mb', 'Mc', 'Alt0', 'Cvd', 'Cnp', 'Mz_std', 'My_std',
                                'Alt0_std', 'Cvd_std', 'Cnp_std'])
            logwriter.writerow([Mx, My, Mz, alt0, Cvd, Cnp, Mz_sstd, My_sstd,
                                alt0_sstd, Cvd_sstd, Cnp_sstd])

        # Find residuals!
        compare = self.get_com_altaz_from_itrf_xyz(obs_in_itrf[:, valid])
        n = 0
        for i in range(len(valid)):
            if valid[i]:
                d_alt = compare[0, n] - obs_data[i][3]
                d_azi = compare[1, n] - obs_data[i][4]
                self._logger.info('Residual for measurement ' + str(i + 1)
                                  + ': Alt: ' + str(np.round(d_alt * 3600, 1))
                                  + ' Azi: ' + str(np.round(d_azi * 3600, 1)) + ' (asec).')
                n += 1
            else:
                self._logger.info('Residual for measurement ' + str(i + 1)
                                  + ': Alt: ----  Azi: ---- .')
        self._logger.info('Aligned.')

    def set_alignment_enu(self):
        """Set the MNT frame equal to the current ENU frame and zero all correction terms.

        Note:
            If the location is changed after set_alignment_enu() the MNT frame will not be updated
            automatically to coincide with the new ENU.
        """
        assert self.is_located, 'No location'
        self._MX_itrf2mnt = self._MX_itrf2enu
        self._MX_mnt2itrf = self._MX_enu2itrf
        self._Alt0 = 0
        self._Cvd = 0
        self._Cnp = 0


class Target:
    """Target to track and start and end times.

    Two types of targets are supported, Astropy *SkyCoord* (any deep space object) and Skyfield
    *EarthSatellite* (an Earth orbiting satellite).

    The target can be set by the property Target.target_object to one of the supported types. It
    can also be created from the right ascension and declination (for deep sky) or the Two Line
    Element (TLE) for a satellite. See Target.set_target_from_ra_dec() and
    Target.set_target_from_tle() respectively.

    You may also give a start and end time (e.g. useful for satellite rise and set times) when
    creating the target or by the method Target.set_start_end_time().

    With a target set, get the ITRF_xyz coordinates at your prefered times with
    Target.get_target_itrf_xyz().

    Note:
        - If the target is a SkyCoord, the ITRF coordinates will be a unit vector in the direction
          of the target.
        - If the target is an EarthSatellite, the ITRF coordinates will be an absolute position (in
          metres) from the centre of Earth to the satellite.
    """

    _allowed_types = (apy_coord.SkyCoord, sgp4.EarthSatellite)

    def __init__(self):
        """Create Alignment instance. See class documentation."""
        self._target = None
        self._rise_time = None
        self._set_time = None

        self._tle_line1 = None
        self._tle_line2 = None
        self._skyfield_ts = sf_api.Loader(_system_data_dir, expire=False).timescale()

    @property
    def has_target(self):
        """bool: Returns True if a target is set."""
        return self._target is not None

    @property
    def target_object(self):
        """Astropy *SkyCoord* or Skyfield *EarthSatellite* or None: Get or set the target object.
        """
        return self._target

    @target_object.setter
    def target_object(self, target):
        if target is None:
            self._target = None
        else:
            assert isinstance(target, self._allowed_types), \
                'Must be None or of type ' + str(self._allowed_types)
            self._target = target

    def set_target_from_ra_dec(self, ra, dec, start_time=None, end_time=None):
        """Create an Astropy *SkyCoord* and set as the target.

        Args:
            ra (float): Right ascension in decimal degrees.
            dec (float): Declination in decimal degrees.
            start_time (astropy *Time*, optional): The start time to set.
            end_time (astropy *Time*, optional): The end time to set.
        """
        self.target_object = apy_coord.SkyCoord(ra, dec, unit='deg')
        self.set_start_end_time(start_time, end_time)

    def set_target_deep_by_name(self, name, start_time=None, end_time=None):
        """Use Astropy name lookup for setting a SkyCoord deep sky target.

        Args:
            name (str): Name to search for.
            start_time (astropy *Time*, optional): The start time to set.
            end_time (astropy *Time*, optional): The end time to set.
        """
        self.target_object = apy_coord.SkyCoord.from_name(name)
        self.set_start_end_time(start_time, end_time)

    def set_target_from_tle(self, tle, start_time=None, end_time=None):
        """Create a Skyfield *EarthSatellite* and set as the target.

        Args:
            tle (tuple or str): 2-tuple with the two TLE rows or a string with newline character
                between lines.
            start_time (astropy *Time*, optional): The start time to set.
            end_time (astropy *Time*, optional): The end time to set.
        """
        if isinstance(tle, str):
            tle = tle.splitlines()
        tle = tuple(tle)
        self.target_object = sgp4.EarthSatellite(tle[0], tle[1])
        self._tle_line1 = tle[0].strip()
        self._tle_line2 = tle[1].strip()
        self.set_start_end_time(start_time, end_time)

    @property
    def start_time(self):
        """astropy *Time* or None: Get or set the tracking start time."""
        return self._rise_time

    @start_time.setter
    def start_time(self, time):
        if time is None:
            self._rise_time = None
        elif isinstance(time, apy_time):
            self._rise_time = time
        else:
            self._rise_time = apy_time(time)

    @property
    def end_time(self):
        """astropy *Time* or None: Get or set the tracking end time."""
        return self._set_time

    @end_time.setter
    def end_time(self, time):
        if time is None:
            self._set_time = None
        elif isinstance(time, apy_time):
            self._set_time = time
        else:
            self._set_time = apy_time(time)

    def set_start_end_time(self, start_time=None, end_time=None):
        """Set the start and end times.

        Args:
            start_time (astropy *Time*, optional): The start time to set.
            end_time (astropy *Time*, optional): The end time to set.
        """
        self.start_time = start_time
        self.end_time = end_time

    def clear_start_end_time(self):
        """Set both start and end time to None."""
        self.start_time = None
        self.end_time = None

    def get_tle_raw(self):
        """Get the TLE of the target.

        Returns:
            tuple of str

        Raises:
            AssertionError: If the current target is not a Skyfield *EarthSatellite*.
        """
        assert isinstance(self._target, sgp4.EarthSatellite), \
            'Current target is not a Skyfield EarthSatellite.'
        return (self._tle_line1, self._tle_line2)

    def get_short_string(self):
        """Get a short descriptive string of the target.

        Returns:
            str
        """
        if self._target is None:
            return 'No target'
        elif isinstance(self._target, sgp4.EarthSatellite):
            return 'TLE #' + str(self._target.model.satnum)
        elif isinstance(self._target, apy_coord.SkyCoord):
            return 'RA:' + str(round(self._target.ra.to_value('deg'), 2)) + DEG \
                   + ' D:' + str(round(self._target.dec.to_value('deg'), 2)) + DEG

    def get_target_itrf_xyz(self, times=None):
        """Get the ITRF_xyz position vector from the centre of Earth to the EarthSatellite (in
        metres) or the ITRF_xyz unit vector to the SkyCoord.

        Args:
            times (Astropy *Time*): Single or array Time for when to calculate the vector.

        Returns:
            numpy.ndarray: Shape (3,) if single input, shape (3,N) if array input.
        """
        assert self.has_target, 'No target set.'
        if times is None:
            times = apy_time.now()
        if isinstance(self._target, apy_coord.SkyCoord):
            itrs = self._target.transform_to(apy_coord.ITRS(obstime=times))
            return np.array(itrs.data.xyz)
        elif isinstance(self._target, sgp4.EarthSatellite):
            ts_time = self._skyfield_ts.from_astropy(times)
            itrf_xyz = (self._target.ITRF_position_velocity_error(ts_time)[0]
                        * apy_unit.au.in_units(apy_unit.m))
            return itrf_xyz
