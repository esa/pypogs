"""Mount interfaces
======================

Current harware support:
    - :class:`pypogs.Mount`: 'Celestron' for Celestron, Orion and SkyWatcher telescopes (using NexStar serial protocol). No additional
      packages required. Tested with Celestron model CPC800.
    - :class:`pypogs.Mount`: 'ASCOM' for ASCOM-enabled mounts. Requires ASCOM platform and mount driver.

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
from time import sleep, time as timestamp
from datetime import datetime
from threading import Thread, Event
import pythoncom
from struct import pack as pack_data
from enum import Enum

# External imports:
import numpy as np
import serial

class Mount:
    """Control a telescope gimbal mount.

    To initialise a Mount a *model* (determines hardware interface) and *identity* (identifying the specific device)
    must be given. If both are given to the constructor the Mount will be initialised immediately (unless
    auto_init=False is passed). Manually initialise with a call to Mount.initialize(); release hardware with a call to
    Mount.deinitialize().

    After the Mount is initialised, the gimbal angles and rates may be read and commanded. Several properties (e.g
    maximum angles and rates) may be set.

    Args:
        model (str, optional): The model used to determine the the hardware control interface. 
            Supported: 
               'Celestron' for Celestron NexStar and Orion/SkyWatcher SynScan (all the same) hand controller communication over serial.
               'ASCOM'     for ASCOM-enabled telescope mounts.
        identity (str or int, optional): String or int identifying the device. For model *Celestron* this can either be
            a string with the serial port (e.g. 'COM3' on Windows or '/dev/ttyUSB0' on Linux) or an int with the index
            in the list of available ports to use (e.g. identity=0 i if only one serial device is connected.)
            For model *ASCOM* this can either be left blank to invoke the ASCOM telescope selection menu, or may specify 
            a specific installed ASCOM driver by (case sensitive) name (e.g. DeviceHub, Celestron, Simulator, SkyWatcher, etc).
        name (str, optional): Name for the device.
        auto_init (bool, optional): If both model and identity are given when creating the Mount and auto_init
            is True (the default), Mount.initialize() will be called after creation.
        debug_folder (pathlib.Path, optional): The folder for debug logging. If None (the default)
            the folder *pypogs*/logs will be used/created.

    Example:
        ::

            # Create instance (will auto initialise)
            mount = pypogs.Mount(model='Celestron', identity='COM3', name='CPC800')
            # Move to position
            mount.move_to_alt_az(30, 10) #degrees; by default blocks until finished
            # Set gimbal rates
            mount.set_rate_alt_az(0, -1.5) #degrees per second
            # Wait for a while
            time.sleep(2)
            # Stop moving
            mount.stop()
            # Disconnect from the mount
            mount.deinitialize()

    Note:
        The Mount class allows two modes of control for moving to positions. The default is rate_control=True, where
        this class will continuously send rate commands until the desired position is reached. It is possible to use the
        internal motion controller in the mount by passing rate_control=False. However, it is slow and implements
        backlash compensation. In our testing the accuracy difference is negligible so the default is recommended.
    """

    _supported_models = ('Celestron','ASCOM','iOptron AZMP')
    _default_model = 'ASCOM'


    def __init__(self, model=None, identity=None, name=None, auto_init=True, debug_folder=None, **properties):
        """Create Mount instance. See class documentation."""
        # Logger setup
        self._debug_folder = None
        if debug_folder is None:
            self.debug_folder = Path(__file__).parent.parent / 'debug'
        else:
            self.debug_folder = debug_folder
        self._logger = logging.getLogger('pypogs.hardware.Mount')
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
        self._logger.debug('Mount called with model=' + str(model) + ' identity=' + str(identity) + ' name=' \
                           + str(name) + ' auto_init=' + str(auto_init))
                           
        self._serial_baud_rate = {
            'Celestron':    9600,
            'iOptron AZMP': 115200,
        }
        self._serial_is_init = False
        self._model = None
        self._identity = None
        self._name = 'UnnamedMount'
        self._is_init = False
        self._max_speed = (4.0, 4.0) #(alt,azi) degrees/sec
        self._alt_limit = (-5, 95) #limit degrees
        self._azi_limit = (None, None) #limit degees
        self._home_pos = (0, 0) #Home position
        self._alt_zero = 0 #Amount to subtract from alt.
        self._is_sidereal_tracking = False
        self._axis_directions = (1, 1)  #set to 1 to use mount default axis direction, -1 to invert direction
        self._supported_models_lowercase = [x.lower() for x in self._supported_models]
        # Only used for model Celestron
        self._serial_port = None
        # Only used for model iOptron AZMP
        self._azmp_command_modes = {b'5035': 'normal', b'9035': 'special'}
        self._azmp_states = {
            '0': 'stopped at non-zero pos', 
            '1': 'tracking with PEC disabled',
            '2': 'slewing',
            '3': 'autoguiding',
            '4': 'meridian flipping',
            '5': 'tracking with PEC enabled',
            '6': 'parked',
            '7': 'stopped at zero pos'
        }
        self._azmp_command_mode_names = self._azmp_command_modes.values()
        self._azmp_command_mode_code = b''
        # Only used for model ASCOM
        self._ascom_telescope = None
        self._ascom_scope_alt_axis = 1
        self._ascom_scope_azi_axis = 0
        self._ascom_availableRatesAlt = [0]
        self._ascom_availableRatesAzi = [0]
        self._ascom_driver_handler = None
        # Thread for rate control
        self._control_thread = None
        self._control_thread_stop = True
        # Cache of the state of the mount
        self._state_cache = {'alt': 0.0, 'azi': 0.0, 'alt_rate': 0.0, 'azi_rate': 0.0}
        if name is not None:
            self.name = name
        if model is not None:
            self.model = model
        if identity is not None:
            self.identity = identity
        if model is not None:
            self.initialize()
            
        available_properties = self.available_properties
        for property_name in properties:
            if property_name in available_properties:
                self._logger.debug('Setting mount property "%s" to value "%s"' % (property_name, properties[property_name]))
                try:
                    setattr(self, property_name, properties[property_name])
                except:
                    self._logger.warning('Failed to set mount property "%s" to value "%s"' % (property_name, properties[property_name]))
            
        # Try to get Python to clean up the object properly
        import atexit, weakref
        atexit.register(weakref.ref(self.__del__))
        self._logger.info('Mount instance created with name: ' + self.name + '.')

    def __del__(self):
        """Destructor, try to stop the mount and disconnect."""
        try:
            self._logger.debug('Destructor called, try stop moving and disconnecting')
        except:
            pass
        try:
            self.stop()
            self._logger.debug('Stopped')
        except:
            pass
        try:
            self.deinitialize()
            self._logger.debug('Deinitialised')
        except:
            pass
        try:
            self._logger.debug('Destructor finished')
        except:
            pass

    @property
    def debug_folder(self):
        """pathlib.Path: Get or set the path for debug logging. Will create folder if not
        existing."""
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
    def state_cache(self):
        """dict: Get cache with the current state of the Mount. Updates on calls to get_alt_az() and set_rate_alt_az().

        Keys:
            azi: float, alt: float, azi_rate: float, alt_rate: float
        """
        if self.is_init:
            return self._state_cache
        else:
            return None

    @property
    def name(self):
        """str: Get or set the name."""
        return self._name
    @name.setter
    def name(self, name):
        self._logger.debug('Setting name to: '+str(name))
        self._name = str(name)
        self._logger.debug('Name set to '+str(self.name))

    @property
    def model(self):
        """str: Get or set the device model.

        Supported:
            - 'Celestron' for Celestron NexStar and Orion/SkyWatcher SynScan hand controllers over serial.
            - 'ASCOM'     for ASCOM-enabled telescope mounts.
        - This will determine which hardware interface is used.
        - Must set before initialising the device and may not be changed for an initialised device.
        """
        return self._model
    @model.setter
    def model(self, model):
        assert not self.is_init, 'Can not change already initialised device model'
        model = str(model).lower()
        self._logger.debug('Setting model to: '+model)
        assert model.lower() in self._supported_models_lowercase,\
                                                'Model type not recognised, allowed: '+str(self._supported_models)
        self._model = model
        self._logger.debug('Model set to '+str(self.model))

    @property
    def identity(self):
        """str: Get or set the device and/or input. Model must be defined first.

        - For model *celestron* or *iptron azmp* this can either be a string with the serial port (e.g. 'COM3' on
          Windows or '/dev/ttyUSB0' on Linux) or an int with the index in the list of available ports to use (e.g.
          identity=0 i if only one serial device is connected.)
        - For model *ASCOM* this can either be left blank to invoke the ASCOM telescope selection menu, or may specify 
          a specific installed ASCOM driver by name (case sensitive) (e.g. DeviceHub, Celestron, Simulator, SkyWatcher, etc).
        - Must set before initialising the device and may not be changed for an initialised device.

        Raises:
            AssertionError: if unable to connect to and verify identity of the mount.
        """
        return self._identity
    @identity.setter
    def identity(self, identity):
        self._logger.debug('Mount identity setter called with "'+str(identity)+'"')    
        assert not self.is_init, 'Can not change already initialised device'
        assert self.model is not None, 'Must define model first'
                
        if self.model.lower() == 'celestron':
            self._logger.debug('Using %s, try to open serial port and confirm model' % self.model)
            serial_port_name = self._serial_find_port(identity) if identity.isnumeric() else identity
            self._logger.debug('Opening serial port: ' + str(serial_port_name))
            r = self._serial_test(serial_port_name, test_command='m', baud=9600, nbytes=2)
            assert r is not None and len(r)>0, 'Serial port "%s" test failed' % (serial_port_name)
            assert len(r)==2 and r[1] == ord('#'), 'Did not get the expected response from the device'
            self._logger.debug('Setting identity to: '+serial_port_name)
            self._identity = serial_port_name
            
        elif self.model.lower() == 'ioptron azmp':
            self._logger.debug('Using %s with string identity, try to open and check model' % self.model)
            serial_port_name = self._serial_find_port(identity) if identity.isnumeric() else identity
            self._logger.debug('Opening serial port: ' + str(serial_port_name))
            r = self._serial_test(serial_port_name, test_command=':MountInfo#', baud=115200, nbytes=4)
            assert r is not None and len(r)>0, 'Serial port "%s" test failed' % (serial_port_name)
            assert len(r)==4 and r == b'5035' or r == b'9035', 'Did not get the expected response from the device'
            self._logger.debug('Setting identity to: '+serial_port_name)
            self._identity = serial_port_name
            
        elif self.model.lower() == 'ascom':
            self._logger.debug('Attempting to connect to ASCOM device "'+str(identity)+'"')
            if self._ascom_driver_handler is None:
                self._logger.debug('Loading ASCOM win32com device handler')
                import win32com.client
                self._ascom_driver_handler = win32com.client
            ascomDriverName = str()
            if identity is not None:
                self._logger.debug('Specified identity: "'+str(identity)+'" ['+str(len(identity))+']')
                if identity.startswith('ASCOM'):
                    ascomDriverName = identity
                else:
                    ascomDriverName = 'ASCOM.'+str(identity)+'.Telescope'
            else:
                ascomSelector = self._ascom_driver_handler.Dispatch("ASCOM.Utilities.Chooser")
                ascomSelector.DeviceType = 'Telescope'
                ascomDriverName = ascomSelector.Choose('None')
                self._logger.info('Selected telescope driver: ' + ascomDriverName)
                if not ascomDriverName:            
                    self._logger.debug('User canceled telescope selection')
                    return False
                try:
                    identity = ascomDriverName.replace('ASCOM.','').replace('.Telescope','')
                except:
                    identity = None
            if not ascomDriverName:
                raise AssertionError('Failed to identify ASCOM telescope')
            #try:
            self._ascom_telescope = self._ascom_driver_handler.Dispatch(ascomDriverName)
            self._ascom_telescope = None
            #except:
            #    raise AssertionError('Failed to connect to ASCOM telescope: '+str(ascomDriverName))
            self._identity = identity
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
        self._logger.debug('Identity set to: '+str(self.identity))

    @property
    def is_init(self):
        """bool: True if the device is initialised (and therefore ready to control)."""
        if not self.model: return False
        if self.model.lower() in self._supported_models_lowercase: 
            return self._is_init
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def available_properties(self):
        """tuple of str: Get all the available properties (settings) supported by this device."""
        if not self.is_init:
            return None
        elif self.model.lower() == 'celestron':
            return ('zero_altitude', 'home_alt_az', 'max_rate', 'alt_limit', 'azi_limit')
        elif self.model.lower() == 'ioptron azmp':
            return ('zero_altitude', 'home_alt_az', 'max_rate', 'alt_limit', 'azi_limit')
        elif self.model.lower() == 'ascom':
            return ('zero_altitude', 'home_alt_az', 'max_rate', 'alt_limit', 'azi_limit', 'axis_directions')
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def zero_altitude(self):
        """float: Get or set the zero altitude angle (degrees). Default 0.

        Normally the mount is initialised with the telescope level. In this case zero_altitude is 0. However, if the
        mount is e.g. initialised with the telescope pointing straight up, zero_altitude must be set to +90.
        """
        return self._alt_zero
    @zero_altitude.setter
    def zero_altitude(self, angle):
        self._logger.debug('Got set zero altitude with: '+str(angle))
        self._alt_zero = float(angle)
        self._logger.debug('Alt zero set to: '+str(self._alt_zero))

    @property
    def home_alt_az(self):
        """tuple of float: Get or set the home position (altitude, azimuth) in degrees. Default (0, 0)"""
        return self._home_pos
    @home_alt_az.setter
    def home_alt_az(self, pos):
        self._logger.debug('Got set home pos with: '+str(pos))
        assert pos in (int, float) or len(pos) == 2, 'Must be scalar or array of length 2'
        if pos in (int, float):
            pos = tuple([float(x) for x in pos])
        else:
            pos = (float(pos), float(pos))
        self._home_pos = pos
        self._logger.debug('Home pos set to: '+str(self.home_alt_az))

    @property
    def max_rate(self):
        """tuple of float: Get or set the max slew rate (degrees per second) for the axes (altitude, azimith).
        Default (4.0, 4.0).

        If a scalar is set, both axes' rates will be set to this value.
        """
        return self._max_speed
    @max_rate.setter
    def max_rate(self, maxrate):
        self._logger.debug('Got set max rate with: '+str(maxrate))
        assert maxrate in (int, float) or len(maxrate) == 2, 'Must be scalar or array of length 2'
        if maxrate in (int, float):
            maxrate = tuple([float(x) for x in maxrate])
        else:
            maxrate = (float(maxrate[0]), float(maxrate[1]))
        self._max_speed = maxrate
        self._logger.debug('Set max to: '+str(self.max_rate))

    @property
    def alt_limit(self):
        """tuple of float: Get or set the altitude limits (degrees) where the mount can safely move.
            May be set to None. Default (-5, 95). Not enforced when slewing (set_rate) the mount.
        """
        return self._alt_limit
    @alt_limit.setter
    def alt_limit(self, altlim):
        if altlim is None:
            self._logger.debug('Setting alt limit to None')
            self._alt_limit = (None, None)
        else:
            assert isinstance(altlim, (tuple, list)) and len(altlim)==2, 'Must be 2-tuple'
            self._logger.debug('Got set alt limits with: '+str(altlim))
            self._alt_limit = (float(altlim[0]) if altlim[0] is not None else None \
                               , float(altlim[1]) if altlim[1] is not None else None)
        self._logger.debug('Set alt limit to: '+str(self._alt_limit))

    @property
    def azi_limit(self):
        """tuple of float: Get or set the azimuth limits (degrees) where the mount can safely move.
            May be set to None. Default (None, None). Not enforced when slewing (set_rate) the mount.
        """
        return self._azi_limit
    @azi_limit.setter
    def azi_limit(self, azilim):
        if azilim is None:
            self._logger.debug('Setting azi limit to None')
            self._azi_limit = (None, None)
        assert isinstance(azilim, (tuple, list)) and len(azilim)==2, 'Must be 2-tuple'
        self._logger.debug('Got set azi limits with: '+str(azilim))
        self._azi_limit = (float(azilim[0]) if azilim[0] is not None else None \
                           , float(azilim[1]) if azilim[1] is not None else None)
        self._logger.debug('Set azi limit to: '+str(self._azi_limit))

    @property
    def axis_directions(self):
        """tuple of float: Get or set the azimuth limits (degrees) where the mount can safely move.
            May be set to None. Default (None, None). Not enforced when slewing (set_rate) the mount.
        """
        return self._axis_directions
    @axis_directions.setter
    def axis_directions(self, axis_dirs):
        if axis_dirs is None:
            self._logger.debug('Setting axis directions to 1')
            self._axis_directions = (1, 1)
        assert isinstance(axis_dirs, (tuple, list)) and len(axis_dirs)==2, 'Must be 2-tuple'
        assert axis_dirs[0] in [-1, 1] and axis_dirs[1] in [-1, 1], 'Axis directions must be 1 or -1'
        self._logger.debug('Got set axis directions with: '+str(axis_dirs))
        self._axis_directions = (int(axis_dirs[0]) if axis_dirs[0] is not None else None \
                               , int(axis_dirs[1]) if axis_dirs[1] is not None else None)
        self._logger.debug('Set axis directions to: '+str(self._axis_directions))
        
    @property
    def is_sidereal_tracking(self):
        """bool: True if the device is in sidereal tracking mode."""
        if self._is_init:
            if self.model.lower() == 'celestron':
                tracking_mode = self._serial_query('t', ord('#'))
                self._is_sidereal_tracking = (tracking_mode is not None and tracking_mode[0] == '1')
            elif self.model.lower() == 'ioptron azmp':
                # Sidereal tracking (and state query) is only available in normal commanding mode.
                if self._azmp_command_mode == 'normal':
                    mount_state = self._serial_query(':GLS#', ord('#')).decode('ASCII')
                    # The 18th digit indicates system status: 1 = tracking with PEC disabled, 5 means tracking with PEC enabled
                    status = mount_state[14]
                    self._logger.debug('Mount tracking state: "%s"' % status)
                    self._is_sidereal_tracking = (status == '1' or status == '5')
                else:
                    self._is_sidereal_tracking = False
            elif self.model.lower() == 'ascom':
                self._is_sidereal_tracking = (self._ascom_telescope is not None and self._ascom_telescope.Tracking)
        return self._is_sidereal_tracking        
    @is_sidereal_tracking.setter
    def is_sidereal_tracking(self, enable_sidereal):
        """bool: Set to True to enable sidereal tracking mode."""
        elf._logger.debug('Setting is_sidereal_tracking to '+str(enable_sidereal))
        if enable_sidereal:
            self._logger.debug('Enabling sidereal tracking') 
            self.start_sidereal_tracking()
        else:
            self._logger.debug('Disabling sidereal tracking') 
            self.stop_sidereal_tracking()
        
    def initialize(self):
        """Initialise (make ready to start) the device. The model and identity must be defined."""
        self._logger.debug('Initialising')
        assert not self.is_init, 'Already initialised'
        #assert not None in (self.model, self.identity), 'Must define model and identity before initialising'
        assert not None in (self.model, ), 'Must define model before initialising'
        if self.model.lower() == 'celestron':
            assert self.identity is not None, 'Must define identity before initialising'
            self._logger.debug('Using Celestron, try to initialise')
            self._logger.debug('Opening serial port '+self.identity)
            self._baud = 9600
            self._eol_byte = b'#'
            self._serial_port_open(self.identity)
            self._logger.debug('Opened serial port, sending test command')
            res = self._serial_query('m')
            self._logger.debug('Mount responded with: ' + str(res))
            self._logger.debug('Set tracking to off')
            self._cel_tracking_off()
            self._is_init = True

        elif self.model.lower() == 'ioptron azmp':
            assert self.identity is not None, 'Must define identity before initialising'
            self._logger.debug('Using %s, try to initialise' % self.model)
            self._logger.debug('Opening serial port '+self.identity)
            self._baud = 115200
            self._eol_byte = b'#'
            self._serial_port_open(self.identity)
            self._logger.debug('Opened serial port, checking command mode')
            # Command mode persists across resets. Expect either mode initially, and try to get to special mode.
            self._azmp_get_command_mode()
            assert self._azmp_command_mode in self._azmp_command_mode_names, 'Failed to get initial mount commanding mode.'
            self._logger.debug('Initial command mode: %s' % self._azmp_command_mode)
            if self._azmp_command_mode == 'normal':
                self._logger.debug('Ensure sidereal tracking is off and transition to special')
                self.stop_sidereal_tracking()
                self._azmp_set_command_mode('special')
                assert self._azmp_command_mode == 'special', 'Unable to switch mount to special command mode.'
            self._is_init = True

        elif self.model.lower() == 'ascom':
            if self._ascom_telescope is not None:
                raise RuntimeError('There is already an ASCOM telescope object here')
            self._logger.debug('Attempting to connect to ASCOM device "'+str(self.identity)+'"')
            if self._ascom_driver_handler is None:
                import win32com.client
                self._ascom_driver_handler = win32com.client
            if self.identity is not None:
                self._logger.debug('Specified identity: "'+str(self.identity)+'" ['+str(len(self.identity))+']')
                if self.identity.startswith('ASCOM'):
                    ascomDriverName = self.identity
                else:
                    ascomDriverName = 'ASCOM.'+str(self.identity)+'.Telescope'
            else:
                ascomSelector = self._ascom_driver_handler.Dispatch("ASCOM.Utilities.Chooser")
                ascomSelector.DeviceType = 'Telescope'
                ascomDriverName = ascomSelector.Choose('None')
                self._logger.info("Selected telescope driver: "+ascomDriverName)
                if not ascomDriverName:            
                    self._logger.debug('User canceled telescope selection')
                    return False
            self._identity = ascomDriverName.replace('ASCOM.','').replace('.Telescope','')
            assert ascomDriverName, 'Unable to identify ASCOM telescope.'
            assert self._ascom_driver_handler is not None, 'Unable to access win32com driver handler'
            self._logger.info('Loading ASCOM telescope driver: '+ascomDriverName)
            self._ascom_telescope = self._ascom_driver_handler.Dispatch(ascomDriverName)
            assert self._ascom_telescope is not None, 'Failed to intialize ASCOM telescope'
            assert hasattr(self._ascom_telescope, 'Connected'), "Unable to access telescope driver"
            self._logger.debug('Connecting to telescope')
            self._ascom_telescope.Connected = True
            assert self._ascom_telescope.Connected, "Failed to connect to telescope"
            self._logger.debug('Connected to ASCOM telescope')
            if hasattr(self._ascom_telescope, 'CanSetTracking') and self._ascom_telescope.CanSetTracking:
                self._ascom_telescope.Tracking = False  #turn off tracking
            try:
                self._ascom_canSlewAltAz = self._ascom_telescope.CanSlewAltAz
            except:
                self._ascom_canSlewAltAz = False
            max_speed = [0, 0]
            for axis in [0, 1]:
                self._logger.debug('axis ' + str(axis) + ' rate count: ' + str(self._ascom_telescope.AxisRates(axis).Count))
                for i in range(1, self._ascom_telescope.AxisRates(axis).Count+1):
                    self._logger.debug('axis rate ' + str(i) + ' min: ' + str(self._ascom_telescope.AxisRates(axis).Item(i).Minimum) + ', max: ' + str(self._ascom_telescope.AxisRates(axis).Item(i).Maximum))
                max_speed[axis] = self._ascom_telescope.AxisRates(axis).Item(i).Maximum
            for i in range(1, self._ascom_telescope.AxisRates(self._ascom_scope_alt_axis).Count+1):
                self._ascom_availableRatesAlt.append(float(self._ascom_telescope.AxisRates(self._ascom_scope_alt_axis).Item(i).Maximum))
            for i in range(1, self._ascom_telescope.AxisRates(self._ascom_scope_azi_axis).Count+1):
                self._ascom_availableRatesAzi.append(float(self._ascom_telescope.AxisRates(self._ascom_scope_azi_axis).Item(i).Maximum))
            self.max_rate = (max_speed[self._ascom_scope_alt_axis], max_speed[self._ascom_scope_azi_axis])
            self._is_init = True
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
        
        if self._is_init:
            self._logger.info('Mount initialised.')
            try:
                self.get_alt_az() #Get cache to update
            except AssertionError:
                self._logger.debug('Failed to set state cache', exc_info=True)
        else:
            self._logger.info('Mount not initialised.')
        

    def deinitialize(self):
        """De-initialise the device and release hardware (serial port). Will stop the mount if it is moving."""
        self._logger.debug('De-initialising')
        assert self.is_init, 'Not initialised'
        try:
            self._logger.debug('Stopping mount')
            self.stop()
        except:
            self._logger.debug('Did not stop', exc_info=True)
        if self.model.lower() == 'celestron':
            self._logger.debug('Using celestron, closing and deleting serial port')
            self._serial_port_close()
            self._is_init = False
            self._logger.info('Mount deinitialised')
        elif self.model.lower() == 'ioptron azmp':
            self._logger.debug('Using iOptron AZMP, reverting mount commanding mode to normal')
            self._azmp_set_command_mode('normal')
            self._logger.debug('Closing and deleting serial port')
            self._serial_port_close()
            self._is_init = False
            self._logger.info('Mount deinitialised')
        elif self.model.lower() == 'ascom':
            self._logger.debug('Disconnecting ASCOM telescope mount')
            if self._ascom_telescope is not None:
                try:
                    if self._ascom_telescope.Connected:
                        self._ascom_telescope.AbortSlew()
                    self._ascom_telescope.Connected = False
                except:
                    pass
            self._is_init = False
            pythoncom.CoUninitialize()
            self._ascom_telescope = None
            self._logger.info('Mount deinitialised')
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def is_moving(self):
        """Returns True if the mount is currently moving."""
        assert self.is_init, 'Must be initialised'
        self._logger.debug('Got is moving request, checking thread')
        if self._control_thread is not None and self._control_thread.is_alive():
            self._logger.debug('Has active control thread')
            return True
        if self.model.lower() == 'celestron':
            self._logger.debug('Using celestron, asking if moving')
            ret = [None]
            def _is_moving_to(ret):
                self._serial_send_text_command('L')
                ret[0] = self._serial_read_to_eol()
            t = Thread(target=_is_moving_to, args=(ret,))
            t.start()
            t.join()
            moving = not ret[0] == b'0'
            self._logger.debug('Mount returned: ' + str(ret[0]) + ', is moving: ' + str(moving))
            return moving

        elif self.model.lower() == 'ioptron azmp':
            is_moving = False
            self._logger.debug('Using %s in %s command mode, asking if moving' % (self.model, self._azmp_command_mode))
            if self._azmp_command_mode == 'special':
                azi_axis_rate = self._serial_query(':Q0#').decode('ASCII')
                alt_axis_rate = self._serial_query(':Q1#').decode('ASCII')
                self._logger.debug('azi_axis_rate: "%s", alt_axis_rate: "%s"' % (azi_axis_rate, alt_axis_rate))
                try:
                    is_moving = (int(azi_axis_rate or 0) != 0 or int(alt_axis_rate or 0) != 0)
                except:
                    raise AssertionError('invalid rate query response (azi_axis_rate: "%s", alt_axis_rate: "%s")' % \
                                                                                    (azi_axis_rate, alt_axis_rate))
            else:
                mount_system_status_char = ''
                mount_state = self._serial_query(':GLS#')
                print(mount_state)
                assert mount_state is not None and len(mount_state)>14, 'Unexpected AZMP state query response: "%s"' % mount_state                
                try:
                    mount_system_status_char = mount_state.decode('ASCII')[14]
                    print(mount_system_status_char)
                    mount_system_state = self._azmp_states[mount_system_status_char]
                    self._logger.debug('AZMP system state: "%s" (%s)' % (mount_system_state, mount_system_status_char))
                except KeyError:
                    self._logger.warning('Unexpected AZMP state query response: "%s", status byte: %s' % (mount_state, mount_state.decode[14]))
                assert mount_system_status_char in self._azmp_states, 'Invalid AZMP state index: %s' % mount_system_status_char
                is_moving = mount_system_status_char in '12345'  # azmp motion states are between 1 and 5 inclusive
            return is_moving
            
        elif self.model.lower() == 'ascom':
            return self._ascom_telescope.Slewing or self._ascom_telescope.Tracking

        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def move_to_alt_az(self, alt, azi, block=True, rate_control=True, tolerance_deg=0.001, min_speed=0.001):
        """Move the mount to the given position. Must be initialised.

        Args:
            alt (float): Altitude angle (degrees).
            azi (float): Azimuth angle (degrees).
            block (bool, optional): If True (the default) the call to this method will block until the move is finished.
            rate_control (bool, optional): If True (the default) the rate of the mount will be controlled until position
                is reached, if False the position command will be sent to the mount for execution.
        """
        assert self.is_init, 'Must be initialised'
        assert self._alt_limit[0] is None or alt >= self._alt_limit[0], 'Altitude outside range!'
        assert self._alt_limit[1] is None or alt <= self._alt_limit[1], 'Altitude outside range!'
        assert self._azi_limit[0] is None or azi >= self._azi_limit[0], 'Azimuth outside range!'
        assert self._azi_limit[1] is None or azi <= self._azi_limit[1], 'Azimuth outside range!'
        self._logger.debug('Got move command with: alt=' + str(alt) + ' azi=' + str(azi) + ' block='+str(block) \
                           + ' rate_control=' + str(rate_control))
        self._logger.debug('Stopping mount first')
        self.stop()
        if self.model.lower() in ('celestron', 'ioptron azmp', 'ascom'):
            self._logger.debug('Adjusting range to -180 to 180')
            alt = self.degrees_to_n180_180(alt - self._alt_zero)
            alt = self.degrees_to_n180_180(alt)
            azi = self.degrees_to_n180_180(azi)
            self._logger.debug('Will command to alt=' + str(alt) + ' azi=' + str(azi))                                                                                    

            if self.model.lower() == 'ioptron azmp':            
                self._azmp_set_command_mode('special')
                
            if not rate_control: # Command mount natively
                self._logger.debug('Sending move command to mount')
                success = [False]
                def _move_to_alt_az(alt, azi, success):
                    success[0] = _command_to_alt_az(alt, azi)
                t = Thread(target=_move_to_alt_az, args=(alt, azi, success))
                t.start()
                t.join()
                self._logger.debug('Send successful')
                if block:
                    self._logger.debug('Waiting for mount to finish')
                    self.wait_for_move_to()

            else: # Use own control thread
                self._logger.debug('Starting rate controller')            
                Kp = 0.8
                self._control_thread_stop = False
                success = [False]
                def _loop_slew_to(alt, azi, success):
                    if self.model.lower() == 'ascom':  pythoncom.CoInitialize()
                    while not self._control_thread_stop:
                        curr_pos = self.get_alt_az()
                        # Get current position error
                        error_alt = self.degrees_to_n180_180(alt - curr_pos[0])
                        error_azi = self.degrees_to_n180_180(azi - curr_pos[1])
                        
                        if abs(error_alt) < tolerance_deg:
                            rate_alt = 0
                        else:
                            rate_alt = Kp * error_alt
                            # Clip to maximum and minimum speeds
                            if abs(rate_alt) > self.max_rate[0]: rate_alt = self.max_rate[0] * np.sign(rate_alt)
                            if abs(rate_alt) < min_speed: rate_alt = min_speed * np.sign(rate_alt)
                        
                        if abs(error_azi) < tolerance_deg:
                            rate_azi = 0
                        else:
                            rate_azi = Kp * error_azi
                            # Clip to maximum and minimum speeds
                            if abs(rate_azi) > self.max_rate[1]: rate_azi = self.max_rate[1] * np.sign(rate_azi)
                            if abs(rate_azi) < min_speed: rate_azi = min_speed * np.sign(rate_azi)

                        self.set_rate_alt_az(rate_alt, rate_azi)
                        if rate_alt == 0 and rate_azi == 0:
                            success[0] = True
                            break

                    self._control_thread_stop = True
            self._control_thread = Thread(target=_loop_slew_to, args=(alt, azi, success))
            self._control_thread.start()
            if block:
                self._logger.debug('Waiting for thread to finish')
                self._control_thread.join()
                #assert success[0], 'Failed moving with rate controller'

        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def _command_to_alt_az(self, alt, azi):
        """PRIVATE: Command mount to slew to alt/az coordinates. Must be initialised.

        Args:
            alt (float): Altitude (degrees).
            azi (float): Azimuth (degrees).
        """
        self._logger.debug('Got request to command to alt: %0.3f, azi: %0.3f' % (alt, azi))
        assert self.is_init, 'Must be initialised'
        if self.model.lower() == 'celestron':
            self._logger.debug('Sending move command to mount')
            success = [False]
            def _move_to_alt_az(alt, azi, success):
                #azi = azi %360 #Mount uses 0-360
                # TODO check alt zero correct
                altRaw = int(self.degrees_to_0_360(alt - self._alt_zero) / 360 * 2**32) & 0xFFFFFF00
                aziRaw = int(self.degrees_to_0_360(azi) / 360 * 2**32) & 0xFFFFFF00
                altFormatted = '{0:0{1}X}'.format(altRaw,8)
                aziFormatted = '{0:0{1}X}'.format(aziRaw,8)
                command = 'b' + aziFormatted + ',' + altFormatted
                self._serial_send_text_command(command)
                assert self._serial_check_ack(), 'Mount did not acknowledge'
                success[0] = True
            t = Thread(target=_move_to_alt_az, args=(alt, azi, success))
            t.start()
            t.join()
            assert success[0], 'Failed communicating with mount'
            self._logger.debug('Send successful')

        elif self.model.lower() == 'ioptron azmp':
            # TODO check alt zero correct
            self._azmp_set_command_mode('special')
            # Azimuth:
            command = 'T0%+i#' % int(self.degrees_to_0_360(azi) * 3600 / 0.01 )
            self._serial_send_text_command(command)
            # Altitude:
            command = 'T1%+i#' % int(self.degrees_to_0_360(alt - self._alt_zero) * 3600 / 0.01 )
            self._serial_send_text_command(command)
            
        elif self.model.lower() == 'ascom':
            if not self._ascom_telescope.CanSlewAltAz:
                raise RuntimeError('ASCOM mount does not support alt/az go-to commanding')
                return False
            if self._ascom_telescope.AtPark:
                raise RuntimeError('ASCOM mount is parked; cannot command alt/az slew')
                return False
            if self._ascom_telescope.Tracking:
                raise RuntimeError('ASCOM mount is tracking; cannot command alt/az slew')
                return False                
            self._ascom_telescope.SlewToAltAz(alt, azi)
            return True
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
            
    def get_alt_az(self):
        """Get the current alt and azi angles of the mount.

        Returns:
            tuple of float: the (altitude, azimuth) angles of the mount in degrees (-180, 180].
        """
        assert self.is_init, 'Must be initialised'
        #self._logger.debug('Requesting mount position')
        if self.model.lower() == 'celestron':
                                                                            
            ret = [None, None]
               
            def _get_alt_az(ret):
                command = bytes([ord('z')]) #Get precise AZM-ALT
                self._serial_port.write(command)
                # The command returns ASCII encoded text of HEX values!
                res = self._serial_read_to_eol().decode('ASCII')
                r2 = res.split(',')
                ret[0] = int(r2[1], 16)
                ret[1] = int(r2[0], 16)
            ret = [None, None]
            t = Thread(target=_get_alt_az, args=(ret,))
            t.start()
            t.join()

            alt = self.degrees_to_n180_180( float(ret[0]) / 2**32 * 360 + self._alt_zero)
            azi = self.degrees_to_n180_180( float(ret[1]) / 2**32 * 360 )
            self._logger.debug('Mount position: alt=' + str(ret[0]) + ' azi=' + str(ret[1]) \
                               + ' => alt=' + str(alt) + ' azi=' + str(azi))
            self._state_cache['alt'] = alt
            self._state_cache['azi'] = azi
            return (alt, azi)

        elif self.model.lower() == 'ioptron azmp':
            (alt, azi) = (None, None)
            if self._azmp_command_mode == 'special':
                # returns integer units of 0.01 arcsec
                for attempt in (0, 1):
                    azi_raw = self._serial_query(':P0#').decode('ASCII')
                    alt_raw = self._serial_query(':P1#').decode('ASCII')
                    try:
                        azi = self.degrees_to_n180_180( int(azi_raw) * 0.01 / 3600 + 180 )
                        alt = self.degrees_to_n180_180( 90 - int(alt_raw) * 0.01 / 3600 + self._alt_zero)
                        self._logger.debug('Mount position: alt=' + str(alt_raw) + ' azi=' + str(azi_raw) \
                                            + ' => alt=' + str(alt) + ' azi=' + str(azi))
                        break
                    except:
                        self._logger.info('WARNING: invalid responses from mount (alt: "%s", azi: "%s")' % (alt_raw, azi_raw))
            elif self._azmp_command_mode == 'normal':
                # returns char array: : “sTTTTTTTTTTTTTTTTT#”
                mount_altaz_info = self._serial_query(':GAC#').decode('ASCII')
                assert mount_altaz_info is not None, 'Failed to get mount position.'
                alt_raw = int(mount_altaz_info[0:9])
                azi_raw = int(mount_altaz_info[9:18])
                azi = self.degrees_to_n180_180( int(azi_raw) * 0.01 / 3600 )
                alt = self.degrees_to_n180_180( int(alt_raw) * 0.01 / 3600 + self._alt_zero)
                self._logger.debug('Mount position: alt=' + str(alt_raw) + ' azi=' + str(azi_raw) \
                                    + ' => alt=' + str(alt) + ' azi=' + str(azi))
            self._state_cache['alt'] = alt
            self._state_cache['azi'] = azi
            return (alt, azi)

        elif self.model.lower() == 'ascom':
            alt = self._ascom_telescope.Altitude
            azi = self._ascom_telescope.Azimuth
            self._logger.debug('Mount position: alt=' + str(alt) + ' azi=' + str(azi))
            self._state_cache['alt'] = alt
            self._state_cache['azi'] = azi
            return (alt, azi)            

        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def move_home(self, block=True, rate_control=True):
        """Move to the position defined by Mount.home_alt_az.

        Args:
            block (bool, optional): If True (the default) the call to this method will block until the move is finished.
            rate_control (bool, optional): If True (the default) the rate of the mount will be controlled until position
                is reached, if False the position command will be sent to the mount for execution.
        """
        self.move_to_alt_az(*self.home_alt_az, block=block, rate_control=rate_control)

    def set_rate_alt_az(self, alt, azi):
        """Set the mount slew rate. Must be initialised.

        Args:
            alt (float): Altitude rate (degrees per second).
            azi (float): Azimuth rate (degrees per second).
        """
        assert self.is_init, 'Must be initialised'
        self._logger.debug('Got rate command. alt=' + str(alt) + ' azi=' + str(azi))
        if (abs(alt) > self._max_speed[0]) or  (abs(azi) > self._max_speed[1]):
            raise ValueError('Above maximum speed! ('+str(abs(alt))+', '+str(abs(azi))+')')
        if self.model.lower() == 'celestron':
            self._logger.debug('Using celestron, sending rate command to mount')
            success = [False]
            def _set_rate_alt_az(alt, azi, success):
                #Altitude
                rate = int(round(alt*3600*4))
                if rate >= 0:
                    rateLo = rate & 0xFF
                    rateHi = rate>>8 & 0xFF
                    command_bytes = [ord('P'),3,17,6,rateHi,rateLo,0,0]
                else:
                    rateLo = -rate & 0xFF
                    rateHi = -rate>>8 & 0xFF
                    command_bytes = [ord('P'),3,17,7,rateHi,rateLo,0,0]
                self._serial_send_bytes_command(command_bytes)
                self._logger.debug('Sending: '+str(command_bytes))
                assert self._serial_check_ack(), 'Mount did not acknowledge!'
                #Azimuth
                rate = int(round(azi*3600*4))
                if rate >= 0:
                    rateLo = rate & 0xFF
                    rateHi = rate>>8 & 0xFF
                    self._serial_send_bytes_command([ord('P'),3,16,6,rateHi,rateLo,0,0])
                else:
                    rateLo = -rate & 0xFF
                    rateHi = -rate>>8 & 0xFF
                    self._serial_send_bytes_command([ord('P'),3,16,7,rateHi,rateLo,0,0])
                assert self._serial_check_ack(), 'Mount did not acknowledge!'
                success[0] = True
            t = Thread(target=_set_rate_alt_az, args=(alt, azi, success))
            t.start()
            t.join()
            assert success[0], 'Failed communicating with mount'
            self._logger.debug('Send successful')
            self._state_cache['alt_rate'] = alt
            self._state_cache['azi_rate'] = azi

        elif self.model.lower() == 'ioptron azmp':
            #self._logger.debug('Using %s, sending rate command to mount' % self.model)
            self._serial_port.reset_input_buffer()
            if self._azmp_command_mode == 'special':
                # convert rates to integer units of 0.01 arcsec/second
                alt_rate_command = ':M1%+i#' % int(round(-1*alt*3600/0.01))
                azi_rate_command = ':M0%+i#' % int(round(azi*3600/0.01))
                self._serial_query(alt_rate_command, eol_byte = b'1')
                self._serial_query(azi_rate_command, eol_byte = b'1')
            elif self._azmp_command_mode == 'normal' and alt==0 and azi==0:
                # support zero-rate commanding in normal mode specifically to support the stop method.
                self._serial_query(':Q#', eol_byte = b'1')  # "quit slew" command stops 
            else:
                raise AssertionError('Non-zero rate requested while mount is not in compatible mode.')
            
            self._logger.debug('Send successful')
            self._state_cache['alt_rate'] = alt
            self._state_cache['azi_rate'] = azi

        elif self.model.lower() == 'ascom':        
            if alt==0 and azi==0:
                try:
                    self._ascom_telescope.AbortSlew()
                except:
                    pass                        
                self._ascom_telescope.MoveAxis(self._ascom_scope_alt_axis, 0)
                self._ascom_telescope.MoveAxis(self._ascom_scope_azi_axis, 0)
                rates = [0, 0]
            else:
                requested_rates = [0, 0]
                requested_rates[self._ascom_scope_alt_axis] = alt
                requested_rates[self._ascom_scope_azi_axis] = azi
                rates = [0, 0]
                # Verify requested rates are within allowable ranges, or round down to nearest allowed range:
                for axis in [0, 1]:
                    requested_rate_mag = abs(requested_rates[axis])
                    requested_rate_sign = 1 if requested_rates[axis]>=0 else -1
                    self._logger.debug('axis ' + str(axis) + ' requested rate mag: ' + str(requested_rate_mag) + ', sign: ' + str(requested_rate_sign))
                    for i in range(1, self._ascom_telescope.AxisRates(axis).Count+1):
                        if requested_rate_mag >= self._ascom_telescope.AxisRates(axis).Item(i).Minimum:
                            if requested_rate_mag <= self._ascom_telescope.AxisRates(axis).Item(i).Maximum:
                                rates[axis] = requested_rate_mag * requested_rate_sign
                                break
                            else:
                                rates[axis] = self._ascom_telescope.AxisRates(axis).Item(i).Maximum * requested_rate_sign
                        else:
                            break
                    rates[axis] *= self._axis_directions[axis]
                self._logger.debug('Commanding alt rate: '+str(rates[self._ascom_scope_alt_axis]))
                self._ascom_telescope.MoveAxis(self._ascom_scope_alt_axis, rates[self._ascom_scope_alt_axis])
                self._logger.debug('Commanding azi rate: '+str(rates[self._ascom_scope_azi_axis]))
                self._ascom_telescope.MoveAxis(self._ascom_scope_azi_axis, rates[self._ascom_scope_azi_axis])
                
            self._state_cache['alt_rate'] = rates[self._ascom_scope_alt_axis]
            self._state_cache['azi_rate'] = rates[self._ascom_scope_azi_axis]

        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def start_sidereal_tracking(self):
        """Enable sidereal tracking."""
        self._logger.debug('Got request to start sidereal tracking')
        assert self.is_init, 'Must be initialised'
        if self.model.lower() == 'celestron':
            success = [False]
            def _set_tracking_on(success):
                #self._serial_send_bytes_command([ord('T'),1])
                #assert self._serial_check_ack(ord('#')), 'Mount did not acknowledge!'
                assert self._serial_query('T1') is not None, 'Mount did not acknowledge!'
                success[0] = True
                self._is_sidereal_tracking = True
            t = Thread(target=_set_tracking_on, args=(success,))
            t.start()
            t.join()
            assert success[0], 'Failed communicating with mount'   
            self._is_sidereal_tracking = True
        elif self.model.lower() == 'ioptron azmp':
            # sidereal tracking (and state check) is only supported in normal commanding mode.
            self._azmp_set_command_mode('normal')
            assert self._azmp_command_mode == 'normal', 'Cannot enable sidereal tracking in "%s" mode' % self._azmp_command_mode
            assert self._serial_query(':ST1#', b'1') is not None, 'Mount did not acknowledge!'
            #self._serial_send_text_command(':ST1#')
            #assert self._serial_check_ack('1'), 'Mount did not acknowledge!'
            self._is_sidereal_tracking = True
        elif self.model.lower() == 'ascom':
            if hasattr(self._ascom_telescope, 'CanSetTracking') and self._ascom_telescope.CanSetTracking:
                try:
                    self._ascom_telescope.Tracking = True  #turn on tracking
                    self._is_sidereal_tracking = True
                except:
                    self._logger.warning('Failed to start sidereal tracking.')
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
                
    def stop_sidereal_tracking(self):
        """Disable sidereal tracking."""
        self._logger.debug('Got request to stop sidereal tracking')
        if self.is_init:
            if self.model.lower() == 'celestron':
                success = [False]
                def _set_tracking_off(success):
                    self._serial_send_bytes_command([ord('T'),0])
                    assert self._serial_check_ack('#'), 'Mount did not acknowledge!'
                    success[0] = True
                    self._is_sidereal_tracking = False
                t = Thread(target=_set_tracking_off, args=(success,))
                t.start()
                t.join()
                assert success[0], 'Failed communicating with mount'
            elif self.model.lower() == 'ioptron azmp':
                # sidereal tracking (and state check) is only supported in normal commanding mode.
                #self._azmp_get_command_mode()
                if self._azmp_command_mode == 'normal':
                    #self._serial_send_text_command(':ST0#')
                    #assert self._serial_check_ack('1'), 'Mount did not acknowledge!'
                    assert self._serial_query(':ST0#', b'1') is not None, 'Mount did not acknowledge!'
                    self._is_sidereal_tracking = False
                    #self._azmp_set_command_mode('special')
                    #assert self._azmp_command_mode == 'special', 'Unable to switch mount to special command mode.'
                    #self.get_alt_az()
                else:
                    self._is_sidereal_tracking = False
            elif self.model.lower() == 'ascom':
                if hasattr(self._ascom_telescope, 'CanSetTracking') and self._ascom_telescope.CanSetTracking:
                    try:
                        self._ascom_telescope.Tracking = False  #turn off tracking
                        self._is_sidereal_tracking = False
                    except:
                        self._logger.warning('Failed to stop sidereal tracking.')                        
            else:
                self._logger.warning('Forbidden model string defined.')
                raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def stop(self):
        """Stop moving."""
                                                  
        self._logger.debug('Got stop command, check thread')
        if self.is_init:
            #if self.is_moving:
            self._logger.info('stopping mount')
            if self._control_thread is not None and self._control_thread.is_alive():
                self._logger.debug('Stopping control thread')
                self._control_thread_stop = True
                self._control_thread.join()
                self._logger.debug('Stopped')
            if self._serial_is_init:
                #sleep(0.2)
                self._serial_port.reset_input_buffer()
                self._serial_port.reset_output_buffer()                    
            #sleep(0.2)
            self._logger.debug('Sending zero rate command')                
            self.set_rate_alt_az(0, 0)
            #sleep(0.2)
            if self._is_sidereal_tracking:
                self.stop_sidereal_tracking()
        self._logger.debug('Stopped mount')

    def wait_for_move_to(self, timeout=120):
        """Wait for mount to finish move.

        Args:
            timeout (int, optional): Maximum time (seconds) to wait before raising TimeoutError. Default 120.
        """
        assert self.is_init, 'Must be initialised'
        t_start = timestamp()
        self._logger.debug('Waiting for move to, start time: '+str(t_start))
        try:
            while timestamp() - t_start < timeout:
                if self.is_moving:
                    sleep(.5)
                else:
                    return
        except KeyboardInterrupt:
            self._logger.debug('Waiting interrupted', exc_info=True)

        raise TimeoutError('Waiting for mount move took more than ' + str(timeout) + 'seconds.')

    @staticmethod
    def list_available_ports():
        """List the available serial port names and descriptions.

        Returns:
            list of tuple: (device, description) for each available serial port (see serial.tools.list_ports).
        """
        import serial.tools.list_ports as ports
        port_list = ports.comports()
        return [(x.device, x.description) for x in port_list] if len(port_list) > 0 else []

    @staticmethod
    def degrees_to_0_360(number):
        """float: Convert angle (degrees) to range [0, 360)."""
        return float(number)%360

    @staticmethod
    def degrees_to_n180_180(number):
        """float: Convert angle (degrees) to range (-180, 180]"""
        return 180 - (180-float(number))%360

    def _serial_find_port(self, port_index):
        """PRIVATE: Get serial port name at a given index, starting at zero"""
        port_index = int(port_index)
        self._logger.debug('Searching for serial port at index: ' +str(port_index))
        ports = self.list_available_ports()
        self._logger.debug('Found ports: ' + str(ports))
        try:
            return ports[port_index][0]
        except IndexError:
            self._logger.debug('Index error', exc_info=True)
            raise AssertionError('No serial port for index: '+str(identity))

    def _serial_test(self, port_name, baud, test_command, nbytes):
        """PRIVATE: Test if serial communication is established by sending """
        self._logger.debug('Testing serial port "%s" with test command "%s", reading %i bytes' % (
                                                                                port_name, test_command, nbytes))
        self._logger.debug('Testing serial port "%s", baud %i' % (port_name, baud))                                                                               
        try:
            with serial.Serial(port_name, baud, timeout=3.5, write_timeout=3.5) as ser:
                ser.write(test_command.encode('ASCII'))
                ser.flush()
                response = ser.read(nbytes)
                self._logger.debug('Got response: '+str(response))
                return response
        except serial.SerialException:
            self._logger.warning('Failed to communicate on serial port ' + str(port_name), exc_info=True)
            return []
            #raise
        
        
        
    def _serial_port_open(self, port_name, baud=None, timeout=3.5):
        """PRIVATE: Opens serial port specified by port_name string."""
        self._logger.debug('Got open serial port with name: ' + str(port_name))
        baud = baud or self._baud  # take from self if not given
        try:
            self._serial_port = serial.Serial(port_name, baud, timeout=timeout, write_timeout=timeout)
            self._serial_is_init = True
            self._logger.debug('Successfully opened port')
        except serial.SerialException:
            self._logger.waring('Failed to open serial port at ' + str(port_name), exc_info=True)
            raise
            
    def _serial_port_close(self):
        """PRIVATE:  Closes serial port."""
        self._logger.debug('Got close serial port')
        if self._serial_port is not None:
            self._serial_port.close()
            self._serial_port = None
            self._logger.debug('Port closed')
            self._serial_is_init = False
        
    def _serial_query(self, command, eol_byte=None):
        """PRIVATE: Encodes as ASCII and sends command string to mount, then reads and returns 
        response string from mount ending in indicated end-of-line character.
        inputs:
          command (str):   command message to be sent to mount.
          eol_byte (byte, optional): expected terminating character at end of mount response.
        returns:  ASCII string response from mount up to and including EOL byte character.
        """
        assert self._serial_port is not None and self._serial_is_init, 'Serial port is not initialized'
        response = b''
        self._logger.debug('Sending serial command "%s" to mount' % str(command))
        eol_byte = eol_byte or self._eol_byte  # Take from self if not given
        try:
            self._serial_port.reset_input_buffer()
            self._serial_port.reset_output_buffer()            
            self._serial_send_text_command(command)
            #self._serial_port.flush()
            response = self._serial_read_to_eol(eol_byte)
            if response is None:
                self._logger.warning('No response from mount (query: "%s")' % command)
                response = b''
        except:
            self._logger.debug('Failed to communicate', exc_info=True)
            raise
        return response


    def _serial_send_text_command(self, command):
        """PRIVATE: Encode as ASCII and send to mount."""
        assert self._serial_is_init, 'Serial port is not initialized'
        self._serial_port.write(command.encode('ASCII'))
        self._serial_port.flush() #Push out data

    def _serial_send_bytes_command(self, command):
        """PRIVATE: Send bytes to mount."""
        assert self._serial_is_init, 'Serial port is not initialized'
        self._serial_port.write(bytes(command))
        self._serial_port.flush() #Push out data

    def _serial_check_ack(self, ack_byte = None):
        """PRIVATE: Read one byte and compare to the acknowledge byte character."""
        assert self._serial_is_init, 'Serial port is not initialized'
        ack_byte = ack_byte or self._eol_byte
        b = self._serial_port.read()
        return b == ack_byte

    '''
    def _serial_read_to_eol(self, eol_byte=None):
        """PRIVATE: Read response to the EOL byte character. Return bytes."""
        assert self._serial_is_init, 'Serial port is not initialized'
        eol_byte = eol_byte or self._eol_byte  # Take from self if not given
        response = b'' #Empty type 'bytes'
        while True:
            r = self._serial_port.read()
            if r == b'': #If we didn't get anything/timeout
                #raise RuntimeError('Mount serial timed out!')
                self._logger.warning('Mount serial timed out!')
            else:
                if r == eol_byte:
                    self._logger.debug('Read from mount: '+str(response))
                    return response
                else:
                    response += r
    '''
                    
    def _serial_read_to_eol(self, eol_byte=None, timeout=3):
        """PRIVATE: Read response to the EOL byte character. Return bytes."""
        assert self._serial_is_init, 'Serial port is not initialized'
        eol_byte = eol_byte or self._eol_byte  # Take from self if not given
        timeout_time = timestamp() + timeout
        response = b'' #Empty type 'bytes'
        while timestamp() < timeout_time:
            r = self._serial_port.read(1)
            if r == eol_byte:
                #self._logger.debug('Read from mount: '+str(response))
                return response
            else:
                response += r
                
            if timestamp() > timeout_time:
                self._logger.info('timed out waiting for serial response (read: "%s", looking for eol byte: %s)' % (response, eol_byte))
                return None
            sleep(0.0001)

    def _cel_tracking_off(self):
        """PRIVATE: Disable sidreal tracking on celestron mount."""
        success = [False]
        def _set_tracking_off(success):
            self._serial_send_bytes_command([ord('T'),0])
            assert self._serial_check_ack(), 'Mount did not acknowledge!'
            success[0] = True
        t = Thread(target=_set_tracking_off, args=(success,))
        t.start()
        t.join()
        assert success[0], 'Failed communicating with mount'        

    def _azmp_get_command_mode(self):
        """PRIVATE: Get the iOptron AZMP command mode. Returns either 'normal' or 'special'"""
        self._logger.debug('Checking AZMP command mode')
        assert self._serial_is_init, 'Serial port is not initialized'
        # Empty both buffers
        self._serial_port.reset_input_buffer()
        self._serial_port.reset_output_buffer()
        self._serial_send_text_command(':MountInfo#')
        command_mode_code = self._serial_port.read(4)
        assert command_mode_code in (b'5035', b'9035'), 'Failed to get AZMP command mode. Mount returned: ' + str(command_mode_code)
        self._azmp_command_mode = 'normal' if command_mode_code == b'5035' else 'special'
        self._logger.info('AZMP command mode is "%s" (%s)' % (self._azmp_command_mode, str(command_mode_code)))

    def _azmp_set_command_mode(self, to_mode):
        self._logger.debug('Got request to transition mount to %s commanding mode' % to_mode)
        self._azmp_get_command_mode()
        if self._azmp_command_mode == to_mode:
            self._logger.debug('Mout is already in command mode "%s"' % self._azmp_command_mode)
        else:
            self._logger.debug('Commanding AZMP mode transition')
            self._serial_send_text_command(':ZZZ#')
            sleep(2.5)
            self._azmp_get_command_mode()
            assert self._azmp_command_mode == to_mode, 'Failed to transition mount to %s commanding mode"' % to_mode