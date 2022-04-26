"""Hardware interfaces
======================

Current harware support:
    - :class:`pypogs.Camera`: 'ptgrey' for FLIR (formerly Point Grey) machine vision cameras. Requires Spinnaker API and PySpin, see the
      installation instructions. Tested with Blackfly S USB3 model BFS-U3-31S4M.

    - :class:`pypogs.Mount`: 'celestron' for Celestron, Orion and SkyWatcher telescopes (using NexStar serial protocol). No additional
      packages required. Tested with Celestron model CPC800.

    - :class:`pypogs.Receiver`: 'ni_daq' for National Instruments DAQ data acquisition cards. Requires NI-DAQmx API and nidaqmx, see the
      installation instructions. Tested with NI DAQ model USB-6211.

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
from datetime import datetime, time
from threading import Thread, Event
from struct import pack as pack_data
import threading

# External imports:
import numpy as np
import serial
from harvesters.core import Harvester, TimeoutException  # type: ignore

class Camera:
    """Control acquisition and receive images from a camera.

    To initialise a Camera a *model* (determines hardware interface) and *identity* (identifying the specific device)
    must be given. If both are given to the constructor the Camera will be initialised immediately (unless
    auto_init=False is passed). Manually initialise with a call to Camera.initialize(); release hardware with a call to
    Camera.deinitialize().

    After the Camera is intialised, acquisition properties (e.g. exposure_time and frame_rate) may be set and images
    received. The Camera also supports event-driven acquisition, see Camera.add_event_callback(), where new images are
    automatically passed on to the desired functions.

    Args:
        model (str, optional): The model used to determine the correct hardware API. Supported: 'ptgrey' for
            PointGrey/FLIR Machine Vision cameras (using Spinnaker and PySpin).
        identity (str, optional): String identifying the device. For model *ptgrey* this is 'serial number' *as a
            string*.
        name (str, optional): Name for the device.
        auto_init (bool, optional): If both model and identity are given when creating the Camera and auto_init
            is True (the default), Camera.initialize() will be called after creation.
        debug_folder (pathlib.Path, optional): The folder for debug logging. If None (the default)
            the folder *pypogs*/debug will be used/created.

    Example:
        ::

            # Create instance and set parameters (will auto initialise)
            cam = pypogs.Camera(model='ptgrey', identity='18285284', name='CoarseCam')
            cam.gain = 0 #decibel
            cam.exposure_time = 100 #milliseconds
            cam.frame_rate_auto = True
            # Start acquisition
            cam.start()
            # Wait for a while
            time.sleep(2)
            # Read the latest image
            img = cam.get_latest_image()
            # Stop the acquisition
            cam.stop()
            # Release the hardware
            cam.deinitialize()
    """
    _supported_models = ('ptgrey','phfocus')

    def __init__(self, model=None, identity=None, name=None, auto_init=True, debug_folder=None):
        """Create Camera instance. See class documentation."""
        # Logger setup
        self._debug_folder = None
        if debug_folder is None:
            self.debug_folder = Path(__file__).parent / 'debug'
        else:
            self.debug_folder = debug_folder
        self._logger = logging.getLogger('pypogs.hardware.Camera')
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
        self._logger.debug('Creating instance. Constructor input: Model:'+str(model)+' ID:'+str(identity)\
                           +' Name:'+str(name) +' AutoInit:'+str(auto_init))
        self._model = None
        self._identity = None
        self._name = 'UnnamedCamera'
        self._plate_scale = 1.0
        self._rotation = 0.0
        self._flipX = False
        self._flipY = False
        self._rot90 = 0 #Number of times to rotate by 90 deg, done after flips
        #Only used for ptgrey
        self._ptgrey_camera = None
        self._ptgrey_camlist = None
        self._ptgrey_system = None
        #Only used for phfocus
        self._phfocus_dev = None
        self._phfocus_ia = None
        self._phfocus_on_frame_ready = None
        self._phfocus_is_running = False
        #Callbacks on image event
        self._call_on_image = set()
        self._got_image_event = Event()
        self._image_data = None
        self._image_timestamp = None
        self._imgs_since_start = 0

        self._logger.debug('Calling self on constructor input')
        if model is not None:
            self.model = model
        if identity is not None:
            self.identity = identity
        if name is not None:
            self.name = name
        if auto_init and not None in (model, identity):
            try:
                self._logger.debug('Trying to auto-initialise')
                self.initialize()
            except:
                self._logger.info('Camera initialization failed')
        self._logger.debug('Registering destructor')
        # TODO: Should we register deinitialisor instead? (probably yes...)
        import atexit, weakref
        atexit.register(weakref.ref(self.__del__))
        self._logger.info('Camera instance created with name: ' + self.name + '.')

    def __del__(self):
        """Destructor. Releases hardware."""
        try:
            self._log_debug('Destructor called for name: '+str(self.name))
        except:
            pass
        if self.is_init:
            try:
                self._log_debug('Is initialised, de-initing')
            except:
                pass
            self.deinitialize()
        try:
            self._log_debug('Instance deleted')
        except:
            pass

    # Loggers with added names
    def _log_debug(self, msg, **kwargs):
        self._logger.debug(self.name + ': ' + msg, **kwargs)
    def _log_info(self, msg, **kwargs):
        self._logger.info(self.name + ': ' + msg, **kwargs)
    def _log_warning(self, msg, **kwargs):
        self._logger.warning(self.name + ': ' + msg, **kwargs)
    def _log_exception(self, msg, **kwargs):
        self._logger.exception(self.name + ': ' + msg, **kwargs)

    def _ptgrey_release(self):
        """PRIVATE: Release Point Grey hardware resources."""
        self._log_debug('PointGrey hardware release called')
        if self._ptgrey_camera is not None:
            self._log_debug('Deleting PtGrey camera object')
            del(self._ptgrey_camera) #Preferred over =None according to PtGrey
            self._ptgrey_camera = None
        if self._ptgrey_camlist is not None:
            self._log_debug('Clearing and deleting PtGrey camlist')
            self._ptgrey_camlist.Clear()
            del(self._ptgrey_camlist)
            self._ptgrey_camlist = None
        if self._ptgrey_system is not None:
            self._log_debug('Has PtGrey system. Is in use? '+str(self._ptgrey_system.IsInUse()))
            if not self._ptgrey_system.IsInUse():
                self._log_debug('Not in use, releasing and deleting')
                self._ptgrey_system.ReleaseInstance()
                del(self._ptgrey_system)
                self._ptgrey_system = None
        self._log_debug('Hardware released')

    def _phfocus_release(self):
        """PRIVATE: Release PhotonFocus hardware resources."""
        self._log_debug('Photon Focus hardware release called')
        if self._phfocus_ia:
            self._phfocus_ia.stop_acquisition()
            try:
                self._phfocus_ia.destroy()
            except:
                pass
            del (self._phfocus_ia)
            self._phfocus_ia = None
        if self._phfocus_dev:
            try:
                self._phfocus_dev.reset()
            except:
                pass
            del (self._phfocus_dev)
            self._phfocus_dev = None
        self._log_debug('PhotonFocus Hardware released')

    def set_on_frame_ready(self, on_frame_ready):
        """Add a method to be called on new frame arrival"""
        self._phfocus_on_frame_ready = on_frame_ready

    @property
    def debug_folder(self):
        """pathlib.Path: Get or set the path for debug logging. Will create folder if not existing."""
        return self._debug_folder
    @debug_folder.setter
    def debug_folder(self, path):
        # Do not do logging in here! This will be called before the logger is set up
        path = Path(path) #Make sure pathlib.Path
        if path.is_file():
            path = path.parent
        if not path.is_dir():
            path.mkdir(parents=True)
        self._debug_folder = path

    @property
    def name(self):
        """str: Get or set the name."""
        return self._name
    @name.setter
    def name(self, name):
        self._log_debug('Setting name to: '+str(name))
        self._name = str(name)
        self._log_debug('Name set to '+str(self.name))

    @property
    def model(self):
        """str: Get or set the device model.

        Supported:
            - 'ptgrey' for FLIR/Point Grey cameras (using Spinnaker/PySpin SDKs).

        - This will determine which hardware API that is used.
        - Must set before initialising the device and may not be changed for an initialised device.
        """
        return self._model
    @model.setter
    def model(self, model):
        self._log_debug('Setting model to: '+str(model))
        assert not self.is_init, 'Can not change already initialised device model'
        model = str(model)
        assert model.lower() in self._supported_models,\
                                                'Model type not recognised, allowed: '+str(self._supported_models)
        #TODO: Check that the APIs are available.
        self._model = model
        self._log_debug('Model set to '+str(self.model))

    @property
    def identity(self):
        """str: Get or set the device and/or input. Model must be defined first.

        - For model *ptgrey* this is the serial number *as a string*
        - Must set before initialising the device and may not be changed for an initialised device.
        """
        return self._identity
    @identity.setter
    def identity(self, identity):
        self._log_debug('Setting identity to: '+str(identity))
        assert not self.is_init, 'Can not change already initialised device'
        assert self.model is not None, 'Must define model first'
        identity = str(identity)
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PtGrey, checking vailidity')
            import PySpin
            if not self._ptgrey_system:
                self._ptgrey_system = PySpin.System.GetInstance() #Get singleton
            self._ptgrey_camlist = self._ptgrey_system.GetCameras()
            self._log_debug('Got cam list, size:'+str(self._ptgrey_camlist.GetSize()))
            self._ptgrey_camera = self._ptgrey_camlist.GetBySerial(identity)
            valid = self._ptgrey_camera.IsValid()
            self._log_debug('Got object, valid: '+str(valid))
            if valid:
                self._log_debug('Already init: '+str(self._ptgrey_camera.IsInitialized()))
            if not valid:
                self._log_debug('Invalid camera object. Cleaning up')
                del(self._ptgrey_camera)
                self._ptgrey_camera = None
                self._ptgrey_camlist.Clear()
                raise AssertionError('The camera was not found')
            elif self._ptgrey_camera.IsInitialized():
                self._log_debug('Camera object already in use. Cleaning up')
                del(self._ptgrey_camera)
                self._ptgrey_camera = None
                self._ptgrey_camlist.Clear()
                raise RuntimeError('The camera is already in use')
            else:
                self._log_debug('Seems valid. Setting identity and cleaning up')
                del(self._ptgrey_camera)
                self._ptgrey_camera = None
                self._identity = identity
                self._ptgrey_camlist.Clear()
        elif self.model.lower() == 'phfocus':
            self._identity = identity
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
        self._log_debug('Identity set to: '+str(self.identity))

    @property
    def is_init(self):
        """bool: True if the device is initialised (and therefore ready to start)."""
        if not self.model: return False
        if self.model.lower() == 'ptgrey':
            init = self._ptgrey_camera is not None and self._ptgrey_camera.IsInitialized()
            return init
        if self.model.lower() == 'phfocus':
            #TODO: DM finish this
            init = self._phfocus_dev is not None
            return init
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def initialize(self):
        """Initialise (make ready to start) the device. The model and identity must be defined."""
        self._log_debug('Initialising')
        assert not self.is_init, 'Already initialised'
        assert not None in (self.model, self.identity), 'Must define model and identity before initialising'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin, try to initialise')
            import PySpin
            if self._ptgrey_camera is not None:
                raise RuntimeError('There is already a camera object here')
            if not self._ptgrey_system: self._ptgrey_system = PySpin.System.GetInstance() #Get singleton
            if self._ptgrey_camlist: #Clear old list and get fresh one
                self._ptgrey_camlist.Clear()
                del(self._ptgrey_camlist)
            self._ptgrey_camlist = self._ptgrey_system.GetCameras()
            self._log_debug('Getting pyspin object and initialising')
            self._ptgrey_camera = self._ptgrey_camlist.GetBySerial(self.identity)
            self._ptgrey_camera.Init()
            # BASIC SETUP
            self._log_debug('Setting pixel format to mono16')
            self._ptgrey_camera.PixelFormat.SetValue(PySpin.PixelFormat_Mono16)
            self._log_debug('Setting gamma off')
            nodemap = self._ptgrey_camera.GetNodeMap()
            PySpin.CBooleanPtr(nodemap.GetNode('GammaEnable')).SetValue(False)
            self._log_debug('Setting acquisition mode to continuous')
            self._ptgrey_camera.AcquisitionMode.SetIntValue(PySpin.AcquisitionMode_Continuous)
            self._log_debug('Setting stream mode to newest only')
            self._ptgrey_camera.TLStream.StreamBufferHandlingMode.SetIntValue(
                                                                        PySpin.StreamBufferHandlingMode_NewestOnly)
            class PtGreyEventHandler(PySpin.ImageEventHandler):
                """Barebones event handler for ptgrey, just pass along the event to the Camera class."""
                def __init__(self, parent):
                    assert parent.model.lower() == 'ptgrey', 'Trying to attach ptgrey event handler to non ptgrey model'
                    super().__init__()
                    self.parent = parent

                def OnImageEvent(self, img_ptr):
                    """Read out the image and a timestamp, reshape to array, pass to parent"""
                    self.parent._log_debug('Image event! Unpack and release pointer')
                    self.parent._image_timestamp = datetime.utcnow()
                    try:
                        img = img_ptr.GetData().reshape((img_ptr.GetHeight(), img_ptr.GetWidth()))
                        if self.parent._flipX:
                            img = np.fliplr(img)
                        if self.parent._flipY:
                            img = np.flipud(img)
                        if self.parent._rot90:
                            img = np.rot90(img, self.parent._rot90)
                        self.parent._image_data = img
                    except:
                        self.parent._log_warning('Failed to unpack image', exc_info=True)
                        self.parent._image_data = None
                    finally:
                        img_ptr.Release()
                    self.parent._got_image_event.set()
                    self.parent._log_debug('Time: ' + str(self.parent._image_timestamp) \
                                           + ' Size:' + str(self.parent._image_data.shape) \
                                           + ' Type:' + str(self.parent._image_data.dtype))
                    for func in self.parent._call_on_image:
                        try:
                            self.parent._log_debug('Calling back to: ' + str(func))
                            func(self.parent._image_data, self.parent._image_timestamp)
                        except:
                            self.parent._log_warning('Failed image callback', exc_info=True)
                    self.parent._imgs_since_start += 1
                    self.parent._log_debug('Event handler finished.')

            self._ptgrey_event_handler = PtGreyEventHandler(self)
            self._log_debug('Created ptgrey image event handler')
            self._ptgrey_camera.RegisterEventHandler( self._ptgrey_event_handler )
            self._log_debug('Registered ptgrey image event handler')
            self._log_info('Camera successfully initialised')
        elif self.model.lower() == 'phfocus':
            self._log_debug('Using GenICam, try to initialise')
            h = Harvester()
            locs = [
                r"~/tools/mvImpact/lib/x86_64/mvGenTLProducer.cti",
                r"C:\Program Files\MATRIX VISION\mvIMPACT Acquire\bin\x64\mvGenTLProducer.cti"
            ]
            cti = ""
            for loc in locs:
                if Path(loc).expanduser().exists():
                    cti = loc
            if not cti:
                raise FileNotFoundError("Could not locate cti file: mvGenTLProducer.cti")

            cti = str(Path(cti).expanduser())
            h.add_file(cti)
            h.update()
            # len(h.device_info_list) #For debug purpose
            # h.device_info_list[0]
            ia = h.create_image_acquirer(0)
            self._phfocus_dev = h
            self._phfocus_ia = ia

            #print('Nodes:', dir(self._phfocus_ia.remote_device.node_map))    # get all available properties of genicam
            # BASIC SETUP
            self._log_debug('Setting pixel format to mono12')
            self._log_debug('Setting acquisition mode to continuous')
            self._phfocus_ia.remote_device.node_map.PixelFormat.value = 'Mono12'
            self._phfocus_ia.remote_device.node_map.AcquisitionMode.value = 'Continuous'
            self._phfocus_ia.remote_device.node_map.EnAcquisitionFrameRate.value = 'True'

            from harvesters.core import Callback
            class CallbackOnNewBuffer(Callback):
                """Barebones event handler for phfocus, just pass along the event to the Camera class."""
                def __init__(self, parent):
                    super().__init__()
                    self.parent = parent

                def emit(self, context):
                    """Read out the image and a timestamp, reshape to array, pass to parent"""
                    with self.parent._phfocus_ia.fetch_buffer() as buffer:
                        # Work with the fetched buffer.
                        """Read out the image and a timestamp, reshape to array, pass to cam object"""
                        self.parent._log_debug('Image event! Unpack ')
                        self.parent._image_timestamp = datetime.utcnow()
                        try:
                            component = buffer.payload.components[0]
                            data = component.data.reshape(component.height, component.width)
                            img = data.copy()
                            if self.parent._flipX:
                                img = np.fliplr(img)
                            if self.parent._flipY:
                                img = np.flipud(img)
                            if self.parent._rot90:
                                img = np.rot90(img, self.parent._rot90)
                            self.parent._image_data = img
                        except:
                            self.parent._log_warning('Failed to read image', exc_info=True)
                            self.parent._image_data = None
                        self.parent._got_image_event.set()
                        self.parent._log_debug('Time: ' + str(self.parent._image_timestamp) \
                                        + ' Size:' + str(self.parent._image_data.shape) \
                                        + ' Type:' + str(self.parent._image_data.dtype))
                        for func in self.parent._call_on_image:
                            try:
                                self.parent._log_debug('Calling back to: ' + str(func))
                                func(self.parent._image_data, self.parent._image_timestamp)
                            except:
                                self.parent._log_warning('Failed image callback', exc_info=True)
                        self.parent._imgs_since_start += 1
                        self.parent._log_debug('Image event handler finished.')
                        self.parent._phfocus_is_running = True

            # Add method to the callback method for camera NEW_BUFFER event. PhotonFocus
            on_new_buffer = CallbackOnNewBuffer(self)
            ia.add_callback(
                ia.Events.NEW_BUFFER_AVAILABLE,
                on_new_buffer
            )

        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def deinitialize(self):
        """De-initialise the device and release hardware resources. Will stop the acquisition if it is running."""
        self._log_debug('De-initialising')
        assert self.is_init, 'Not initialised'
        if self.is_running:
            self._log_debug('Is running, stopping')
            self.stop()
            self._log_debug('Stopped')
        if self._ptgrey_camera:
            self._log_debug('Found PtGrey camera, deinitialising')
            try:
                self._ptgrey_camera.UnregisterEventHandler(self._ptgrey_event_handler)
                self._log_debug('Unregistered event handler')
            except:
                self._log_exception('Failed to unregister event handler')
            try:
                self._ptgrey_camera.DeInit()
                del(self._ptgrey_camera)
                self._ptgrey_camera = None
                self._log_debug('Deinitialised PtGrey camera object and deleted')
            except:
                self._log_exception('Failed to close task')
            self._log_debug('Trying to release PtGrey hardware resources')
            self._ptgrey_release()

        elif self._phfocus_dev:
            self._log_debug('Found PhtonFocus camera, deinitialising')
            self._phfocus_release()
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def available_properties(self):
        """tuple of str: Get all the available properties (settings) supported by this device."""
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            return ('flip_x', 'flip_y', 'rotate_90', 'plate_scale', 'rotation', 'binning', 'size_readout', 'frame_rate_auto',\
                    'frame_rate', 'gain_auto', 'gain', 'exposure_time_auto', 'exposure_time')
        elif self.model.lower() == 'phfocus':
            #TODO: check whether its vailidity
            return ('flip_x', 'flip_y', 'rotate_90', 'plate_scale', 'rotation', 'size_readout',\
                    'frame_rate', 'gain', 'exposure_time')
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def flip_x(self):
        """bool: Get or set if the image X-axis should be flipped. Default is False."""
        self._log_debug('Get flip-X called')
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PtGrey camera. Will flip the received image array ourselves: ' +str(self._flipX))
            return self._flipX
        elif self.model.lower() == 'phfocus':
            self._log_debug('Using phfocus camera. Will flip the received image array ourselves: ' +str(self._flipX))
            return self._flipX
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @flip_x.setter
    def flip_x(self, flip):
        self._log_debug('Set flip-X called with: '+str(flip))
        assert self.is_init, 'Camera must be initialised'
        flip = bool(flip)
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PtGrey camera. Will flip the received image array ourselves.')
            self._flipX = flip
            self._log_debug('_flipX set to: '+str(self._flipX))
        elif self.model.lower() == 'phfocus':
            self._log_debug('Using phfocus camera. Will flip the received image array ourselves.')
            self._flipX = flip
            self._log_debug('_flipX set to: '+str(self._flipX))
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def flip_y(self):
        """bool: Get or set if the image Y-axis should be flipped. Default is False."""
        self._log_debug('Get flip-Y called')
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PtGrey camera. Will flip the received image array ourselves: ' + str(self._flipY))
            return self._flipY
        elif self.model.lower() == 'phfocus':
            self._log_debug('Using Phfocus camera. Will flip the received image array ourselves: ' + str(self._flipY))
            return self._flipY
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @flip_y.setter
    def flip_y(self, flip):
        self._log_debug('Set flip-Y called with: '+str(flip))
        assert self.is_init, 'Camera must be initialised'
        flip = bool(flip)
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PtGrey camera. Will flip the received image array ourselves.')
            self._flipY = flip
            self._log_debug('_flipY set to: '+str(self._flipY))
        elif self.model.lower() == 'phfocus':
            self._log_debug('Using PhotonFocus camera. Will flip the received image array ourselves.')
            self._flipY = flip
            self._log_debug('_flipY set to: '+str(self._flipY))
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def rotate_90(self):
        """int: Get or set how many times the image should be rotated by 90 degrees. Applied *after* flip_x and flip_y.
        """
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey' or self.model.lower() == 'phfocus':
            return self._rot90
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @rotate_90.setter
    def rotate_90(self, k):
        self._log_debug('Set rot90 called with: '+str(k))
        assert self.is_init, 'Camera must be initialised'
        k = int(k)
        if self.model.lower() == 'ptgrey' or self.model.lower() == 'phfocus':
            self._log_debug('Using PtGrey or Phfocus camera. Will rotate the received image array ourselves.')
            self._rot90 = k
            self._log_debug('rot90 set to: '+str(self._rot90))
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def plate_scale(self):
        """float: Get or set the plate scale of the Camera in arcsec per pixel.

        This will not affect anything in this class but is used elsewhere. Set this to the physical pixel plate scale
        *before* any binning. When getting the plate scale it will be scaled by the binning factor.
        """
        return self._plate_scale * self.binning
    @plate_scale.setter
    def plate_scale(self, arcsec):
        self._log_debug('Set plate scale called with: '+str(arcsec))
        self._plate_scale = float(arcsec)
        self._log_debug('Plate scale set to: '+str(self.plate_scale))

    @property
    def rotation(self):
        """float: Get or set the camera rotation relative to the horizon in degrees.

        This does not affect the received images, but is used elsewhere. Use rotate_90 first to keep this rotation
        small.
        """
        return self._rotation
    @rotation.setter
    def rotation(self, rot):
        self._log_debug('Set rotation called with: '+str(rot))
        self._rotation = float(rot)
        self._log_debug('Rotation set to: '+str(self.rotation))

    @property
    def frame_rate_auto(self):
        """bool: Get or set automatic frame rate. If True camera will run as fast as possible."""
        self._log_debug('Get frame rate auto called')
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CBooleanPtr(nodemap.GetNode('AcquisitionFrameRateEnable'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node):
                self._log_debug('Node not available')
                raise RuntimeError('Unable to read from camera')
            else:
                val = node.GetValue()
                self._log_debug('Returning not '+str(val))
                return not val
        elif self.model.lower() == 'phfocus':
            self._log_debug('Phfocus camera will not use auto frame rate')
            return False
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @frame_rate_auto.setter
    def frame_rate_auto(self, auto):
        self._log_debug('Set frame rate called with: '+str(auto))
        assert self.is_init, 'Camera must be initialised'
        auto = bool(auto)
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CBooleanPtr(nodemap.GetNode('AcquisitionFrameRateEnable'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
                self._log_debug('Node not available or not writable. Available:'+str(PySpin.IsAvailable(node))\
                                   +' Writable:'+str(PySpin.IsWritable(node)))
                raise RuntimeError('Unable to command camera')
            else:
                self._log_debug('Setting frame rate')
                node.SetValue(not auto)
        elif self.model.lower() == 'ptgrey':
            self._log_debug('PhFocus auto frame rate will not be used')
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def frame_rate_limit(self):
        """tuple of float: Get the minimum and maximum frame rate in Hz supported."""
        self._log_debug('Get frame rate limit called')
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node1 = PySpin.CFloatPtr(nodemap.GetNode('FrameRateHz_Min'))
            node2 = PySpin.CFloatPtr(nodemap.GetNode('FrameRateHz_Max'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node1) or not PySpin.IsAvailable(node2):
                self._log_debug('One node not available. Node1:'+str(PySpin.IsAvailable(node1))\
                                   +' Node2:'+str(PySpin.IsAvailable(node2)))
                raise RuntimeError('Unable to read from camera')
            else:
                val = (node1.GetValue(), node2.GetValue())
                self._log_debug('Returning '+str(val))
                return val
        elif self.model.lower() == 'phfocus':
            self._log_debug('Using phfocus frame rate limits')
            frame_rate_max = self._phfocus_ia.remote_device.node_map.AcquisitionFrameRateMax.value
            frame_rate_min = 2.1 #Hz
            return frame_rate_min, frame_rate_max
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def frame_rate(self):
        """float: Get or set the camera frame rate in Hz. Will set auto frame rate to False."""
        self._log_debug('Get frame rate called')
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CFloatPtr(nodemap.GetNode('AcquisitionFrameRate'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node):
                self._log_debug('Node not available')
                raise RuntimeError('Unable to read from camera')
            else:
                val = node.GetValue()
                self._log_debug('Returning '+str(val))
                return val
        elif self.model.lower() == 'phfocus':
            if self._phfocus_ia:
                return self._phfocus_ia.remote_device.node_map.AcquisitionFrameRate.value
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @frame_rate.setter
    def frame_rate(self, frame_rate_hz):
        self._log_debug('Set frame rate called with: '+str(frame_rate_hz))
        assert self.is_init, 'Camera must be initialised'
        frame_rate_hz = float(frame_rate_hz)
        if self.frame_rate_auto:
            self._log_debug('Frame rate is set to auto. Command auto off')
            self.frame_rate_auto = False
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CFloatPtr(nodemap.GetNode('AcquisitionFrameRate'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
                self._log_debug('Node not available or not writable. Available:'+str(PySpin.IsAvailable(node))\
                                   +' Writable:'+str(PySpin.IsWritable(node)))
                raise RuntimeError('Unable to command camera')
            else:
                self._log_debug('Setting frame rate')
                try:
                    node.SetValue(frame_rate_hz)
                except PySpin.SpinnakerException as e:
                    if 'OutOfRangeException' in e.message:
                        raise AssertionError('The commanded value is outside the allowed range. See frame_rate_limit')
                    else:
                        raise #Rethrows error
        elif self.model.lower() == 'phfocus':
            if self._phfocus_ia:
                try:
                    if frame_rate_hz< self._phfocus_ia.remote_device.node_map.AcquisitionFrameRateMax.value:
                        self._phfocus_ia.remote_device.node_map.AcquisitionFrameRate.value = frame_rate_hz
                    else:
                        self._log_warning('Frame rate above max limit. Change exposure time.')
                except self._phfocus_ia as e:
                    if 'OutOfRangeException' in e.message:
                        raise AssertionError('The commanded value is outside the allowed range. See frame_rate_limit')
                    else:
                        raise #Rethrows error
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def gain_auto(self):
        """bool: Get or set automatic gain. If True the gain will be continuously updated."""
        self._log_debug('Get gain auto called')
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CEnumerationPtr(nodemap.GetNode('GainAuto'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node):
                self._log_debug('Node not available')
                raise RuntimeError('Unable to read from camera')
            else:
                val = node.GetCurrentEntry()
                val = val.GetDisplayName().lower()
                self._log_debug('Node value: '+str(val))
                if val == 'off':
                    self._log_debug('Returning False')
                    return False
                elif val == 'continuous':
                    self._log_debug('Returning True')
                    return True
                else:
                    self._log_debug('Unexpected return value')
                    raise RuntimeError('Unknow response from camera')
        elif self.model.lower() == 'phfocus':
            self._log_debug('Using phfocus auto gain is always disable')
            return False
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @gain_auto.setter
    def gain_auto(self, auto):
        self._log_debug('Set gain called with: '+str(auto))
        assert self.is_init, 'Camera must be initialised'
        auto = bool(auto)
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            if auto:
                set_to = 'Continuous'
            else:
                set_to = 'Off'
            self._log_debug('Will set gain auto to: '+set_to)
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CEnumerationPtr(nodemap.GetNode('GainAuto'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
                self._log_debug('Node not available or not writable. Available:'+str(PySpin.IsAvailable(node))\
                                   +' Writable:'+str(PySpin.IsWritable(node)))
                raise RuntimeError('Unable to command camera')
            else:
                self._log_debug('Setting gain')
                node.SetIntValue(node.GetEntryByName(set_to).GetValue())
        elif self.model.lower() == 'phfocus':
            self._log_debug('Using phfocus auto gain is always disable')
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def gain_limit(self):
        """tuple of float: Get the minimum and maximum gain in dB supported."""
        self._log_debug('Get gain limit called')
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node1 = PySpin.CFloatPtr(nodemap.GetNode('GainDB_Min'))
            node2 = PySpin.CFloatPtr(nodemap.GetNode('GainDB_Max'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node1) or not PySpin.IsAvailable(node2):
                self._log_debug('One node not available. Node1:'+str(PySpin.IsAvailable(node1))\
                                   +' Node2:'+str(PySpin.IsAvailable(node2)))
                raise RuntimeError('Unable to read from camera')
            else:
                val = (node1.GetValue(), node2.GetValue())
                self._log_debug('Returning '+str(val))
                return val
        elif self.model.lower() == 'phfocus':
            #TODO: find pffocus gain limits
            min_g = 0.1
            max_g = 16
            return min_g, max_g
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def gain(self):
        """float: Get or set the camera gain in dB. Will set auto frame rate to False."""
        self._log_debug('Get gain called')
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CFloatPtr(nodemap.GetNode('Gain'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node):
                self._log_debug('Node not available')
                raise RuntimeError('Unable to read from camera')
            else:
                val = node.GetValue()
                self._log_debug('Returning '+str(val))
                return val
        elif self.model.lower() == 'phfocus':
            gain_val = self._phfocus_ia.remote_device.node_map.Gain.value
            self._log_debug('Returning '+str(gain_val))
            return gain_val
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @gain.setter
    def gain(self, gain_db):
        self._log_debug('Set gain called with: '+str(gain_db))
        assert self.is_init, 'Camera must be initialised'
        gain_db = float(gain_db)
        if self.gain_auto:
            self._log_debug('Gain is set to auto. Command auto off')
            self.gain_auto = False
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CFloatPtr(nodemap.GetNode('Gain'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
                self._log_debug('Node not available or not writable. Available:'+str(PySpin.IsAvailable(node))\
                                   +' Writable:'+str(PySpin.IsWritable(node)))
                raise RuntimeError('Unable to command camera')
            else:
                self._log_debug('Setting gain')
                try:
                    node.SetValue(gain_db)
                except PySpin.SpinnakerException as e:
                    if 'OutOfRangeException' in e.message:
                        raise AssertionError('The commanded value is outside the allowed range. See gain_limit')
                    else:
                        raise #Rethrows error
        elif self.model.lower() == 'phfocus':
            self._log_debug('Setting gain phfocus'+str(gain_db))
            self._phfocus_ia.remote_device.node_map.Gain.value = gain_db
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def exposure_time_auto(self):
        """bool: Get or set automatic exposure time. If True the exposure time will be continuously updated."""
        self._log_debug('Get exposure time auto called')
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CEnumerationPtr(nodemap.GetNode('ExposureAuto'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node):
                self._log_debug('Node not available')
                raise RuntimeError('Unable to read from camera')
            else:
                val = node.GetCurrentEntry()
                val = val.GetDisplayName().lower()
                self._log_debug('Node value: '+str(val))
                if val == 'off':
                    self._log_debug('Returning False')
                    return False
                elif val == 'continuous':
                    self._log_debug('Returning True')
                    return True
                else:
                    self._log_debug('Unexpected return value')
                    raise RuntimeError('Unknow response from camera')
        elif self.model.lower() == 'phfocus':
            self._log_debug('Phfocus auto exposure always disable')
            return False
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @exposure_time_auto.setter
    def exposure_time_auto(self, auto):
        self._log_debug('Set expsure time called with: '+str(auto))
        assert self.is_init, 'Camera must be initialised'
        auto = bool(auto)
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            if auto:
                set_to = 'Continuous'
            else:
                set_to = 'Off'
            self._log_debug('Will set exposure auto auto to: '+set_to)
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CEnumerationPtr(nodemap.GetNode('ExposureAuto'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
                self._log_debug('Node not available or not writable. Available:'+str(PySpin.IsAvailable(node))\
                                   +' Writable:'+str(PySpin.IsWritable(node)))
                raise RuntimeError('Unable to command camera')
            else:
                self._log_debug('Setting exposure auto to: '+set_to)
                node.SetIntValue(node.GetEntryByName(set_to).GetValue())
        elif self.model.lower() == 'phfocus':
            self._log_debug('Phfocus auto exposure always disable')
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def exposure_time_limit(self):
        """tuple of float: Get the minimum and maximum expsure time in ms supported."""
        self._log_debug('Get gain limit called')
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node1 = PySpin.CFloatPtr(nodemap.GetNode('ExposureTime_FloatMin'))
            node2 = PySpin.CFloatPtr(nodemap.GetNode('ExposureTime_FloatMax'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node1) or not PySpin.IsAvailable(node2):
                self._log_debug('One node not available. Node1:'+str(PySpin.IsAvailable(node1))\
                                   +' Node2:'+str(PySpin.IsAvailable(node2)))
                raise RuntimeError('Unable to read from camera')
            else:
                val = (node1.GetValue()/1000, node2.GetValue()/1000)
                self._log_debug('Returning '+str(val))
                return val
        elif self.model.lower() == 'phfocus':
        #TODO: get exposure limits from camera
            min_ex = 0.01
            max_ex = 671
            return min_ex, max_ex
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def exposure_time(self):
        """float: Get or set the camera expsure time in ms. Will set auto exposure time to False."""
        self._log_debug('Get exposure time called')
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CFloatPtr(nodemap.GetNode('ExposureTime'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node):
                self._log_debug('Node not available')
                raise RuntimeError('Unable to read from camera')
            else:
                val = node.GetValue() / 1000 #microseconds used in PtGrey
                self._log_debug('Returning '+str(val))
                return val
        elif self.model.lower() == 'phfocus':
            factor = 1000
            if self._phfocus_ia:
                return int(self._phfocus_ia.remote_device.node_map.ExposureTime.value / factor) #out is in ms
            return 0.0
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @exposure_time.setter
    def exposure_time(self, exposure_ms):
        self._log_debug('Set exposure time called with: '+str(exposure_ms))
        assert self.is_init, 'Camera must be initialised'
        exposure_ms = float(exposure_ms)
        if self.exposure_time_auto:
            self._log_debug('Exposure time is set to auto. Command auto off')
            self.exposure_time_auto = False
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node = PySpin.CFloatPtr(nodemap.GetNode('ExposureTime'))
            self._log_debug('Got the node')
            if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
                self._log_debug('Node not available or not writable. Available:'+str(PySpin.IsAvailable(node))\
                                   +' Writable:'+str(PySpin.IsWritable(node)))
                raise RuntimeError('Unable to command camera')
            else:
                self._log_debug('Setting exposure time to (us): ' + str(exposure_ms*1000))
                try:
                    node.SetValue(exposure_ms*1000)
                except PySpin.SpinnakerException as e:
                    if 'OutOfRangeException' in e.message:
                        raise AssertionError('The commanded value is outside the allowed range.'\
                                             +' See exposure_time_limit')
                    else:
                        raise #Rethrows error
        elif self.model.lower() == 'phfocus':
            factor = 1000   #convert to ms
            if self._phfocus_ia:
                self._phfocus_ia.remote_device.node_map.ExposureTime.value = exposure_ms * factor
            self._log_debug('Setting exposure time phfocus to (ms): ' + str(exposure_ms))
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def binning(self):
        """int: Number of pixels to bin in each dimension (e.g. 2 gives 2x2 binning). Bins by summing.

        Setting will stop and restart camera if running. Will scale size_readout to show the same sensor area.
        """
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node_horiz = PySpin.CIntegerPtr(nodemap.GetNode('BinningHorizontal'))
            node_vert = PySpin.CIntegerPtr(nodemap.GetNode('BinningVertical'))
            self._log_debug('Got the nodes')
            try:
                val_horiz = node_horiz.GetValue()
                val_vert = node_vert.GetValue()
                self._log_debug('Got '+str(val_horiz)+' '+str(val_vert))
                if val_horiz != val_vert:
                    self._log_warning('Horzontal and vertical binning is not equal.')
                return val_horiz
            except PySpin.SpinnakerException:
                self._log_warning('Failed to read', exc_info=True)
        elif self.model.lower() == 'phfocus':
            try:
                val_horiz = self._phfocus_ia.remote_device.node_map.BinningHorizontal.value
                val_vert = self._phfocus_ia.remote_device.node_map.BinningVertical.value
                self._log_debug('Got '+str(val_horiz)+' '+str(val_vert))
                if val_horiz != val_vert:
                    self._log_warning('Horzontal and vertical binning is not equal.')
                return val_horiz
            except:
                self._log_warning('Failed to read', exc_info=True)
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @binning.setter
    def binning(self, binning):
        self._log_debug('Set binning called with: '+str(binning))
        assert self.is_init, 'Camera must be initialised'
        binning = int(binning)
        was_running = self.is_running
        if self.is_running:
            self._log_debug('Camera is running, stop it and restart immediately after.')
            self.stop()
        initial_size = self.size_readout
        initial_bin = self.binning
        self._log_debug('Initial sensor readout area and binning: '+str(initial_size)+' ,'+str(initial_bin))
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node_horiz = PySpin.CIntegerPtr(nodemap.GetNode('BinningHorizontal'))
            node_vert = PySpin.CIntegerPtr(nodemap.GetNode('BinningVertical'))
            self._log_debug('Got the nodes')
            try:
                node_horiz.SetValue(binning)
                node_vert.SetValue(binning)
            except PySpin.SpinnakerException as e:
                self._log_debug('Failure setting', exc_info=True)
                if 'OutOfRangeException' in e.message:
                    raise ValueError('Commanded value not allowed.')
                elif 'AccessException' in e.message:
                    raise AssertionError('Not allowed to change binning now.')
                else:
                    raise #Rethrows error
        elif self.model.lower() == 'phfocus':
            try:
                self._phfocus_ia.remote_device.node_map.BinningHorizontal.value = binning
                self._phfocus_ia.remote_device.node_map.BinningVertical.value = binning
            except:
                self._log_warning('Failure setting binning of phfocus', exc_info=True)
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
        new_bin = self.binning
        bin_scaling = new_bin/initial_bin
        new_size = [round(sz/bin_scaling) for sz in initial_size]
        self._log_debug('New binning and new size to set: '+str(new_bin)+' ,'+str(new_size))
        try:
            self.size_readout = new_size
            self._log_debug('Set new size to: ' + str(self.size_readout))
        except:
            self._log_warning('Failed to scale readout after binning change', exc_info=True)
        if was_running:
            try:
                self.start()
                self._log_debug('Restarted')
            except Exception:
                self._log_debug('Failed to restart: ', exc_info=True)

    @property
    def size_max(self):
        """tuple of int: Get the maximum allowed readout size (width, height) in pixels."""
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node_width = PySpin.CIntegerPtr(nodemap.GetNode('WidthMax'))
            node_height = PySpin.CIntegerPtr(nodemap.GetNode('HeightMax'))
            self._log_debug('Got the nodes')
            try:
                val_w = node_width.GetValue()
                val_h = node_height.GetValue()
                self._log_debug('Got '+str(val_w)+' '+str(val_h))
            except PySpin.SpinnakerException:
                self._log_debug('Failure reading', exc_info=True)
                raise
            return (val_w, val_h)
        elif self.model.lower() == 'phfocus':
            try:
                val_w = self._phfocus_ia.remote_device.node_map.WidthMax.value
                val_h = self._phfocus_ia.remote_device.node_map.HeightMax.value
                self._log_debug('Got '+str(val_w)+' '+str(val_h))
            except:
                self._log_debug('Failure reading', exc_info=True)
                raise
            return (val_w, val_h)
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def size_readout(self):
        """tuple of int: Get or set the number of pixels read out (width, height). Will automatically center.

        This applies after binning, i.e. this is the size the output image will be.

        Setting will stop and restart camera if running.
        """
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node_width = PySpin.CIntegerPtr(nodemap.GetNode('Width'))
            node_height = PySpin.CIntegerPtr(nodemap.GetNode('Height'))
            self._log_debug('Got the nodes')
            try:
                val_w = node_width.GetValue()
                val_h = node_height.GetValue()
                self._log_debug('Got '+str(val_w)+' '+str(val_h))
            except PySpin.SpinnakerException:
                self._log_debug('Failure reading', exc_info=True)
                raise
            return (val_w, val_h)
        elif self.model.lower() == 'phfocus':
            try:
                val_w = self._phfocus_ia.remote_device.node_map.Width.value
                val_h = self._phfocus_ia.remote_device.node_map.Height.value
                self._log_debug('Got '+str(val_w)+' '+str(val_h))
                return (val_w, val_h)
            except:
                self._log_debug('Failure reading', exc_info=True)
                raise
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @size_readout.setter
    def size_readout(self, size):
        assert self.is_init, 'Camera must be initialised'
        self._log_debug('Got set readout with: ' + str(size))
        if isinstance(size, (int, float)): size = (size, size)
        size = tuple([int(x) for x in size])
        was_running = self.is_running
        if self.is_running:
            self._log_debug('Camera is running, stop it and restart immediately after.')
            self.stop()
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            nodemap = self._ptgrey_camera.GetNodeMap()
            node_offs_x = PySpin.CIntegerPtr(nodemap.GetNode('OffsetX'))
            node_offs_y = PySpin.CIntegerPtr(nodemap.GetNode('OffsetY'))
            node_width = PySpin.CIntegerPtr(nodemap.GetNode('Width'))
            node_height = PySpin.CIntegerPtr(nodemap.GetNode('Height'))
            self._log_debug('Got nodes for offset and size')
            try:
                node_offs_x.SetValue(0)
                node_offs_y.SetValue(0)
                self._log_debug('Set offsets to zero')
                node_width.SetValue(size[0])
                node_height.SetValue(size[1])
                self._log_debug('Set desired size')
            except PySpin.SpinnakerException as e:
                self._log_debug('Failure setting', exc_info=True)
                if 'OutOfRangeException' in e.message:
                    raise ValueError('Commanded value not allowed.')
                elif 'AccessException' in e.message:
                    raise AssertionError('Not allowed to change readout now.')
                else:
                    raise #Rethrows error
            self._log_debug('Read max size and set offsets to center.')
            try:
                max_size = self.size_max
            except:
                self._log_warning('Failure reading max size to center', exc_info=True)
            new_offset = None
            try:
                actual_w = node_width.GetValue() #Read what we set before to be sure
                actual_h = node_height.GetValue()
                self._log_debug('Actual and max, w,h {} {} {} {}'.format(actual_w, actual_h, max_size[0], max_size[1]))
                new_offset = (round((max_size[0] - actual_w) / 2), round((max_size[1] - actual_h) / 2))
                self._log_debug('Neccessary offset: ' + str(new_offset))
            except PySpin.SpinnakerException:
                self._log_warning('Failure centering readout', exc_info=True)
            if new_offset is not None:
                try:
                    node_offs_x.SetValue(new_offset[0])
                    node_offs_y.SetValue(new_offset[1])
                except PySpin.SpinnakerException:
                    self._log_warning('Failure centering readout', exc_info=True)
        elif self.model.lower() == 'phfocus':
            nodemap = self._phfocus_ia.remote_device.node_map
            self._log_debug('Got nodes for offset and size')
            try:
                self._log_debug('Set offsets to zero')
                nodemap.Width.value = size[0]
                nodemap.Height.value = size[1]
                self._log_debug('Set desired size')
            except:
                self._log_debug('Failure setting', exc_info=True)
                raise #Rethrows error
            self._log_debug('Read max size and set offsets to center.')
            try:
                max_size = self.size_max
            except:
                self._log_warning('Failure reading max size to center', exc_info=True)
            new_offset = None
            try:
                actual_w = nodemap.Width.value #Read what we set before to be sure
                actual_h = nodemap.Height.value
                self._log_debug('Actual and max, w,h {} {} {} {}'.format(actual_w, actual_h, max_size[0], max_size[1]))
                new_offset = (round((max_size[0] - actual_w) / 2), round((max_size[1] - actual_h) / 2))
                self._log_debug('Neccessary offset: ' + str(new_offset))
            except:
                self._log_warning('Failure centering readout', exc_info=True)
            if new_offset is not None:
                try:
                    nodemap.OffsetX.value = new_offset[0]
                    nodemap.OffsetY.value = new_offset[1]
                except:
                    self._log_warning('Failure centering readout', exc_info=True)
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
        if was_running:
            try:
                self.start()
                self._log_debug('Restarted')
            except Exception:
                self._log_debug('Failed to restart: ', exc_info=True)

    def add_event_callback(self, method):
        """Add a method to be called when a new image shows up.

        The method should have the signature (image, timestamp, \*args, \*\*kwargs) where:

        - image (numpy.ndarray): The image data as a 2D numpy array.
        - timestamp (datetime.datetime): UTC timestamp when the image event occurred (i.e. when the capture
          finished).
        - \*args, \*\*kwargs should be allowed for forward compatibility.

        The callback should *not* be used for computations, make sure the method returns as fast as possible.

        Args:
            method: The method to be called, with signature (image, timestamp, \*args, \*\*kwargs).
        """
        self._log_debug('Adding to callbacks: ' + str(method))
        self._call_on_image.add(method)

    def remove_event_callback(self, method):
        """Remove method from event callbacks."""
        try:
            self._call_on_image.remove(method)
        except:
            self._log_warning('Could not remove callback', exc_info=True)

    @property
    def is_running(self):
        """bool: True if device is currently acquiring data."""
        self._log_debug('Checking if running')
        if not self.is_init: return False
        if self.model.lower() == 'ptgrey':
            return self._ptgrey_camera is not None and self._ptgrey_camera.IsStreaming()
        elif self.model.lower() == 'phfocus':
            #TODO: DM finish it
            if self._image_timestamp is not None:
                time_since_last_image = (datetime.utcnow() - self._image_timestamp).total_seconds()
                if time_since_last_image >0.5: #If last image was 1sec ago
                    self._phfocus_is_running = False
            return self._phfocus_dev is not None and self._phfocus_is_running
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def start(self):
        """Start the acquisition. Device must be initialised."""
        assert self.is_init, 'Must initialise first'
        if self.is_running:
            self._log_info('Camera already running, name: '+self.name)
            return
        self._log_debug('Got start command')
        self._imgs_since_start = 0
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PySpin')
            import PySpin
            try:
                self._ptgrey_camera.BeginAcquisition()
            except PySpin.SpinnakerException as e:
                self._log_debug('Could not start:', exc_info=True)
                if 'already streaming' in e.message:
                    self._log_warning('The camera was already streaming...')
                else:
                    raise RuntimeError('Failed to start camera acquisition') from e
        elif self.model.lower() == 'phfocus':
            #TODO: DM finish it, add logs and protections
            try:
                self._phfocus_ia.stop_acquisition()
                self._phfocus_ia.start_acquisition(run_in_background=True)
            except:
                self._log_warning('Phfocus camera cant start, trying reconnect..')
                raise
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
        self._log_info('Acquisition started, name: '+self.name)

    def stop(self):
        """Stop the acquisition."""
        if not self.is_running:
            self._log_info('Camera was not running, name: '+self.name)
            return
        self._log_debug('Got stop command')
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PtGrey')
            try:
                self._ptgrey_camera.EndAcquisition()
            except:
                self._log_debug('Could not stop:', exc_info=True)
                raise RuntimeError('Failed to stop camera acquisition')
        elif self.model.lower() == 'phfocus':
            try:
                self._phfocus_ia.stop_acquisition()
                self._phfocus_is_running = False
            except:
                self._log_debug('Could not stop PhFocus:', exc_info=True)
                raise RuntimeError('Failed to stop PhFocus camera acquisition')
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
        self._image_data = None
        self._image_timestamp = None
        self._got_image_event.clear()
        self._log_info('Acquisition stopped, name: '+self.name)

    def get_next_image(self, timeout=10):
        """Get the next image to be completed. Camera does not have to be running.

        Args:
            timeout (float): Maximum time (seconds) to wait for the image before raising TimeoutError.

        Returns:
            numpy.ndarray: 2d array with image data.
        """
        self._log_debug('Got next image request')
        assert self.is_init, 'Camera must be initialised'
        if not self.is_running:
            self._log_debug('Camera was not running, start and grab the first image')
            self._got_image_event.clear()
            self.start()
            if not self._got_image_event.wait(timeout):
                raise TimeoutError('Getting image timed out')
            img = self._image_data
            self.stop()
        else:
            self._log_debug('Camera running, grab the first image to show up')
            self._got_image_event.clear()
            if not self._got_image_event.wait(timeout):
                raise TimeoutError('Getting image timed out')
            img = self._image_data
        return img

    def get_new_image(self, timeout=10):
        """Get an image guaranteed to be started *after* calling this method. Camera does not have to be running.

        Args:
            timeout (float): Maximum time (seconds) to wait for the image before raising TimeoutError.

        Returns:
            numpy.ndarray: 2d array with image data.
        """
        self._log_debug('Got next image request')
        assert self.is_init, 'Camera must be initialised'
        if not self.is_running:
            self._log_debug('Camera was not running, start and grab the first image')
            self._got_image_event.clear()
            self.start()
            if not self._got_image_event.wait(timeout):
                raise TimeoutError('Getting image timed out')
            img = self._image_data
            self.stop()
        else:
            self._log_debug('Camera running, grab the second image to show up')
            self._got_image_event.clear()
            if not self._got_image_event.wait(timeout/2):
                raise TimeoutError('Getting image timed out')
            self._got_image_event.clear()
            if not self._got_image_event.wait(timeout/2):
                raise TimeoutError('Getting image timed out')
            img = self._image_data
        return img

    def get_latest_image(self):
        """Get latest image in the cache immediately. Camera must be running.

        Returns:
            numpy.ndarray: 2d array with image data.
        """
        self._log_debug('Got latest image request')
        assert self.is_running, 'Camera must be running'
        return self._image_data


class Mount:
    """Control a telescope gimbal mount.

    To initialise a Mount a *model* (determines hardware interface) and *identity* (identifying the specific device)
    must be given. If both are given to the constructor the Mount will be initialised immediately (unless
    auto_init=False is passed). Manually initialise with a call to Mount.initialize(); release hardware with a call to
    Mount.deinitialize().

    After the Mount is initialised, the gimbal angles and rates may be read and commanded. Several properties (e.g
    maximum angles and rates) may be set.

    Args:
        model (str, optional): The model used to determine the the hardware control interface. Supported: 'celestron'
            for Celestron NexStar and Orion/SkyWatcher SynScan (all the same) hand controller communication over serial.
        identity (str or int, optional): String or int identifying the device. For model *celestron* this can either be
            a string with the serial port (e.g. 'COM3' on Windows or '/dev/ttyUSB0' on Linux) or an int with the index
            in the list of available ports to use (e.g. identity=0 i if only one serial device is connected.)
        name (str, optional): Name for the device.
        auto_init (bool, optional): If both model and identity are given when creating the Mount and auto_init
            is True (the default), Mount.initialize() will be called after creation.
        debug_folder (pathlib.Path, optional): The folder for debug logging. If None (the default)
            the folder *pypogs*/logs will be used/created.

    Example:
        ::

            # Create instance (will auto initialise)
            mount = pypogs.Mount(model='celestron', identity='COM3', name='CPC800')
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

    _supported_models = ('celestron',)

    def __init__(self, model=None, identity=None, name=None, auto_init=True, debug_folder=None):
        """Create Mount instance. See class documentation."""
        # Logger setup
        self._debug_folder = None
        if debug_folder is None:
            self.debug_folder = Path(__file__).parent / 'debug'
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
        self._model = None
        self._identity = None
        self._name = 'UnnamedMount'
        self._is_init = False
        self._max_speed = (4.0, 4.0) #(alt,azi) degrees/sec
        self._alt_limit = (-5, 95) #limit degrees
        self._azi_limit = (None, None) #limit degees
        self._home_pos = (0, 0) #Home position
        self._alt_zero = 0 #Amount to subtract from alt.
        # Only used for model celestron
        self._cel_serial_port = None
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
        if model is not None and identity is not None:
            self.initialize()
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
            - 'celestron' for Celestron NexStar and Orion/SkyWatcher SynScan hand controllers over serial.

        - This will determine which hardware interface is used.
        - Must set before initialising the device and may not be changed for an initialised device.
        """
        return self._model
    @model.setter
    def model(self, model):
        self._logger.debug('Setting model to: '+str(model))
        assert not self.is_init, 'Can not change already initialised device model'
        model = str(model)
        assert model.lower() in self._supported_models,\
                                                'Model type not recognised, allowed: '+str(self._supported_models)
        self._model = model.lower()
        self._logger.debug('Model set to '+str(self.model))

    @property
    def identity(self):
        """str: Get or set the device and/or input. Model must be defined first.

        - For model *celestron* this can either be a string with the serial port (e.g. 'COM3' on Windows or
          '/dev/ttyUSB0' on Linux) or an int with the index in the list of available ports to use (e.g. identity=0 i if
          only one serial device is connected.)
        - Must set before initialising the device and may not be changed for an initialised device.

        Raises:
            AssertionError: if unable to connect to and verify identity of the mount.
        """
        return self._identity
    @identity.setter
    def identity(self, identity):
        self._logger.debug('Setting identity to: '+str(identity))
        assert not self.is_init, 'Can not change already initialised device'
        assert isinstance(identity, (str, int)), 'Identity must be a string or an int'
        assert self.model is not None, 'Must define model first'
        if self.model == 'celestron':
            if isinstance(identity, int):
                self._logger.debug('Got int instance, finding open ports')
                ports = self.list_available_ports()
                self._logger.debug('Found ports: '+str(ports))
                try:
                    identity = ports[identity][0]
                except IndexError:
                    self._logger.debug('Index error', exc_info=True)
                    raise AssertionError('No serial port for index: '+str(identity))
            self._logger.debug('Using celestron with string identity, try to open and check model')
            try:
                with serial.Serial(identity, 9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,\
                                   timeout=3.5, write_timeout=3.5) as ser:
                    ser.write(b'm')
                    ser.flush()
                    r = ser.read(2)
                    self._logger.debug('Got response: '+str(r))
                    assert len(r)==2 and r[1] == ord('#'), 'Did not get the expected response from the device'
            except serial.SerialException:
                self._logger.debug('Failed to open port', exc_info=True)
                raise AssertionError('Failed to open the serial port named: '+str(identity))
            self._identity = identity
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
        self._logger.debug('Identity set to: '+str(self.identity))

    @property
    def is_init(self):
        """bool: True if the device is initialised (and therefore ready to control)."""
        if not self.model: return False
        if self.model == 'celestron':
            return self._is_init
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def available_properties(self):
        """tuple of str: Get all the available properties (settings) supported by this device."""
        assert self.is_init, 'Mount must be initialised'
        if self.model.lower() == 'celestron':
            return ('zero_altitude', 'home_alt_az', 'max_rate', 'alt_limit', 'azi_limit')
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
        try:
            pos = tuple([float(x) for x in pos])
            assert len(pos)==2
        except TypeError:
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
        try:
            maxrate = tuple([float(x) for x in maxrate])
            assert len(maxrate)==2
        except TypeError:
            maxrate = (float(maxrate), float(maxrate))
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

    def initialize(self):
        """Initialise (make ready to start) the device. The model and identity must be defined."""
        self._logger.debug('Initialising')
        assert not self.is_init, 'Already initialised'
        assert not None in (self.model, self.identity), 'Must define model and identity before initialising'
        if self.model == 'celestron':
            self._logger.debug('Using Celestron, try to initialise')
            try:
                self._cel_serial_port = serial.Serial(self.identity, 9600, parity=serial.PARITY_NONE,\
                                                      stopbits=serial.STOPBITS_ONE, timeout=3.5, write_timeout=3.5)
            except serial.SerialException:
                self._logger.debug('Failed to open', exc_info=True)
                raise RuntimeError('Failed to connect to the mount during initialisation')
            self._logger.debug('Sending model check')
            self._cel_serial_port.write(b'm')
            self._cel_serial_port.flush()
            r = self._cel_serial_port.read(2)
            self._logger.debug('Got response: '+str(r))
            assert len(r)==2 and r[1] == ord('#'), 'Did not get the expected response during initialisation'
            self._logger.debug('Ensure sidreal tracking is off.')
            self._cel_tracking_off()
            self._is_init = True
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
        self._logger.info('Mount initialised.')
        try:
            self.get_alt_az() #Get cache to update
        except AssertionError:
            self._logger.debug('Failed to set state cache', exc_info=True)

    def deinitialize(self):
        """De-initialise the device and release hardware (serial port). Will stop the mount if it is moving."""
        self._logger.debug('De-initialising')
        assert self.is_init, 'Not initialised'
        try:
            self._logger.debug('Stopping mount')
            self.stop()
        except:
            self._logger.debug('Did not stop', exc_info=True)
        if self.model == 'celestron':
            self._logger.debug('Using celestron, closing and deleting serial port')
            self._cel_serial_port.close()
            self._cel_serial_port = None
            self._is_init = False
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
            self._logger.debug('Has active celestron control thread')
            return True
        if self.model == 'celestron':
            self._logger.debug('Using celestron, asking if moving')
            ret = [None]
            def _is_moving_to(ret):
                self._cel_send_text_command('L')
                ret[0] = self._cel_read_to_eol()
            t = Thread(target=_is_moving_to, args=(ret,))
            t.start()
            t.join()
            moving = not ret[0] == b'0'
            self._logger.debug('Mount returned: ' + str(ret[0]) + ', is moving: ' + str(moving))
            return moving
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def move_to_alt_az(self, alt, azi, block=True, rate_control=True):
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
        if self.model == 'celestron':
            self._logger.debug('Using celestron, ensure range -180 to 180')
#            alt = self.degrees_to_n180_180(alt - self._alt_zero)
            alt = self.degrees_to_n180_180(alt)
            azi = self.degrees_to_n180_180(azi)
            self._logger.debug('Will command: alt=' + str(alt) + ' azi=' + str(azi))
            if rate_control: #Use own control thread
                self._logger.debug('Starting rate controller')
                Kp = 1.5
                self._control_thread_stop = False
                success = [False]
                def _loop_slew_to(alt, azi, success):
                    while not self._control_thread_stop:
                        curr_pos = self.get_alt_az()
                        eAlt = Kp * self.degrees_to_n180_180(alt - curr_pos[0])
                        eAzi = Kp * self.degrees_to_n180_180(azi - curr_pos[1])
                        if eAlt < -self._max_speed[0]: eAlt = -self._max_speed[0]
                        if eAlt > self._max_speed[0]: eAlt = self._max_speed[0]
                        if eAzi < -self._max_speed[1]: eAzi = -self._max_speed[1]
                        if eAzi > self._max_speed[1]: eAzi = self._max_speed[1]

                        if abs(eAlt)<.001 and abs(eAzi)<.001:
                            self.set_rate_alt_az(0, 0)
                            success[0] = True
                            break
                        else:
                            self.set_rate_alt_az(eAlt, eAzi)
                    self._control_thread_stop = True
                self._control_thread = Thread(target=_loop_slew_to, args=(alt, azi, success))
                self._control_thread.start()
                if block:
                    self._logger.debug('Waiting for thread to finish')
                    self._control_thread.join()
                    assert success[0], 'Failed moving with rate controller'
            else:
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
                    self._cel_send_text_command(command)
                    assert self._cel_check_ack(), 'Mount did not acknowledge'
                    success[0] = True
                t = Thread(target=_move_to_alt_az, args=(alt, azi, success))
                t.start()
                t.join()
                assert success[0], 'Failed communicating with mount'
                self._logger.debug('Send successful')
                if block:
                    self._logger.debug('Waiting for mount to finish')
                    self.wait_for_move_to()
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def get_alt_az(self):
        """Get the current alt and azi angles of the mount.

        Returns:
            tuple of float: the (altitude, azimuth) angles of the mount in degrees (-180, 180].
        """
        assert self.is_init, 'Must be initialised'
        if self.model == 'celestron':
            self._logger.debug('Using celestron, requesting mount position')
            def _get_alt_az(ret):
                command = bytes([ord('z')]) #Get precise AZM-ALT
                self._cel_serial_port.write(command)
                # The command returns ASCII encoded text of HEX values!
                res = self._cel_read_to_eol().decode('ASCII')
                r2 = res.split(',')
                ret[0] = int(r2[1], 16)
                ret[1] = int(r2[0], 16)
            ret = [None, None]
            t = Thread(target=_get_alt_az, args=(ret,))
            t.start()
            t.join()
            alt = self.degrees_to_n180_180( float(ret[0]) / 2**32 * 360 + self._alt_zero)
            azi = self.degrees_to_n180_180( float(ret[1]) / 2**32 * 360 )
            self._logger.debug('Mount returned: alt=' + str(ret[0]) + ' azi=' + str(ret[1]) \
                               + ' => alt=' + str(alt) + ' azi=' + str(azi))
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
        if (abs(alt) > self._max_speed[0]) or  (abs(azi) > self._max_speed[0]):
            raise ValueError('Above maximum speed!')
        if self.model == 'celestron':
            self._logger.debug('Using celestron, sending rate command to mount')
            success = [False]
            def _set_rate_alt_az(alt, azi, success):
                #Altitude
                rate = int(round(alt*3600*4))
                if rate >= 0:
                    rateLo = rate & 0xFF
                    rateHi = rate>>8 & 0xFF
                    self._cel_send_bytes_command([ord('P'),3,17,6,rateHi,rateLo,0,0])
                else:
                    rateLo = -rate & 0xFF
                    rateHi = -rate>>8 & 0xFF
                    self._cel_send_bytes_command([ord('P'),3,17,7,rateHi,rateLo,0,0])
                assert self._cel_check_ack(), 'Mount did not acknowledge!'
                #Azimuth
                rate = int(round(azi*3600*4))
                if rate >= 0:
                    rateLo = rate & 0xFF
                    rateHi = rate>>8 & 0xFF
                    self._cel_send_bytes_command([ord('P'),3,16,6,rateHi,rateLo,0,0])
                else:
                    rateLo = -rate & 0xFF
                    rateHi = -rate>>8 & 0xFF
                    self._cel_send_bytes_command([ord('P'),3,16,7,rateHi,rateLo,0,0])
                assert self._cel_check_ack(), 'Mount did not acknowledge!'
                success[0] = True
            t = Thread(target=_set_rate_alt_az, args=(alt, azi, success))
            t.start()
            t.join()
            assert success[0], 'Failed communicating with mount'
            self._logger.debug('Send successful')
            self._state_cache['alt_rate'] = alt
            self._state_cache['azi_rate'] = azi
        else:
            self._logger.warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def stop(self):
        """Stop moving."""
        assert self.is_init, 'Must be initialised'
        self._logger.debug('Got stop command, check thread')
        if self._control_thread is not None and self._control_thread.is_alive():
            self._logger.debug('Stopping celestron control thread')
            self._control_thread_stop = True
            self._control_thread.join()
            self._logger.debug('Stopped')
        self._logger.debug('Sending zero rate command')
        self.set_rate_alt_az(0, 0)
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

    def _cel_tracking_off(self):
        """PRIVATE: Disable sidreal tracking on celestron mount."""
        success = [False]
        def _set_tracking_off(success):
            self._cel_send_bytes_command([ord('T'),0])
            assert self._cel_check_ack(), 'Mount did not acknowledge!'
            success[0] = True
        t = Thread(target=_set_tracking_off, args=(success,))
        t.start()
        t.join()
        assert success[0], 'Failed communicating with mount'

    def _cel_send_text_command(self,command):
        """PRIVATE: Encode and send str to mount."""
        # Given command as type 'str', send to mount as ASCII text
        self._cel_serial_port.write(command.encode('ASCII'))
        self._cel_serial_port.flush() #Push out data

    def _cel_send_bytes_command(self,command):
        """PRIVATE: Send bytes to mount."""
        # Given command as list of integers, send to mount as bytes
        self._cel_serial_port.write(bytes(command))
        self._cel_serial_port.flush() #Push out data

    def _cel_check_ack(self):
        """PRIVATE: Read one byte and check that it is the ack #."""
        # Checks if '#' is returned as expected
        b = self._cel_serial_port.read()
        return int.from_bytes(b, 'big') == ord('#')

    def _cel_read_to_eol(self):
        """PRIVATE: Read response to the EOL # character. Return bytes."""
        # Read from mount until EOL # character. Return as type 'bytes'
        response = b'' #Empty type 'bytes'
        while True:
            r = self._cel_serial_port.read()
            if r == b'': #If we didn't get anything/timeout
                raise RuntimeError('No response from mount!')
            else:
                if int.from_bytes(r, 'big') == ord('#'):
                    self._logger.debug('Read from mount: '+str(response))
                    return response
                else:
                    response += r


class Receiver:
    """Control acquisition and read received power from a photodetector.

    To initialise a Receiver a *model* (determines hardware interface) and *identity* (identifying the specific device)
    must be given. If both are given to the constructor the Receiver will be initialised immediately (unless
    auto_init=False is passed). Manually initialise with a call to Receiver.initialize(); release hardware with a call
    to Receiver.deinitialize().

    The raw data can be saved to a file by specifying data_folder (filenames are auto-generated). While the acquisition
    is running the instantaneous (last measurement) and (exponentially) smoothed power can be read.

    Args:
        model (str, optional): The model used to determine the correct hardware API. Supported: 'ni_daq' for
            National Instruments DAQ cards (tested on USB-6211).
        identity (str, optional): String identifying the device and input. For *ni_daq* this is 'device/input' eg.
            'Dev1/ai1' for device 'Dev1' and analog input 1; only differential input is supported for *ni_daq*.
        name (str, optional): Name for the device.
        auto_init (bool, optional): If both model and identity are given when creating the Receiver and auto_init
            is True (the default), Receiver.initialize() will be called after creation.
        data_folder (pathlib.Path, optional): The folder for data saving. If None (the default) no data will be saved.
        debug_folder (pathlib.Path, optional): The folder for debug logging. If None (the default) the folder
            *pypogs*/debug will be used/created.

    Example:
        ::

            # Create instance and set parameters (will auto initialise)
            rec = pypogs.Receiver(model='ni_daq', identity='Dev1/ai1', name='PhotoDiode')
            rec.sample_rate = 1000 #Samples per second
            rec.smoothing_parameter = 100 #number of samples to smooth over
            rec.measurement_range = (-10, 10) #Volts for ni_daq
            # Add a save path (filenames are auto-generated)
            rec.data_folder = pathlib.Path('./datafolder')
            # Start acquisition
            rec.start()
            # Wait for a while
            time.sleep(2)
            # Read the smooth and instantaneous powers
            print('Smoothed power is: ' + str(rec.smooth_power))
            print('Instant power is: ' + str(rec.instant_power))
            # Stop the acquisition
            rec.stop()

    """
    _supported_models = ('ni_daq',)

    def __init__(self, model=None, identity=None, name=None, auto_init=True, data_folder=None,\
                 debug_folder=None):
        """Create Receiver instance. See class documentation."""
        # Logger setup
        self._debug_folder = None
        if debug_folder is None:
            self.debug_folder = Path(__file__).parent / 'debug'
        else:
            self.debug_folder = debug_folder
        self._logger = logging.getLogger('pypogs.hardware.Receiver')
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
        self._logger.debug('Receiver called with: model=' + str(model) + ' identity=' + str(identity) + ' name=' \
                           +str(name) + ' auto_init=' + str(auto_init) + ' data_folder=' + str(data_folder))
        self._is_init = False
        self._is_running = False
        self._model = None
        self._identity = None
        self._name = 'UnnamedReceiver'
        self._data_folder = None
        self._data_file = None
        # Power values stored from the receiver
        self._instant_power = None
        self._smooth_power = None
        self._smoothing_parameter = 100
        #Only used for NI DAQ devices
        self._ni_task = None
        if model is not None:
            self.model = model
        if identity is not None:
            self.identity = identity
        if name is not None:
            self.name = name
        if data_folder is not None:
            self.data_folder = data_folder
        self._logger.info('Instance created'+(': '+self.name) if self.name is not None else '')
        if auto_init and not None in (model, identity):
            self.initialize()
        import atexit, weakref
        atexit.register(weakref.ref(self.__del__))
        self._logger.info('Receiver instance created with name: ' + self.name + '.')

    def __del__(self):
        """Receiver destructor. Will close connections to device before destruction."""
        try:
            self._logger.debug('Deleter called')
        except:
            pass
        try:
            self.deinitialize()
            try:
                self._logger.debug('Deinitialised')
            except:
                pass
        except:
            self._logger.debug('Did not deinitialise', exc_info=True)
        try:
            self._logger.debug('Instance deleted')
        except:
            pass

    @property
    def data_folder(self):
        """pathlib.Path: Get or set the path for data saving. Will create folder if not existing."""
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
    def name(self):
        '''str: Get or set the name.'''
        return self._name
    @name.setter
    def name(self, name):
        self._logger.debug('Setting name to: '+str(name))
        assert isinstance(name, str), 'Name must be a string'
        self._name = name
        self._logger.debug('Name set')

    @property
    def model(self):
        """str: Get or set the device model.

        Supported:
            - 'ni_daq' for National Instruments DAQ devices (e.g. USB-6211).

        - This will determine which hardware API that is used.
        - Must set before initialising the device and may not be changed for an initialised device.
        """
        return self._model
    @model.setter
    def model(self, model):
        self._logger.debug('Setting model to: '+str(model))
        assert not self.is_init, 'Can not change already initialised device model'
        assert isinstance(model, str), 'Model type must be a string'
        assert model.lower() in self._supported_models,\
                                                'Model type not recognised, allowed: '+str(self._supported_models)
        self._model = model
        self._logger.debug('Model set')

    @property
    def identity(self):
        """str: Get or set the device and/or input. Model must be defined first.


        - For model *ni_daq* this is 'device/input' eg. 'Dev1/ai1' for device 'Dev1' and analog input 1. Only
          differential input is supported for NI DAQ.
        - Must set before initialising the device and may not be changed for an initialised device.
        """
        return self._identity
    @identity.setter
    def identity(self, identity):
        self._logger.debug('Setting identity to: '+str(identity))
        assert not self.is_init, 'Can not change already initialised device'
        assert isinstance(identity, str), 'Identity must be a string'
        assert self.model is not None, 'Must define model first'
        if self.model.lower() == 'ni_daq':
            self._logger.debug('Using NI DAQ, checking validity by opening a task')
            import nidaqmx as ni
            t = ni.Task()
            try:
                t.ai_channels.add_ai_voltage_chan(identity)
                self._identity = identity
            except ni.DaqError:
                self._logger.debug('Verification unsucessful', exc_info=True)
                raise AssertionError('The identity was not found')
            finally:
                try:
                    self._logger.debug('Deleting task')
                    t.close()
                    del(t)
                except:
                    self._logger.debug('Failed to delete task used to test identity', exc_info=True)
        else:
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

        self._logger.debug('Identity set')

    @property
    def is_init(self):
        """bool: True if the device is initialised (and therefore ready to start)."""
        return self._is_init

    def initialize(self):
        """Initialise (make ready to start) the device. The model and identity must be defined."""
        self._logger.debug('Initialising')
        assert not self.is_init, 'Already initialised'
        assert not None in (self.model, self.identity), 'Must define model and identity before initialising'
        if self.model.lower() == 'ni_daq':
            import nidaqmx as ni
            self._logger.debug('Using NI DAQ, create a task')
            try:
                self._ni_task = ni.Task(self.name) if self.name is not None else ni.Task()
            except ni.DaqError:
                self._logger.debug('Failed to create task', exc_info=True)
                raise RuntimeError('Failed to initialise, may conflict with existing instance')
            try:
                self._ni_task.ai_channels.add_ai_voltage_chan(self.identity)
                self._ni_task.timing.cfg_samp_clk_timing(rate=1000, sample_mode=ni.constants.AcquisitionType.CONTINUOUS)
                self._logger.info('Successfully initialised'+(': '+self.name) if self.name is not None else '')
                self._is_init = True
            except ni.DaqError:
                self._logger.debug('Failed to initialise', exc_info=True)
                try:
                    self._ni_task.close()
                    self._ni_task = None
                    self._logger.debug('Closed the task')
                except:
                    self._logger.debug('Failed to close task', exc_info=True)
                raise RuntimeError('Failed to initialise, may conflict with existing instance')
        else:
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def deinitialize(self):
        """De-initialise the device. Will stop the acquisition if it is running."""
        self._logger.debug('De-initialising')
        assert self.is_init, 'Not initialised'
        if self.is_running:
            self._logger.debug('Is running, stopping')
            self.stop()
            self._logger.debug('Stopped')
        if self._ni_task is not None:
            self._logger.debug('Found NI DAQ task, closing and removing')
            try:
                self._ni_task.close()
                self._ni_task = None
                self._logger.debug('Closed task')
            except:
                self._logger.exception('Failed to close task')
            self._is_init = False
        else:
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def available_properties(self):
        """tuple of str: Get all the available properties (settings) supported by this device."""
        assert self.is_init, 'Must be initialised'
        if self.model.lower() == 'ni_daq':
            return ('sample_rate', 'measurement_range', 'smoothing_parameter')
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def sample_rate(self):
        """int or float: Get or set the sample rate (in Hz) of the device. Must initialise the device first."""
        assert self.is_init, 'Must initialise first'
        self._logger.debug('Getting sample rate')
        if self.model.lower() == 'ni_daq':
            self._logger.debug('Using NI DAQ, trying to get')
            return self._ni_task.timing.samp_clk_rate
        else:
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @sample_rate.setter
    def sample_rate(self, rate_hz):
        self._logger.debug('Setting sample rate to (Hz): '+str(rate_hz))
        assert isinstance(rate_hz, (float,int)), 'Sample rate must be a scalar (float or int)'
        assert self.is_init, 'Must initialise first'
        assert not self.is_running, 'Cant change rate while running'
        if self.model.lower() == 'ni_daq':
            import nidaqmx as ni
            self._logger.debug('Using NI DAQ, trying to set')
            self._logger.debug('Checking valid rate')
            assert 0 < rate_hz <= self._ni_task.timing.samp_clk_max_rate, 'Requested rate is not allowed, '\
                +'maximum rate is: '+str(self._ni_task.timing.samp_clk_max_rate)
            try:
                self._ni_task.timing.cfg_samp_clk_timing(rate=rate_hz,\
                                                         sample_mode=ni.constants.AcquisitionType.CONTINUOUS)
                self._logger.debug('Sampling rate set to: '+str(self._ni_task.timing.samp_clk_rate))
            except:
                self._logger.exception('Failed to set sample rate: ')
                raise
        else:
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def measurement_range(self):
        """tuple, int or float: Get or set the measurement range (lower_limit, upper_limit).

        - If given as a scalar the range will be set to +- the supplied value.
        """
        assert self.is_init, 'Must initialise first'
        self._logger.debug('Getting measurement range')
        if self.model.lower() == 'ni_daq':
            self._logger.debug('Using NI DAQ, trying to get')
            try:
                maxval = self._ni_task.ai_channels[0].ai_max
                minval = self._ni_task.ai_channels[0].ai_min
                return (minval, maxval)
            except:
                self._logger.exception('Failed to get range')
        else:
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @measurement_range.setter
    def measurement_range(self, meas_range):
        assert self.is_init, 'Must initialise first'
        assert not self.is_running, 'Cant change range while running'
        self._logger.debug('Setting measurement range to: '+str(meas_range))
        assert isinstance(meas_range,(int,float,tuple)), 'Input must be scalar (int/float) or a 2-tuple of scalars'
        if isinstance(meas_range,tuple):
            assert len(meas_range) == 2, 'Input must be scalar (int or float) or a 2-tuple of scalars'
            assert all( isinstance(x,(int,float)) for x in meas_range ),\
                                         'Input must be scalar (int or float) or a 2-tuple of scalars'
        else:
            meas_range = (-meas_range, meas_range)
        self._logger.debug('Decoded input: '+str(meas_range))
        if self.model.lower() == 'ni_daq':
            import nidaqmx as ni
            self._logger.debug('Using NI DAQ, trying to set')
            self._logger.debug('NOTE: nidaqmx is broken, must manually check if range is allowed (-10, 10)...')
            assert min(meas_range)>=-10 and max(meas_range)<=10, 'Values must be <=10 and >=-10'
            self._logger.debug('NOTE: Passed manual value check')
            try:
                self._logger.debug('Setting new values')
                self._ni_task.ai_channels[0].ai_max = meas_range[1]
                self._ni_task.ai_channels[0].ai_min = meas_range[0]
                self._logger.debug('Range set to: '+str(self.measurement_range))
            except ni.DaqError:
                self._logger.exception('Failed to set new values, this may cause strange behaviour. De-init and re-init.')
        else:
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def smoothing_parameter(self):
        """int or float: Get or set the smoothing parameter. It roughly corresponds to the number of samples to average.

        - Exponential smoothing is used. smoothing_parameter is the *inverse* of 'alpha'. Each smoothed value s is
          defined from the measurements x by:

              ``s[n] = alpha*x[n] + (1-alpha)*s[n-1]; s[0] = x[0]``
        """
        return self._smoothing_parameter
    @smoothing_parameter.setter
    def smoothing_parameter(self, param):
        assert isinstance(param, (int, float)), 'Parameter must be scalar (float or int)'
        assert param > 0, 'Parameter must be >0'
        self._smoothing_parameter = param
        self._logger.debug('Smoothing parameter set to '+str(param))

    @property
    def instant_power(self):
        """float: Get the latest raw measurement."""
        if not self.is_running: return None
        self._get_update_from_hardware()
        return self._instant_power

    @property
    def smooth_power(self):
        """float: Get the current smoothed measurement (see smoothing_parameter)."""
        if not self.is_running: return None
        self._get_update_from_hardware()
        return self._smooth_power

    @property
    def is_running(self):
        """bool: True if device is currently acquiring data."""
        return self._is_running

    def start(self):
        """Start the acquisition. Device must be initialised. Data will only be saved if data_folder is set."""
        assert self.is_init, 'Must initialise first'
        assert not self.is_running, 'Acquisition already running'
        self._logger.debug('Got start command')
        if self.data_folder is not None:
            self._logger.debug('Data folder exists, creating file and header')
            self._create_data_file()
        else:
            self._logger.debug('No save path set')
        if self.model.lower() == 'ni_daq':
            import nidaqmx as ni
            self._logger.debug('Using NI DAQ, setting up callback')
            cb_count = int(min(1000, max(1, self.sample_rate/10))) #Typically 10Hz, min 1 and max 1000 per callback

            def _ni_buffering_callback(task_handle, event_type, number_of_samples, callback_data):
                self._logger.debug('Got a callback')
                try:
                    self._get_update_from_hardware()
                except:
                    logging.error('Could not update from hardware')
                return 0

            try:
                self._ni_task.register_every_n_samples_acquired_into_buffer_event(cb_count, _ni_buffering_callback)
                self._logger.debug('Registered event every n='+str(cb_count)+' samples')
            except ni.DaqError:
                self._logger.debug('Unable to register event, trying to unregister and try again')
                self._ni_task.register_every_n_samples_acquired_into_buffer_event(cb_count, None)
                self._ni_task.register_every_n_samples_acquired_into_buffer_event(cb_count, _ni_buffering_callback)
                self._logger.debug('Registered event every n='+str(cb_count)+' samples')
            self._ni_task.start()
            self._logger.info('Started acquisition from receiver')
            self._is_running = True
        else:
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def stop(self):
        """Stop the acquisition. Will ensure all data in the buffer is read before stopping."""
        assert self.is_running, 'Acquisition is not running'
        self._logger.debug('Got stop command')
        if self.model.lower() == 'ni_daq':
            self._logger.debug('Using NI DAQ, stopping task')
            self._is_running = False
            self._ni_task.stop()
            self._logger.debug('Stopped task')
            self._logger.info('Stopped acquisition from receiver')
        else:
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def _get_update_from_hardware(self):
        """PRIVATE: Read all available data from the device and call _update_stored_values. Save if data_folder is set."""
        self._logger.debug('Got hardware update command')
        if self.model.lower() == 'ni_daq':
            import nidaqmx as ni
            self._logger.debug('Using NI DAQ, reading all available')
            try:
                data = self._ni_task.read(ni.constants.READ_ALL_AVAILABLE)
                self._logger.debug('Data of length '+str(len(data))+' and class '+str(type(data)))
            except:
                data = None
                if self.is_running:
                    self._logger.exception('Failed to read data')
                else:
                    self._loger.debug('Got a callback after stop command')
            if data:
                try:
                    self._update_stored_values(data)
                except:
                    self._logger.exception('Failed to update stored values')
                if self.data_folder is not None:
                    try:
                        self._write_data_to_data_file(data)
                    except:
                        self._logger.exception('Failed to save update')
        else:
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def _update_stored_values(self, data):
        """PRIVATE: Update the stored instantaneous and smoothed measurement."""
        self._logger.debug('Got update request with length '+str(len(data)))
        if None in (self._smooth_power, self._instant_power): #No old data, need to initialise
            self._logger.debug('No previous values')
            self._instant_power = data[-1] #Last read data saved here
            if len(data) == 1: #If only one point
                self._smooth_power = data[0]
            else:
                k = len(data)
                a = 1/self._smoothing_parameter
                data = np.array(data)
                facs = (1-a)**np.arange(k) #Smoothing factors
                self._smooth_power =      a*np.sum( facs[:-1]*data[:-1] ) + facs[-1]*data[-1]
        else: #Doing a normal update
            self._logger.debug('Previous smooth and instant powers are: '\
                                                 +str(self._smooth_power)+' '+str(self._instant_power))
            self._instant_power = data[-1] #Last read data saved here
            k = len(data)
            a = 1/self._smoothing_parameter
            if k == 1: #If only one point
                self._smooth_power = a*data[0] + (1-a)*self._smooth_power
            else:
                data = np.array(data)
                facs = (1-a)**np.arange(k+1) #Smoothing factors
                self._smooth_power = a*np.sum( facs[:-1]*data ) + facs[-1]*self._smooth_power

        self._logger.debug('Smooth and instant power are now: '+str(self._smooth_power)+' '+str(self._instant_power))

    def _write_data_to_data_file(self, data):
        """PRIVATE: Write data to the data file."""
        self._logger.debug('Writing to data file, got '+str(len(data))+' measurements')
        assert self._data_file is not None, 'No logfile is defined...'
        with open(self._data_file, 'ba') as file:
            dpack = pack_data('%df' % len(data), *data) #Create binary (dobule) representation of data
            file.write(dpack)

    def _create_data_file(self):
        """"PRIVATE: Create data file and write the header."""
        assert self.data_folder is not None, 'No save path here...'
        self._logger.debug('Creating data file')
        timestamp = datetime.utcnow()
        filename = timestamp.strftime('%Y-%m-%dT%H%M%S') + '_Receiver.dat'
        self._data_file = self.data_folder / Path(filename)
        self._logger.debug('File: ' + str(filename))
        header = 'TIME: ' + timestamp.isoformat() + '; ' \
                +'NAME: ' + str(self.name) + '; ' \
                +'MODEL: ' + str(self.model) + '; ' \
                +'IDENTITY: ' + str(self.identity) + '; ' \
                +'SAMPLE_RATE: ' + str(self.sample_rate) + '; ' \
                +'MEASUREMENT_RANGE: ' + str(self.measurement_range) +'; ' \
                +'FORMAT: ' + 'STRUCT_PACK_FLOAT32' + ';\n'
        self._logger.debug('Header: ' + header)
        with open(self._data_file, 'a') as file:
            file.write(header)