"""Camera interfaces
======================

Current harware support:
    - :class:`pypogs.Camera`: 'ptgrey' for FLIR (formerly Point Grey) machine vision cameras. Requires Spinnaker API and PySpin, see the
      installation instructions. Tested with Blackfly S USB3 model BFS-U3-31S4M.

    - :class:`pypogs.Camera`: 'ascom' for ASCOM-enabled cameras.  Requires ASCOM platform, ASCOM drivers, and native drivers.
      Tested with FIXME.
      
      
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
from struct import pack as pack_data

# External imports:
import numpy as np
import serial


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
    _supported_models = ('ptgrey','ascom')

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
        #Only used for ascom
        self._ascom_camera = None
        self._exposure_sec = 0.1
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
        if auto_init and model is not None:
            self._logger.debug('Trying to auto-initialise')
            self.initialize()
        else:
            self._logger.debug('Skipping auto-initialise')
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
                self._log_debug('Is initialized, de-initalizing')
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
        
    def _ascom_release(self):
        """PRIVATE: Release ASCOM hardware resources."""
        self._log_debug('ASCOM camera release called')
        if self._ascom_camera is not None:
            if self._ascom_camera.Connected:
                if self._ascom_camera.CanAbortExposure:
                    self._ascom_camera.AbortExposure()
                self._ascom_camera.Connected = False
            self._log_debug('ASCOM camera disconnected')
            del(self._ascom_camera)
        self._log_debug('ASCOM camera hardware released')

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
            - 'ascom'  for ASCOM-enabled cameras.

        - This will determine which hardware API that is used.
        - Must set before initialising the device and may not be changed for an initialised device.
        
        """
        return self._model
    @model.setter
    def model(self, model):
        self._log_debug('Setting model to: '+str(model))
        assert not self.is_init, 'Cannot change already intialised device model'
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
        
        - For model *ascom*, a driver name may be specified if known, (i.e. DSLR, ASICamera1, ASICamera2, 
        QHYCCD, QHYCCD_GUIDER, QHYCCD_CAM2, AtikCameras, AtikCameras2, etc), otherwise the ASCOM driver
        selector will open.
        
        """
        return self._identity
    @identity.setter
    def identity(self, identity):
        self._log_debug('Setting identity to: '+str(identity))
        assert not self.is_init, 'Cannot change already intialised device'
        assert not None in (self.model, identity), 'Must define camera model and identity first'
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
                
        elif self.model.lower() == 'ascom':
            self._log_debug('Checking ASCOM camera identity and availability')
            assert identity is not None, 'ASCOM camera identity not resolved'
            if not identity.startswith('ASCOM'):
                identity = 'ASCOM.'+identity+'.camera'
            #self._log_debug('Loading ASCOM camera driver: '+str(identity))
            #try:
            #    ascom_camera = self._ascom_driver_handler.Dispatch(identity)
            #except:
            #    raise RuntimeError('Failed to load camera driver') 
            #if not ascom_camera or not hasattr(ascom_camera, 'Connected'):
            #    raise RuntimeError('Failed to load camera driver (2)')
            #if self._ascom_camera.Connected:
            #    self._log_debug("Camera was already connected")
            #    raise RuntimeError('The camera is already in use')
            #else:
            #    self._log_debug("Camera available. Setting identity.")
            self._identity = identity
            #ascom_camera = None

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
        elif self.model.lower() == 'ascom':
            init = hasattr(self,'_ascom_camera') and self._ascom_camera is not None and self._ascom_camera.Connected
            return init
            #FIXME: bypassed this because camera preserves .Connected state when GUI restarts without disconnecting
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def initialize(self):
        """Initialise (make ready to start) the device. The model and identity must be defined."""
        self._log_debug('Initialising')
        assert not self.is_init, 'Already initialised'
        assert not None in (self.model, ), 'Must define model before initialising'            
        if self.model.lower() == 'ptgrey':
            assert self.identity is not None, 'Must define identity of Pt Grey camera before initialising'            
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
            class PtGreyEventHandler(PySpin.ImageEvent):
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
            self._ptgrey_camera.RegisterEvent( self._ptgrey_event_handler )
            self._log_debug('Registered ptgrey image event handler')
            self._log_info('Camera successfully initialised')
            
        elif self.model.lower() == "ascom":
            if self._ascom_camera is not None:
                raise RuntimeError('There is already an ASCOM camera object here')
            self._log_debug('Attempting to connect to ASCOM device "'+str(self.identity)+'"')
            if not hasattr(self, '_ascom_driver_handler'):
                import win32com.client
                self._ascom_driver_handler = win32com.client
            camDriverName = str()
            if self.identity is not None:
                self._log_debug('Specified identity: "'+str(self.identity)+'" ['+str(len(self.identity))+']')
                if self.identity.startswith('ASCOM'):
                    camDriverName = self.identity
                else:
                    camDriverName = 'ASCOM.'+str(self.identity)+'.camera'
            else:
                ascomSelector = self._ascom_driver_handler.Dispatch("ASCOM.Utilities.Chooser")
                ascomSelector.DeviceType = 'Camera'
                camDriverName = ascomSelector.Choose('None')
                self._log_debug("Selected camera driver: "+camDriverName)
                if not camDriverName:            
                    self._log_debug('User canceled camera selection')
            assert camDriverName, 'Unable to identify ASCOM camera.'
            self._log_debug('Loading ASCOM camera driver: '+camDriverName)
            self._ascom_camera = self._ascom_driver_handler.Dispatch(camDriverName)
            assert hasattr(self._ascom_camera, 'Connected'), "Unable to access camera driver"
            self._log_debug('Connecting to camera')
            self._ascom_camera.Connected = True
            assert self._ascom_camera.Connected, "Failed to connect to camera"
            #self.identity = camDriverName
            assert self._ascom_camera is not None, 'ASCOM camera not initialized'
            
            class AscomCameraImagingLoopHandler():
                def __init__(self, parent):
                    super().__init__()
                    self.parent = parent
                    self._is_running = False
                    
                def start_imaging_loop(self):
                    assert self.parent._ascom_camera, 'Cannot start imaging - camera not initialized'
                    self.parent._log_debug('Starting ASCOM camera imaging loop')
                    self.continue_imaging = True
                    self.imagethread = Thread(target=self.imaging_loop)
                    self.imagethread.start()
                    
                def stop_imaging_loop(self):
                    self.parent._log_debug('Stopping ASCOM camera imaging loop')
                    self.continue_imaging = False
                    if self._is_running:
                        self.parent._log_debug('Waiting ASCOM camera imaging loop')
                        polling_period_sec = 0.05
                        while self._is_running:
                            sleep(polling_period_sec)
                        self.parent._log_debug('Stopped ASCOM camera imaging loop')
            
                def imaging_loop(self):
                    assert self.parent._ascom_camera, 'Cannot start imaging - ASCOM camera driver not loaded'
                    assert self.parent._ascom_camera.Connected, 'Cannot start imaging - ASCOM camera not connected'                        
                    self._is_running = True
                    self.parent._log_debug('Starting ASCOM camera imaging loop')
                    while self.continue_imaging and self.parent._ascom_camera.Connected:
                        #self.parent._log_debug('Starting ASCOM camera exposure')
                        self.parent._ascom_camera.StartExposure(self.parent._exposure_sec,True)
                        waited_time = 0
                        timeout = self.parent._exposure_sec + 0.5
                        polling_period_sec = 0.05
                        while not self.parent._ascom_camera.ImageReady and not waited_time >= timeout:
                            waited_time += polling_period_sec
                            sleep(polling_period_sec)
                        if not self.parent._ascom_camera.ImageReady:
                            self.parent._log_debug('Timed out waiting for image')
                            self.parent._log_debug('Camera connected: '+str(self.parent._ascom_camera.Connected))
                        else:
                            self.parent._image_timestamp = datetime.utcnow()
                            try:
                                img = np.asarray(self.parent._ascom_camera.ImageArray, dtype=np.uint8).T;
                                if self.parent._flipX:
                                    img = np.fliplr(img)
                                if self.parent._flipY:
                                    img = np.flipud(img)
                                if self.parent._rot90:
                                    img = np.rot90(img, self.parent._rot90)
                                self.parent._image_data = img
                            except:
                                self.parent._log_debug('Failed to access image.')
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
                            #self.parent._log_debug('ASCOM image loop finished.')
                        if self.continue_imaging:
                            sleep(0.01)
                    self._is_running = False
                    
            self._ascom_camera_imaging_handler = AscomCameraImagingLoopHandler(self)
            self._ascom_camera_imaging_handler.start_imaging_loop()
            
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    def deinitialize(self):
        """De-initialise the device and release hardware resources. Will stop the acquisition if it is running."""
        self._log_debug('De-initialising')
        #assert self.is_init, 'Not initialised'
        if self.is_running:
            self._log_debug('Is running, stopping')
            self.stop()
            self._log_debug('Stopped')
        if self._ptgrey_camera:
            self._log_debug('Found PtGrey camera, deinitialising')
            try:
                self._ptgrey_camera.UnregisterEvent(self._ptgrey_event_handler)
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
        elif self._ascom_camera:
            self._log_debug('Deinitialising ASCOM camera')
            self.stop()
            self._log_debug('Deinitialised ASCOM camera')
            self._ascom_release()
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
        elif self.model.lower() == 'ascom':
            return ('flip_x', 'flip_y', 'rotate_90', 'plate_scale', 'rotation', 'binning', 'size_readout',\
                     'gain', 'exposure_time')    # FIXME             
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
        elif self.model.lower() == 'ascom':
            self._log_debug('Using ASCOM camera. Will flip the received image array ourselves: ' +str(self._flipX))
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
        elif self.model.lower() == 'ascom':
            self._log_debug('Using ASCOM camera. Will flip the received image array ourselves.')
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
            self._log_debug('Using PtGrey camera. Will flip the received image array ourselves: ' +str(self._flipX))
            return self._flipY
        elif self.model.lower() == 'ascom':
            self._log_debug('Using ASCOM camera. Will flip the received image array ourselves: ' +str(self._flipX))
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
        elif self.model.lower() == 'ascom':
            self._log_debug('Using ASCOM camera. Will flip the received image array ourselves.')
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
        if self.model.lower() == 'ptgrey':
            return self._rot90
        elif self.model.lower() == 'ascom':
            return self._rot90
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @rotate_90.setter
    def rotate_90(self, k):
        self._log_debug('Set rot90 called with: '+str(k))
        assert self.is_init, 'Camera must be initialised'
        k = int(k)
        if self.model.lower() == 'ptgrey':
            self._log_debug('Using PtGrey camera. Will rotate the received image array ourselves.')
            self._rot90 = k
            self._log_debug('rot90 set to: '+str(self._rot90))
        elif self.model.lower() == 'ascom':
            self._log_debug('Using ASCOM camera. Will rotate the received image array ourselves.')
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
        elif self.model.lower() == 'ascom':
            self._log_debug('frame_rate_auto not supported in ASCOM')
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
        elif self.model.lower() == 'ascom':
            self._log_debug('frame_rate_auto not supported in ASCOM')
            return False
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
        elif self.model.lower() == 'ascom':
            self._log_debug('frame_rate_auto not supported in ASCOM')
            return False
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
        elif self.model.lower() == 'ascom':
            self._log_debug('frame rate not supported in ASCOM')
            return 0
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
        elif self.model.lower() == 'ascom':
            self._log_debug('auto gain not implemented in ASCOM')
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
        elif self.model.lower() == 'ascom':
            self._log_debug('auto gain not implemented in ASCOM')
            return False
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
        elif self.model.lower() == 'ascom':
            self._log_debug('gain setting not yet implemented in ASCOM')
            return 0
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
        elif self.model.lower() == 'ascom':
            self._log_debug('gain setting not yet implemented in ASCOM')
            return 0
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
        elif self.model.lower() == 'ascom':
            self._log_debug('gain setting not yet implemented in ASCOM')
            return 0
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
        elif self.model.lower() == 'ascom':
            self._log_debug('auto exposure not implemented in ASCOM')
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
        elif self.model.lower() == 'ascom':
            self._log_debug('auto exposure not implemented in ASCOM')
            return 0
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
        elif self.model.lower() == 'ascom':
            return self._ascom_camera.ExposureMin, self._ascom_camera.ExposureMax
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
        elif self.model.lower() == 'ascom':
            try:
                return self._ascom_camera.LastExposureDuration
            except:
                return 0
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
        elif self.model.lower() == 'ascom':
            exposure_sec = exposure_ms/1000
            if exposure_sec < self._ascom_camera.ExposureMin or exposure_sec > self._ascom_camera.ExposureMax:
                self._log_debug('Exposure time out of allowable range ('+str(self._ascom_camera.ExposureMin)+':'+str(self._ascom_camera.ExposureMax))
                raise AssertionError('Requested exposure time ['+str(exposure_sec)+'] out of allowable range.')                
            self._exposure_sec = exposure_ms/1000
            
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
        elif self.model.lower() == 'ascom':
            binMax = self._ascom_camera.MaxBinX
            if binMax:
                return binMax
            else:
                self._log_debug('Error reading camera bin value')            
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
        elif self.model.lower() == 'ascom':
            binMax = self._ascom_camera.MaxBinX
            if binMax and binning <= binMax:
                try:
                    print("setting binning to ",binning)
                    self._ascom_camera.BinX = binning
                    self._ascom_camera.BinY = binning
                    self._ascom_camera.NumX = self._ascom_camera.CameraXSize/binning
                    self._ascom_camera.NumY = self._ascom_camera.CameraYSize/binning
                    print(self._ascom_camera.BinX, binning)
                except:
                    raise AssertionError('Unable to set camera binning')
            else:
                raise ValueError('exceeds camera max bin val ',binMax)
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
        elif self.model.lower() == 'ascom':
            try:
                val_w = self._ascom_camera.CameraXSize
                val_h = self._ascom_camera.CameraYSize
                return (val_w, val_h)
            except:
                self._log_debug('Unable to read ASCOM camera max image dimensions', exc_info=True)
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
        elif self.model.lower() == 'ascom':
            try:
                val_w = self._ascom_camera.NumX
                val_h = self._ascom_camera.NumY
                return (val_w, val_h)
            except:
                self._log_debug('Unable to read ASCOM camera image dimensions', exc_info=True)
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
        elif self.model.lower() == 'ascom':
            try:
                max_w = self._ascom_camera.CameraXSize
                max_h = self._ascom_camera.CameraYSize
                binning = self._ascom_camera.binning
                if not max_h or not max_w:
                    raise AssertionError('Unable to read ASCOM camera image size limits.')
                if not binning:
                    raise AssertionError('Unable to read ASCOM camera binning value.')
                try:
                    self._ascom_camera.NumX = max_w/binning
                    self._ascom_camera.NumY = max_h/binning
                except:
                    raise AssertionError('Unable to set ASCOM camera image size.')
            except:
                raise AssertionError('Unable to read ASCOM camera image size limits.')
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
        - timestamp (datetime.datetime): UTC timestamp when the image event occured (i.e. when the capture
          finished).
        - \*args, \*\*kwargs should be allowed for forward compatability.

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
        #self._log_debug('Checking if running')
        if not self.is_init: return False
        if self.model.lower() == 'ptgrey':
            return self._ptgrey_camera is not None and self._ptgrey_camera.IsStreaming()
        elif self.model.lower() == 'ascom':
            return self._ascom_camera.Connected
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
        elif self.model.lower() == 'ascom':
            self._ascom_camera_imaging_handler.start_imaging_loop()
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
        elif self.model.lower() == 'ascom':
            self._ascom_camera_imaging_handler.stop_imaging_loop()
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