"""Camera hardware interface
======================

Current hardware support:
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
from time import sleep, time as timestamp, perf_counter as precision_timestamp
from datetime import datetime
from threading import Thread, Event
from struct import pack as pack_data
# External imports:
import numpy as np
import serial

# Hardware support imports:
import zwoasi

_zwoasi_bayer = {0:'RGGB', 1:'BGGR', 2:'GRBG', 3:'GBRG'}                                                        

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
    _supported_models = ('ptgrey','zwoasi','ascom')
    _default_model = 'zwoasi'

    def __init__(self, model=None, identity=None, name=None, auto_init=True, debug_folder=None, properties=[]):
        """Create Camera instance. See class documentation."""
        # Logger setup
        self._debug_folder = None
        if debug_folder is None:
            self.debug_folder = Path(__file__).parent.parent / 'debug'
        else:
            self.debug_folder = debug_folder
        self._logger = logging.getLogger('pypogs.hardware.Camera')
        if not self._logger.hasHandlers():
            # Add new handlers to the logger if there are none
            self._logger.setLevel(logging.DEBUG)
            # Console handler at INFO level
            ch = logging.StreamHandler()
            ch.setLevel(logging.INFO)   #CHANGE MEEEEEEE
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
        self._binning = 1
        self._flipX = False
        self._flipY = False
        self._rot90 = 0 #Number of times to rotate by 90 deg, done after flips
        self._color_bin = True # Downscale when debayering instead of interpolating for speed
        #Only used for ptgrey
        self._ptgrey_camera = None
        self._ptgrey_camlist = None
        self._ptgrey_system = None
        #Only used for zwoasi
        self._zwo_camera_index = None
        self._zwoasi_camera = None
        self._zwoasi_is_init = False
        self._zwoasi_image_handler = None
        #Only used for ascom
        self._ascom_driver_handler = None
        self._zwoasi_property = None
        self._ascom_camera = None
        self._exposure_sec = 0.1
        #Callbacks on image event
        self._call_on_image = set()
        self._got_image_event = Event()
        self._image_data = None
        self._image_timestamp = None
        self._imgs_since_start = 0
        self._average_frame_time = None  # Running average of time between frames in ms
        self._image_precision_timestamp = None  # Precision timestamp of last frame

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
            
        available_properties = self.available_properties
        for property_name in properties:
            if property_name in available_properties:
                self._logger.debug('Setting property "%s" to value "%s"' % (property_name, properties[property_name]))
                try:
                    setattr(self, property_name, properties[property_name])
                except:
                    self._logger.warning('Failed to set camera property "%s" to value "%s"' % (property_name, properties[property_name]))
            
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

        
    #FIXME:  do we need release method for zwo?
        
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
            self._ascom_pythoncom.CoUninitialize()
            self._ascom_camera = None
        self._log_debug('ASCOM camera hardware released')

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
            - 'zwoasi' for ZWO ASI cameras.
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
        - Must set before initialising the device and may not be changed for an initialised device.
        - For model *ptgrey* this is the serial number *as a string*
        - For model *zwoasi* this is the index (starting at zero)
        - For model *ascom*, a driver name may be specified if known, (i.e. DSLR, ASICamera1, ASICamera2, Simulator,
        QHYCCD, QHYCCD_GUIDER, QHYCCD_CAM2, AtikCameras, AtikCameras2, etc), otherwise the ASCOM driver
        selector will open.
        """
        return self._identity
    @identity.setter
    def identity(self, identity):
        self._log_debug('Setting identity to: '+str(identity))
        assert not self.is_init, 'Cannot change already initialised device'
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
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi, first load and initialise the package')
            library_path = Path(__file__).parent.parent / '_system_data' / 'ASICamera2'
            self._log_debug('Initialising with files at ' + str(library_path.resolve()))
            try:
                zwoasi.init(str(library_path.resolve()))
            except zwoasi.ZWO_Error as e:
                if not str(e) == 'Library already initialized':
                    raise # Throw error if any other problem than already initialised
            
            self._log_debug('Library intialised, checking if identity is available')

            # Get count and list of detected ZWO cameras:
            zwo_num_cameras = zwoasi.get_num_cameras()
            assert zwo_num_cameras > 0, 'No ZWO cameras detected.'
            zwo_camera_names = zwoasi.list_cameras()
            self._log_info('Detected '+str(zwo_num_cameras)+' ZWO cameras: '+str(zwo_camera_names))            
            
            # Derive ZWO camera index from identity:
            self._zwo_camera_index = None            
            if identity is None:
                # Disallow for now. Later, consider populating a selection dialog.
                raise AssertionError('Identity is none')
            elif identity.isdigit():
                self._log_info('specified camera identity as index ('+identity+')')
                self._zwo_camera_index = int(identity)
            elif identity.lower().startswith('zwo') or identity.lower().startswith('asi'):
                self._log_info('specified camera identity by string ('+identity+')')
                for detected_camera_idx, detected_camera in enumerate(zwo_camera_names):
                    if detected_camera.lower().replace('zwo ','') == identity.lower().replace('zwo ',''):
                        self._zwo_camera_index = detected_camera_idx
                        break
            else:
                raise AssertionError('Unrecognized identity')

            self._log_info('Selected ZWO camera: index '+str(self._zwo_camera_index)+', name "'+zwo_camera_names[self._zwo_camera_index]+'"')
            assert self._zwo_camera_index is not None, 'Unrecognized ZWO camera identity: "'+identity+'"'
            assert self._zwo_camera_index < zwo_num_cameras, ('Selected identity is greater than the available cameras,'
                                         'largest possible is one less than ' + str(num_cams))
            # TODO: test if in use. Turns out API allows you to initialise several objects
            # connected to the same hardware without complaining... Must keep own list?
            
            #self._log_debug('Identity available, testing if in use')
            #try...
            #self._zwoasi_camera = zwoasi.Camera(identity)
                
            #except...
            
            #finally... close
            
            self._identity = identity

        elif self.model.lower() == 'ascom':
            self._log_debug('Checking ASCOM camera identity and availability')
            assert identity is not None, 'ASCOM camera identity not resolved'
            if not identity.startswith('ASCOM'):
                identity = 'ASCOM.'+identity+'.Camera'
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
            self._log_debug('Specified identity: "'+str(self.identity)+'" ['+str(len(self.identity))+']')
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
        elif self.model.lower() == 'zwoasi':
            return self._zwoasi_is_init
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
                    last_timestamp = self.parent._image_precision_timestamp
                    self.parent._image_precision_timestamp = precision_timestamp()                                                                                  
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
                            #self.parent._log_debug('Calling back to: ' + str(func))
                            func(self.parent._image_data, self.parent._image_timestamp)
                        except:
                            self.parent._log_warning('Failed image callback', exc_info=True)
                    self.parent._imgs_since_start += 1
                    if last_timestamp is not None:
                            new_frame_time = self.parent._image_precision_timestamp - last_timestamp
                            if self.parent._average_frame_time is None:
                                self.parent._average_frame_time = new_frame_time
                            else:
                                self.parent._average_frame_time = .8*self.parent._average_frame_time + .2*new_frame_time                                                                                                                        
                    self.parent._log_debug('Event handler finished.')

            self._ptgrey_event_handler = PtGreyEventHandler(self)
            self._log_debug('Created ptgrey image event handler')
            self._ptgrey_camera.RegisterEventHandler( self._ptgrey_event_handler )
            self._log_debug('Registered ptgrey image event handler')
            self._log_info('Camera successfully initialised')
            
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi, try to initialise')
            assert self._zwo_camera_index is not None, 'ZWO camera index not determined from identity'
            self._zwoasi_camera = zwoasi.Camera(self._zwo_camera_index)
            
            # Set to normal mode and 16 bit mode by default
            self._zwoasi_camera.set_camera_mode(zwoasi.ASI_MODE_NORMAL)
            self._zwoasi_camera.set_image_type(zwoasi.ASI_IMG_RAW16)
            
            self._zwoasi_property = self._zwoasi_camera.get_camera_property()
            
            # Set everything to default to be safe
            set_to_default = {'Exposure':zwoasi.ASI_EXPOSURE, 'Gain':zwoasi.ASI_GAIN,
                              'Flip':zwoasi.ASI_FLIP, 'BandWidth':zwoasi.ASI_BANDWIDTHOVERLOAD,
                              'HardwareBin':zwoasi.ASI_HARDWARE_BIN, 'WB_B':zwoasi.ASI_WB_B,
                              'WB_R':zwoasi.ASI_WB_R, 'Offset':zwoasi.ASI_OFFSET,
                              'HighSpeedMode':zwoasi.ASI_HIGH_SPEED_MODE, 'MonoBin':zwoasi.ASI_MONO_BIN}
            controls = self._zwoasi_camera.get_controls()
            for k in set_to_default.keys():
                if k in controls: # Check that our model has this property
                    self._zwoasi_camera.set_control_value(set_to_default[k], controls[k]['DefaultValue'])
            
            # Handler class to deal with the image stream
            class ZwoAsiImageHandler():
                """Barebones class to start/stop camera and read images"""
                def __init__(self, parent):
                    self.parent = parent
                    self._thread = None
                    self._stop_running = False
                def start(self):
                    self.parent._log_info('Starting zwoasi imaging thread')
                    self._thread = Thread(target = self._run)
                    self._stop_running = False
                    self._thread.start()
                def stop(self):
                    self.parent._log_info('Stopping zwoasi imaging thread')
                    self._stop_running = True
                    self._thread.join()
                    self.parent._zwoasi_camera.stop_video_capture()
                    self.parent._log_info('zwoasi imaging thread has been stopped')
                @property
                def is_running(self):
                    return self._thread is not None and self._thread.is_alive()
                def _run(self):
                    """Start camera and continiously read out data"""
                    cam = self.parent._zwoasi_camera
                    cam.start_video_capture()
                    timeout_ms = self.parent.exposure_time + 500
                    while not self._stop_running:
                        try:
                            img = cam.capture_video_frame(timeout = timeout_ms)
                            self.parent._image_timestamp = datetime.utcnow()
                            last_timestamp = self.parent._image_precision_timestamp
                            self.parent._image_precision_timestamp = precision_timestamp()
                        except zwoasi.ZWO_IOError as e:
                            if str(zwoasi.ZWO_IOError) == 'Camera closed':
                                self.parent._log_debug('zwoasi Camera closed, probably deinitialising')
                            else:
                                raise
                        if self._stop_running:
                            break
                        self.parent._log_debug('New image captured! Unpack and set image event')
                        if self.parent._rot90:
                            img = np.rot90(img, self.parent._rot90)
                        if len(img.shape) == 3:
                            # Camera gives GRB, reverse to RGB
                            img = img[:, :, ::-1]
                            
                        # If color camera we may need to debayer
                        if self.parent._zwoasi_property['IsColorCam'] and len(img.shape) < 3:
                            pattern = _zwoasi_bayer[self.parent._zwoasi_property['BayerPattern']]
                            t0_debayer = precision_timestamp()
                            self.parent._image_data = _debayer_image(img, order=pattern,
                                                                     downsize=self.parent.color_bin)
                            t_debayer = precision_timestamp() - t0_debayer
                            self.parent._log_debug('Debayered image in ' + str(t_debayer))
                        else:
                            self.parent._image_data = img
                        
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
                        if last_timestamp is not None:
                            new_frame_time = self.parent._image_precision_timestamp - last_timestamp
                            if self.parent._average_frame_time is None:
                                self.parent._average_frame_time = new_frame_time
                            else:
                                self.parent._average_frame_time = .8*self.parent._average_frame_time + .2*new_frame_time
                        self.parent._log_debug('Event handler finished.')

            self._zwoasi_image_handler = ZwoAsiImageHandler(self)
            self._zwoasi_is_init = True
            
        elif self.model.lower() == "ascom":
            if self._ascom_camera is not None:
                raise RuntimeError('There is already an ASCOM camera object here')
            self._log_debug('Attempting to connect to ASCOM device "'+str(self.identity)+'"')
            if self._ascom_driver_handler is None:
                import pythoncom
                self._ascom_pythoncom = pythoncom
                import win32com.client
                self._ascom_driver_handler = win32com.client
            camDriverName = str()
            if self.identity is not None:
                self._log_debug('Specified identity: "'+str(self.identity)+'" ['+str(len(self.identity))+']')
                if self.identity.startswith('ASCOM'):
                    camDriverName = self.identity
                else:
                    camDriverName = 'ASCOM.'+str(self.identity)+'.Camera'
            else:
                ascomSelector = self._ascom_driver_handler.Dispatch("ASCOM.Utilities.Chooser")
                ascomSelector.DeviceType = 'Camera'
                camDriverName = ascomSelector.Choose('None')
                self._logger.info("Selected camera driver: "+camDriverName)
                if not camDriverName:            
                    self._log_debug('User canceled camera selection')
            assert camDriverName, 'Unable to identify ASCOM camera.'
            self._identity = camDriverName.replace('ASCOM.','').replace('.Camera','')
            self._logger.info('Loading ASCOM camera driver: '+camDriverName)
            self._ascom_pythoncom.CoInitialize()
            try:
                self._ascom_camera = self._ascom_driver_handler.Dispatch(camDriverName)
            except self._ascom_pythoncom.com_error:
                raise AssertionError('Error attaching to device "%s", check name.' % camDriverName)
            assert hasattr(self._ascom_camera, 'Connected'), "Unable to access camera driver"
            self._log_debug('Connecting to camera')
            self._ascom_camera.Connected = True
            assert self._ascom_camera.Connected, "Failed to connect to camera"
            assert self._ascom_camera is not None, 'ASCOM camera not initialized'
            self._log_debug('ReadoutMode: '+str(self._ascom_camera.ReadoutModes[self._ascom_camera.ReadoutMode]))
            self._log_debug('SensorType: '+str(self._ascom_camera.SensorType))
            self._log_debug('CameraState: '+str(self._ascom_camera.CameraState))
            
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
                        self.parent._log_debug('Waiting for ASCOM camera imaging loop to stop')
                        polling_period = 0.05 #sec
                        while self._is_running:
                            sleep(polling_period)
                        self.parent._log_debug('Stopped ASCOM camera imaging loop')
                        
                def imaging_loop(self):
                    assert self.parent._ascom_camera, 'Cannot start imaging - ASCOM camera driver not loaded'
                    assert self.parent._ascom_camera.Connected, 'Cannot start imaging - ASCOM camera not connected'                        
                    self._is_running = True
                    self.parent._log_debug('Starting ASCOM camera imaging loop')
                    timeout = 0.5 # sec
                    polling_period = 0.001 # sec
                    while self.continue_imaging and self.parent._ascom_camera.Connected:
                        #self.parent._log_debug('Starting ASCOM camera exposure')
                        # Start exposure:
                        self.parent._ascom_camera.StartExposure(self.parent._exposure_sec,True)
                        
                        # Wait for image to be ready:
                        sleep(self.parent._exposure_sec * 0.95)
                        waited_time = 0
                        while not self.parent._ascom_camera.ImageReady and waited_time < timeout:
                            sleep(polling_period)
                            waited_time += polling_period
                        if not self.parent._ascom_camera.ImageReady:
                            self.parent._log_debug('Timed out waiting for image')
                            self.parent._log_debug('Camera connected: '+str(self.parent._ascom_camera.Connected))
                            continue
                        # Get and pre-process image:
                        got_image = False
                        try:
                            img = np.array(self.parent._ascom_camera.ImageArray, dtype=np.float).copy().T
                            if self.parent._flipX:
                                img = np.fliplr(img)
                            if self.parent._flipY:
                                img = np.flipud(img)
                            if self.parent._rot90:
                                img = np.rot90(img, self.parent._rot90)
                            self.parent._image_timestamp = datetime.utcnow()
                            self.parent._image_data = img
                            got_image = True
                        except:
                            self.parent._log_debug('Failed to access image.')
                            self.parent._image_data = None
                            
                        # Signal image ready and run callbacks:
                        if got_image:
                            self.parent._got_image_event.set()
                            self.parent._log_debug('Time: ' + str(self.parent._image_timestamp) \
                                                   + ' Size:' + str(self.parent._image_data.shape) \
                                                   + ' Type:' + str(self.parent._image_data.dtype))
                            for func in self.parent._call_on_image:
                                try:
                                    #self.parent._log_debug('Calling back to: ' + str(func))
                                    func(self.parent._image_data, self.parent._image_timestamp)
                                except:
                                    self.parent._log_warning('Failed image callback', exc_info=True)
                            self.parent._imgs_since_start += 1
                            
                        #self.parent._log_debug('ASCOM image loop finished.')
                        if self.continue_imaging:
                            sleep(0.001)
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
        elif self._zwoasi_camera:
            self._log_debug('Found zwoasi camera, deinitialising')
            self._zwoasi_camera.close()
            self._zwoasi_is_init = False
            self._zwoasi_camera = None
            self._zwoasi_property = None
            self._log_debug('Closed, set deinit flag, deleted object')
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
            return ('flip_x', 'flip_y', 'rotate_90', 'plate_scale', 'rotation', 'binning', 'size_readout',
                    'frame_rate_auto', 'frame_rate', 'gain_auto', 'gain', 'exposure_time_auto', 'exposure_time')
        elif self.model.lower() == 'zwoasi':
            return ('flip_x', 'flip_y', 'rotate_90', 'plate_scale', 'rotation', 'binning', 'size_readout',
                    'frame_rate_auto', 'gain', 'gain_auto', 'exposure_time_auto', 'exposure_time', 'color_bin')
        elif self.model.lower() == 'ascom':
            return ('flip_x', 'flip_y', 'rotate_90', 'plate_scale', 'rotation', 'binning', 'size_readout',\
                     'gain', 'exposure_time')           
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
        elif self.model.lower() == 'zwoasi':
            flipmode = self._zwoasi_camera.get_control_value(zwoasi.ASI_FLIP)[0]
            return (flipmode == 1) or (flipmode == 3) # mode 1 is flip horizontal, mode 3 is flip both
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
        elif self.model.lower() == 'zwoasi':
            if not flip: # Disable horizontal flipping
                if not self.flip_y:
                    # No flipping
                    self._zwoasi_camera.set_control_value(zwoasi.ASI_FLIP, 0)
                else:
                    # Set to only vertical flipping
                    self._zwoasi_camera.set_control_value(zwoasi.ASI_FLIP, 2)
            else: # Enable horizontal flipping
                if not self.flip_y:
                    # Flip only horizontal
                    self._zwoasi_camera.set_control_value(zwoasi.ASI_FLIP, 1)
                else:
                    # Flip both
                    self._zwoasi_camera.set_control_value(zwoasi.ASI_FLIP, 3)
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
        elif self.model.lower() == 'zwoasi':
            flipmode = self._zwoasi_camera.get_control_value(zwoasi.ASI_FLIP)[0]
            return (flipmode == 2) or (flipmode == 3) # mode 2 is flip vertical, mode 3 is flip both
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
        elif self.model.lower() == 'zwoasi':
            if not flip: # Disable vertical flipping
                if not self.flip_x:
                    # No flipping
                    self._zwoasi_camera.set_control_value(zwoasi.ASI_FLIP, 0)
                else:
                    # Set to only horizontal flipping
                    self._zwoasi_camera.set_control_value(zwoasi.ASI_FLIP, 1)
            else: # Enable vertical flipping
                if not self.flip_x:
                    # Flip only vertical
                    self._zwoasi_camera.set_control_value(zwoasi.ASI_FLIP, 2)
                else:
                    # Flip both
                    self._zwoasi_camera.set_control_value(zwoasi.ASI_FLIP, 3)
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
        if self.model.lower() in ('ptgrey','zwoasi','ascom'):
            return self._rot90
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @rotate_90.setter
    def rotate_90(self, k):
        self._log_debug('Set rot90 called with: '+str(k))
        assert self.is_init, 'Camera must be initialised'
        k = int(k)
        if self.model.lower() in ('ptgrey','zwoasi','ascom'):
            self._log_debug('Will rotate the received image array ourselves.')
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
        return self._plate_scale * self._binning
    @plate_scale.setter
    def plate_scale(self, arcsec):
        self._log_debug('Set plate scale called with: '+str(arcsec))
        self._plate_scale = float(arcsec)
        self._log_debug('Plate scale set to: '+str(self._plate_scale))

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
        elif self.model.lower() == 'zwoasi':
            return True # Only auto frame rate available in normal mode
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
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi')
            if not auto:
                self._log_warning('zwoasi does not support fixed frame rate, staying in auto mode.')
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
            self._log_debug('frame rate not supported in ASCOM camera class')
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
        elif self.model.lower() == 'ascom':
            self._log_debug('frame rate not supported in ASCOM camera class')
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def frame_rate_actual(self):
        """float: Get the actual image frame rate in Hz. Returns None if not running."""
        return 1/self._average_frame_time if self._average_frame_time is not None else None

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
        elif self.model.lower() == 'zwoasi':
            return self._zwoasi_camera.get_control_value(zwoasi.ASI_GAIN)[1]
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
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi')
            if not self.gain_auto == auto:
                self._log_debug('Changing gain auto mode to: ' + str(auto))
                controls = self._zwoasi_camera.get_controls()
                default = controls['Gain']['DefaultValue']
                self._log_debug('Setting gain to auto ' + str(auto) + ' and default: ' + str(default))
                self._zwoasi_camera.set_control_value(zwoasi.ASI_GAIN, default, auto)
                self._log_debug('Set gain auto to: ' + str(self.gain_auto))
            else:
                self._log_warning('Gain auto mode already set to: ' + str(auto) + ', doing nothing')
        elif self.model.lower() == 'ascom':
            self._log_debug('auto gain not implemented in ASCOM')
            return False
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def gain_limit(self):
        """tuple of float: Get the minimum and maximum gain supported in the camera's native unit."""
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
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi')
            controls = self._zwoasi_camera.get_controls()
            min = controls['Gain']['MinValue']
            max = controls['Gain']['MaxValue']
            self._log_debug('Camera gave min ' + str(min) + ' and max ' + str(max))
            return (min, max)
        elif self.model.lower() == 'ascom':
            return (self._ascom_camera.GainMin, self._ascom_camera.GainMax)
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def gain(self):
        """float: Get or set the camera gain in the camera's native unit."""
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
        elif self.model.lower() == 'zwoasi':
            return self._zwoasi_camera.get_control_value(zwoasi.ASI_GAIN)[0]
        elif self.model.lower() == 'ascom':
            return self._ascom_camera.Gain
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @gain.setter
    def gain(self, gain):
        self._log_debug('Set gain called with: '+str(gain))
        assert self.is_init, 'Camera must be initialised'
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
                    node.SetValue(float(gain))
                except PySpin.SpinnakerException as e:
                    if 'OutOfRangeException' in e.message:
                        raise AssertionError('The commanded value is outside the allowed range. See gain_limit')
                    else:
                        raise #Rethrows error
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi')
            self._zwoasi_camera.set_control_value(zwoasi.ASI_GAIN, int(gain))
            self._log_debug('Set gain to ' + str(self.gain))
        elif self.model.lower() == 'ascom':
            if gain < self._ascom_camera.GainMin or gain > self._ascom_camera.GainMax:
                self._log_debug('Requested gain out of allowable range ('+str(self._ascom_camera.GainMin)+':'+str(self._ascom_camera.GainMax)+').')
                raise AssertionError('Requested gain ['+str(gain)+'] out of allowable range ('+str(self._ascom_camera.GainMin)+' - '+str(self._ascom_camera.GainMax)+').')
            self._ascom_camera.Gain = gain
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
        elif self.model.lower() == 'zwoasi':
            return self._zwoasi_camera.get_control_value(zwoasi.ASI_EXPOSURE)[1]
        elif self.model.lower() == 'ascom':
            self._log_debug('auto exposure not implemented in ASCOM')
            return False
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
    @exposure_time_auto.setter
    def exposure_time_auto(self, auto):
        self._log_debug('Set exposure time auto called with: '+str(auto))
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
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi')
            if not self.exposure_time_auto == auto:
                self._log_debug('Changing exposure auto mode to: ' + str(auto))
                controls = self._zwoasi_camera.get_controls()
                default = controls['Exposure']['DefaultValue']
                self._log_debug('Setting exposure to auto ' + str(auto) + ' and default: ' + str(default))
                self._zwoasi_camera.set_control_value(zwoasi.ASI_EXPOSURE, default, auto)
                self._log_debug('Set exposure auto to: ' + str(self.exposure_time_auto))
            else:
                self._log_warning('Exposure auto mode already set to: ' + str(auto) + ', doing nothing')
        elif self.model.lower() == 'ascom':
            self._log_debug('auto exposure not implemented in ASCOM')
            return 0
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))

    @property
    def exposure_time_limit(self):
        """tuple of float: Get the minimum and maximum expsure time in ms supported."""
        self._log_debug('Get exposure time limit called')
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
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi')
            controls = self._zwoasi_camera.get_controls()
            min = controls['Exposure']['MinValue']
            max = controls['Exposure']['MaxValue']
            self._log_debug('Camera gave min ' + str(min) + ' and max ' + str(max))
            return (min/1000, max/1000)
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
        elif self.model.lower() == 'zwoasi':
            return self._zwoasi_camera.get_control_value(zwoasi.ASI_EXPOSURE)[0] / 1000 #microseconds used in zwoasi
        elif self.model.lower() == 'ascom':
            self._log_debug('Returning '+str(self._exposure_sec*1000))
            return self._exposure_sec*1000
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
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi, setting to ' + str(int(exposure_ms*1000)))
            self._zwoasi_camera.set_control_value(zwoasi.ASI_EXPOSURE, int(exposure_ms*1000))
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
        *ptgrey* cameras bin by summing, *zwoasi* cameras bin by averaging.

        Setting will stop and restart camera if running. Will scale size_readout to show the same sensor area.
        """
        assert self.is_init, 'Camera must be initialised'
        #self._log_debug('Get binning called')
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
        elif self.model.lower() == 'zwoasi':
            return self._zwoasi_camera.get_bin()
        elif self.model.lower() == 'ascom':
            try:
                val_horiz = self._ascom_camera.BinX
                val_vert = self._ascom_camera.BinY
                #self._log_debug('Got '+str(val_horiz)+' '+str(val_vert))
                if val_horiz != val_vert:
                    self._log_warning('Horizontal and vertical binning is not equal.')
                self._binning = val_horiz
                return val_horiz
            except PySpin.SpinnakerException:
                self._log_warning('Failed to read binning property', exc_info=True)
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
        if self.color_bin:
            initial_size = [2*x for x in initial_size]
        initial_bin = self._binning
        self._log_debug('Initial sensor readout area and binning: '+str(initial_size)+' ,'+str(initial_bin))
        
        # Calculate what the new ROI needs to be set to
        bin_scaling = binning/initial_bin
        new_size = [round(sz/bin_scaling) for sz in initial_size]  
        self._log_debug('New binning and new size to set: '+str(binning)+' ,'+str(new_size))
              
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
                self._binning = binning
            except PySpin.SpinnakerException as e:
                self._log_debug('Failure setting', exc_info=True)
                if 'OutOfRangeException' in e.message:
                    raise ValueError('Commanded value not allowed.')
                elif 'AccessException' in e.message:
                    raise AssertionError('Not allowed to change binning now.')
                else:
                    raise #Rethrows error
            # Correctly set the ROI to adjust for new binning size
            try:
                self.size_readout = new_size
                self._log_debug('Set new size to: ' + str(self.size_readout))
            except:
                self._log_warning('Failed to scale readout after binning change', exc_info=True) 
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi, width must be multiple of 8 and height multiple of 2')
            new_size = [new_size[0] - (new_size[0] % 8), new_size[1] - (new_size[1] % 2)]
            self._log_debug('Adjusted to allowable size: ' + str(new_size))
            self._zwoasi_camera.set_roi(width = new_size[0], height = new_size[1], bins = binning)
            self._log_debug('Set binning to ' + str(self.binning) + ' and readout to ' + str(self.size_readout))
        elif self.model.lower() == 'ascom':
            binMax = self._ascom_camera.MaxBinX
            if binMax and binning <= binMax:
                try:
                    self._logger.info("setting binning to %i" % binning)
                    self._ascom_camera.BinX = binning
                    self._ascom_camera.BinY = binning
                    self._ascom_camera.NumX = self._ascom_camera.CameraXSize/binning
                    self._ascom_camera.NumY = self._ascom_camera.CameraYSize/binning
                    self._binning = binning
                except:
                    raise AssertionError('Unable to set camera binning')
            else:
                raise ValueError('exceeds camera max bin val ',binMax)
            #new_bin = self._binning
            #bin_scaling = new_bin/initial_bin
            #new_size = [round(sz/bin_scaling) for sz in initial_size]
            #self._log_debug('New binning and new size to set: '+str(new_bin)+' ,'+str(new_size))
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
        
        #try:
        #    self.size_readout = new_size
        #    self._log_debug('Set new size to: ' + str(self.size_readout))
        #except:
        #    self._log_warning('Failed to scale readout after binning change', exc_info=True)
        
        if was_running:
            self._log_debug('Restarting camera imaging loop.')
            try:
                self.start()
                self._log_debug('Restarted')
            except Exception:
                self._log_debug('Failed to restart: ', exc_info=True)
        else:
            self._log_debug('Camera imaging loop was not previously running.')

    @property
    def color_bin(self):
        """bool: Get or set if colour binning is active. Defaults to True for colour cameras. Is always False for mono
        cameras.
        
        When colour binning is True, each 2x2 Bayer group on the image sensor will form one RGB pixel in the output.
        If set to False, interpolation will be used to create an RGB image at full resolution. Interpolation may slow
        down the image processing significantly.
        """
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            return False
        elif self.model.lower() == 'zwoasi':
            return self._zwoasi_property['IsColorCam'] and self._color_bin
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
            
    @color_bin.setter
    def color_bin(self, bin):
        self._log_debug('Set color bin called with: '+str(bin))
        assert self.is_init, 'Camera must be initialised'
        if self.model.lower() == 'ptgrey':
            raise RuntimeError('ptgrey cameras do not support color binning')
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi, check if we use a colour camera')
            assert self._zwoasi_property['IsColorCam'], 'Must have colour camera to do colour binning'
            self._color_bin = bool(bin)
            self._log_debug('Set color bin to: ' + str(self._color_bin))
        else:
            self._log_warning('Forbidden model string defined.')                                                                                

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
        elif self.model.lower() == 'zwoasi':
            properties = self._zwoasi_camera.get_camera_property()
            bin = self.binning
            width = int(properties['MaxWidth'] / bin)
            width -= width % 8 # Must be multiple of 8
            height = int(properties['MaxHeight'] / bin)
            height -= height % 2 # Must be multiple of 2
            return (width, height) if not self.color_bin else (width//2, height//2)
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

        For model *zwoasi* the set size will be rounded down to the nearest multiple of 8 in width and 2 in height.

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
        elif self.model.lower() == 'zwoasi':
            (width, height) = tuple(self._zwoasi_camera.get_roi_format()[:2])
            return (width, height) if not self.color_bin else (width//2, height//2)
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
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Using zwoasi, check if colour binning and adjust')
            if self.color_bin:
                size = tuple([x*2 for x in size])
                self._log_debug('Size adjusted to ' + str(size))
            self._log_debug('Adjust desired size to allowable size:')                                                                     
            size = [size[0] - (size[0] % 8), size[1] - (size[1] % 2)]
            self._zwoasi_camera.set_roi(width = size[0], height = size[1])
            self._log_debug('Set readout to ' + str(self.size_readout))
        elif self.model.lower() == 'ascom':
            try:
                max_w = self._ascom_camera.CameraXSize
                max_h = self._ascom_camera.CameraYSize
                if not max_h or not max_w:
                    raise AssertionError('Unable to read ASCOM camera image size limits.')
                try:
                  bin_w = self._ascom_camera.BinX
                  bin_h = self._ascom_camera.BinY
                except:
                  raise AssertionError('Unable to read ASCOM camera binning value.')
                if not bin_h or not bin_w:
                    raise AssertionError('Unable to read ASCOM camera binning value.')
                try:
                    self._ascom_camera.NumX = max_w/bin_w
                    self._ascom_camera.NumY = max_h/bin_h
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
        #self._log_debug('Checking if running')
        if not self.is_init: return False
        if self.model.lower() == 'ptgrey':
            return self._ptgrey_camera is not None and self._ptgrey_camera.IsStreaming()
        elif self.model.lower() == 'zwoasi':
            return self._zwoasi_camera is not None and self._zwoasi_image_handler.is_running
        elif self.model.lower() == 'ascom':
            return self._ascom_camera.Connected and self._ascom_camera_imaging_handler._is_running
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
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Calling start on zwoasi image handler')
            self._zwoasi_image_handler.start()
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
        elif self.model.lower() == 'zwoasi':
            self._log_debug('Calling stop on zwoasi image handler')
            self._zwoasi_image_handler.stop()
        elif self.model.lower() == 'ascom':
            self._ascom_camera_imaging_handler.stop_imaging_loop()
        else:
            self._log_warning('Forbidden model string defined.')
            raise RuntimeError('An unknown (forbidden) model is defined: '+str(self.model))
        self._image_data = None
        self._image_timestamp = None
        self._average_frame_time = None
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
        timeout = min(timeout, self.exposure_time*1000)
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
        #self._log_debug('Got latest image request')
        assert self.is_running, 'Camera must be running'
        return self._image_data
def _debayer_image(img, order='RGGB', downsize=False):
    """PRIVATE: Debayer image (2d np array) with given pixel order. 
    Order can be RGGB (default), BGGR, GBRG, or GRBG.
    If downsize is set to True, the output image will be half the size of the original.
    """
    
    (height, width) = img.shape
    datatype = img.dtype
    # Unpack pixels. Determine the pixel offset (position in each quad group)
    offset = {}
    green2 = False
    for c, offs in zip(order.upper(), [(0, 0), (0, 1), (1, 0), (1, 1)]):
        if c == 'G':
            if not green2:
                offset['G1'] = offs
                green2 = True
            else:
                offset['G2'] = offs
        else:
            offset[c] = offs
    
    # Unpack channels to individual arrays
    img_red = img[offset['R'][0]::2, offset['R'][1]::2]
    img_green_1 = img[offset['G1'][0]::2, offset['G1'][1]::2]
    img_green_2 = img[offset['G2'][0]::2, offset['G2'][1]::2]
    img_blue = img[offset['B'][0]::2, offset['B'][1]::2]

    if downsize:
        # Use red and blue as is, average green. Gives half outsize.
        out = np.zeros((height//2, width//2, 3), dtype=datatype)
        # Red green and blue channels. Green has a thrid dim for the two pixels in each group.
        out[:, :, 0] = img_red
        out[:, :, 1] = ((img_green_1.astype(np.float32) + img_green_2.astype(np.float32))/2).astype(datatype)
        out[:, :, 2] = img_blue
    
    else:
        # Bilinear interpolation to give approximate full outsize.
        out = np.zeros((height, width, 3), dtype=datatype)

        def insert_red_blue(oy, ox, image):
            out = np.zeros(np.array(image.shape)*2, dtype=np.float32)
            # Pad array to deal with edges
            image = np.pad(image, 1, mode='edge').astype(np.float32)
            # Set output array according to bilinear interpolation
            out[oy::2, ox::2] = image[1:-1, 1:-1]
            out[(oy+1)%2::2, ox::2] = (image[(oy+1)%2:(oy+1)%2-2, 1:-1] 
                                     + image[(oy+1)%2+1:((oy+1)%2-1 or None), 1:-1])/2
            out[oy::2, (ox+1)%2::2] = (image[1:-1, (ox+1)%2:(ox+1)%2-2]
                                     + image[1:-1, (ox+1)%2+1:((ox+1)%2-1 or None)])/2
            out[(oy+1)%2::2, (ox+1)%2::2] = (image[(oy+1)%2:(oy+1)%2-2, (ox+1)%2+1:((ox+1)%2-1 or None)]
                                           + image[(oy+1)%2+1:((oy+1)%2-1 or None):, (ox+1)%2+1:((ox+1)%2-1 or None)]
                                           + image[(oy+1)%2:(oy+1)%2-2, (ox+1)%2:(ox+1)%2-2]
                                           + image[(oy+1)%2+1:((oy+1)%2-1 or None), (ox+1)%2:(ox+1)%2-2])/4
            return out
            
        def insert_green(oy1, ox1, image1, oy2, ox2, image2):
            out = np.zeros(np.array(image1.shape)*2, dtype=np.float32)
            # Pad arrays to deal with edges
            image1 = np.pad(image1, 1, mode='edge').astype(np.float32)
            image2 = np.pad(image2, 1, mode='edge').astype(np.float32)
            out[oy1::2, ox1::2] = image1[1:-1, 1:-1]
            out[(oy1+1)%2::2, (ox1+1)%2::2] = image2[1:-1, 1:-1]
            out[(oy1+1)%2::2, ox1::2] = (image1[(oy1+1)%2:(oy1+1)%2-2, 1:-1] 
                                       + image1[(oy1+1)%2+1:((oy1+1)%2-1 or None), 1:-1]
                                       + image2[1:-1, (ox2+1)%2+1:((ox2+1)%2-1 or None)]
                                       + image2[1:-1, (ox2+1)%2:(ox2+1)%2-2])/4

            out[oy1::2, (ox1+1)%2::2] = (image1[1:-1, (ox1+1)%2:(ox1+1)%2-2] 
                                       + image1[1:-1, (ox1+1)%2+1:((ox1+1)%2-1 or None)]
                                       + image2[(oy2+1)%2+1:((oy2+1)%2-1 or None), 1:-1]
                                       + image2[(oy2+1)%2:(oy2+1)%2-2, 1:-1])/4
            return out

        # Debayer/interpolate red and blue channels
        out[:, :, 0] = insert_red_blue(offset['R'][0], offset['R'][1], img_red).astype(datatype)
        out[:, :, 2] = insert_red_blue(offset['B'][0], offset['B'][1], img_blue).astype(datatype)
        
        # Debayer/interpolate green channel
        out[:, :, 1] = insert_green(offset['G1'][0], offset['G1'][1], img_green_1,
                                    offset['G2'][0], offset['G2'][1], img_green_2,).astype(datatype)
        
    return out