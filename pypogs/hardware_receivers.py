"""Receiver interfaces
======================

Current harware support:
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
from datetime import datetime
from threading import Thread, Event
from struct import pack as pack_data

# External imports:
import numpy as np
import serial


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
    _default_model = 'ni_daq'

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
        assert not self.is_init, 'Can not change already intialised device model'
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
        assert not self.is_init, 'Can not change already intialised device'
        assert isinstance(identity, str), 'Identity must be a string'
        assert self.model is not None, 'Must define model first'
        if self.model.lower() == 'ni_daq':
            self._logger.debug('Using NI DAQ, checking vailidity by opening a task')
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