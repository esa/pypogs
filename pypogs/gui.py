"""Graphical User Interface
===========================

:class:`pypogs.GUI` implements the GUI. Call pypogs.GUI(*sys*) where *sys* is an instance of pypogs.System to start.

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
import tkinter as tk
import tkinter.ttk as ttk
from tkinter import filedialog
from astropy.time import Time as apy_time
from astropy.coordinates import SkyCoord
from skyfield.sgp4lib import EarthSatellite
from ast import literal_eval as parse_expression
from PIL import Image, ImageTk
import numpy as np
from pathlib import Path
import logging
import sys, os, traceback

COMMAND = 0
MOUNT = 1
ENU = 2
DEG = chr(176) # Degree (unicode) character

NONE = 0
STAR_OL = 1
COARSE_CCL = 2
FINE_FCL = 3

class GUI:
    """Create a Graphical User Interface which controls the given System.

    Args:
        pypogs_system (pypogs.System): The System instance to control.
        gui_update_ms (int, optional): How often (in milliseconds) the GUI should update (default 500).
        debug_folder (pathlib.Path, optional): The folder for debug logging. If None (the default)
            the folder *pypogs*/debug/ will be used/created.
    """
    def __init__(self, pypogs_system, gui_update_ms=500, debug_folder=None):
        # Logger setup
        self._debug_folder = None
        if debug_folder is None:
            self.debug_folder = Path(__file__).parent / 'debug'
        else:
            self.debug_folder = debug_folder

        self.logger = logging.getLogger('pypogs.gui')
        if not self.logger.hasHandlers():
            # Add new handlers to the logger if there are none
            self.logger.setLevel(logging.DEBUG)
            # Console handler at INFO level
            ch = logging.StreamHandler()
            ch.setLevel(logging.INFO)
            # File handler at DEBUG level
            fh = logging.FileHandler(self.debug_folder / 'gui.txt')
            fh.setLevel(logging.DEBUG)
            # Format and add
            formatter = logging.Formatter('%(asctime)s:%(name)s-%(levelname)s: %(message)s')
            fh.setFormatter(formatter)
            ch.setFormatter(formatter)
            self.logger.addHandler(fh)
            self.logger.addHandler(ch)

        self.logger.debug('GUI called with: pypogs_system={} gui_update_ms={}' \
                                                   .format(pypogs_system, gui_update_ms))
        gui_update_ms = int(gui_update_ms)
        self.root = tk.Tk()
        self.root.title('pypogs: the PYthon Portable Optical Ground Station')
        self.root.resizable(False, False)
        self.sys = pypogs_system
        self.logger.debug('Setting styles')
        ttk.Style().configure('TButton', background='gray25', justify='center', anchor='center')
        ttk.Style().configure('TLabel', background='gray25', foreground='white', justify='center', anchor='center')
        ttk.Style().configure('TRadiobutton', background='gray25', foreground='white')
        ttk.Style().configure('TCheckbutton', background='gray25', foreground='white')
        ttk.Style().configure('TFrame', background='gray25')

        self.root.configure(background='gray25')
        self.logger.debug('Loading HardwareFrame')
        self.hardware_frame = HardwareFrame(self.root, self.sys, self.logger)
        self.hardware_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=(10,0), sticky=tk.N)
        self.logger.debug('Loading LiveViewFrame')
        self.live_frame = LiveViewFrame(self.root, self.sys, self.logger)
        self.live_frame.grid(row=0, rowspan=2, column=2, padx=(0,10), pady=10)
        self.live_frame.start(gui_update_ms)
        self.logger.debug('Configuring main window columns')
        self.col1_frame = ttk.Frame(self.root)
        self.col1_frame.grid(row=1, column=0, padx=(10,0), pady=0, sticky=tk.E+tk.W+tk.N)
        self.col2_frame = ttk.Frame(self.root)
        self.col2_frame.grid(row=1, column=1, padx=(0,0), pady=0, sticky=tk.E+tk.W+tk.N)
        self.logger.debug('Loading MountControlFrame')
        self.manual_control_frame = MountControlFrame(self.col1_frame, self.sys, self.logger)
        self.manual_control_frame.grid(column=0, pady=(0,10), sticky=tk.E+tk.W+tk.N)
        self.manual_control_frame.start(gui_update_ms)
        self.logger.debug('Loading AlignmentFrame')
        self.alignment_frame = AlignmentFrame(self.col1_frame, self.sys, self.logger)
        self.alignment_frame.grid(column=0, pady=(0,10), sticky=tk.E+tk.W+tk.N)
        self.alignment_frame.start(gui_update_ms)

        self.logger.debug('Loading TargetFrame')
        self.target_frame = TargetFrame(self.col1_frame, self.sys, self.logger)
        self.target_frame.grid(column=0, pady=(0,10), sticky=tk.E+tk.W+tk.N)
        self.target_frame.start(gui_update_ms)
        self.logger.debug('Loading ControlPropertiesFrame')
        self.control_prop_frame = ControlPropertiesFrame(self.col1_frame, self.sys, self.logger)
        self.control_prop_frame.grid(column=0, pady=(0,10), sticky=tk.E+tk.W+tk.N)
        self.logger.debug('Loading TrackingControlFrame')
        self.tracking_control_frame = TrackingControlFrame(self.col1_frame, self.sys, self.logger)
        self.tracking_control_frame.grid(column=0, pady=(0,10), sticky=tk.E+tk.W+tk.N)
        self.tracking_control_frame.start(gui_update_ms)

        self.logger.debug('Loading StatusFrame')
        self.status_frame = StatusFrame(self.col2_frame, self.sys, self.logger)
        self.status_frame.grid(column=0, pady=(0,10), sticky=tk.E+tk.W+tk.N)
        self.status_frame.start(gui_update_ms)

        self.logger.info('pypogs GUI created')
        self.logger.debug('Setting delete function and starting tkinter mainloop')

        self.root.protocol('WM_DELETE_WINDOW', self.__del__)
        self.root.mainloop()

    def __del__(self):
        try:
            self.status_frame.stop()
        except:
            pass
        try:
            self.target_frame.stop()
        except:
            pass
        try:
            self.live_view.stop()
        except:
            pass
        try:
            self.root.destroy()
        except:
            pass

    @property
    def debug_folder(self):
        """pathlib.Path: Get or set the path for debug logging. Will create folder if not existing."""
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


class ControlPropertiesFrame(ttk.Frame):
    """Extends tkinter.Frame for setting up control loop parameters"""
    def __init__(self, master, pypogs_system, logger):
        self.logger = logger
        self.logger.debug('Creating ControlPropertiesFrame')
        super().__init__(master)
        self.sys = pypogs_system

        self.logger.debug('Creating label and buttons')
        ttk.Label(self, text='Controller Properties').pack(fill=tk.BOTH, expand=True)
        tk.Frame(self, height=1, bg='gray50').pack(fill=tk.BOTH, expand=True)

        ttk.Button(self, text='Feedback', command=self.controller_callback, width=12) \
                                                                .pack(fill=tk.BOTH, expand=True)
        ttk.Button(self, text='Coarse Tracker', command=self.coarse_callback, width=12) \
                                                                .pack(fill=tk.BOTH, expand=True)
        ttk.Button(self, text='Fine Tracker', command=self.fine_callback, width=12) \
                                                                .pack(fill=tk.BOTH, expand=True)
        self.controller_popup = None
        self.coarse_popup = None
        self.fine_popup = None

    def controller_callback(self):
        try:
            if self.controller_popup is None:
                self.controller_popup = self.ControlPopup(self, device=self.sys.control_loop_thread,\
                                                          title='Feedback Properties')
            else:
                self.controller_popup.update()
                self.controller_popup.deiconify()
        except Exception as err:
            self.logger.debug('Could not open feedback popup', exc_info=True)
            ErrorPopup(self, err, self.logger)

    def coarse_callback(self):
        try:
            assert self.sys.coarse_track_thread is not None, 'No coarse tracker (requires mount and coarse camera).'
            if self.coarse_popup is None:
                self.coarse_popup = self.ControlPopup(self, device=self.sys.coarse_track_thread.spot_tracker,\
                                                      title='Coarse Tracker')
            else:
                self.coarse_popup.update()
                self.coarse_popup.deiconify()
        except Exception as err:
            self.logger.debug('Failed to popup coarse control popup', exc_info=True)
            ErrorPopup(self, err, self.logger)

    def fine_callback(self):
        try:
            assert self.sys.fine_track_thread is not None, 'No fine tracker (requires mount and fine camera).'
            if self.fine_popup is None:
                self.fine_popup = self.ControlPopup(self, device=self.sys.fine_track_thread.spot_tracker,\
                                                      title='Fine Tracker')
            else:
                self.fine_popup.update()
                self.fine_popup.deiconify()
        except Exception as err:
            self.logger.debug('Failed to popup fine control popup', exc_info=True)
            ErrorPopup(self, err, self.logger)


    class ControlPopup(tk.Toplevel):
        """Extends tk.Toplevel for controller settings"""
        def __init__(self, master, device, title='Control'):
            super().__init__(master, padx=10, pady=10, bg=ttk.Style().lookup('TFrame', 'background'))
            self.logger = master.logger
            self.device = device
            self.title(title)
            self.resizable(False, False)

            self.properties_frame = ttk.Frame(self)
            self.properties_frame.pack(fill=tk.BOTH, expand=True)
            self.property_entries = []
            ttk.Button(self, text='Set', command=self.set_properties).pack(fill=tk.BOTH, expand=True)
            self.protocol('WM_DELETE_WINDOW', self.withdraw)
            self.update()

        def update(self):
            self.logger.debug('ControlPopup got update request')
            #if self.properties_frame is not None: self.properties_frame.destroy()
            try:
                property_list = self.device.available_properties
            except (AssertionError, AttributeError):
                property_list = None
                self.logger.debug('Got no properties to list', exc_info=True)
            i = 0
            if property_list is not None:
                # Keep track of position
                row = 0
                column = 0
                maxrows = 10
                for prop in property_list:
                    if prop in ('CCL_enable', 'FCL_enable', 'CTFSP_enable'):
                        continue #These have dedicated checkbuttons
                    if row >= maxrows:
                        row = 0
                        column += 2
                    try:
                        (old_name, entry, old_value, label) =  self.property_entries[i]
                        entry.delete(0, 'end')
                        label['text'] = ''
                    except IndexError:
                        entry = ttk.Entry(self.properties_frame, width=20)
                        label = ttk.Label(self.properties_frame, text='')
                    entry.grid(row=row, column=column+1)
                    label.grid(row=row, column=column, sticky=tk.E)
                    try:
                        name = prop
                        value = str(getattr(self.device, name))
                        entry.insert(0, value)
                        label['text'] = name
                    except AttributeError as err:
                        ErrorPopup(self, err, self.logger)

                    try:
                        self.property_entries[i] = (name, entry, value, label)
                    except IndexError:
                        self.property_entries.append((name, entry, value, label))

                    row += 1
                    i += 1
            # Hide any widgets no longer in use
            while i < len(self.property_entries):
                (old_name, entry, old_value, label) =  self.property_entries[i]
                entry.grid_forget()
                label.grid_forget()
                i += 1

        def set_properties(self):
            self.logger.debug('Got set properties. Here are the entries:')
            for name, entry, old_value, label in self.property_entries:
                new_value = entry.get()
                if not new_value == old_value:
                    if name in ('sigma_mode', 'bg_subtract_mode'):
                        parsed = new_value
                    else:
                        try:
                            parsed = parse_expression(new_value)
                        except Exception as err:
                            parsed = None
                            ErrorPopup(self, err, self.logger)
#                    if parsed is not None:
                    try:
                        setattr(self.device, name, parsed)
                        self.logger.debug('Set ' + str(name) + ' to: ' + str(parsed))
                    except Exception as err:
                        ErrorPopup(self, err, self.logger)
                else:
                    self.logger.debug('Did not change: ' + str(name))
            self.update()


class TrackingControlFrame(ttk.Frame):
    """Extends tkinter.Frame for starting and stopping tracking"""
    def __init__(self, master, pypogs_system, logger):
        self.logger = logger
        self.logger.debug('Creating TrackingControlFrame')
        super().__init__(master)
        self.sys = pypogs_system
        self._update_stop = True
        self._update_after = 1000
        
        self.logger.debug('Creating label and buttons')

        ttk.Label(self, text='Tracking').pack(fill=tk.BOTH, expand=True)
        tk.Frame(self, height=1, bg='gray50').pack(fill=tk.BOTH, expand=True)

        input_frame = ttk.Frame(self)
        input_frame.pack(fill=tk.BOTH, expand=True)
        self.logger.debug('Creating checkboxes')
        self.enable_ccl = tk.BooleanVar()
        self.enable_ctfsp = tk.BooleanVar()
        self.enable_fcl = tk.BooleanVar()
        mode_frame = ttk.Frame(input_frame)
        input_frame.columnconfigure((0,1), weight=1)
        input_frame.rowconfigure(0, weight=1)
        mode_frame.grid(row=0, column=0, sticky=tk.E+tk.W+tk.N+tk.E, padx=(10,0))
        ttk.Label(mode_frame, text='Allow Modes:').grid(sticky=tk.W)
        ttk.Checkbutton(mode_frame, text='CCL', variable=self.enable_ccl, command=self.enable_callback) \
                                                                                            .grid(sticky=tk.W)
        ttk.Checkbutton(mode_frame, text='CTFSP', variable=self.enable_ctfsp, command=self.enable_callback) \
                                                                                            .grid(sticky=tk.W)
        ttk.Checkbutton(mode_frame, text='FCL', variable=self.enable_fcl, command=self.enable_callback) \
                                                                                            .grid(sticky=tk.W)
        self.auto_coarse = tk.BooleanVar()
        self.auto_fine = tk.BooleanVar()
        auto_frame = ttk.Frame(input_frame)
        auto_frame.grid(row=0, column=1, sticky=tk.E+tk.W+tk.N+tk.E, padx=(10,0))
        ttk.Label(auto_frame, text='Auto Acquire:').grid(sticky=tk.W)
        ttk.Checkbutton(auto_frame, text='Coarse', variable=self.auto_coarse, command=self.coarse_callback) \
                                                                                            .grid(sticky=tk.W)
        ttk.Checkbutton(auto_frame, text='Fine', variable=self.auto_fine, command=self.fine_callback) \
                                                                                            .grid(sticky=tk.W)
        ttk.Button(self, text='Start Tracking', command=self.start_tracking_callback, width=12) \
                                                                .pack(fill=tk.BOTH, expand=True)
        ttk.Button(self, text='Stop Tracking', command=self.stop_tracking_callback, width=12) \
                                                                .pack(fill=tk.BOTH, expand=True)
        self.update()

    def update(self):
        self.logger.debug('TrackingControlFrame got update request')
        self.enable_ccl.set(self.sys.control_loop_thread.CCL_enable)
        self.enable_ctfsp.set(self.sys.control_loop_thread.CTFSP_enable)
        self.enable_fcl.set(self.sys.control_loop_thread.FCL_enable)
        if self.sys.coarse_track_thread is not None:
            self.auto_coarse.set(self.sys.coarse_track_thread.auto_acquire_track)
        else:
            self.auto_coarse.set(False)
        if self.sys.fine_track_thread is not None:
            self.auto_fine.set(self.sys.fine_track_thread.auto_acquire_track)
        else:
            self.auto_fine.set(False)
        if not self._update_stop:
            self.logger.debug('TrackingControlFrame updating self after {} ms'.format(self._update_after))
            self.after(self._update_after, self.update)

    def coarse_callback(self):
        try:
            assert self.sys.coarse_track_thread is not None, 'No coarse tracker (requires mount and coarse camera).'
            self.sys.coarse_track_thread.auto_acquire_track = self.auto_coarse.get()
        except Exception as err:
            self.logger.debug('Could not set coarse to auto', exc_info=True)
            ErrorPopup(self, err, self.logger)

    def fine_callback(self):
        try:
            assert self.sys.fine_track_thread is not None, 'No fine tracker (requires mount and fine camera).'
            self.sys.fine_track_thread.auto_acquire_track = self.auto_fine.get()
        except Exception as err:
            self.logger.debug('Could not set fine to auto', exc_info=True)
            ErrorPopup(self, err, self.logger)

    def enable_callback(self):
        try:
            self.sys.control_loop_thread.CCL_enable = self.enable_ccl.get()
        except Exception as err:
            self.logger.debug('Could not set CCL enable', exc_info=True)
            ErrorPopup(self, err, self.logger)
        try:
            self.sys.control_loop_thread.CTFSP_enable = self.enable_ctfsp.get()
        except Exception as err:
            self.logger.debug('Could not set CTFSP enable', exc_info=True)
            ErrorPopup(self, err, self.logger)
        try:
            self.sys.control_loop_thread.FCL_enable = self.enable_fcl.get()
        except Exception as err:
            self.logger.debug('Could not set FCL enable', exc_info=True)
            ErrorPopup(self, err, self.logger)
#        self.update()

    def start_tracking_callback(self):
        if self.sys.mount is not None and self.sys.mount.is_init and self.sys.mount._is_sidereal_tracking:
            self.logger.debug('Sidereal tracking is on.  Will turn off.')
            self.sys.mount.stop_sidereal_tracking()
        try:
            self.sys.start_tracking()
        except Exception as err:
            self.logger.debug('Did not start tracking', exc_info=True)
            ErrorPopup(self, err, self.logger)
    def stop_tracking_callback(self):
        self.logger.debug('TrackingControlFrame got stop request')
        try:
            self.sys.stop()
        except Exception as err:
            self.logger.debug('Did not stop', exc_info=True)
            ErrorPopup(self, err, self.logger)

    def start(self, after=None):
        """Give number of milliseconds to wait between updates."""
        self.logger.debug('TrackingControlFrame got start request with after={}'.format(after))
        if after is not None: self._update_after = after
        self._update_stop = False
        self.update()

    def stop(self):
        """Stop updating."""
        self.logger.debug('TrackingControlFrame got stop request')
        self._update_stop = True


class LiveViewFrame(ttk.Frame):
    """Extends tkinter.Frame for showing live camera images"""
    def __init__(self, master, pypogs_system, logger):
        self.logger = logger
        self.logger.debug('Creating LiveViewFrame')
        super().__init__(master)
        self.sys = pypogs_system
        self._update_after = 1000
        self._update_stop = True

        self.logger.debug('Creating Frames')
        self.image_frame = ttk.Frame(self)
        self.image_frame.grid(row=1, column=0, sticky=tk.W+tk.E+tk.N+tk.S)
        self.top_frame = ttk.Frame(self)
        self.top_frame.grid(row=0, column=0)
        self.bottom_frame1 = ttk.Frame(self)
        self.bottom_frame1.grid(row=2, column=0, pady=(5,0))
        self.bottom_frame2 = ttk.Frame(self)
        self.bottom_frame2.grid(row=3, column=0, pady=(5,0))

        self.logger.debug('Filling top frame with label')
        ttk.Label(self.top_frame, text='Live View and Interactive Control').grid(row=0, column=0, padx=(250,0))
        self.zoom_variable = tk.IntVar()
        self.zoom_variable.set(1)
        ttk.Label(self.top_frame, text='Zoom:').grid(row=0, column=1, padx=(50,0))
        ttk.Radiobutton(self.top_frame, text='1x', variable=self.zoom_variable, value=1) \
                .grid(row=0, column=2, padx=(5,0))
        ttk.Radiobutton(self.top_frame, text='2x', variable=self.zoom_variable, value=2) \
                .grid(row=0, column=3, padx=(5,0))
        ttk.Radiobutton(self.top_frame, text='4x', variable=self.zoom_variable, value=4) \
                .grid(row=0, column=4, padx=(5,0))
        ttk.Radiobutton(self.top_frame, text='8x', variable=self.zoom_variable, value=8) \
                .grid(row=0, column=5, padx=(5,0))

        self.logger.debug('Creating image canvas')
        self.canvas_size = (800, 640)
        self.image_size = self.canvas_size
        self.image_scale = 1
        self.canvas = tk.Canvas(self.image_frame, width=self.canvas_size[0], height=self.canvas_size[1], bg='black')
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.click_canvas_callback)
        self.tk_image = None
        self.canvas_image = None
        self.logger.debug('Creating radiobuttons for camera selection')

        # Top row under live view
        self.logger.debug('Filling bottom frame with interactive controls')
        ttk.Label(self.bottom_frame1, text='Camera (Tracker):').grid(row=0, column=0)
        self.camera_variable = tk.IntVar()
        self.camera_variable.set(NONE)
        ttk.Radiobutton(self.bottom_frame1, text='None', variable=self.camera_variable, value=NONE) \
                .grid(row=0, column=1, padx=(5,0))
        ttk.Radiobutton(self.bottom_frame1, text='Star (OL)', variable=self.camera_variable, value=STAR_OL) \
                .grid(row=0, column=2, padx=(5,0))
        ttk.Radiobutton(self.bottom_frame1, text='Coarse (CCL)', variable=self.camera_variable, value=COARSE_CCL) \
                .grid(row=0, column=4, padx=(5,0))
        ttk.Radiobutton(self.bottom_frame1, text='Fine (FCL)', variable=self.camera_variable, value=FINE_FCL) \
                .grid(row=0, column=5, padx=(5,0))
        self.logger.debug('Creating entry and checkbox for image max value control')
        ttk.Label(self.bottom_frame1, text='Max Value:').grid(row=0, column=6, padx=(20,0))
        self.max_entry = ttk.Entry(self.bottom_frame1, width=10)
        self.max_entry.grid(row=0, column=7, padx=(5,0))
        self.auto_max_variable = tk.BooleanVar()
        self.auto_max_variable.set(True)
        ttk.Checkbutton(self.bottom_frame1, variable=self.auto_max_variable, text='Auto')\
                                                                            .grid(row=0, column=8, padx=(5,0))

        self.logger.debug('Creating entry for quick exposure time control')
        ttk.Label(self.bottom_frame1, text='Exposure time:').grid(row=0, column=9, padx=(20,0))
        self.exposure_entry = ttk.Spinbox(self.bottom_frame1, width=10, from_=0, to=10000, increment=.01,
                                          command=lambda: self.exposure_entry_callback(None))
        self.exposure_entry.grid(row=0, column=10, padx=(5,0))
        self.exposure_entry.bind(('<Return>',), self.exposure_entry_callback)

        # Bottom row under live view
        self.set_search_variable = tk.BooleanVar()
        self.set_search_variable.set(False)
        ttk.Checkbutton(self.bottom_frame2, text='Manual Acquire', variable=self.set_search_variable) \
                                                            .grid(row=0, column=1)
        ttk.Button(self.bottom_frame2, text='Clear Track', command=self.clear_tracker_callback) \
                                                            .grid(row=0, column=2, padx=(5,0))

        self.add_offset_variable = tk.BooleanVar()
        self.add_offset_variable.set(False)
        ttk.Checkbutton(self.bottom_frame2, text='Add Offset', variable=self.add_offset_variable) \
                                                            .grid(row=0, column=3, padx=(50,0))
        ttk.Button(self.bottom_frame2, text='Clear Offset', command=self.clear_offset_callback) \
                                                            .grid(row=0, column=4, padx=(5,0))

        self.set_goal_variable = tk.BooleanVar()
        self.set_goal_variable.set(False)
        ttk.Checkbutton(self.bottom_frame2, text='Intercam Alignment', variable=self.set_goal_variable) \
                                                            .grid(row=0, column=5, padx=(50,0))

        self.annotate_variable = tk.BooleanVar()
        self.annotate_variable.set(True)
        self.goal_handles = [None]*4
        self.offset_handles = [None]*4
        self.track_circle_handle = None
        self.search_circle_handle = None
        self.track_cross_handles = [None]*2
        ttk.Checkbutton(self.bottom_frame2, text='Annotate', variable=self.annotate_variable) \
                                                            .grid(row=0, column=6, padx=(50,0))											

        self.logger.debug('Finished creating. Calling update on self')
        self.update()

    def clear_tracker_callback(self):
        """Clear the track and the offset of the current tracker."""
        self.logger.debug('Clicked on clear tracker')
        cam = self.camera_variable.get()
        if cam == STAR_OL:
            self.logger.info('Clearing OL tracker.')
            try: #Only set offset to zero, there is nothing else to do
                self.sys.control_loop_thread.OL_goal_offset_x_y = (0,0)
            except Exception as err:
                self.logger.debug('Could not clear coarse tracker', exc_info=True)
                ErrorPopup(self, err, self.logger)
        elif cam == COARSE_CCL:
            self.logger.info('Clearing CCL tracker.')
            try:
                self.sys.coarse_track_thread.spot_tracker.clear_tracker()
            except Exception as err:
                self.logger.debug('Could not clear coarse tracker', exc_info=True)
                ErrorPopup(self, err, self.logger)
        elif cam == FINE_FCL:
            self.logger.info('Clearing FCL tracker.')
            try:
                self.sys.fine_track_thread.spot_tracker.clear_tracker()
            except Exception as err:
                self.logger.debug('Could not clear fine tracker', exc_info=True)
                ErrorPopup(self, err, self.logger)
        else:
            self.logger.debug('No camera selected in clear tracker callback')

    def clear_offset_callback(self):
        """Clear the offset *of the preceding tracker*."""
        self.logger.debug('Clicked on clear offset')
        cam = self.camera_variable.get()
        if cam == COARSE_CCL or cam == STAR_OL:
            self.logger.info('Clearing OL offset.')
            try:
                self.sys.control_loop_thread.OL_goal_offset_x_y = (0,0)
            except Exception as err:
                self.logger.debug('Could not set OL offset', exc_info=True)
                ErrorPopup(self, err, self.logger)
        elif cam == FINE_FCL:
            self.logger.info('Clearing CCL offset.')
            try:
                self.sys.coarse_track_thread.goal_offset_x_y = np.array((0,0))
            except Exception as err:
                self.logger.debug('Could not set CCL offset', exc_info=True)
                ErrorPopup(self, err, self.logger)

    def exposure_entry_callback(self, event):
        """User requested a new exposure time"""
        self.logger.info('exposure_entry_callback, event: ' + str(event))
        old_exposure = None
        updated_value = None
        new_value = None
        if event is None: # Clicked increment or decrement button
            entry_value = float(self.exposure_entry.get())
            # Figure out the camera
            cam = self.camera_variable.get()
            self.logger.debug('Incrementing exposure for camera: ' + str(cam))
            if cam == STAR_OL:
                old_exposure = self.sys.star_camera.exposure_time
                new_value = round(old_exposure*(1/1.26 if entry_value < old_exposure else 1.26), 2)
                self.sys.star_camera.exposure_time = new_value
                updated_value = self.sys.star_camera.exposure_time
            elif cam == COARSE_CCL:
                old_exposure = self.sys.coarse_camera.exposure_time
                new_value = round(old_exposure*(1/1.26 if entry_value < old_exposure else 1.26), 2)
                self.sys.coarse_camera.exposure_time = new_value
                updated_value = self.sys.coarse_camera.exposure_time
            elif cam == FINE_FCL:
                old_exposure = self.sys.fine_camera.exposure_time
                new_value = round(old_exposure*(1/1.26 if entry_value < old_exposure else 1.26), 2)
                self.sys.fine_camera.exposure_time = new_value
                updated_value = self.sys.fine_camera.exposure_time

        elif event: # Hit enter
            cam = self.camera_variable.get()
            new_value = self.exposure_entry.get()
            self.logger.debug('Hit enter on exposure entry, set to camera ' + str(cam) + ' value ' + str(new_value))
            if cam == STAR_OL:
                self.sys.star_camera.exposure_time = new_value
                updated_value = self.sys.star_camera.exposure_time
            elif cam == COARSE_CCL:
                self.sys.coarse_camera.exposure_time = new_value
                updated_value = self.sys.coarse_camera.exposure_time
            elif cam == FINE_FCL:
                self.sys.fine_camera.exposure_time = new_value
                updated_value = self.sys.fine_camera.exposure_time
        # Update box afterwords
        if updated_value: 
            self.logger.debug('Setting exposure time box to ' + str(updated_value))
            self.exposure_entry.delete(0, 'end')
            self.exposure_entry.insert(0, updated_value)
            self.logger.debug('exposure_entry_callback succeeded (%s)' % str([event, old_exposure, new_value, updated_value]))
        elif old_exposure:
            self.logger.debug('exposure_entry_callback failed (%s)' % str([event, old_exposure, new_value, updated_value]))
            self.exposure_entry.delete(0, 'end')
            self.exposure_entry.insert(0, old_exposure)
        else:
            self.logger.warning('exposure invalid')

    def click_canvas_callback(self, event):
        self.logger.debug('Canvas click callback')
        if event.x > self.image_size[0] or event.y > self.image_size[1]:
            self.logger.debug('Outside image')
            x_image = None
            y_image = None
        else:
            x_image = event.x - self.image_size[0] / 2
            y_image = self.image_size[1] / 2 - event.y #Camera coordinates are opposite
            self.logger.debug('Image coordinates x:' + str(x_image) + ' y:' + str(y_image))
        if self.set_goal_variable.get() and not None in (x_image, y_image):
            cam = self.camera_variable.get()
            if cam == STAR_OL:
                self.logger.debug('Setting goal for OL tracker from Star camera')
                try:
                    plate_scale = self.sys.star_camera.plate_scale
                    goal = [x_image / self.image_scale * plate_scale, y_image / self.image_scale * plate_scale]
                    self.logger.info('Setting OL goal to: ' + str(goal))
                    self.sys.control_loop_thread.OL_goal_x_y = goal
                except Exception as err:
                    self.logger.debug('Could not set OL goal', exc_info=True)
                    ErrorPopup(self, err, self.logger)
            elif cam == COARSE_CCL:
                self.logger.debug('Setting goal for CCL tracker from Coarse camera')
                try:
                    plate_scale = self.sys.coarse_camera.plate_scale
                    goal = [x_image / self.image_scale * plate_scale, y_image / self.image_scale * plate_scale]
                    self.logger.info('Setting coarse goal to: ' + str(goal))
                    self.sys.coarse_track_thread.goal_x_y = goal
                except Exception as err:
                    self.logger.debug('Could not set CCL goal', exc_info=True)
                    ErrorPopup(self, err, self.logger)
            elif cam == FINE_FCL:
                self.logger.debug('Setting goal for FCL tracker from Fine camera')
                try:
                    plate_scale = self.sys.fine_camera.plate_scale
                    goal = [x_image / self.image_scale * plate_scale, y_image / self.image_scale * plate_scale]
                    self.logger.info('Setting fine goal to: ' + str(goal))
                    self.sys.fine_track_thread.goal_x_y = goal
                except Exception as err:
                    self.logger.debug('Could not set FCL goal', exc_info=True)
                    ErrorPopup(self, err, self.logger)
            else:
                self.logger.debug('No camera selected in canvas click callback')
            self.set_goal_variable.set(False)
        elif self.add_offset_variable.get() and not None in (x_image, y_image):
            cam = self.camera_variable.get()
            if cam == STAR_OL:
                self.logger.debug('Setting offset for OL tracker from Coarse camera')
                try:
                    plate_scale = self.sys.star_camera.plate_scale
                    rotation = self.sys.star_camera.rotation
                    click = [x_image / self.image_scale * plate_scale, y_image / self.image_scale * plate_scale]
                    offset = np.array(click) - np.array(self.sys.coarse_track_thread.goal_x_y)
                    rotmx = np.array([[np.cos(np.deg2rad(rotation)), np.sin(np.deg2rad(rotation))],
                                  [-np.sin(np.deg2rad(rotation)), np.cos(np.deg2rad(rotation))]])
                    offset = rotmx @ offset
                    self.logger.info('Subtracting from OL offset: ' + str(offset))
                    old_offset = self.sys.control_loop_thread.OL_goal_offset_x_y
                    self.sys.control_loop_thread.OL_goal_offset_x_y = np.array(old_offset) - offset
                except Exception as err:
                    self.logger.debug('Could not set OL offset', exc_info=True)
                    ErrorPopup(self, err, self.logger)
            elif cam == COARSE_CCL:
                self.logger.debug('Setting offset for OL tracker from Coarse camera')
                try:
                    plate_scale = self.sys.coarse_camera.plate_scale
                    rotation = self.sys.coarse_camera.rotation
                    click = [x_image / self.image_scale * plate_scale, y_image / self.image_scale * plate_scale]
                    offset = np.array(click) - np.array(self.sys.coarse_track_thread.goal_x_y)
                    rotmx = np.array([[np.cos(np.deg2rad(rotation)), np.sin(np.deg2rad(rotation))],
                                  [-np.sin(np.deg2rad(rotation)), np.cos(np.deg2rad(rotation))]])
                    offset = rotmx @ offset
                    self.logger.info('Subtracting from OL offset: ' + str(offset))
                    old_offset = self.sys.control_loop_thread.OL_goal_offset_x_y
                    self.sys.control_loop_thread.OL_goal_offset_x_y = np.array(old_offset) - offset
                except Exception as err:
                    self.logger.debug('Could not set OL offset', exc_info=True)
                    ErrorPopup(self, err, self.logger)
            elif cam == FINE_FCL:
                self.logger.debug('Setting offset for CCL tracker from Fine camera')
                try:
                    plate_scale = self.sys.fine_camera.plate_scale
                    rotation_fine = self.sys.fine_camera.rotation
                    rotation_coarse = self.sys.coarse_camera.rotation
                    click = [x_image / self.image_scale * plate_scale, y_image / self.image_scale * plate_scale]
                    offset = np.array(click) - np.array(self.sys.fine_track_thread.goal_x_y)
                    rotmx1 = np.array([[np.cos(np.deg2rad(rotation_fine)), np.sin(np.deg2rad(rotation_fine))],
                                  [-np.sin(np.deg2rad(rotation_fine)), np.cos(np.deg2rad(rotation_fine))]])
                    rotmx2 = np.array([[np.cos(np.deg2rad(rotation_coarse)), -np.sin(np.deg2rad(rotation_coarse))],
                                  [np.sin(np.deg2rad(rotation_coarse)), np.cos(np.deg2rad(rotation_coarse))]])
                    offset = rotmx2 @ (rotmx1 @ offset)
                    self.logger.info('Subtracting from CCL offset: ' + str(offset))
                    old_offset = self.sys.coarse_track_thread.goal_offset_x_y
                    self.sys.coarse_track_thread.goal_offset_x_y = np.array(old_offset) - offset
                except Exception as err:
                    self.logger.debug('Could not set CCL offset', exc_info=True)
                    ErrorPopup(self, err, self.logger)
            else:
                self.logger.debug('No camera selected in canvas click callback')
            self.add_offset_variable.set(False)
        elif self.set_search_variable.get() and not None in (x_image, y_image):
            cam = self.camera_variable.get()
            if cam == COARSE_CCL:
                self.logger.debug('Setting search pos for CCL')
                plate_scale = self.sys.coarse_camera.plate_scale
                search_pos = [x_image / self.image_scale * plate_scale, y_image / self.image_scale * plate_scale]
                self.logger.info('Setting coarse search position to: ' + str(search_pos))
                self.sys.coarse_track_thread.pos_search_x_y = search_pos
            elif cam == FINE_FCL:
                self.logger.debug('Setting search pos for CCL')
                plate_scale = self.sys.fine_camera.plate_scale
                search_pos = [x_image / self.image_scale * plate_scale, y_image / self.image_scale * plate_scale]
                self.logger.info('Setting fine search position to: ' + str(search_pos))
                self.sys.fine_track_thread.pos_search_x_y = search_pos
            else:
                self.logger.error('No camera selected in canvas click callback')
            self.set_search_variable.set(False)

    def update(self):
        """Update the canvas with an image"""
        self.logger.debug('LiveViewFrame got update request')
        # Read desired camera
        cam = self.camera_variable.get()
        self.logger.debug('Selected camera is {} (1 star, 2 coarse, 3 fine, 0 none)'.format(cam))
        # Check if we have it, otherwise switch to none
        if cam == STAR_OL and (self.sys.star_camera is None or not self.sys.star_camera.is_init): cam = NONE
        if cam == COARSE_CCL and (self.sys.coarse_camera is None or not self.sys.coarse_camera.is_init): cam = NONE
        if cam == FINE_FCL and (self.sys.fine_camera is None or not self.sys.fine_camera.is_init): cam = NONE
        self.logger.debug('After validity checks setting camera to {}'.format(cam))
        self.camera_variable.set(cam)
        img = None
        goal_pos = (None, None)
        offset_pos = (None, None)
        plate_scale = None
        rotation = None
        mean_pos = (None, None)
        track_sd = None
        track_pos = (None, None)
        search_pos = (None, None)
        search_rad = None
        exposure = None
        if cam == STAR_OL:
            self.logger.debug('Trying to get star OL data')
            try:
                if not self.sys.star_camera.is_running: self.sys.star_camera.start()
                img = self.sys.star_camera.get_latest_image()
                plate_scale = self.sys.star_camera.plate_scale
                rotation = self.sys.star_camera.rotation
                goal_pos = self.sys.control_loop_thread.OL_goal_x_y
                exposure = self.sys.star_camera.exposure_time
                try:
                    offset_pos = np.array(goal_pos) + np.array(self.sys.control_loop_thread.OL_goal_offset_x_y)
                except:
                    self.logger.debug('Could not get offset', exc_info=True)
            except:
                self.logger.debug('Failed', exc_info=True)
        elif cam == COARSE_CCL:
            self.logger.debug('Trying to get coarse CCL data')
            try:
                if not self.sys.coarse_camera.is_running: self.sys.coarse_camera.start()
                img = self.sys.coarse_camera.get_latest_image()
                goal_pos = self.sys.coarse_track_thread.goal_x_y
                try:
                    offset_pos = np.array(goal_pos) + np.array(self.sys.coarse_track_thread.goal_offset_x_y)
                except:
                    self.logger.debug('Could not get offset', exc_info=True)
                plate_scale = self.sys.coarse_camera.plate_scale
                rotation = self.sys.coarse_camera.rotation
                mean_pos = self.sys.coarse_track_thread.mean_x_y_absolute
                track_sd = self.sys.coarse_track_thread.track_sd
                track_pos = self.sys.coarse_track_thread.track_x_y_absolute
                search_pos = self.sys.coarse_track_thread.pos_search_x_y
                search_rad = self.sys.coarse_track_thread.pos_search_rad
                exposure = self.sys.coarse_camera.exposure_time
            except:
                self.logger.debug('Failed', exc_info=True)
        elif cam == FINE_FCL:
            self.logger.debug('Trying to get fine FCL data')
            try:
                if not self.sys.fine_camera.is_running: self.sys.fine_camera.start()
                img = self.sys.fine_camera.get_latest_image()
                goal_pos = self.sys.fine_track_thread.goal_x_y
                try:
                    offset_pos = np.array(goal_pos) + np.array(self.sys.fine_track_thread.goal_offset_x_y)
                except:
                    self.logger.debug('Could not get offset', exc_info=True)
                plate_scale = self.sys.fine_camera.plate_scale
                rotation = self.sys.fine_camera.rotation
                mean_pos = self.sys.fine_track_thread.mean_x_y_absolute
                track_sd = self.sys.fine_track_thread.track_sd
                track_pos = self.sys.fine_track_thread.track_x_y_absolute
                search_pos = self.sys.fine_track_thread.pos_search_x_y
                search_rad = self.sys.fine_track_thread.pos_search_rad
                exposure = self.sys.fine_camera.exposure_time
            except:
                self.logger.debug('Failed', exc_info=True)

        if img is not None:
            zoom = self.zoom_variable.get()
            (height, width) = img.shape[0:2]
            offs_x = round(width / 2 * (1 - 1/zoom))
            offs_y = round(height / 2 * (1 - 1/zoom))
            width = width//zoom
            height = height//zoom
            try:
                img = img[offs_y:offs_y+height, offs_x:offs_x+width]
            except:
                self.logger.warning('Failed to zoom image')

        #self.logger.debug('Setting image to: ' + str(img))
        if img is not None:
            if self.auto_max_variable.get(): #Auto set max scaling
                maxval = int(np.max(img))
                self.max_entry.delete(0, 'end')
                self.max_entry.insert(0, str(maxval))
                self.logger.debug('Using auto max scaling with maxval {}'.format(maxval))
            else:
                try:
                    maxval = int(self.max_entry.get())
                    self.logger.debug('Using manual maxval {}'.format(maxval))
                except:
                    self.logger.debug('Failed to convert max entry value {} to int'\
                                       .format(self.max_entry.get()))
                    maxval = 255
            # Scale and convert to 8-bit
            img_scaled = (img / max(1, int(.8 * maxval / 255))).clip(0, 255).astype('uint8')
            pil_img = Image.fromarray(img_scaled)
            # Resize to fit inside canvas
            (in_width, in_height) = pil_img.size
            self.image_scale = min(self.canvas_size[0]/in_width, self.canvas_size[1]/in_height)
            self.image_size = (round(self.image_scale*in_width), round(self.image_scale*in_height))
            self.logger.debug('Resizing image to: ' + str(self.image_size))
            pil_img = pil_img.resize(self.image_size, resample=Image.NEAREST)
            # Set canvas image (must keep reference to image, otherwise will be garbage collected)
            self.tk_image = ImageTk.PhotoImage(image=pil_img)
            if self.canvas_image is None:
                self.logger.debug('Creating image on canvas')
                self.canvas_image = self.canvas.create_image(0, 0, image=self.tk_image, anchor=tk.NW)
            else:
                self.canvas.itemconfig(self.canvas_image, image=self.tk_image)
                self.logger.debug('Image was updated')

            if self.annotate_variable.get() and not None in (goal_pos[0], goal_pos[1], plate_scale):
                self.logger.debug('Annotating Goal: ' + str(goal_pos) + ' scale: ' + str(plate_scale))
                gap = 5
                length = 30
                width = 4
                xhair_x = goal_pos[0] / plate_scale * self.image_scale + self.image_size[0] / 2
                xhair_y = -goal_pos[1] / plate_scale * self.image_scale + self.image_size[1] / 2
                self.logger.debug('Crosshair goes at: ' + str((xhair_x, xhair_y)))
                xhair = np.array([[0, -gap],[0, -gap-length-2],
                                  [0, gap], [0, gap+length],
                                  [gap, 0], [gap+length+2, 0],
                                  [-gap, 0], [-gap-length, 0]]).transpose()
                rotmx = np.array([[np.cos(np.deg2rad(rotation)), np.sin(np.deg2rad(rotation))],
                                  [-np.sin(np.deg2rad(rotation)), np.cos(np.deg2rad(rotation))]])
                xhair = (rotmx @ xhair) + np.array([xhair_x, xhair_y]).reshape((2,1))

                if None in self.goal_handles:
                    self.goal_handles[0] = self.canvas.create_line(*xhair[:,0], *xhair[:,1], fill='blue',\
                                                                   width=width, tag='goal', arrow=tk.LAST,\
                                                                   arrowshape=(10,14,6))
                    self.goal_handles[1] = self.canvas.create_line(*xhair[:,2], *xhair[:,3], fill='blue',\
                                                                   width=width, tag='goal')
                    self.goal_handles[2] = self.canvas.create_line(*xhair[:,4], *xhair[:,5], fill='blue',\
                                                                   width=width, tag='goal', arrow=tk.LAST,\
                                                                   arrowshape=(10,14,6))
                    self.goal_handles[3] = self.canvas.create_line(*xhair[:,6], *xhair[:,7], fill='blue',\
                                                                   width=width, tag='goal')
                else:
                    self.canvas.coords(self.goal_handles[0], *xhair[:,0], *xhair[:,1])
                    self.canvas.coords(self.goal_handles[1], *xhair[:,2], *xhair[:,3])
                    self.canvas.coords(self.goal_handles[2], *xhair[:,4], *xhair[:,5])
                    self.canvas.coords(self.goal_handles[3], *xhair[:,6], *xhair[:,7])
                    self.canvas.tag_raise('goal')
            else:
                self.logger.debug('Hiding goal')
                self.canvas.tag_lower('goal')

            if self.annotate_variable.get() and not None in (offset_pos[0], offset_pos[1], plate_scale):
                self.logger.debug('Annotating Offset: ' + str(offset_pos) + ' scale: ' + str(plate_scale))
                gap = 5
                length = 10
                width = 4
                xhair_x = offset_pos[0] / plate_scale * self.image_scale + self.image_size[0] / 2
                xhair_y = -offset_pos[1] / plate_scale * self.image_scale + self.image_size[1] / 2
                self.logger.debug('Crosshair goes at: ' + str((xhair_x, xhair_y)))
                xhair = np.array([[0, -gap],[0, -gap-length],
                                  [0, gap], [0, gap+length],
                                  [gap, 0], [gap+length, 0],
                                  [-gap, 0], [-gap-length, 0]]).transpose()
                rotmx = np.array([[np.cos(np.deg2rad(rotation)), np.sin(np.deg2rad(rotation))],
                                  [-np.sin(np.deg2rad(rotation)), np.cos(np.deg2rad(rotation))]])
                xhair = (rotmx @ xhair) + np.array([xhair_x, xhair_y]).reshape((2,1))
                if None in self.offset_handles:
                    self.offset_handles[0] = self.canvas.create_line(*xhair[:,0], *xhair[:,1], fill='royal blue', \
                                                                       width=width, tag='offset')
                    self.offset_handles[1] = self.canvas.create_line(*xhair[:,2], *xhair[:,3], fill='royal blue', \
                                                                       width=width, tag='offset')
                    self.offset_handles[2] = self.canvas.create_line(*xhair[:,4], *xhair[:,5], fill='royal blue', \
                                                                       width=width, tag='offset')
                    self.offset_handles[3] = self.canvas.create_line(*xhair[:,6], *xhair[:,7], fill='royal blue', \
                                                                       width=width, tag='offset')
                else:
                    self.canvas.coords(self.offset_handles[0], *xhair[:,0], *xhair[:,1])
                    self.canvas.coords(self.offset_handles[1], *xhair[:,2], *xhair[:,3])
                    self.canvas.coords(self.offset_handles[2], *xhair[:,4], *xhair[:,5])
                    self.canvas.coords(self.offset_handles[3], *xhair[:,6], *xhair[:,7])
                    self.canvas.tag_raise('offset')
            else:
                self.logger.debug('Hiding offset')
                self.canvas.tag_lower('offset')

            if self.annotate_variable.get() and not None in (mean_pos[0], mean_pos[1], track_sd, plate_scale):
                self.logger.debug('Annotating Mean: ' + str(mean_pos) + ' scale: ' + str(track_sd))
                canvas_x = mean_pos[0] / plate_scale * self.image_scale + self.image_size[0] / 2 - .5
                canvas_y = -mean_pos[1] / plate_scale * self.image_scale + self.image_size[1] / 2 + .5
                canvas_sd = track_sd / plate_scale * self.image_scale
                coords = (canvas_x-canvas_sd, canvas_y-canvas_sd, canvas_x+canvas_sd, canvas_y+canvas_sd)
                self.logger.debug('Circle goes at: ' + str((canvas_x, canvas_y)) + ' radius: ' + str(canvas_sd))
                colour = 'green'
                if self.track_circle_handle is None:
                    self.track_circle_handle = self.canvas.create_oval(*coords, outline=colour, width=2, tag='mean')
                else:
                    self.canvas.coords(self.track_circle_handle, *coords)
                    self.canvas.tag_raise('mean')
            else:
                self.logger.debug('Hiding mean')
                self.canvas.tag_lower('mean')

            if self.annotate_variable.get() and not None in (search_pos[0], search_pos[1], search_rad, plate_scale):
                self.logger.debug('Annotating Search: ' + str(search_pos) + ' radius: ' + str(search_rad))
                canvas_x = search_pos[0] / plate_scale * self.image_scale + self.image_size[0] / 2 - .5
                canvas_y = -search_pos[1] / plate_scale * self.image_scale + self.image_size[1] / 2 + .5
                canvas_rad = search_rad / plate_scale * self.image_scale
                coords = (canvas_x-canvas_rad, canvas_y-canvas_rad, canvas_x+canvas_rad, canvas_y+canvas_rad)
                self.logger.debug('Circle goes at: ' + str((canvas_x, canvas_y)) + ' radius: ' + str(canvas_rad))
                colour = 'blue'
                if self.search_circle_handle is None:
                    self.search_circle_handle = self.canvas.create_oval(*coords, outline=colour, width=2, tag='search')
                else:
                    self.canvas.coords(self.search_circle_handle, *coords)
                    self.canvas.tag_raise('search')
            else:
                self.logger.debug('Hiding mean')
                self.canvas.tag_lower('search')

            if self.annotate_variable.get() and not None in (track_pos[0], track_pos[1], plate_scale):
                self.logger.debug('Annotating Search: ' + str(track_pos) + ' radius: ' + str(search_rad))
                canvas_x = track_pos[0] / plate_scale * self.image_scale + self.image_size[0] / 2 - .5
                canvas_y = -track_pos[1] / plate_scale * self.image_scale + self.image_size[1] / 2 + .5
                length = 6
                width = 4
                colour = 'green'
                coords_h = (canvas_x-length, canvas_y, canvas_x+length, canvas_y)
                coords_v = (canvas_x, canvas_y-length, canvas_x, canvas_y+length)
                self.logger.debug('Cross goes at: ' + str((canvas_x, canvas_y)))
                if None in self.track_cross_handles:
                    self.track_cross_handles[0] = self.canvas.create_line(coords_h, fill=colour, width=width,\
                                                                          tag='track')
                    self.track_cross_handles[1] = self.canvas.create_line(coords_v, fill=colour, width=width,\
                                                                          tag='track')
                else:
                    self.canvas.coords(self.track_cross_handles[0], coords_h)
                    self.canvas.coords(self.track_cross_handles[1], coords_v)
                    self.canvas.tag_raise('track')
            else:
                self.logger.debug('Hiding mean')
                self.canvas.tag_lower('track')
        else:
            self.logger.debug('Did not get an image to show')
            self.tk_image = ImageTk.PhotoImage(image=Image.new(mode='L', size=self.canvas_size))
            self.image_size = self.canvas_size
            if self.canvas_image is None:
                self.logger.debug('Creating image on canvas')
                self.canvas_image = self.canvas.create_image(0, 0, image=self.tk_image, anchor=tk.NW)
            else:
                self.canvas.itemconfig(self.canvas_image, image=self.tk_image)
                self.logger.debug('Image was updated')
            self.canvas.tag_lower('goal')
            self.canvas.tag_lower('offset')
            self.canvas.tag_lower('mean')
            self.canvas.tag_lower('search')
            self.canvas.tag_lower('track')

        # Update exposure box, unless it is in focus
        if exposure and not self.exposure_entry.instate(('focus',)):
            self.logger.debug('Setting exposure time box to ' + str(exposure))
            self.exposure_entry.delete(0, 'end')
            self.exposure_entry.insert(0, exposure)

        if not self._update_stop:
            self.logger.debug('Calling update on self after {} ms'.format(self._update_after))
            self.after(self._update_after, self.update)

    def start(self, after=None):
        """Give number of milliseconds to wait between updates."""
        self.logger.debug('LiveViewFrame got start request with after={}'.format(after))
        if after is not None: self._update_after = after
        self._update_stop = False
        self.update()

    def stop(self):
        """Stop updating."""
        self.logger.debug('LiveViewFrame got stop request')
        self._update_stop = True


class HardwareFrame(ttk.Frame):
    """Extends tkinter.Frame for controlling System hardware"""
    def __init__(self, master, pypogs_system, logger):
        self.logger = logger
        self.logger.debug('Creating HardwareFrame')
        super().__init__(master)
        self.sys = pypogs_system
        self.logger.debug('Configure button styles')
        ttk.Style().configure('mount.TButton')
        ttk.Style().configure('star.TButton')
        ttk.Style().configure('coarse.TButton')
        ttk.Style().configure('fine.TButton')
        ttk.Style().configure('rec.TButton')
        self.logger.debug('Create label and hardware buttons')
        ttk.Label(self, text='Hardware Setup and Properties').grid(row=0, column=0, columnspan=6, sticky=tk.W+tk.E)
        tk.Frame(self, height=1, bg='gray50').grid(row=1, column=0, columnspan=6, sticky=tk.W+tk.E)
        self.mount_button = ttk.Button(self, text='Mount\n(Not set)', style='mount.TButton', \
                                       command=self.mount_callback, width=12)
        self.mount_button.grid(row=2, column=0)
        self.star_button = ttk.Button(self, text='Star Cam\n(Not set)', style='star.TButton', \
                                      command=self.star_callback, width=12)
        self.star_button.grid(row=2, column=1)
        self.coarse_button = ttk.Button(self, text='Coarse Cam\n(Not set)', style='coarse.TButton', \
                                        command=self.coarse_callback, width=12)
        self.coarse_button.grid(row=2, column=2)
        self.fine_button = ttk.Button(self, text='Fine Cam\n(Not set)', style='fine.TButton', \
                                      command=self.fine_callback, width=12)
        self.fine_button.grid(row=2, column=3)
        self.receiver_button = ttk.Button(self, text='Receiver\n(Not set)', style='rec.TButton', \
                                          command=self.receiver_callback, width=12)
        self.receiver_button.grid(row=2, column=4)
        self.logger.debug('Create init/deinit frame and buttons')
        self.init_frame = ttk.Frame(self)
        self.init_frame.grid(row=3, column=0, columnspan=5, sticky=tk.W+tk.E)
        ttk.Button(self.init_frame, text='Initialize All', command=self.init_callback, width=12) \
                                                                .pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ttk.Button(self.init_frame, text='Deinitialize All', command=self.deinit_callback, width=12) \
                                                                .pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.mount_popup = None
        self.star_popup = None
        self.coarse_popup = None
        self.fine_popup = None
        self.receiver_popup = None

        self.logger.debug('Finished creating. Call update on self')
        self.update() # Set the state

    def update(self):
        """Update the status of all buttons."""
        self.logger.debug('HardwareFrame got update request')
        # Mount
        if self.sys.mount is None:
            ttk.Style().configure('mount.TButton',\
                                 background=ttk.Style().lookup('TButton', 'background'), \
                                 foreground=ttk.Style().lookup('TButton', 'foreground'))
            self.mount_button['text'] = 'Mount\n(None)'
        elif self.sys.mount.is_init:
            ttk.Style().configure('mount.TButton', background='green', foreground='green')
            self.mount_button['text'] = 'Mount\n(Ready)'
        else:
            ttk.Style().configure('mount.TButton', background='red', foreground='red')
            self.mount_button['text'] = 'Mount\n(Not init)'
        # Star cam
        if self.sys.star_camera is None:
            ttk.Style().configure('star.TButton',\
                                  background=ttk.Style().lookup('TButton', 'background'), \
                                  foreground=ttk.Style().lookup('TButton', 'foreground'))
            self.star_button['text'] = 'Star Cam\n(None)'
        elif self.sys.star_camera.is_init:
            ttk.Style().configure('star.TButton', background='green', foreground='green')
            self.star_button['text'] = 'Star Cam\n(Ready)'
        else:
            ttk.Style().configure('star.TButton', background='red', foreground='red')
            self.star_button['text'] = 'Star Cam\n(Not init)'
        # Coarse cam
        if self.sys.coarse_camera is None:
            ttk.Style().configure('coarse.TButton',\
                                  background=ttk.Style().lookup('TButton', 'background'), \
                                  foreground=ttk.Style().lookup('TButton', 'foreground'))
            self.coarse_button['text'] = 'Coarse Cam\n(None)'
        elif self.sys.coarse_camera.is_init:
            ttk.Style().configure('coarse.TButton', background='green', foreground='green')
            self.coarse_button['text'] = 'Coarse Cam\n(Ready)'
        else:
            ttk.Style().configure('coarse.TButton', background='red', foreground='red')
            self.coarse_button['text'] = 'Coarse Cam\n(Not init)'
        # Fine cam
        if self.sys.fine_camera is None:
            ttk.Style().configure('fine.TButton',\
                                  background=ttk.Style().lookup('TButton', 'background'), \
                                  foreground=ttk.Style().lookup('TButton', 'foreground'))
            self.fine_button['text'] = 'Fine Cam\n(None)'
        elif self.sys.fine_camera.is_init:
            ttk.Style().configure('fine.TButton', background='green', foreground='green')
            self.fine_button['text'] = 'Fine Cam\n(Ready)'
        else:
            ttk.Style().configure('fine.TButton', background='red', foreground='red')
            self.fine_button['text'] = 'Fine Cam\n(Not init)'
        # Receiver
        if self.sys.receiver is None:
            ttk.Style().configure('rec.TButton',\
                                  background=ttk.Style().lookup('TButton', 'background'), \
                                  foreground=ttk.Style().lookup('TButton', 'foreground'))
            self.receiver_button['text'] = 'Receiver\n(None)'
        elif self.sys.receiver.is_init:
            ttk.Style().configure('rec.TButton', background='green', foreground='green')
            self.receiver_button['text'] = 'Receiver\n(Ready)'
        else:
            ttk.Style().configure('rec.TButton', background='red', foreground='red')
            self.receiver_button['text'] = 'Receiver\n(Not init)'
        self.logger.debug('Updated all button statuses.')

    def mount_callback(self):
        self.logger.debug('HardwareFrame mount button clicked')
        try:
            if self.mount_popup is None:
                self.mount_popup = self.HardwarePopup(self, 'mount', self.sys.mount, self.sys.add_mount, self.sys.clear_mount, \
                                                      title='Mount', default_name='Mount')
            else:
                self.mount_popup.update()
                self.mount_popup.deiconify()
        except Exception as err:
            self.logger.debug('Could not open mount popup', exc_info=True)
            ErrorPopup(self, err, self.logger)

    def star_callback(self):
        self.logger.debug('HardwareFrame star button clicked')
        try:
            if self.star_popup is None:
                self.star_popup = self.HardwarePopup(self, 'camera', self.sys.star_camera, self.sys.add_star_camera, \
                                                     self.sys.clear_star_camera, title='Star camera', \
                                                     default_name='StarCamera', link_device=self.sys.coarse_camera, \
                                                     link_func=self.sys.add_coarse_camera_from_star)
            else:
                self.star_popup.update()
                self.star_popup.deiconify()
        except Exception as err:
            self.logger.debug('Could not open star popup', exc_info=True)
            ErrorPopup(self, err, self.logger)

    def coarse_callback(self):
        self.logger.debug('HardwareFrame coarse button clicked')
        try:
            if self.coarse_popup is None:
                self.coarse_popup = self.HardwarePopup(self, 'camera', self.sys.coarse_camera, self.sys.add_coarse_camera, \
                                                       self.sys.clear_coarse_camera, title='Coarse camera', \
                                                       default_name='CoarseCamera', link_device=self.sys.star_camera, \
                                                       link_func=self.sys.add_star_camera_from_coarse)
            else:
                self.coarse_popup.update()
                self.coarse_popup.deiconify()
        except Exception as err:
            self.logger.debug('Could not open coarse popup', exc_info=True)
            ErrorPopup(self, err, self.logger)

    def fine_callback(self):
        self.logger.debug('HardwareFrame fine button clicked')
        try:
            if self.fine_popup is None:
                self.fine_popup = self.HardwarePopup(self, 'camera', self.sys.fine_camera, self.sys.add_fine_camera, \
                                                     self.sys.clear_fine_camera, title='Fine camera', \
                                                     default_name='FineCamera')
            else:
                self.fine_popup.update()
                self.fine_popup.deiconify()
        except Exception as err:
            self.logger.debug('Could not open fine popup', exc_info=True)
            ErrorPopup(self, err, self.logger)

    def receiver_callback(self):
        self.logger.debug('HardwareFrame receiver button clicked')
        try:
            if self.receiver_popup is None:
                self.receiver_popup = self.HardwarePopup(self, 'receiver', self.sys.receiver, self.sys.add_receiver, \
                                                     self.sys.clear_receiver, title='Receiver', \
                                                     default_name='UnnamedReceiver')
            else:
                self.receiver_popup.update()
                self.receiver_popup.deiconify()
        except Exception as err:
            self.logger.debug('Could not open receiver popup', exc_info=True)
            ErrorPopup(self, err, self.logger)

    def init_callback(self):
        self.logger.debug('HardwareFrame init button clicked')
        try:
            self.sys.initialize()
            self.logger.debug('System was initialised')
        except Exception as err:
            ErrorPopup(self, err, self.logger)
        self.update()

    def deinit_callback(self):
        self.logger.debug('HardwareFrame deinit button clicked')
        try:
            self.sys.deinitialize()
            self.logger.debug('System was deinitialised')
        except Exception as err:
            ErrorPopup(self, err, self.logger)
        self.update()

    class HardwarePopup(tk.Toplevel):
        """Extends tk.Toplevel for hardware settings

        For star/coarse camera, pass the other one in link_device to get the option to join them.
        """
        def __init__(self, master, device_type, device, add_func, clear_func, link_device=0, link_func=0, properties_frame=None, \
                     title='Hardware', default_name=''):
            super().__init__(master, padx=10, pady=10, bg=ttk.Style().lookup('TFrame', 'background'))
            self.logger = master.logger
            self.title(title)
            self.resizable(False, False)
            self.device = device
            self.add_func = add_func
            self.clear_func = clear_func
            self.link_device = link_device
            self.link_func = link_func
            self.default_name = default_name
            
            setup_frame = ttk.Frame(self)
            setup_frame.grid(row=0, column=0, sticky=tk.S)
            self.linked_bool = tk.BooleanVar()
            r = 0
            if link_device != 0:
                ttk.Checkbutton(setup_frame, text='Link Star and Coarse', variable=self.linked_bool) \
                                                                .grid(row=r, column=0)
                r+=1

            ttk.Label(setup_frame, text='Model:').grid(row=r, column=0); r+=1
            self.model_combo = ttk.Combobox(setup_frame, values=master.sys._supported_models[device_type])
            self.model_combo.grid(row=r, column=0); r+=1
            self.model_combo.set(device.model if device and device.model else master.sys._default_model[device_type] or '')
            ttk.Label(setup_frame, text='Identity:').grid(row=r, column=0); r+=1
            self.identity_entry = ttk.Entry(setup_frame, width=20)
            self.identity_entry.grid(row=r, column=0); r+=1
            self.identity_entry.insert(0, device.identity if device and device.identity else '')
            ttk.Label(setup_frame, text='Name:').grid(row=r, column=0); r+=1
            self.name_entry = ttk.Entry(setup_frame, width=20)
            self.name_entry.grid(row=r, column=0); r+=1
            ttk.Button(setup_frame, text='Add', command=self.connect_callback) \
                                                .grid(row=r, column=0, sticky=tk.W+tk.E); r+=1
            ttk.Button(setup_frame, text='Remove', command=self.clear_callback) \
                                                .grid(row=r, column=0, sticky=tk.W+tk.E); r+=1

            self.properties_frame = ttk.Frame(self)
            self.property_entries = []
            self.properties_label = ttk.Label(self.properties_frame, text='Device properties:')
            self.set_prop_button = ttk.Button(self.properties_frame, text='Set', command=self.set_properties)
            self.protocol('WM_DELETE_WINDOW', self.withdraw)
            self.update()

        def update(self):
            self.logger.debug('HardwarePopup got update request')
            #self.model_entry.delete(0, 'end')
            #self.identity_entry.delete(0, 'end')
            #self.name_entry.delete(0, 'end')
            if self.device is None:
                #model = ''
                #identity = ''
                name = self.default_name
            else:
                model = self.device.model
                if model is None: model = ''
                identity = self.device.identity
                if identity is None: identity = ''
                name = self.device.name
                #self.update_properties()
                self.identity_entry.delete(0, tk.END)
                self.identity_entry.insert(0, identity)
            self.name_entry.delete(0, tk.END)
            self.name_entry.insert(0, name)
            self.linked_bool.set(self.device is not None and self.device is self.link_device)
            self.master.update()   
            self.update_properties()

        def clear_callback(self):
            self.logger.debug('HardwarePopup clear button clicked')
            self.clear_func()
            self.device = None
            self.update()

        def connect_callback(self):
            self.logger.debug('HardwarePopup connect button clicked')
            # Read the entries
            model = self.model_combo.get()
            if not model.strip(): model = None
            identity = self.identity_entry.get()
            if not identity.strip(): identity = None
            name = self.name_entry.get()
            if not name.strip(): name = None
            self.logger.debug(str(model)+str(identity)+str(name))
            try:
                self.device = self.add_func(model=model, identity=identity, name=name)
            except Exception as err:
                ErrorPopup(self, err, self.logger)
            if self.linked_bool.get():
                self.logger.debug('Will also set the other device!')
                try:
                    self.link_device = self.link_func()
                except Exception as err:
                    ErrorPopup(self, err, self.logger)
            self.update()

        def update_properties(self):
            try:
                property_list = self.device.available_properties
            except (AssertionError, AttributeError):
                property_list = None
                self.logger.debug('Got no properties to list', exc_info=True)
            i = 0
            if property_list is not None:
                row = 1
                column = 1
                maxrows = 6
                for prop in property_list:
                    if row > maxrows:
                        row = 1
                        column += 2
                    try:
                        (old_name, entry, old_value, label) =  self.property_entries[i]
                        entry.delete(0, 'end')
                        label['text'] = ''
                    except IndexError:
                        entry = ttk.Entry(self.properties_frame, width=20)
                        label = ttk.Label(self.properties_frame, text='')
                    entry.grid(row=row, column=column+1)
                    label.grid(row=row, column=column, sticky=tk.E)
                    try:
                        name = prop
                        value = str(getattr(self.device, name))
                        entry.insert(0, value)
                        label['text'] = name
                    except AttributeError as err:
                        ErrorPopup(self, err, self.logger)
                    try:
                        self.property_entries[i] = (name, entry, value, label)
                    except IndexError:
                        self.property_entries.append((name, entry, value, label))
                    row += 1
                    i += 1
                self.properties_label.grid(row=0, column=1, columnspan=column+1)
                self.set_prop_button.grid(row=maxrows+1, column=1, columnspan=column+1, sticky=tk.W+tk.E)
                self.properties_frame.grid(row=0, column=1, padx=(20,0), sticky=tk.S)
            else:
                self.properties_frame.grid_forget() #Hide entirely from the popup
            # Hide any widgets no longer in use
            while i < len(self.property_entries):
                (old_name, entry, old_value, label) =  self.property_entries[i]
                entry.grid_forget()
                label.grid_forget()
                i += 1

        def set_properties(self):
            self.logger.debug('Got set properties. Here are the entries:')
            for name, entry, old_value, label in self.property_entries:
                new_value = entry.get()
                if not new_value == old_value:
                    try:
                        setattr(self.device, name, parse_expression(new_value))
                        self.logger.debug('Set ' + str(name) + ' to: ' + str(parse_expression(new_value)))
                    except Exception as err:
                        ErrorPopup(self, err, self.logger)
                else:
                    self.logger.debug('Did not change: ' + str(name))
            self.update()


class TargetFrame(ttk.Frame):
    """Extends tkinter.Frame for controlling System.target"""
    def __init__(self, master, pypogs_system, logger):
        self.logger = logger
        self.logger.debug('Creating TargetFrame')
        super().__init__(master)
        self.sys = pypogs_system
        self._update_after = 1000
        self._update_stop = True
        # Create widgets and layout        
        #ttk.Label(self, text='Controller Properties').pack(fill=tk.BOTH, expand=True)
        #tk.Frame(self, height=1, bg='gray50').pack(fill=tk.BOTH, expand=True)
        #ttk.Button(self, text='Feedback', command=self.controller_callback, width=12) \
        #                                                        .pack(fill=tk.BOTH, expand=True)

        
        ttk.Label(self, text='Target').pack(fill=tk.BOTH, expand=True)
        tk.Frame(self, height=1, bg='gray50').pack(fill=tk.BOTH, expand=True)
        self.status_label = ttk.Label(self, font='TkFixedFont')
        self.status_label.pack(fill=tk.BOTH, expand=True)
        ttk.Button(self, text='Set Target', command=self.manual_button_callback, width=15).pack(fill=tk.BOTH, expand=True)
        ttk.Button(self, text='Go To Target', command=self.go_to_target_callback).pack(fill=tk.BOTH, expand=True)
        self.manual_popup = None
        self.update()

    def update(self):
        #self.logger.debug('TargetFrame got update request')
        """Update the target status"""
        target_string = self.sys.target.get_short_string()
        t_start = self.sys.target.start_time
        start_string = ('Start: ' + t_start.strftime('%H:%M:%S')) if t_start is not None else 'No start time'
        t_end = self.sys.target.end_time
        end_string = ('End: ' + t_end.strftime('%H:%M:%S')) if t_end is not None else 'No end time'
        try:
            itrf_xyz = self.sys.get_itrf_direction_of_target()
            enu_altaz = self.sys.alignment.get_enu_altaz_from_itrf_xyz(itrf_xyz)
            altaz_string = 'Alt:' + str(round(enu_altaz[0],1)) + DEG + ' Az:' + str(round(enu_altaz[1],1)) + DEG
        except AssertionError:
            altaz_string = ''
        self.status_label['text'] = target_string + '\n' + start_string + '\n' + end_string + '\n' + altaz_string

        if not self._update_stop:
            self.after(self._update_after, self.update)

    def start(self, after=None):
        """Give number of milliseconds to wait between updates."""
        if after is not None: self._update_after = after
        self._update_stop = False
        self.update()

    def stop(self):
        """Stop updating."""
        self._update_stop = True

    def manual_button_callback(self):
        self.logger.debug('TargetFrame manual button clicked')
        try:
            if self.manual_popup is None:
                self.manual_popup = self.TargetPopup(self)
            else:
                self.manual_popup.update()
                self.manual_popup.deiconify()
        except Exception as err:
            self.logger.debug('Could not open manual popup', exc_info=True)
            ErrorPopup(self, err, self.logger)


    def go_to_target_callback(self):
        self.logger.debug('MountControlFrame Go to target clicked')
        assert self.sys.mount is not None and self.sys.mount.is_init, 'No mount or not initialised'
        try:
            itrf_xyz = self.sys.get_itrf_direction_of_target()
            enu_altaz = self.sys.alignment.get_enu_altaz_from_itrf_xyz(itrf_xyz)
            altaz_string = 'Alt:' + str(round(enu_altaz[0],3)) + DEG + ' Az:' + str(round(enu_altaz[1],3)) + DEG
            self.logger.debug('Target coordinates: '+altaz_string)
            self.logger.debug('Send in EastNorthUp')
            self.sys.mount.move_to_alt_az(*enu_altaz, block=False)
        except Exception as err:
            ErrorPopup(self.master, err, self.logger)

    class TargetPopup(tk.Toplevel):
        """Extends tk.Toplevel for setting target manually."""
        def __init__(self, master):
            super().__init__(master, padx=10, pady=10, bg=ttk.Style().lookup('TFrame', 'background'))
            self.logger = master.logger
            self.title('Target Selection Methods')
            self.resizable(False, False)
                        
#            self.grab_set() #Grab control

            # list common targets
            target_selection_frame = ttk.Frame(self)
            target_selection_frame.grid(row=0, column=0, columnspan=4, padx=(0,10), pady=10, sticky=tk.E+tk.W)
            target_selection_frame.columnconfigure(index=(0, 1, 2, 3), weight=1, uniform="equal")
            ttk.Label(target_selection_frame, text='Select satellite:').grid(row=0, column=0, sticky=tk.W)
            self.target_selection_combo = ttk.Combobox(target_selection_frame, values=list(self.master.sys.saved_targets.keys()))
            self.target_selection_combo.grid(row=0, column=2, sticky="EW")
            self.target_selection_combo.set('ISS')
            ttk.Button(target_selection_frame, text='Get', command=self.get_tle_for_selected_satellite).grid(row=0, column=3, sticky="EW")

            # Fetch TLE Input:
            get_tle_frame = ttk.Frame(self)
            get_tle_frame.grid(row=1, column=0, columnspan=4, padx=(0,10), pady=10, sticky=tk.E+tk.W)
            get_tle_frame.columnconfigure(index=(0, 1, 2, 3), weight=1, uniform="equal")
            ttk.Label(get_tle_frame, text='Set from NORAD ID:').grid(row=0, column=0, sticky=tk.W)
            self.sat_norad_id_entry = ttk.Entry(get_tle_frame, width=15, font='TkFixedFont')
            self.sat_norad_id_entry.grid(row=0, column=2, sticky=tk.E+tk.W)
            ttk.Button(get_tle_frame, text='Set', command=self.get_and_set_tle_callback).grid(row=0, column=3, sticky=tk.E+tk.W)

            # Ephemeris Input:
            get_ephem_frame = ttk.Frame(self)
            get_ephem_frame.grid(row=2, column=0, columnspan=4, padx=(0,10), pady=10, sticky=tk.E+tk.W)
            get_ephem_frame.columnconfigure(index=(0, 1, 2, 3), weight=1, uniform="equal")            
            ttk.Label(get_ephem_frame, text='Set from NAIF ID:').grid(row=0, column=0, sticky=tk.W)
            self.naif_obj_id_entry = ttk.Entry(get_ephem_frame, width=15, font='TkFixedFont')
            self.naif_obj_id_entry.grid(row=0, column=2, sticky=tk.E+tk.W)
            ttk.Button(get_ephem_frame, text='Set', command=self.get_ephem_callback).grid(row=0, column=3, sticky=tk.E+tk.W)

            # Name label:
            name_label_frame = ttk.Frame(self)
            name_label_frame.grid(row=3, column=0, columnspan=4, padx=(0,10), pady=10, sticky=tk.E+tk.W)
            name_label_frame.columnconfigure(index=(0, 1, 2, 3), weight=1, uniform="equal")            
            ttk.Label(name_label_frame, text='Selected:').grid(row=0, column=0, sticky=tk.W)
            self.sat_name_label = ttk.Label(name_label_frame, font='TkFixedFont', text='None')
            self.sat_name_label.grid(row=0, column=1, columnspan=3, padx=10, sticky=tk.E+tk.W)

            # TLE Manual Input:
            tle_frame = ttk.Frame(self)
            tle_frame.grid(row=4, column=0, columnspan=4, padx=(0,10), pady=10, sticky=tk.E+tk.W)
            tle_frame.columnconfigure(index=(0, 1, 2, 3), weight=1, uniform="equal")
            ttk.Label(tle_frame, text='Set from TLE:').grid(row=0, column=0, sticky=tk.W)
            ttk.Button(tle_frame, text='Load from file', command=self.target_from_file_button_callback, width=15, state='DISABLE') \
                                                .grid(row=0, column=1, sticky=tk.E+tk.W)
            ttk.Button(tle_frame, text='Clear', command=self.clear_tle_callback).grid(row=0, column=2, sticky=tk.E+tk.W)
            ttk.Button(tle_frame, text='Set', command=self.set_tle_callback).grid(row=0, column=3, sticky=tk.E+tk.W)
            self.tle_line1_entry = ttk.Entry(tle_frame, width=69, font='TkFixedFont')
            self.tle_line1_entry.grid(row=1, column=0, columnspan=4, sticky=tk.W+tk.E)
            self.tle_line2_entry = ttk.Entry(tle_frame, width=69, font='TkFixedFont')
            self.tle_line2_entry.grid(row=2, column=0, columnspan=4, sticky=tk.W+tk.E)

            # RA/Dec Input:
            radec_frame = ttk.Frame(self)
            radec_frame.grid(row=5, column=0, columnspan=3, padx=(10,0), pady=10)
            ttk.Label(radec_frame, text='Set from RA/Dec:').grid(row=0, column=0, columnspan=2)
            ttk.Label(radec_frame, text='RA: (deg)').grid(row=1, column=0, sticky=tk.E)
            self.ra_entry = ttk.Entry(radec_frame, width=25, font='TkFixedFont')
            self.ra_entry.grid(row=1, column=1)
            ttk.Label(radec_frame, text='Dec: (deg)').grid(row=2, column=0, sticky=tk.E)
            self.dec_entry = ttk.Entry(radec_frame, width=25, font='TkFixedFont')
            self.dec_entry.grid(row=2, column=1)
            ttk.Button(radec_frame, text='Set', command=self.set_radec_callback) \
                                                .grid(row=3, column=1, sticky=tk.W+tk.E)

            # Tracking Time Input:
            time_frame = ttk.Frame(self)
            time_frame.grid(row=5, column=3, columnspan=3, padx=(10,0), pady=10)
            ttk.Label(time_frame, text='Set tracking time (optional):').grid(row=0, column=0, columnspan=3)
            ttk.Label(time_frame, text='Start: (UTC)').grid(row=1, column=0, sticky=tk.E)
            self.start_entry = ttk.Entry(time_frame, width=25, font='TkFixedFont')
            self.start_entry.grid(row=1, column=1, columnspan=2)
            ttk.Label(time_frame, text='End: (UTC)').grid(row=2, column=0, sticky=tk.E)
            self.end_entry = ttk.Entry(time_frame, width=25, font='TkFixedFont')
            self.end_entry.grid(row=2, column=1, columnspan=2)
            ttk.Button(time_frame, text='Clear', width=10, command=self.clear_time_callback) \
                                                            .grid(row=3, column=1, sticky=tk.W+tk.E)
            ttk.Button(time_frame, text='Set', width=10, command=self.set_time_callback) \
                                                            .grid(row=3, column=2, sticky=tk.W+tk.E)

            # Application Link
            '''
            tcp_link_frame = ttk.Frame(self)
            tcp_link_frame.grid(row=6, column=0, columnspan=4, padx=(0,10), pady=10, sticky=tk.E+tk.W)
            tcp_link_frame.columnconfigure(index=(0, 1, 2, 3), weight=1, uniform="equal")
            self.tcp_link_enabled = tk.IntVar()
            ttk.Checkbutton(tcp_link_frame, text="Enable TCP host", variable=self.tcp_link_enabled, command = lambda: self.toggle_tcp_link(),) \
                .grid(row=0, column=0, sticky=tk.W)
            ttk.Label(tcp_link_frame, text='Port:').grid(row=0, column=1, sticky=tk.E)
            self.tcp_port_entry = ttk.Entry(tcp_link_frame, width=15, font='TkFixedFont')
            self.tcp_port_entry.grid(row=0, column=2, sticky=tk.E+tk.W)
            self.tcp_port_entry.insert(0, '12345')            
            ttk.Button(tcp_link_frame, text='Set', command=self.get_ephem_callback).grid(row=0, column=3, sticky=tk.E+tk.W)
            '''


            self.protocol('WM_DELETE_WINDOW', self.withdraw)
            self.update()

        def update(self):
            self.logger.debug('TargetPopup got update request')
            """Read the target status and fill in fields."""
            target = self.master.sys.target.target_object
            # Clear everything
            self.tle_line1_entry.delete(0, 'end')
            self.tle_line2_entry.delete(0, 'end')
            self.ra_entry.delete(0, 'end')
            self.dec_entry.delete(0, 'end')
            # Fill with current
            if isinstance(target, SkyCoord):
                ra = target.ra.to_value('deg')
                dec = target.dec.to_value('deg')
                self.ra_entry.insert(0, str(ra))
                self.dec_entry.insert(0, str(dec))
            elif isinstance(target, EarthSatellite):
                (l1, l2) = self.master.sys.target.get_tle_raw()
                self.tle_line1_entry.insert(0, l1)
                self.tle_line2_entry.insert(0, l2)
            else:
                self.logger.debug('No target set, leaving clear.')
            # Clear times
            self.start_entry.delete(0, 'end')
            self.end_entry.delete(0, 'end')
            # Set with current
            t_start = self.master.sys.target.start_time
            t_end = self.master.sys.target.end_time
            self.start_entry.insert(0, str(t_start) if t_start is not None else '')
            self.end_entry.insert(0, str(t_end) if t_end is not None else '')
            self.master.update()

        def toggle_tcp_link(self):
            print(self.tcp_link_enabled.get())

        def get_tle_callback(self):
            self.logger.debug("TLE requested for sat ID: " + self.sat_norad_id_entry.get())        
            try:
                self.sat_id = int(self.sat_norad_id_entry.get())
            except:
                self.logger.debug("sat ID invalid")
                self.sat_name_label['text'] = ''
                self.sat_id = None
            if self.sat_id:
                tle = self.master.sys.target.get_tle_from_sat_id(self.sat_id)
                if tle is not None and len(tle)==3:
                    self.tle_line1_entry.delete(0, tk.END)
                    self.tle_line1_entry.insert(0,tle[0])
                    self.tle_line2_entry.delete(0, tk.END)
                    self.tle_line2_entry.insert(0,tle[1])
                    sat_name = tle[2]
                    self.sat_name_label['text'] = sat_name or ''
                    self.logger.debug('successfully fetched TLE for sat ID '+str(self.sat_id)+', "'+sat_name+'"')
                    self.logger.debug(tle[0])
                    self.logger.debug(tle[1])
                    self.logger.debug(tle[2])
                else:
                    self.logger.info('Failed to retrieve TLE for sat ID ' +str(self.sat_id))
                    self.sat_name_label['text'] = '(failed)'                    

        def get_and_set_tle_callback(self):
            self.get_tle_callback()
            if self.sat_id is not None:
                self.set_tle_callback()
                
        def clear_tle_callback(self):
            self.tle_line1_entry.delete(0, tk.END)
            self.tle_line2_entry.delete(0, tk.END)            
                
        def get_tle_for_selected_satellite(self):
            selected_sat_name = self.target_selection_combo.get()
            self.logger.info('Selected target name: '+selected_sat_name)
            if selected_sat_name in self.master.sys.saved_targets:
                self.sat_id = self.master.sys.saved_targets[selected_sat_name]
            self.sat_norad_id_entry.delete(0, tk.END)
            self.sat_norad_id_entry.insert(0,str(self.sat_id))
            self.get_and_set_tle_callback()
            
        def set_tle_callback(self):
            try:
                line1 = self.tle_line1_entry.get()
                line2 = self.tle_line2_entry.get()
                self.master.sys.target.set_target_from_tle((line1, line2))
                self.clear_time_callback()
                #self.update() called in clear_time_callback()
                self.master.sys.target.set_source('TLE')
            except Exception as err:
                ErrorPopup(self, err, self.logger)

        def target_from_file_button_callback(self):
            try:
                raise NotImplementedError('Feature coming soon!')
            except Exception as err:
                ErrorPopup(self, err, self.logger)
                
        def set_radec_callback(self):
            try:
                ra = self.ra_entry.get()
                dec = self.dec_entry.get()
                self.master.sys.target.set_target_from_ra_dec(ra, dec)
                self.clear_time_callback()
                #self.update() called in clear_time_callback()
                self.master.sys.target.set_source('RADEC')
            except Exception as err:
                ErrorPopup(self, err, self.logger)
                
        def set_time_callback(self):
            try:
                start = self.start_entry.get()
                self.logger.debug(start)
                start = None if not start.strip() else apy_time(start)
                self.logger.debug(start)
                end = self.end_entry.get()
                self.logger.debug(end)
                end = None if not end.strip() else apy_time(end)
                self.logger.debug(start)
                self.master.sys.target.set_start_end_time(start, end)
                self.update()
            except Exception as err:
                ErrorPopup(self, err, self.logger)
                
        def clear_time_callback(self):
            try:
                self.start_entry.delete(0, 'end')
                self.start_entry.insert(0, '')
                self.end_entry.delete(0, 'end')
                self.end_entry.insert(0, '')
                self.master.sys.target.clear_start_end_time()
                self.update()
            except Exception as err:
                ErrorPopup(self, err, self.logger)

        def get_ephem_callback(self):
            try:
                self.logger.debug("Ephemeris requested for NAIF object ID: " + self.naif_obj_id_entry.get())
                obj_id = self.naif_obj_id_entry.get()
                (lat, lon, elevation_m) = self.master.sys.alignment.get_location_lat_lon_height()
                assert lat is not None and lon is not None and elevation_m is not None, 'Location not initialized'
                self.logger.info("Ephemeris requested for sat ID: " + obj_id)
                self.master.sys.target.get_ephem(obj_id, lat, lon, elevation_m)
                if self.master.sys.target._ephem.target_name is not None:
                    self.logger.info('Ephemeris target object: "%s"' % self.master.sys.target._ephem.target_name)
                self.sat_name_label['text'] = self.master.sys.target._ephem.target_name or ''
                self.master.sys.target.set_source('EPHEM')
            except Exception as err:
                ErrorPopup(self, err, self.logger)
                self.sat_name_label['text'] = 'None'

class AlignmentFrame(ttk.Frame):
    """Extends tkinter.Frame for controlling System.alignment"""
    def __init__(self, master, pypogs_system, logger):
        self.logger = logger
        self.logger.debug('Creating AlignmentFrame')
        super().__init__(master)
        self.sys = pypogs_system
        self._update_after = 1000
        self._update_stop = True
        # Create widgets and layout
        ttk.Label(self, text='Location and Alignment').grid(row=0, column=0, columnspan=2)
        tk.Frame(self, height=1, bg='gray50').grid(row=1, column=0, columnspan=2, sticky=tk.W+tk.E)
        self.status_label = ttk.Label(self, font='TkFixedFont')
        self.status_label .grid(row=2, column=0, columnspan=2)
        self.update() # Sets status
        ttk.Button(self, text='Set Location', command=self.location_button_callback, width=15).grid(row=3, column=0)
        ttk.Button(self, text='Set Alignment', command=self.alignment_button_callback, width=15).grid(row=3, column=1)
        self.location_popup = None
        self.alignment_popup = None


    def update(self):
        """Update the alignment status"""
        self.logger.debug('AlignmentFrame got update request')
        try:
            (lat, lon, height) = self.sys.alignment.get_location_lat_lon_height()
            loc_string = 'N:' + str(round(lat, 3)) + DEG + ' E:' + str(round(lon, 3)) + DEG
        except AssertionError:
            loc_string = 'No location'
        if self.sys.alignment.is_aligned:
            align_string = 'Is aligned'
        else:
            align_string = 'Not aligned'
        self.status_label['text'] = loc_string + '\n' + align_string

        if not self._update_stop:
            self.after(self._update_after, self.update)

    def start(self, after=None):
        """Give number of milliseconds to wait between updates."""
        if after is not None: self._update_after = after
        self._update_stop = False
        self.update()

    def stop(self):
        """Stop updating."""
        self._update_stop = True



    def location_button_callback(self):
        self.logger.debug('AlignmentFrame location button clicked')
        try:
            if self.location_popup is None:
                self.location_popup = self.LocationPopup(self)
            else:
                self.location_popup.update()
                self.location_popup.deiconify()
        except Exception as err:
            self.logger.debug('Could not open location popup', exc_info=True)
            ErrorPopup(self, err, self.logger)

    class LocationPopup(tk.Toplevel):
        """Extends tk.Toplevel for setting location."""
        def __init__(self, master):
            super().__init__(master, padx=10, pady=10, bg=ttk.Style().lookup('TFrame', 'background'))
            self.logger = master.logger
            self.title('Location')
            self.resizable(False, False)
#            self.grab_set() #Grab control
            try:
                (old_lat, old_lon, old_height) = master.sys.alignment.get_location_lat_lon_height()
            except AssertionError:
                (old_lat, old_lon, old_height) = (0.0,)*3

            ttk.Label(self, text='Set Location').grid(row=0, column=0, columnspan=2)
            ttk.Label(self, text='Latitude: (deg)').grid(row=1, column=0, sticky=tk.E)
            self.lat_entry = ttk.Entry(self, width=10)
            self.lat_entry.insert(0, str(round(old_lat, 4)))
            self.lat_entry.grid(row=1, column=1)
            ttk.Label(self, text='Longitude: (deg)').grid(row=2, column=0, sticky=tk.E)
            self.lon_entry = ttk.Entry(self, width=10)
            self.lon_entry.insert(0, str(round(old_lon, 4)))
            self.lon_entry.grid(row=2, column=1)
            ttk.Label(self, text='Height: (m)').grid(row=3, column=0, sticky=tk.E)
            self.height_entry = ttk.Entry(self, width=10)
            self.height_entry.insert(0, str(round(old_height, 4)))
            self.height_entry.grid(row=3, column=1)
            ttk.Button(self, text='Set', width=20, command=self.set_callback).grid(row=4, column=0, columnspan=2)
            self.protocol('WM_DELETE_WINDOW', self.withdraw)

        def set_callback(self):
            try:
                lat = float(self.lat_entry.get())
                lon = float(self.lon_entry.get())
                height = float(self.height_entry.get())
                self.master.sys.alignment.set_location_lat_lon(lat, lon, height)
#                self.master.update()
#                self.withdraw()
            except Exception as err:
                ErrorPopup(self, err, self.logger)

    def alignment_button_callback(self):
        self.logger.debug('AlignmentFrame alignment button clicked')
        try:
            if self.alignment_popup is None:
                self.alignment_popup = self.AlignmentPopup(self)
            else:
                self.alignment_popup.update()
                self.alignment_popup.deiconify()
        except Exception as err:
            self.logger.debug('Could not open alignment popup', exc_info=True)
            ErrorPopup(self, err, self.logger)

    class AlignmentPopup(tk.Toplevel):
        """Extends tk.Toplevel for setting alignment."""
        def __init__(self, master):
            super().__init__(master, padx=10, pady=10, bg=ttk.Style().lookup('TFrame', 'background'))
            self.logger = master.logger
            self.title('Alignment')
            self.resizable(False, False)
#            self.grab_set() #Grab control
            ttk.Label(self, text='Set Alignment').grid(row=0, column=0, columnspan=3)
            ttk.Button(self, text='Do auto-align', width=18, command=self.auto_callback).grid(row=1, column=0)
            ttk.Button(self, text='EastNorthUp', width=18, command=self.enu_callback).grid(row=1, column=1)
            ttk.Button(self, text='Load from file', width=18, command=self.load_callback).grid(row=1, column=2)
            self.protocol('WM_DELETE_WINDOW', self.withdraw)

        def auto_callback(self):
            try:
                self.master.sys.do_auto_star_alignment()
#                self.master.update()
#                self.withdraw()
            except Exception as err:
                ErrorPopup(self, err, self.logger)
        def enu_callback(self):
            try:
                self.master.sys.alignment.set_alignment_enu()
#                self.master.update()
#                self.withdraw()
            except Exception as err:
                ErrorPopup(self, err, self.logger)
        def load_callback(self):
            try:
                filename = filedialog.askopenfilename(
                    initialdir = self.master.sys.data_folder, 
                    title = 'Select alignment file (*_Alignment_from_obs.csv)',
                    filetypes = (("CSV Files","csv",),("all files","*.*"))
                )
                self.master.sys.alignment.get_alignment_data_form_file(filename)
            except Exception as err:
                ErrorPopup(self, err, self.logger)


class StatusFrame(ttk.Frame):
    """Extends tkinter.Frame for displaying Mount status"""
    def __init__(self, master, pypogs_system, logger):
        self.logger = logger
        self.logger.debug('Creating StatusFrame')
        super().__init__(master)
        self.sys = pypogs_system
        self._update_stop = True
        self._update_after = 1000
        # Create widgets and layout
        ttk.Label(self, text='Status', width=32).grid(row=0, column=0)
        tk.Frame(self, height=1, bg='gray50').grid(row=1, column=0, sticky=tk.W+tk.E)
        self.status_label = ttk.Label(self, font='TkFixedFont')
        self.status_label.grid(row=2, column=0)
        self.update() # Sets status

    def update(self):
        self.logger.debug('StatusFrame got update request')
        """Update status once. Auto update with start() and stop() instead."""
        keys = ('alt', 'azi', 'alt_rate', 'azi_rate')
        mount_state = self.sys.mount.state_cache if self.sys.mount is not None else None
        status_string = ''
        for key in keys:
            try:
                status_string += ('{:>13s}:'.format(key) + '{: 7.2f}'.format(mount_state[key]) + '\n')
            except:
                status_string += ('{:>13s}:'.format(key) + '  ---  ' + '\n')
                
        control_state = self.sys.control_loop_thread.state_cache
            
        # Modify mode indicator
        '''
        if self.sys.mount is not None and self.sys.mount.is_init:
            if control_state['mode'] in (None, 'SDRL'):
                if self.sys.mount.is_sidereal_tracking:
                    if control_state['mode'] is None:  # recheck since could have changed while checking if sidereal
                        control_state['mode'] = 'SDRL'
                else:
                    control_state['mode'] = None
        '''
                    

        for key in control_state.keys():
            try:
                if key in ('mode', 'ct_has_track', 'ft_has_track'):
                    assert control_state[key] is not None
                    status_string += ('{:>13s}:'.format(key) + '  {: <5}'.format(str(control_state[key])) + '\n')
                else:
                    status_string += ('{:>13s}:'.format(key) + '{: 7.2f}'.format(control_state[key]) + '\n')
            except:
                status_string += ('{:>13s}:'.format(key) + '  ---  ' + '\n')

        self.status_label['text'] = status_string.rstrip('\n')
        if not self._update_stop:
            self.after(self._update_after, self.update)

    def start(self, after=None):
        """Give number of milliseconds to wait between updates."""
        if after is not None: self._update_after = after
        self._update_stop = False
        self.update()

    def stop(self):
        """Stop updating."""
        self._update_stop = True


class MountControlFrame(ttk.Frame):
    """Extends tkinter.Frame for manual control of Mount"""
    def __init__(self, master, pypogs_system, logger):
        self.logger = logger
        self.logger.debug('Creating ManualControlFrame')
        super().__init__(master)
        self.sys = pypogs_system
        self._update_stop = True
        self._update_after = 1000
        
        # Create widgets and layout
        ttk.Label(self, text='Manual Control').grid(row=0, column=0, columnspan=2)
        tk.Frame(self, height=1, bg='gray50').grid(row=1, column=0, columnspan=2, sticky=tk.W+tk.E)
        ttk.Label(self, text='Altitude: (deg)').grid(row=2, column=0)
        self.alt_spinbox = ttk.Spinbox(self, from_=0, to=90, width=10)
        self.alt_spinbox.grid(row=3, column=0)
        self.alt_spinbox.delete(0, 'end')
        self.alt_spinbox.insert(0, '0.0') #Set default zero
        ttk.Label(self, text='Azimuth: (deg)').grid(row=4, column=0)
        self.azi_spinbox = ttk.Spinbox(self, from_=-180, to=180, width=10)
        self.azi_spinbox.delete(0, 'end')
        self.azi_spinbox.insert(0, '0.0') #Set default zero
        self.azi_spinbox.grid(row=5, column=0)

        rb_frame = ttk.Frame(self)
        rb_frame.grid(row=2, rowspan=4, column=1)
        ttk.Label(rb_frame, text='Coordinates:').grid()
        self.coord_variable = tk.IntVar()
        ttk.Radiobutton(rb_frame, text='COM', variable=self.coord_variable, value=COMMAND).grid(sticky=tk.W)
        ttk.Radiobutton(rb_frame, text='MNT', variable=self.coord_variable, value=MOUNT).grid(sticky=tk.W)
        ttk.Radiobutton(rb_frame, text='ENU', variable=self.coord_variable, value=ENU).grid(sticky=tk.W)

        ttk.Button(self, text='Send', command=self.send_button_callback, width=15).grid(row=6, column=0)
        ttk.Button(self, text='Stop', command=self.stop_button_callback, width=15).grid(row=6, column=1)
        ttk.Style().configure('sidereal.TButton')
        self.sidereal_button = ttk.Button(self, text='Sidereal Tracking', style='sidereal.TButton', command=self.toggle_sidereal_tracking)
        self.sidereal_button.grid(row=7, column=0, columnspan=2, sticky=tk.W+tk.E)

        self.update()
        
    def update(self):
        self.logger.debug('MountControlFrame got update request')
        
        if self.sys.mount is not None and self.sys.mount.is_init:
            if self.sys.mount._is_sidereal_tracking:
                ttk.Style().configure('sidereal.TButton', background='green', foreground='green')
                self.sidereal_button['text'] = 'Stop sidereal tracking'
                #self.sys.mount.get_alt_az()
            else:
                ttk.Style().configure('sidereal.TButton',\
                                     background=ttk.Style().lookup('TButton', 'background'), \
                                     foreground=ttk.Style().lookup('TButton', 'foreground'))
                self.sidereal_button['text'] = 'Start sidereal tracking'

        if not self._update_stop:
            self.after(self._update_after, self.update)
        
    def start(self, after=None):
        """Give number of milliseconds to wait between updates."""
        if after is not None: self._update_after = after
        self._update_stop = False
        self.update()

    def stop(self):
        """Stop updating."""
        self._update_stop = True
        
    def stop_button_callback(self):
        self.logger.debug('MountControlFrame stop clicked')
        assert self.sys.mount is not None and self.sys.mount.is_init, 'No mount or not initialised'
        self.sys.stop()

    def send_button_callback(self):
        self.logger.debug('MountControlFrame send clicked')
        # Read inputs and check
        input_ok = True
        try:
            alt = float(self.alt_spinbox.get())
            assert 0 <= alt <= 90
        except Exception as err:
            ErrorPopup(self, err, self.logger)
            alt = 0.0
            input_ok = False
        try:
            azi = float(self.azi_spinbox.get())
            assert -360 <= azi <= 360
        except Exception as err:
            ErrorPopup(self, err, self.logger)
            azi = 0.0
            input_ok = False
        # Set inputs to our decoded value
        self.alt_spinbox.delete(0, 'end')
        self.alt_spinbox.insert(0, str(alt))
        self.azi_spinbox.delete(0, 'end')
        self.azi_spinbox.insert(0, str(azi))
        self.logger.debug('Values alt='+str(alt)+' azi='+str(azi))
        try:
            if input_ok:
                # Check mount ready
                assert self.sys.mount is not None and self.sys.mount.is_init, 'No mount or not initialised'
                mode = self.coord_variable.get()
                if mode == COMMAND:
                    self.logger.debug('Send direct to mount')
                    self.sys.mount.move_to_alt_az(alt, azi, block=False)
                elif mode == MOUNT:
                    self.logger.debug('Send in corrected mount coordinates')
                    com_altaz = self.sys.alignment.get_com_altaz_from_mnt_altaz((alt, azi))
                    self.logger.debug('Command: ' + str(com_altaz))
                    self.sys.mount.move_to_alt_az(*com_altaz, block=False)
                elif mode == ENU:
                    self.logger.debug('Send in EastNorthUp')
                    com_altaz = self.sys.alignment.get_com_altaz_from_enu_altaz((alt, azi))
                    self.logger.debug('Command: ' + str(com_altaz))
                    self.sys.mount.move_to_alt_az(*com_altaz, block=False)
                else:
                    raise RuntimeError('Unknown coordinate choice')
            else:
                raise ValueError('Forbidden input')
        except Exception as err:
            ErrorPopup(self.master, err, self.logger)

    def toggle_sidereal_tracking(self):
        self.logger.debug('Sidereal tracking button clicked')
        if self.sys.mount._is_sidereal_tracking:
            self.logger.debug('Sidereal tracking is on.  Will turn off.')            
            self.sys.mount.stop_sidereal_tracking()
        else:
            self.logger.debug('Sidereal tracking is off.  Will turn on.')
            # Require mount initialized
            assert self.sys.mount is not None and self.sys.mount.is_init, 'No mount or not initialised'
            # Stop satellite tracking
            try:
                self.logger.debug('Stopping control loops')
                self.sys.stop()
            except Exception as err:
                self.logger.debug('Did not stop', exc_info=True)
                ErrorPopup(self, err, self.logger)
            # Start sidereal tracking
            self.logger.debug('Starting sidereal tracking')
            self.sys.mount.start_sidereal_tracking()
            
class ErrorPopup(tk.Toplevel):
    """Extends tkinter.Toplevel for error popups"""
    def __init__(self, master, error, logger):
        logger.debug('ErrorPopup: Got error: ' + str(error))
        print(traceback.format_exc())
        super().__init__(master, padx=10, pady=10, bg=ttk.Style().lookup('TFrame', 'background'))
        self.title('Error')
        self.grab_set() #Grab control
        ttk.Label(self, text=str(error), width=50).pack()
        ttk.Button(self, text='Close', command=self.destroy).pack(fill=tk.BOTH)
