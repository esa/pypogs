'''
horizons_ephem.py, by R. Kinnett, 2021

Utility for fetching ephemerides from JPL Horizons REST API (primarily for interplanetary spacecraft).

:class: Ephem
  Methods:
    
    fetch( object_id, start_date_time_utc, end_date_time_utc, time_step, lat, lon, elevation_m)
      Queries JPL Horizons ephemeris REST API and tabulates results across specified time span
      for interpolation later by provided methods.  This should be performed upon target
      selection to pre-cache reference data.  
    
      Parameters:
        object_id:  THE SPICE convention uses negative integers as spacecraft ID codes. 
                    The code assigned to interplanetary spacecraft is normally the negative of
                    the code assigned to the same spacecraft by JPL's Deep Space Network (DSN)
                    as determined the NASA control authority at Goddard Space Flight Center.
                    
                    Examples:  HST: -48, JWST: -170, Europa Clipper: -159
                    
        start_date_time_utc:  'YYYY-MM-DD' or 'YYYY-MM-DD HH:MM:SS'
        end_date_time_utc:    'YYYY-MM-DD' or 'YYYY-MM-DD HH:MM:SS'
        time_step:            Time step in minutes between ephemerides
        lat:                  Ground station North latitude in decimal degrees
        lon:                  Ground station East latitude in decimal degrees
        elevation_m:          Ground station elevation in meters above mean sea level.
    
    interp( julian_date )
      Interpolates alt, azi coordinates at specified time.
      
    now()
      Interpolates current alt, azi coordinates for present time.
  
  Example:
    ephem = Ephem()
    ephem.fetch(-170, '2021-12-25 12:50:00', '2021-12-31', 34.2, -118.2, 500)
    print(ephem.now())

'''

import requests 
import numpy as np
from astropy.time import Time

class Ephem:
  jd  = []
  azi = []
  alt = []
  len = 0
  target_name = ''
  obj_id = None
  is_init = False
  
  def __init__(self, object_id, start_date_time_utc, end_date_time_utc, time_step_minutes, lat, lon, elevation_m):
    request_str = 'https://ssd.jpl.nasa.gov/api/horizons.api?format=text' \
                + '&COMMAND="'+str(object_id)+'"' \
                + '&MAKE_EPHEM="YES"' + '&EPHEM_TYPE="OBSERVER"' + '&CENTER="coord@399"' + '&COORD_TYPE="GEODETIC"' \
                + '&SITE_COORD="'+str(lon)+','+str(lat)+','+str(elevation_m/1000)+'"' \
                + '&START_TIME="'+start_date_time_utc+'"' \
                + '&STOP_TIME="'+end_date_time_utc+'"' \
                + '&STEP_SIZE="'+str(time_step_minutes)+' MINUTES"' \
                + '&CAL_FORMAT="JD"' \
                + '&QUANTITIES="4"' \
                + '&CSV_FORMAT="YES"'
    http_request = requests.get(request_str)
    self.len = 0
    self.jd  = []
    self.alt = []
    self.azi = []
    self.obj_id = object_id
    if http_request.status_code == 200:
      found_ephem_start = False
      found_ephem_end = False
      for line in http_request.text.splitlines():
        if not found_ephem_start:
          if line == '$$SOE':
            found_ephem_start = True
          elif line.startswith('Target body name'):
            try:
              self.target_name = line[18: line.find(' (') or len(line)]
            except:
              pass
        elif not found_ephem_end:
          if line == '$$EOE':
            found_ephem_end = True
            break
          #print(line)
          ephem_line = line.split(',')
          self.jd.append(float(ephem_line[0]))
          self.azi.append(float(ephem_line[3]))
          self.alt.append(float(ephem_line[4]))
          self.len += 1
          #print(ephem.jd[-1],ephem.azi[-1],ephem.alt[-1])
      if self.len > 0:
        self.is_init = True
      #else:
        #print('Received response but no ephemeris')
        #print(http_request.text)

    #else:
      #print('status code: %i' % http_request.status_code)
      #print(request_str)

    
  def interp(self,jdate):
    assert self.is_init, 'Ephemeris not loaded.'
    def circular_interpolation(x, xp, fp, range_min=0, range_max=360): 
      period = range_max - range_min
      midpoint = period/2
      y = np.mod(np.interp(x, xp, np.unwrap(fp, period=period)), period)
      while y>range_max:  y -= period
      while y<range_min:  y += period
      return y

    interpolated_alt = circular_interpolation(jdate, self.jd, self.alt, -180, 180)
    interpolated_azi = circular_interpolation(jdate, self.jd, self.azi, 0, 360)
    return np.array([[interpolated_alt, interpolated_azi]])

  def project_ephem(self, times=None):
    if times is None:
      times = Time.now()
    if times.size == 1:
      az_el_pairs = np.array(self.interp(times.jd))
    else:
      az_el_pairs = np.array(self.interp(times.jd[1]))
      if times.size>1:
        for i in range(1, times.size):
          az_el_pair = self.interp(times.jd[i])
          az_el_pairs = np.concatenate([az_el_pairs, az_el_pair])
    #print(az_el_pairs)
    return az_el_pairs.T

  def now(self):
    ut = Time.now()
    return self.interp(ut.jd)
    
