Welcome to (unofficial) pypogs!
==================

*Experimental fork for satellite imaging*
-----------------------------------------

====

Relation to main ESA project
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This fork is unaffiliated with ESA and is managed outside of the main ESA project, but
coordinates informally with the main project.  This fork pulls updates form the main project 
and in some cases may feed forward new features, developed and tested here, if deemed robust 
and useful to the optical comm goals of the main ESA project.

+ pypogs **main project**:  https://github.com/esa/pypogs  
+ pypogs **documentation**: https://pypogs.readthedocs.io/  

====

Geared toward satellite imaging
^^^^

With several features added for user convenience:

+ ASCOM mount control (partially tested)
+ ASCOM camera control (for non-ZWO astro cameras; limited to ~5 fps)
+ Expanded target selection methods: 
 + "saved" target selection pull-down menu (users can append this short list via startup scripts)
 + TLE lookup by NORAD ID
 + JPL horizons epehemeris lookup (for JWST, Psyche, etc!)
+ Configurable TCP application links to:
 + Stellarium for graphical display of telescope pointing and receipt of go-to requests
 + SkyTrack for receiving target TLEs
+ Expanded configurability of auto-alignment routine
 + Self-alignment vectors configurable by command (to avoid my neighbor's trees) 
 + Additional user controls for settling time, retry counts, etc.
+ Sidereal tracking button (sometimes useful for finding faint satellites)
+ Camera settings (gain, exposure, etc) exposed to initialization commands (useful in startup scripts)
+ Improved start of tracking by projecting intercept point
+ Additional control offset in coarse camera view (to help recover from severe misalignment)


====

Hardware Compatibility
^^^^^^^^^^^^^^^^^^^^^^

| **!!!! Presently restricted to alt-az mounts only !!!!** 
| *Equatorial mount support is under consideration but would require refactoring the entire pypogs architecture.*

The telescope mount must be capable of smoothly executing continuous floating-point rate commands.

**Telescope Mounts:**  

+ Celestron CPC (tested, *recommended*) & NexStar (tested)
+ iOptron AZMP with `latest firmware <https://www.ioptron.com/Articles.asp?ID=290>`_ (tested)
+ Possibly others via ASCOM (*not* tested)
+ Planned:  Meade LX200 (*not* tested)
+ **not compatible**:  Sky-Watcher (lacking support for rate commands)

**Cameras:**

+ Point Gray (tested)
+ ZWO (tested, *recommended*)
+ QHY via ASCOM (tested, limited by driver to 5 fps)
+ Possibly others via ASCOM (not tested, limited by driver to 5 fps)
+ Under consideration:  QHY direct driver (*not* tested)
+ Under consideration:  directshow (for webcams and non-ZWO cameras, *not* tested)

====

Optical Configuration Considerations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Image outside of pypogs** *(for now)*

Presently, pypogs cannot record frames from a camera at full frame rate while simultaneously 
controlling from the same camera.  Therefore, it is recommended that users operate main imaging 
cameras through separate software (e.g. SharpCap, FIreCapture, ASICap, etc) while tracking a 
satellite with one or two dedicated control cameras in pypogs.

**Wide field of view for auto-alignment**

Pypogs provides a brilliant auto-alignment routine which measures and compensates for a mount's 
inherent alignment error.  The auto-alignment routine dramatically improves pypogs target 
acquisition and tracking performance.  It is highly recommended that operators run auto-alignment 
prior to tracking, or load a previous alignment solution if a mount and telescope system has not 
changed.  The plate solver used by pypogs' auto-alignment routine works best with wide fields of 
view, roughly 10 degrees or more.

::

 field of view = arctan( camera sensor width / focal length)

Focal lengths of 35mm and 50mm have been found to work well with with small-chip (asi120, asi290, 
asi178, asi224, etc) and large-chip (asi174) guide cameras, respectively.  C-mount/CS-mount CCTV 
lenses work well, but must be of decent optical quality. The plate solver is sensitive to 
optical distortion from low-quality lenses.  Several ultra-cheap CCTV lenses were found not to 
work due to field distortion and field flatness (corners out of focus).

Recommended star camera lenses:

+ For small-chip guide cameras (asi290, etc):  `Fujinon hf35ha-1s 35mm Lens <https://www.rmaelectronics.com/fujinon-hf35ha-1s/>`_ ($110 USD)
* For large-chip guide cameras (asi174):  `Fujinon hf50ha-1s 50mm Lens <https://www.rmaelectronics.com/fujinon-hf50ha-1s/>`_ ($155 USD)
+ Budget option for large-chip guide cameras (asi174):  `Arducam C-Mount 50mm Lens <https://www.arducam.com/product/50mm-c-mount-lens-for-hq-camera/>`_ ($46 USD, one test article shows noticeable tilt but works reliably)




**Competing constraints:  auto-alignment and bright target acquisition vs tracking precision**

In addition to being better suited for plate solving, a wide field of view coarse camera 
configuration can reduce susceptibility to alignment error during initial target acquisition by 
presenting a larger patch of sky for pypogs to search.  This wide field advantage can only be 
realized with targets that are bright enough (visual magnitude ~3 or less) to be detected in the 
wide view.  Small or distant, dim targets generally require longer focal length to detect and 
track.  Moreover, longer focal length (narrower field of view) yields better tracking performance.  
As a rule of thumb, it is recommended that the finest view used by pypogs have focal length not 
less than about 1/10th that of the primary imaging telescope.  For example, with a C8 at f/10 
(2032 mm focal length), the guide scope focal length should be at least 200 mm.

*In a nutshell, although it may be possible to operate pypogs with a single guide scope and 
camera, competing objectives of auto-alignment, initial target acquisition, and tracking 
generally warrant operating pypogs with at least 2 optical systems - one wide field optical 
system for auto-alignment and bright object initial acquisition, and a separate, longer focal 
length system for dim object initial acquisition and fine guiding.*

**Star Camera, Coarse Camera, or Fine Camera?**

Which camera "role" in pypogs should be associated with which optical system?  It depends.

If you are planning to track only bright objects like ISS and CSS, use a wide field system as
your Coarse Camera, and enable "Link Star/Coarse Cameras" to use this camera in both roles.
Select a narrow field of view system as the Fine Camera.  This way, the wide field system 
will be used for both auto-alignment and initial target acquisition and tracking, and once
the pypogs locks onto the target in the coarse view, it should then automatically search for
and lock onto the target in the fine camera, providing best stabilization for a primary imaging
system (operated outside of pypogs).

If you are planning to track dim objects (visual magnitude >2.5 or so) which cannot be
detected in the wide field camera view, configure the wide field system as your Star Camera
only, and load the narrow field of view camera as the Coarse Camera.


====

Getting Started
^^^^

Check hardware compatiblity before proceeding.

| Follow `installation instructions <https://pypogs.readthedocs.io/en/latest/installation.html>`_ 
 provided from the main project, **but** 
| clone "https://github.com/rkinnett/pypogs.git" 
| instead of "https://github.com/esa/pypogs.git".

Once installed, run graphical pypogs by:

::

  cd examples
  python run_pypogsGUI.py  

This is a starting point configuration without any hardware initialized, and with default 
settings for everything.

The file run_pypogsGUI.py contains many commented-out (via # and ''') configuration commands 
as examples of how to customize a startup configuration.

The user may copy run_pypogsGUI.py to a new file titled "my_pypogs.py" or similar, specifically 
prefixed by "my\_" so that git will not try to configuration manage unique configuration files
when the user updates pypogs via git.


====

pypogs general overview (from main project)
------------------------------------------- 

*pypogs is an automated closed-loop satellite tracker for portable telescopes written in Python.*

Use it to control your optical ground station, auto-align it to the stars, and automatically acquire
and track satellites with closed-loop camera feedback. Additionally we include instructions for how
to build a fibre-coupling Focal Plane Assembly (FPA) replacing the eyepiece in any unmodified
portable telescope.

pypogs includes a platform independent Graphical User Interface (GUI) to manage alignment, tracking
feedback, and hardware settings. The GUI controls the pypogs core through a public API (see
documentation); pypogs may be controlled fully from the command line as well.

The software is available in the `pypogs GitHub repository <https://github.com/esa/pypogs>`_.
All documentation is hosted at the
`pypogs ReadTheDocs website <https://pypogs.readthedocs.io/en/latest/>`_. pypogs is Free and Open
Source Software released by the European Space Agency under the Apache License 2.0. See NOTICE.txt
in the repository for full licensing details.

Performance will vary. Our testing shows approximately 1 arcsecond RMS tracking of stars and 
MEO/GEO satellites, with 4 arcseconds RMS tracking of LEO satellites. With this performance you
can launch the received signal into a 50µm and 150µm core diameter multimode fibre respectively with
the proposed FPA. We require no modifications to the telescope nor a fine steering mirror for these
results; pypogs will enable the lowest cost high-performance optical ground stations yet.

An article describing the system was presented at IEEE ICSOS in 2019; the paper is
`available here <https://ieeexplore.ieee.org/abstract/document/8978992>`_. The GitHub respository
includes a preprint. If you find pypogs useful in your work, please cite:

G. M. Pettersson, J. Perdigues, and Z. Sodnik, "Unmodified Portable Telescope for Space-to-Ground
Optical Links," in *Proc. IEEE International Conference on Space Optical Systems and Applications
(ICSOS)*, 2019.