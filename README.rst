Welcome to pypogs!
==================

*pypogs is an automated closed-loop satellite tracker for portable telescopes written in Python.*

Use it to control your optical ground station, auto-align it to the stars, and automatically acquire
and track satellites with closed-loop camera feedback. Additionally we include instructions for how
to build a fibre-coupling Focal Plane Assembly (FPA) replacing the eyepiece in any unmodified
portable telescope.

pypogs includes a platform idenpendent Graphical User Interface (GUI) to manage alignment, tracking
feedback, and hardware settings. The GUI controls the pypogs core through a public API (see
documentation); pypogs may be controlled fully from the command line as well.

The software is available in the `pypogs GitHub repository <https://github.com/esa/pypogs>`_.
All documentation is hosted at the
`pypogs ReadTheDocs website <https://pypogs.readthedocs.io/en/latest/>`_. pypogs is Free and Open
Source Software released by the European Space Agency under the Apache License 2.0. See NOTICE.txt
in the repository for full licensing details.

Performance will vary. Our testing shows approximately 1 arcsecond RMS tracking of stars and 
MEO/GEO satellites, with 4 arcsecond RMS tracking of LEO satellites. With this performance you
can launch the received signal into a 50µm and 150µm core diameter multimode fibre respectively with
the proposed FPA. We require no modifications to the telescope nor a fine steering mirror for these
results; pypogs will enable the lowest cost high-performance optical ground stations yet.

An article describing the system will be presented at IEEE ICSOS in 2019. The GitHub respository
includes a preprint (to be updated after publication). If you find pypogs useful in your work,
please cite:

G. M. Pettersson, J. Perdigues, and Z. Sodnik, "Unmodified Portable Telescope for Space-to-Ground
Optical Links," in *Proc. IEEE International Conference on Space Optical Systems and Applications
(ICSOS)*, 2019.
