Installation
============

Getting Python
--------------
pypogs is written for Python 3.7 (and therefore runs on almost any platform) and should work with
most modern Python 3 installations. However, it is strongly recommended to use a separate Python
environment for pypogs to avoid issues with clashing or out-of-date requirments as well as keeping
your environment clean. Therefore the preferred method of getting Python is by
`downloading Miniconda for Python 3+ <https://docs.conda.io/en/latest/miniconda.html>`_. If you are
on Windows you will be given the option to `add conda to PATH` during installation. If you do not
select this option, the instructions (including running Python and pypogs) which refer to the
terminal/CMD window will need to be carried out in the `Anaconda Prompt` instead.

Now, open a terminal/CMD window and test that you have conda and python by typing::

    conda
    python
    exit()
    
(On Windows you may be sent to the Windows Store to download Python on the second line. Do not do
this, instead go to `Manage app execution aliases` in the Windows settings and disable the python
and python3 aliases to use the version installed with miniconda.)

Now ensure you are up to date::

    conda update conda
    
You can now run any Python script by typing ``python script_name.py``.

Create environment for pypogs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Now create an environment specific for pypogs by::

    conda create --name pypogs_env python=3.7 pip
    conda activate pypogs_env
    
You will need to activate the environment each time you restart the terminal/CMD window. (To go to
your base environment type ``conda deactivate``.)

Getting pypogs
--------------
pypogs is not available on PIP/PyPI (the Python Package Index). Instead you need to get the software
repository from GitHub and place it in the directory where you wish to use it. (I.e. as a folder
next to the Python project you are writing, or alone if you will just use the provided Graphical
User Interface).

The quick way
^^^^^^^^^^^^^
Go to `the GitHub repository <https://github.com/esa/pypogs>`_, click `Clone or Download` and
`Download ZIP` and extract the pypogs directory to where you want to use it. You will notice that
the `pypogs/tetra3` directory is empty (thus pypogs will not work). Therefore also follow the link
to tetra3 in the GitHub repository, download it the same way and place the contents in the
`pypogs/tetra3` directory.

The good way
^^^^^^^^^^^^
To be able to easily download and contribute updates to pypogs you should install Git. Follow the
instructions for your platform `over here <https://git-scm.com/downloads>`_.

Now open a terminal/CMD window in the directory where you wish to use pypogs and clone the
GitHib repository::

    git clone --recurse-submodules "https://github.com/esa/pypogs.git"
    
You should see the `pypogs` directory created for you with all neccessary files, including that the
`pypogs/tetra3` directory Check the status of your repository by typing::

    cd pypogs
    git status
    
which should tell you that you are on the branch "master" and are up to date with the origin (which
is the GitHub version of pypogs).

If you find that `pypogs/tetra3` is empty (due to cloning without recursion), get it by::

    git submodule update --init

If a new update has come to GitHub you can update yourself by
typing::

    git pull --recurse-submodules

If you wish to contribute (please do!) and are not familiar with Git and GitHub, start by creating
a user on GitHub and setting you username and email::

    git config --global user.name "your_username_here"
    git config --global user.email "email@domain.com"

You will now also be able to push proposed changes to the software. There are many good resources
for learning about Git, `the documentation <https://git-scm.com/doc>`_ which includes the reference,
a free book on Git, and introductory videos is a good place to start. Note that if you contribute
to tetra3 you should push those changes to <https://github.com/esa/tetra3.git>.

Installing pypogs
-----------------
To install the requirements open a terminal/CMD window in the pypogs directory and run::

    pip install -r requirements.txt
    
to install all requirements. Test that everything works by running an example::

    cd examples
    python run_pypogsGUI.py
    
which should create an instace of pypogs and open the Graphical User Interface for you.

Adding hardware support
-----------------------
To connect hardware devices you need to download and install both drivers and Python packages for
the respective device.

Missing support for your hardware? `File an issue <https://github.com/esa/pypogs/issues>`_ and we
will work together to expand pypogs hardware support!

Mount: Celestron, Orion, or SkyWatcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Use `model='celestron'` in pypogs. No extra drivers or packages are neccessary.

Camera: FLIR/PointGrey
^^^^^^^^^^^^^^^^^^^^^^
Use `model='ptgrey'` in pypogs. You will need both the `FLIR Spinnaker` drivers and the `pyspin`
Python package. How to install:
	
    1. Go to `FLIR's Spinnaker page <https://www.flir.com/products/spinnaker-sdk/>`_, and click
       `Download Now` to go to their "download box". Download 
       `Latest Spinnaker Full SDK/SpinnakerSDK_FULL_{...}.exe` for your system
       (most likely Windows x64), and `Latest Python Spinnaker/spinnaker_python-{....}.zip`. Take 
       note that the python .zip should say `cp37` (for Python 3.7) and `amd64` for x64 system or
       `win32` for x86 system (matching the SpinnakerSDK you downloaded previously).  
    2. Run the Spinnaker SDK installer, "Camera Evaluation" mode is sufficient for our needs.
    3. Run the newly installed SpinView program and make sure your camera(s) work as expected.
    4. Extract the contents of the spinnaker_python .zip archive.
    5. Open a terminal/CMD window (or Anaconda Prompt); activate your pypogs_env environment. Go to
       the extracted .zip archive, where the file `spinnaker_python-{...}.whl` is located.
    6. Install the package by::
           
           pip install spinnaker_python-{...}.whl
	
You should now be ready to load and use your FLIR/PointGrey camera! Try running the GUI and add the
camera with model `ptgrey` and the serial number printed on the camera (and shown in SpinView).

Receiver: National Instruments DAQ
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Use `model='ni_daq'` in pypogs. Yoy will need both the `NI DAQmx` drivers and the `nidaqmx` Python
package.

TODO: Detailed steps.

If problems arise
-----------------
Please get in touch by `filing an issue <https://github.com/esa/pypogs/issues>`_.