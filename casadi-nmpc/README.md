# CasADi NMPC

An interface to a CasADi C export of an NMPC for double integrator translational dynamics.

* NLP Solver: sqpmethod (CasADi built-in)
* Integrator: RK4 (CasAdi built-in)
* Optimal Control Problem (OCP) Interface: CasADi Opti interface
* Automatic Differentiation: CasADi AD

`casadi_definition/` contains the description of the NMPC problem. The CasADi description of the NMPC is provided in `3dof_double_integrator_casadi.m`
The code generated here is used to produce `libcasdi_nmpc_export.so`.

`casadi_nmpc_nodelet.cc` is a ROS nodelet interface to the underlying CasADi NMPC C code. This interface is specified by
`3dof_double_integrator.h`.

## Usage

Run this nodelet by inserting into the chained Astrobee launchfiles in `llp.launch`, or run independently with `roslaunch casadi_nmpc casadi_nmpc.launch`.

Activate different control_modes using `execute_asap`, like `rosrun execute_asap execute_asap.py 7`, or writing to `td/tube_mpc/control_mode` directly.

## Requirements

* Python >=3.5 is needed for Pytope. `scripts/` contains a modified version of Pytope that works for 3.5. pycddlib must be manually installed.

* You must install the CasADI shared library. It can be compiled from source in your favorite directory as follows. Alternatively, just go to `external/casadi`:
Now you can build CasADi! From the top-level directory:
```
cd casadi && mkdir build && cd build
cmake ..
make
sudo make install
```

If you want to build CasADi on your own:
```
git clone https://github.com/casadi/casadi
cd casadi/external_packages/osqp/osqp
git submodule update --remote --merge --init .
cd lin_sys/direct/qdldl/qdldl_sources
```

If building from CasADi-exported C code you also need OSQP:
```
git clone https://github.com/jgillis/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build . --target install
```

*libcasadi_nmpc may not find your libcasadi!* In this case, you must update your $LD_LIBRARY_PATH to include `/usr/local/lib` (or wherever `make install` put your casadi.so). You can also edit `/etc/ld.so.conf` and add your library path (e.g., `/usr/local/lib`), the run `sudo ldconfig`.

Pytope has some requirements, mainly for Python3 ROS compatibility and libcdd.

Notes and Useful commands:
Debian OS packages are often compatible; 16.04 derives from stretch
`apt-get download $DEB:$ARCH`

For source:
Download everything
`pip3 download --no-binary ":all:" --dest "./" $PKG` (works best if also from a 16.04 system)
Install everything
`pip3 install $PKG -f ./ --no-index`

For wheels:
`pip download --index-url=https://www.piwheels.org/simple --platform linux_armv7l --no-deps -r requirements.txt`
https://stackoverflow.com/questions/60182080/install-wheel-file-on-off-line-machine-which-has-different-processor
`pip3 download --dest "./" $PKG -i https://www.piwheels.org/simple --no-deps`

export LC_ALL=C needed for pip to work

## Set up pip3
`sudo dpkg -i $PACKAGE`
* python-pip-whl (deb)[https://packages.ubuntu.com/xenial/all/python-pip-whl/download
* python3-wheel (deb)[https://packages.ubuntu.com/xenial/all/python3-wheel/download]
* python3-pip (deb)[https://packages.ubuntu.com/xenial/all/python3-pip/download]
* python3-setuptools(deb)[https://packages.ubuntu.com/xenial/all/python3-setuptools/download]

## Set up ROS Kinetic compatibility for Python3
`sudo dpkg -i $PACKAGE`
* python3-yaml (deb)[https://packages.debian.org/stretch/armhf/python3-yaml/download]
* python3-roman(deb)[https://packages.ubuntu.com/xenial/all/python3-roman/download]
* python3-dateutil(deb)[https://packages.ubuntu.com/xenial/python3-dateutil]
* python3-docutils(deb)[https://packages.ubuntu.com/xenial/all/python3-docutils/download]
* python3-pyparsing(deb)[https://packages.ubuntu.com/xenial/all/python3-pyparsing/download]
* python3-catkin-pkg-modules(deb)[https://index.ros.org/d/python3-catkin-pkg-modules/]
* python3-rospkg-modules(deb)[https://index.ros.org/d/python3-rospkg-modules/]

## Set up special Pytope dependencies
`sudo dpkg -i $PACKAGE`
`pip3 install $PKG -f ./ --no-index`
* python3-numpy (deb)[https://packages.debian.org/stretch/python3-numpy]
* python3-decorator(deb)[https://packages.debian.org/stretch/python3-decorator]
* python3-scipy (deb)[https://packages.debian.org/stretch/python3-scipy]

* python3-tk (deb)[https://packages.ubuntu.com/xenial/python3-tk]
* python3-cycler (deb)[https://packages.ubuntu.com/xenial/python3-cycler]
* python3-tz (deb)[https://packages.ubuntu.com/xenial/python3-tz]

* libgmpxx4ldbl (deb) [https://packages.ubuntu.com/xenial/libgmpxx4ldbl]
* libgmp10 (deb) [https://packages.ubuntu.com/search?keywords=libgmp10]
* libgmp-dev (GNU multiprecision library) (deb)[https://packages.ubuntu.com/xenial/libgmp-dev]

* cython (tar-src)[https://packages.ubuntu.com/xenial/amd64/cython/download]
* pycddlib (tar-src)[https://pypi.org/project/pycddlib/#files]
* matplotlib (tar-src)[https://pypi.org/project/matplotlib/#files] currently not used!


Test for LLP: can we run `rosrun casadi_nmpc z_poly_calc.py` and
`rosrun casadi_nmpc service_call_tester.py` successfully?

## Astrobee python3
Astrobee has the following Python3 packages installed:
* libpython3-stdlib:armhf
* libpython3.5:armhf
* libpython3.5-minimal:armhf
* libpython3.5-stdlib:armhf
* python3
* python3-minimal

## CasADi API

The CasADi API is described in section 5.2 and 5.3 of the manual [here](https://web.casadi.org/docs/#syntax-for-generating-code). Here, method 5.2.1 is used---a shared library is produced, and CasADi's C++ interface is used to call it using `external`.

An example of this usage is available [here](https://github.com/casadi/casadi/blob/master/docs/examples/cplusplus/codegen_usage.cpp#L199).

## Behavior

This nodelet runs passively and loops until chaser_coordinator_nodelet sets `td/tube_mpc/control_mode`, at which point the controller is either in tracking mode
`track` or regulating mode `regulate`. Regulating mode takes a setpoint and holds, while tracking modes follows a dynamic trajectory that has
been sent to the nodelet. To turn off control, chaser_coordinator may once again set `td/tube_mpc/control_mode` to `inactive`.

When tracking, commands will be sent at a predefined rate, `command_rate`. This may differ from the trajectory rate, `traj_rate`.

## Tube MPC Explanation

Two main components are required for a Tube MPC: (1) a nominal MPC planner that operates using restricted constraints and (2) an ancillary tube controller that operates on top of the MPC.

Nominal trajectory setpoints (including state and input) are provided to the nominal MPC planner, which provides a set of *nominal inputs*, v, and *nominal states*, z. Note that the nominal state need not correspond to the current true state. Time horizons, constraints, Q and R weightings, and an uncertainty bound are also specified. Tightened limits are produced and an ancillary controller gain is calculated. Finally, the nominal MPC solution is computed over a time horizon, and is used in computed the ancillary control. The ancillary and nominal control are added to provide input.
