# PARODIS - Pareto Optimal MPC for (discrete) Distributed Systems
PARODIS is a MATLAB framework for Pareto Optimal (scenario-based economic) Model Predictive Control for Distributed Systems. The main strength of PARODIS is, that the underlying optimization problem can be formulated completely symbolically and parametrically. PARODIS uses the YALMIP library by Johan Löfberg for the symbolic representation of optimization problems and for interfacing the numerical solvers.

PARODIS was created by Thomas Schmitt, Jens Engel and Matthias Hoffmann at the [Control Methods and Robotics institute at TU Darmstadt](https://www.rmr.tu-darmstadt.de/).

**Contact:** Thomas Schmitt - [thomas.schmitt@rmr.tu-darmstadt.de](mailto:thomas.schmitt@rmr.tu-darmstadt.de) (for support issues, please open an issue here on GitHub)

## Features
- Fully symbolic and parametric problem description
- Support of wide class of problems through use of YALMIP and the the supported LP, QP and NLP solvers
- (Interactive) Pareto optimization for Pareto optimal MPC, i.e. controlling a system by dynamically finding a compromise solution between multiple objectives. For more information, see the [Wiki](https://github.com/teamparodis/parodis/wiki/)
- Integrated support of distributed or hierarchical MPC through agent-based simulation
- Direct consideration of (predicted) disturbances on system dynamics $`x(k+1) = f(x(k), u(k), d(k))`$
- Efficient problem representation through use of YALMIP's `optimizer` to pre-compile optimization problems
- On-the-fly evaluation of simulation results using user-defined evaluation functions
- Easy-to-use (live) plotting

## Requirements
PARODIS is compatible with any MATLAB Release upwards from R2018a. It further requires a current installation of [YALMIP](https://yalmip.github.io/download/) by Johan Löfberg.

No additional toolboxes are needed, though the MATLAB control toolbox may be recommended, if you need tools for discretizing time-continuous models.

## Installation
To install PARODIS, [download](https://github.com/teamparodis/parodis/releases/latest) the latest release or clone this repository to your desired destination. Then, open MATLAB and run the install script. This script will add the correct directories to your MATLAB path and check if [YALMIP](https://yalmip.github.io/download/) is installed.

```matlab
cd YourPARODISdirectoryGoesHere
install
```

## Getting Started
For a quick glance into how PARODIS works, check out the provided examples in the `examples` directory. Just as PARODIS itself, they don't depend on anything but YALMIP and can be run just like that.

If you want to find out how to use PARODIS and implement your own models and problems, check out the [Getting Started](https://github.com/teamparodis/parodis/wiki/Home) section in the Wiki.

## Documentation
For a detailed description and documentation of how to use PARODIS and its components, check out our [Wiki](https://github.com/teamparodis/parodis/wiki/).

## Attribution & Contribution
PARODIS is released under the BSD-2 license, so feel free to modify and use PARODIS as you wish. We'd really appreciate it though, if you would maybe put some kind of attribution in your project :)

Furthermore, you're very much encouraged to contribute to the development of PARODIS by telling us about any issues you encounter by opening an issue (duh) or by even contributing directly and asking us to pull your fixes.
