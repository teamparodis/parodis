# CCTA Hierarchical Energy Management System Example

This is the example system that was presented in the conference publication of the PARODIS framework.
It is the model of  an energy management system of an office building, which is controlled using a hierarchical MPC approach.
The model consists of a higher level, representing the building with all components and the consideration of one global temperature zone.
The lower level represents the 9 seperate temperature zones of the building.

The higher level controller will derive an optimal input for the entire building using Pareto optimization and the conflicting objectives of minimizing monetary costs and minimizing temperature deviation from 21°C.
The lower level then distributes the allotted total heating/cooling power between the 9 temperature zones using economic MPC to minimize the individual rooms' temperature deviation.

All files for the agents, i.e. the model, cost functions, eval functions and parameter/disturbance sources can be found in the `agents` directory.
In the `/data/` folder, you may find the CSV file containing the real disturbances that are applied to the system during simulation.

This readme should give a brief overview over the system. For a thorough explanation of the model, the approach and the applied Pareto optimization, the user is refered to:

* [PARODIS: One MPC framework to control them all. Almost.](https://tuprints.ulb.tu-darmstadt.de/18600/), T. Schmitt, J.Engel, M. Hoffmann, T. Rodemann, 2021 IEEE Conference on Control Technology and Applications (CCTA), San Diego, Calfiornia, 8.8. - 11.8.2021, DOI: 10.26083/tuprints-00018600, 
* [Multi-objective model predictive control for microgrids](https://www.honda-ri.de/pubs/pdf/4361.pdf), T. Schmitt, T. Rodemann, and J. Adamy, at - Automatisierungstechnik, vol. 68, no. 8, pp. 687 – 702, 2020.

## System Model
The example system represents a company building, which has a
* Stationary buffer battery
* Solar system (Photo-voltaic)
* Combined heat and power plant (CHP)
* Gas radiator
* Electric air conditioning
* Connection to the public power grid

The states of the higher level are:
- The energy stored in the buffer battery `E(k)`
- The building temperature `theta_b(k)`

As inputs to the higher level are considered:
- The power drawn from/delivered into the public power grid `P_grid`
- The power generated from the CHP `P_chp`
- The heating power of the gas radiator `Q_rad`
- The cooling power of the air conditioning `Q_cool`

The system is disturbed by the base load demand of the building `P_dem`, the solar irradiation `P_ren` and the outside temperature `theta_air`.

In the lower level, there is one state for each temperature zone, i.e. `theta_i(k)`.
There are further two inputs to each temperature zone, `Q_heat,i(k)` for the heating power and `Q_cool,i(k)` for the cooling power.
The temperature zones are thermally coupled with each other and are disturbed by the outside temperature `theta_air`.

## Disturbances
For the prediction and realisation of the disturbances, measured historical data is used. These can be found in the `data` directory.
In this example, the real and predicted disturbances are assumed to be the same, i.e. we assume perfect predictions.

## MPC
The system is to be controlled using MPC, such that the monetary operational costs are minimised and the inside temperature in the building is kept at around 21°C.

The first objective is reflected by summing the electricity costs and the gas costs, as well as considering peak costs: The maximal peak demand shall be kept low, favorably below an initially set peak.

The second objective if reflected by minimising the quadratic deviation of the state `theta_b(k)` from a temperature set point of `theta_ref = 21°C`

## Pareto Optimization
The high level system considers two cost functions. In this example, the `ParetoController` is used with AWDS as the front determination scheme. This means, that the two objectives are not weighted statically, but that at each time step within the simulation, a Pareto front will be generated and from it a compromise solution for the weighting selected. For more information on Pareto optimization, the reader is refered to the [Wiki](https://github.com/teamparodis/parodis/wiki) and the paper mentioned above.

## Running the example
Simply run the `ccta_main_example.m` script. If you want to turn on live plotting, set the flag `sim.config.livePlot = true` in the script. If you want to turn of Pareto optimization and control run the system "classicaly", open `createAnsatz_HL.m` and replace the line
```matlab
controller = ParetoController();
```
by
```matlab
controller = SymbolicController();
```
