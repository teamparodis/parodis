# Building Energy Management System MPC Controller Example

This example consists of only one agent, which represents the energy management system of a company building.
All files for this agent, i.e. the model, cost functions, eval functions and parameter/disturbance sources can be found in the `agents/building_ems` directory.
In the `agents/Data/` folder, you may find the CSV file containing the real disturbances that are applied to the system during simulation.

This readme should give a brief overview over the system. For a thorough explanation of the model, the approach and the applied Pareto optimization, the user is refered to:

* [Multi-objective model
predictive control for microgrids](https://www.rmr.tu-darmstadt.de/media/rmr/eigeneseiten_rmr/tschmitt/submittedVerson_article_2020_Schmitt_et_al_Multi-Objective_Model_Predictive_Control_for_Microgrids.pdf), T. Schmitt, T. Rodemann, and J. Adamy, at - Automatisierungstechnik,
vol. 68, no. 8, pp. 687 – 702, 2020.

## System Model
The example system represents a company building, which has a
* Stationary buffer battery
* Solar system (Photo-voltaic)
* Combined heat and power plant (CHP)
* Gas radiator
* Electric air conditioning
* Connection to the public power grid

The states of the system are:
- The energy stored in the buffer battery `E(k)`
- The building temperature `theta_b(k)`

As inputs to the system are considered:
- The power drawn from/delivered into the public power grid `P_grid`
- The power generated from the CHP `P_chp`
- The heating power of the gas radiator `Q_rad`
- The cooling power of the air conditioning `Q_cool`

The system is disturbed by the base load demand of the building `P_dem`, the solar irradiation `P_ren` and the outside temperature `theta_air`.

## Disturbances
For the prediction of the disturbances, a simple disturbance model is used. See `agents/building_ems/dist_prediction.m` for more details. These disturbances are used for calculating the cost function and deriving an optimal input. The real disturbance which will be applied to the system is retrieved from the CSV file `agents/Data/disturbances.csv`.

Note: We assume that the disturbances in the current time step `k` can always be measured. Therefore, we use the `agent.config.disturbanceMeasured = true` flag, which will set the `d_predicted(k) = d_real(k)` for the current time step. All other disturbances in the current prediction horizon are generated from the prediction model.

## MPC
The system is to be controlled using MPC, such that the monetary operational costs are minimised and the inside temperature in the building is kept at around 21°C.

The first objective is reflected by summing the electricity costs and the gas costs, as well as considering peak costs: The maximal peak demand shall be kept low, favorably below an initially set peak.

The second objective if reflected by minimising the quadratic deviation of the state `theta_b(k)` from a temperature set point of `theta_ref = 21°C`

## Pareto Optimization
The example system considers two cost functions. In this example, the `ParetoController` is used with AWDS as the front determination scheme. This means, that the two objectives are not weighted statically, but that at each time step within the simulation, a Pareto front will be generated and from it a compromise solution for the weighting selected. For more information on Pareto optimization, the reader is refered to the [Wiki](https://github.com/teamparodis/parodis/wiki) and the paper mentioned above.

## Running the example
Simply run the `building_control_example.m` script. If you want to turn on live plotting, set the flag `sim.config.livePlot = true` in the script. If you want to turn of Pareto optimization and just run the system classicaly, open `agents/building_ems/create_ems_ansatz.m` and replace the line
```matlab
controller = ParetoController(1);
```
by
```matlab
controller = SymbolicController(1);
```
